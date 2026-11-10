#!/usr/bin/env python3
"""Owned PX4 aircraft lifecycle, stock helm, and independent flight observations."""
import argparse
import json
import math
from pathlib import Path
import re
import signal
import subprocess
import time

from run_fixed_wing import AIRFRAME_PINS, SITE_ALTITUDE_M, FreshStableWorkflow
from run_px4_offboard import FIXTURE_PARAMS, HERE, HOLD, OFFBOARD, STABLE_PIN, fields, main, need

TAKEOFF, RETURN, LAND = ((4 << 16) | (submode << 24) for submode in (2, 5, 6))
VTOL_FORWARD, VTOL_HOVER = 4, 3
VTOL_PINS = {
    'etc/init.d-posix/airframes/10043_sihsim_standard_vtol':
        'e61788a035dc74155ca234f9fa77d33b494ad62135af36482fa6ba5761ebed37',
    'etc/init.d/rc.vtol_defaults':
        '3f02908764d25387b9d44f3e4b2602b6744d847d0a59197ffda8d430939d1dec',
    'etc/init.d/rc.vtol_apps':
        '5c5f95bfac70e53df948849a2123f1507e06c0d3108053a9f955d1fb0986d3ab',
    'etc/init.d/rc.mc_defaults':
        '7b2cd6724104c10ffd048b53656e36e61aaeccfbb9e9c1ab09ad0261f478f711',
    'etc/init.d/rc.fw_defaults': AIRFRAME_PINS['etc/init.d/rc.fw_defaults'],
}
VEHICLES = {
    'quadrotor': ('sihsim_quadx', 'px4_multicopter', 2, 10040, r'Quadcopter', 8, 8, 2, {}),
    'fixed-wing': ('sihsim_airplane', 'px4_fixed_wing', 1, 10041, r'(?i)Fixed[- ]?Wing|Airplane',
                   45, 60, 16, AIRFRAME_PINS),
    'standard-vtol': ('sihsim_standard_vtol', 'px4_standard_vtol', 22, 10043, r'(?i)Standard.*VTOL|VTOL',
                      30, 30, 10, VTOL_PINS),
}


class AircraftWorkflow(FreshStableWorkflow):
    template = 'meta_aircraft'
    probe_args = ('--aircraft',)
    scope = 'owned PX4 v1.17.0 SIH aircraft native lifecycle and stock helm flight'

    def __init__(self, args, out, report):
        spec = VEHICLES[args.vehicle]
        (self.model, self.profile, self.mav_type, self.airframe, self.stable_model_label,
         self.takeoff_height, self.helm_height, self.forward_speed, _) = spec
        self.vtol = args.vehicle == 'standard-vtol'
        self.plane = args.vehicle == 'fixed-wing'
        self.fixture_params = {
            **FIXTURE_PARAMS, 'MIS_TAKEOFF_ALT': self.takeoff_height,
            'COM_DISARM_LAND': -1, 'RTL_LAND_DELAY': -1,
            'RTL_RETURN_ALT': 55 if self.plane else 32 if self.vtol else 14,
            'RTL_DESCEND_ALT': 30 if self.plane else 26 if self.vtol else 8,
        }
        if self.plane:
            self.fixture_params['NAV_LOITER_RAD'] = 55
        super().__init__(args, out, report)
        self.deadline += 175
        self.env['PX4_HOME_ALT'] = str(SITE_ALTITUDE_M)
        self.autonomy = getattr(args, "autonomy", False)
        self.stress = getattr(args, 'stress', False)
        self.command_sequence = 0
        self.report.update(vehicle=args.vehicle, model=self.model, profile=self.profile, airframe=self.airframe,
                           native_actions=[], legs=[], transitions=[])

    def fixture_sources(self):
        return [*super().fixture_sources(), Path(__file__), HERE / "meta_vtol_autonomy.bhv"]

    def generate(self, origin):
        moos = dict(MOOS_PORT=self.db, DB_PATH=self.bindir / 'MOOSDB', APP_PATH=self.app,
                    HELM_PATH=self.helm, LOGGER_PATH=self.bindir / 'pLogger', SITL_PID=self.px4.pid,
                    PLUGIN_DIRECTORY=self.build / 'plugins', BEHAVIOR_DIRECTORY=self.args.behavior_dir,
                    LAT_ORIGIN=origin['latitude_e7'] / 1e7, LONG_ORIGIN=origin['longitude_e7'] / 1e7,
                    EXPECTED_TYPE=self.mav_type, AIRCRAFT_PROFILE=self.profile)
        behavior = dict(INITIAL_LEG='forward' if self.plane else 'hover',
                        FORWARD_SPEED=self.forward_speed, HELM_HEIGHT=self.helm_height)
        for suffix, settings in (('moos', moos), ('bhv', behavior)):
            source = HERE / ('meta_vtol_autonomy.bhv' if self.autonomy and suffix == 'bhv' else f'meta_aircraft.{suffix}')
            target = self.out / f'targ_px4.{suffix}'
            subprocess.run([str(self.nsplug), str(source), str(target), '--strict', '--force', '-x',
                            *(f'{key}={value}' for key, value in settings.items())],
                           cwd=self.out, env=self.env, capture_output=True, text=True, timeout=5, check=True)
            need(target.is_file() and '$(' not in target.read_text(), 'unresolved native aircraft mission')
        return self.out / 'targ_px4.moos'

    def command(self, action, *, height=None):
        self.command_sequence += 1
        identity = f'{action}.{self.command_sequence}'
        values = f'id={identity},action={action}'
        if height is not None:
            values += f',height={height}'
        at = self.poke('MAVMOOS_COMMAND', values)
        return identity, at

    def command_rows(self, identity, at):
        return [row for row in self.rows if row.get('kind') == 'mail' and row.get('source') == 'iMAVLink' and
                row.get('key') == 'MAVMOOS_COMMAND_RESULT' and row['mono'] >= at and fields(row).get('id') == identity]

    def await_result(self, operation, action, status='COMPLETED', timeout=10):
        identity, at = operation

        def terminal():
            for row in self.command_rows(identity, at):
                result = fields(row)
                if result['status'] == status:
                    return row
                if result['status'] in ('FAILED', 'REJECTED', 'TIMED_OUT', 'CANCELLED', 'NO_OP'):
                    raise RuntimeError(f'{identity} failed in bridge: {row["value"]}')
            return None

        final = self.wait(timeout, terminal, f'no {status} from MAVMOOS {identity}')
        rows = self.command_rows(identity, at)
        observed = [fields(row)['status'] for row in rows]
        if status == 'COMPLETED':
            need(all(item in observed for item in ('ADMITTED', 'ACK_OBSERVED', 'CONFIRMED')),
                 f'{identity} lacks the admission, ACK and observed effect receipts')
            tx = [row for row in self.rows if row.get('source') == 'iMAVLink' and
                  row.get('key') == 'MAVMOOS_TX_RESULT' and row['mono'] >= at and
                  fields(row).get('id') == identity and fields(row).get('status') == 'LOCAL_ACCEPTED']
            need(len(tx) == 1, f'{identity} lacks a single actual local send')
        else:
            tx = [row for row in self.rows if row.get('source') == 'iMAVLink' and
                  row.get('key') == 'MAVMOOS_TX_RESULT' and fields(row).get('id') == identity]
            need(not tx, f'{identity} was rejected after reaching the send path')
        evidence = {'id': identity, 'action': action, 'requested_at': at, 'result': final,
                    'stages': rows, 'send': tx}
        self.report['native_actions'].append(evidence)
        return evidence

    def grounded(self, armed, after=0):
        hb = self.latest('wire', 'HEARTBEAT', after)
        state = self.latest('wire', 'EXTENDED_SYS_STATE', after)
        return {'heartbeat': hb, 'state': state} if hb and state and hb['mav_type'] == self.mav_type and \
            hb['autopilot'] == 12 and hb['armed'] is armed and state['landed_state'] == 1 else None

    def airborne_hold(self, after, height=None):
        hb = self.air_mode(HOLD, after)
        mode = self.current_mode(HOLD, HOLD, after)
        position = self.latest('wire', 'GLOBAL_POSITION_INT', after, age=.7)
        if hb and mode and position and (height is None or abs(position['relative_m'] - height) <= (6 if self.plane else 1.2)):
            return {'heartbeat': hb, 'mode': mode, 'position': position}
        return None

    def nav_counts(self):
        state = fields(self.runtime())
        return int(state['nav_tx_accepted']), int(state['px4_hold_tx_accepted'])

    def fresh_helm(self, speed, after=0):
        result = {}
        for key, desired in (('DESIRED_SPEED', speed), ('DESIRED_ALTITUDE', self.helm_height)):
            row = self.latest('mail', key, after, age=.6)
            if not row or row['source'] != 'pHelmIvP' or not isinstance(row['value'], (int, float)) or \
                    abs(row['value'] - desired) > .05:
                return None
            result[key] = row
        for key, expected in (('IVPHELM_STATE', 'DRIVE'), ('DESIRED_HEADING', None)):
            row = self.latest('mail', key, after, age=.6)
            if not row or row['source'] != 'pHelmIvP' or (expected is not None and row['value'] != expected):
                return None
            result[key] = row
        return result

    def leg(self, name, speed, east_distance, *, selection=None, survey=False, north_distance=None):
        self.poke('AIRCRAFT_LEG', selection or name)
        deploy = self.poke('DEPLOY', 'true')
        helm = self.wait(7, lambda: self.fresh_helm(speed, deploy), f'no fresh stock {name} tuple')
        self.wait(5, lambda: self.runtime(permission='false', px4_native_hold_ready=1),
                  'native Hold did not allow a fresh acquisition')
        nav, holds = self.nav_counts()
        acquire = self.poke('MAVMOOS_NAV_CONTROL', 'acquire')
        offboard = self.wait(9, lambda: self.air_mode(OFFBOARD, acquire) and self.current_mode(OFFBOARD, OFFBOARD, acquire),
                             f'no independent Offboard after {name} acquisition')
        start = self.wait(3, lambda: self.latest('wire', 'LOCAL_POSITION_NED', offboard['mono']),
                          f'no independent start position for {name} leg')

        def progressed():
            position = self.latest('wire', 'LOCAL_POSITION_NED', start['mono'], age=.7)
            status = self.runtime(acquire, permission='true', px4_offboard_confirmed=1)
            if position and status and position['east'] - start['east'] >= east_distance and \
                    (north_distance is None or position['north'] - start['north'] >= north_distance) and \
                    int(fields(status)['nav_tx_accepted']) >= nav + 10 and self.current_mode(OFFBOARD, OFFBOARD, acquire):
                return position
            return None

        end = self.wait(23, progressed, f'no measured eastward {name} flight under stock helm')
        if survey:
            north_gate = self.wait(20, lambda: self.latest('mail', 'AUTONOMY_INDEX', acquire, age=12),
                                   'stock survey index missing')
            def completed_survey():
                index = self.latest('mail', 'AUTONOMY_INDEX', north_gate['mono'], age=9)
                if index and index['source'] == 'pHelmIvP' and index['value'] == 2:
                    return index
                return None
            index = self.wait(30, completed_survey, 'stock hover survey did not reach its second waypoint')
            end = self.wait(3, progressed, 'survey did not preserve Offboard navigation')
            fixes = [row for row in self.rows if row.get('message') == 'LOCAL_POSITION_NED'
                     and start['mono'] <= row['mono'] <= index['mono']]
            need(fixes, 'independent survey position trace missing')
            a = max(row['north'] for row in fixes) - min(row['north'] for row in fixes)
            e = max(row['east'] for row in fixes) - min(row['east'] for row in fixes)
            need(a >= 7 and e >= 8, 'independent hover survey did not traverse both axes')
            self.report['autonomy_survey'] = {'index': index, 'north_span_m': a, 'east_span_m': e,
                                               'sample_count': len(fixes)}

        series = [fields(row) for row in self.rows if row.get('source') == 'iMAVLink' and
                  row.get('key') == 'MAVMOOS_RUNTIME_STATUS' and acquire <= row['mono'] <= offboard['mono']]
        prestream = max((int(row.get('px4_prestream_count', 0)) for row in series), default=0)
        hold_count = self.nav_counts()[1] - holds
        need(prestream >= 8 and hold_count >= 8 and offboard['mono'] - acquire >= 1.25,
             f'{name} Offboard lacked qualified prestream')
        self.report['legs'].append({'phase': name, 'stock_inputs': helm, 'acquired_at': acquire,
                                    'offboard': offboard, 'start': start, 'end': end, 'prestream': prestream,
                                    'hold_frames': hold_count, 'east_m': end['east'] - start['east']})

    def hold(self):
        operation = self.command('hold')
        result = self.await_result(operation, 'hold')
        independent = self.wait(6, lambda: self.airborne_hold(operation[1]), 'no independent native Hold')
        self.wait(3, lambda: self.runtime(result['result']['mono'], permission='false'), 'native Hold left helm permission')
        nav, holds = self.nav_counts()
        result['hold'] = independent
        self.wait(3, lambda: time.monotonic() >= independent['mode']['mono'] + 1 and
                  self.runtime(permission='false', nav_tx_accepted=nav, px4_hold_tx_accepted=holds),
                  'stock mail resumed sending without a fresh acquire')

    def revise_forward_route(self, nav, holds, phase_unknown):
        selected = self.poke('AIRCRAFT_LEG', 'forward')
        original = self.wait(4, lambda: self.fresh_helm(self.forward_speed, selected),
                             'stock forward course before reroute absent')
        reroute = self.poke('STRESS_ROUTE_UPDATE', 'points=350,200')

        def changed():
            tuple_ = self.fresh_helm(self.forward_speed, reroute)
            if tuple_:
                course = tuple_['DESIRED_HEADING']['value']
                delta = math.remainder(course - original['DESIRED_HEADING']['value'], 360)
                if delta <= -18:
                    return tuple_
            return None

        revised = self.wait(4, changed, 'stock waypoint did not turn left toward the revised forward route')
        denied_at = self.poke('MAVMOOS_NAV_CONTROL', 'acquire')

        def rejected():
            row = self.latest('mail', 'MAVMOOS_HELM_STATUS', denied_at, age=.5)
            return row if row and row['source'] == 'iMAVLink' and fields(row).get('reason') == 'acquisition_rejected' else None

        denied = self.wait(4, rejected, 'revised stock route was not denied with the forward phase unobserved')
        fenced = self.wait(3, lambda: self.runtime(denied['mono'], permission='false',
                           nav_tx_accepted=nav, px4_hold_tx_accepted=holds),
                           'reroute during phase loss emitted navigation')
        return {'selected_at': selected, 'original': original, 'reroute_at': reroute, 'revised': revised,
                'phase_unknown_at': phase_unknown['mono'], 'denied_at': denied_at,
                'denied': denied, 'fenced_runtime': fenced}

    def transition(self, action, transient, target):
        nav, holds = self.nav_counts()
        operation = self.command(action)
        identity, at = operation
        self.wait(4, lambda: any(fields(row)['status'] == 'ACK_OBSERVED' for row in self.command_rows(identity, at)),
                  f'no addressed ACK for {action}')
        acquire = self.poke('MAVMOOS_NAV_CONTROL', 'acquire')
        fault = None
        if self.args.transition_observation_loss and action == 'transition_fw':
            self.cli('mavlink', 'stream', '-u', '14580', '-s', 'EXTENDED_SYS_STATE', '-r', '0')
            lost_at = self.event('bridge_vtol_phase_stream_disabled')
            stable = self.wait(25, lambda: self.phase_sample(target, lost_at), 'independent transition did not finish')

            def still_fenced():
                phase = self.latest('mail', 'MAVMOOS_PX4_VTOL_PHASE', stable['mono'], age=.5)
                need(not any(fields(row)['status'] in ('CONFIRMED', 'COMPLETED')
                             for row in self.command_rows(identity, at)),
                     'transition was confirmed without post-fault aircraft phase')
                fenced = self.runtime(stable['mono'], permission='false',
                                      nav_tx_accepted=nav, px4_hold_tx_accepted=holds)
                if phase and phase['source'] == 'iMAVLink' and phase['value'] == 0 and fenced:
                    return {'bridge_unknown_phase': phase, 'fenced_runtime': fenced}
                return None

            fault = self.wait(4, still_fenced, 'bridge did not fence authority with an unknown phase')
            fault.update(independent_stable=stable, disabled_at=lost_at)
            if self.stress:
                fault['route_change'] = self.revise_forward_route(nav, holds, fault['bridge_unknown_phase'])
                need(not any(fields(row)['status'] in ('CONFIRMED', 'COMPLETED')
                             for row in self.command_rows(identity, at)),
                     'native transition settled without restored bridge phase')
            self.cli('mavlink', 'stream', '-u', '14580', '-s', 'EXTENDED_SYS_STATE', '-r', '5')
            fault['restore_requested_at'] = self.event('bridge_vtol_phase_stream_restored')
        completed = self.await_result(operation, action, timeout=32)
        sample = self.wait(5, lambda: self.phase_sample(target, at), f'no independent stable {action} phase')
        transitional = [row for row in self.rows if row.get('message') == 'EXTENDED_SYS_STATE' and
                        at <= row['mono'] < sample['mono'] and row['vtol_state'] == transient]
        need(transitional and self.runtime(acquire, permission='false',
             nav_tx_accepted=nav, px4_hold_tx_accepted=holds), f'{action} did not fence the prior phase')
        transition = {'action': action, 'transient': transitional[0], 'stable': sample,
                      'attempted_acquisition': acquire, 'fenced_counts': [nav, holds]}
        if fault:
            need(completed['result']['mono'] >= fault['restore_requested_at'],
                 'transition completed before phase restoration')
            transition['observation_loss'] = fault
            if self.stress:
                observed = completed['result']['mono']
                fault['restored_quiet'] = self.wait(4, lambda: self.fresh_helm(self.forward_speed, observed)
                    and self.runtime(observed + .6, permission='false', px4_native_hold_ready=1,
                                     nav_tx_accepted=nav, px4_hold_tx_accepted=holds),
                    'restored phase or continued stock route bypassed a fresh acquire')
        self.report['transitions'].append(transition)

    def phase_sample(self, phase, after):
        row = self.latest('wire', 'EXTENDED_SYS_STATE', after)
        return row if row and row['vtol_state'] == phase and row['landed_state'] == 2 else None

    def home_distance(self, after=0):
        position = self.latest('wire', 'GLOBAL_POSITION_INT', after, age=.7)
        home = self.latest('wire', 'HOME_POSITION', age=8)
        if position and home:
            lat, lon = position['latitude_e7'] * 1e-7, position['longitude_e7'] * 1e-7
            north = (lat - home['latitude_e7'] * 1e-7) * 111195
            east = (lon - home['longitude_e7'] * 1e-7) * 111195 * math.cos(math.radians(lat))
            return {'distance_m': math.hypot(north, east), 'position': position, 'home': home}
        return None

    def fly_return(self):
        start = self.wait(3, self.home_distance, 'no independent HOME and aircraft position')
        need(start['distance_m'] >= 5, 'Return started too close to home to measure progress')
        operation = self.command('rtl')
        self.await_result(operation, 'rtl')
        mode = self.wait(6, lambda: self.current_mode(RETURN, RETURN, operation[1]), 'no independent native Return')
        decrease = min(12., start['distance_m'] * .35)

        def arrived_closer():
            sample = self.home_distance(mode['mono'])
            if sample and self.current_mode(RETURN, RETURN, operation[1]) and \
                    sample['distance_m'] <= start['distance_m'] - decrease:
                return sample
            return None

        end = self.wait(48, arrived_closer, 'native Return never moved sufficiently toward home')
        self.report['return_motion'] = {'mode': mode, 'start': start, 'end': end,
                                        'homeward_m': start['distance_m'] - end['distance_m']}

    def run(self):
        self.boot()
        startup = self.cli('param', 'show', 'SYS_AUTOSTART')
        need(re.search(r':\s*' + str(self.airframe) + r'(?:\s|$)', startup), 'stock SIH airframe differs')
        self.report['airframe_readback'] = startup
        for stream, rate in (('VFR_HUD', '5'), ('LOCAL_POSITION_NED', '10'), ('GLOBAL_POSITION_INT', '5'),
                             ('HOME_POSITION', '2'), ('EXTENDED_SYS_STATE', '5')):
            self.cli('mavlink', 'stream', '-u', '18570', '-s', stream, '-r', rate)
        self.wait(8, lambda: self.grounded(False) and self.latest('wire', 'HOME_POSITION'),
                  'grounded and disarmed stock aircraft absent')
        denied = self.command('takeoff', height=self.takeoff_height)
        self.await_result(denied, 'takeoff', 'REJECTED')
        for action, armed in (('arm', True), ('disarm', False), ('arm', True)):
            operation = self.command(action)
            evidence = self.await_result(operation, action)
            evidence['ground'] = self.wait(5, lambda: self.grounded(armed, operation[1]),
                                           f'no independent grounded {action}')
        takeoff = self.command('takeoff', height=self.takeoff_height)
        self.await_result(takeoff, 'takeoff', timeout=65)
        self.report['takeoff'] = self.wait(6, lambda: self.airborne_hold(takeoff[1], self.takeoff_height),
                                           'takeoff never reached independently measured airborne Hold')
        self.settle_stable_home(takeoff[1])
        self.leg('forward' if self.plane else 'hover', self.forward_speed if self.plane else 2,
                 32 if self.plane else 8, survey=self.autonomy)
        self.hold()
        if self.vtol:
            self.transition('transition_fw', 1, VTOL_FORWARD)
            self.leg('forward', self.forward_speed, 25, north_distance=7 if self.stress else None)
            self.hold()
            self.transition('transition_mc', 2, VTOL_HOVER)
        self.leg('forward' if self.plane else 'hover', self.forward_speed if self.plane else 2,
                 26 if self.plane else 8, selection='final' if self.autonomy else None)
        self.fly_return()
        land = self.command('land')
        landing = self.await_result(land, 'land', timeout=119)
        landing['ground'] = self.wait(5, lambda: self.grounded(True, land[1]), 'no independent armed touchdown')
        disarm = self.command('disarm')
        final = self.await_result(disarm, 'disarm')
        final['ground'] = self.wait(6, lambda: self.grounded(False, disarm[1]), 'no independent final disarm')
        self.report['landed_disarmed'] = final['ground']


def run_aircraft(vehicle, revision, output, build_manifest, *, build=Path('/candidate'), release_stock=False,
                 transition_observation_loss=False, autonomy=False, stress=False):
    pin = json.loads(STABLE_PIN.read_text())
    pin['files'].update(VEHICLES[vehicle][-1])
    args = argparse.Namespace(
        vehicle=vehicle, transition_observation_loss=transition_observation_loss, autonomy=autonomy,
        stress=stress, output=output, source_revision=revision, db_port=15300, px4_root=Path('/opt/px4'),
        moos_root=Path('/work/moos-ivp'), behavior_dir=Path('/work/altitude/build/lib'),
        build=build, probe=None, nsplug=None, stable_pin=pin, build_manifest=build_manifest, release_stock=release_stock,
    )
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(SystemExit(143)))
    return main(args, AircraftWorkflow)
