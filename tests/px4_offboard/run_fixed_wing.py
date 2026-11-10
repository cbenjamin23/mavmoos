#!/usr/bin/env python3
"""Owned stable PX4 fixed-wing flight with stock helm and independent telemetry."""
import argparse
import json
import math
import os
from pathlib import Path
import re
import signal
import time

from run_px4_offboard import (FIXTURE_PARAMS, HERE, HOLD, OFFBOARD, STABLE_PIN,
                              STABLE_PX4_REVISION, Workflow, fields, main, need)

MODEL = 'sihsim_airplane'
SITE_ALTITUDE_M = 15.0
AIRFRAME_PINS = {
    'etc/init.d-posix/airframes/10041_sihsim_airplane':
        '1541503171a0c8d9ecb70a4afafe430966d804b3988264073f4cdeaa4e26f54f',
    'etc/init.d/rc.fw_defaults':
        '10abe3578afc68023af339b586ae40a5ea7dde9172009c5f27fbc5c83ef7146f',
}
FRESH_BUILD_FILES = (
    'bin/iMAVLink', 'bin/px4_offboard_moos_probe',
    'plugins/mavmoos_telemetry.so', 'plugins/mavmoos_px4_control.so',
)
AIR_MIN, AIR_MAX = 10.0, 35.0
GROUND_MIN, GROUND_MAX = 8.0, 35.0
LAND = (4 << 16) | (6 << 24)


def stock_work_pins(stable, release_build=None):
    stock = {f'/work/{name}': digest for name, digest in stable['retained_work_files'].items()
             if not name.startswith('mavmoos-build/')}
    if release_build is None:
        return stock
    witnesses = release_build.get('stock_witness_sha256')
    need(isinstance(witnesses, dict) and witnesses.keys() == stock.keys() and
         all(isinstance(digest, str) and re.fullmatch(r'[0-9a-f]{64}', digest) for digest in witnesses.values()),
         'release build lacks the exact stock dependency witnesses')
    return witnesses


class FreshStableWorkflow(Workflow):
    """Reuse the stable fixture with a newly built, sealed bridge and probe."""

    def stable_work_pins(self, stable):
        for path in (self.build, Path('/source')):
            need(os.statvfs(path).f_flag & os.ST_RDONLY, f'{path} must be mounted read-only')
        candidate = json.loads(self.args.build_manifest.read_text())
        artifacts = candidate['artifacts']
        need(candidate['source_revision'] == self.args.source_revision and
             set(artifacts) == set(FRESH_BUILD_FILES) and
             all(re.fullmatch(r'[0-9a-f]{64}', value) for value in artifacts.values()),
             'candidate build manifest does not match the selected revision and artifacts')
        stock = stock_work_pins(stable, candidate if getattr(self.args, 'release_stock', False) else None)
        stock = {Path(name): digest for name, digest in stock.items()}
        return {**stock, **{self.build / name: digest for name, digest in artifacts.items()}}

    def fixture_sources(self):
        return [*super().fixture_sources(), Path(__file__), self.args.build_manifest]


class FixedWingWorkflow(FreshStableWorkflow):
    model = MODEL
    mav_type = 1
    profile = 'px4_fixed_wing'
    stable_model_label = r'(?i)Fixed[- ]?Wing|Airplane'
    template = 'meta_fixed_wing'
    fixture_params = {**FIXTURE_PARAMS, 'MIS_TAKEOFF_ALT': 45, 'NAV_LOITER_RAD': 55}
    probe_args = ('--fixed-wing',)
    scope = 'owned PX4 1.17.0 SIH airplane stock helm Offboard, handoff and failsafe Hold'

    def __init__(self, args, out, report):
        super().__init__(args, out, report)
        self.deadline += 100
        # Keep the stock SIH altitude-reference initialization close to the
        # flight site while still exercising a nonzero above-home conversion.
        self.env['PX4_HOME_ALT'] = str(SITE_ALTITUDE_M)
        self.minimum_east = 65.0
        self.report.update(profile=self.profile, model=self.model,
                           motion_limits={'airspeed_mps': [AIR_MIN, AIR_MAX],
                                          'ground_speed_mps': [GROUND_MIN, GROUND_MAX],
                                          'relative_altitude_m': [20, 130],
                                          'minimum_east_m': self.minimum_east})

    def stock_inputs(self, after=0):
        expected = {'DESIRED_HEADING': (35, 145), 'DESIRED_SPEED': (15.9, 16.1),
                    'DESIRED_ALTITUDE': (59.9, 60.1)}
        inputs = {}
        for key, (low, high) in expected.items():
            row = self.latest('mail', key, after, age=.55)
            if not row or row['source'] != 'pHelmIvP' or not isinstance(row['value'], (float, int)) or \
                    not low <= row['value'] <= high:
                return None
            inputs[key] = row
        drive = self.latest('mail', 'IVPHELM_STATE', after, age=.55)
        return inputs if drive and drive['source'] == 'pHelmIvP' and drive['value'] == 'DRIVE' else None

    def flight_sample(self, after=0):
        local = self.latest('wire', 'LOCAL_POSITION_NED', after, age=.6)
        global_position = self.latest('wire', 'GLOBAL_POSITION_INT', after, age=.6)
        hud = self.latest('wire', 'VFR_HUD', after, age=.6)
        if not local or not global_position or not hud or not isinstance(hud['airspeed'], (int, float)):
            return None
        speed = math.hypot(local['vx'], local['vy'])
        if not (GROUND_MIN <= speed <= GROUND_MAX and AIR_MIN <= hud['airspeed'] <= AIR_MAX and
                20 <= global_position['relative_m'] <= 130):
            return None
        return {'position': local, 'altitude': global_position, 'airspeed': hud,
                'ground_speed_mps': speed}

    def enter_offboard(self, hold_before):
        acquire = self.poke('MAVMOOS_NAV_CONTROL', 'acquire')
        offboard = self.wait(8, lambda: self.air_mode(OFFBOARD, acquire),
                             'fixed-wing never independently entered Offboard')
        sequence = [fields(row) for row in self.rows if row.get('key') == 'MAVMOOS_RUNTIME_STATUS' and
                    acquire <= row['mono'] <= offboard['mono']]
        prestream = max((int(row.get('px4_prestream_count', 0)) for row in sequence), default=0)
        holds = max((int(row.get('px4_hold_tx_accepted', 0)) for row in sequence), default=0) - hold_before
        need(prestream >= 10 and holds >= 10 and offboard['mono'] - acquire >= 1.25,
             'independent fixed-wing Offboard lacked the measured prestream')
        self.wait(4, lambda: self.runtime(acquire, permission='true', fc_mode=OFFBOARD,
                                         px4_offboard_confirmed=1, reason='px4_offboard_motion'),
                  'bridge did not accept fixed-wing motion after observed Offboard')
        return {'acquire_at': acquire, 'independent_offboard': offboard,
                'prestream_count': prestream, 'hold_tx_accepted': holds}

    def acquire_and_travel(self, nav_before, hold_before):
        entry = self.enter_offboard(hold_before)
        acquire, offboard = entry['acquire_at'], entry['independent_offboard']
        start = self.wait(2, lambda: self.flight_sample(offboard['mono']),
                          'no advancing independent fixed-wing airspeed and position after Offboard')

        def traveled():
            sample = self.flight_sample(offboard['mono'])
            status = fields(self.runtime())
            if not sample or not self.air_mode(OFFBOARD, acquire) or status.get('permission') != 'true':
                return None
            east = sample['position']['east'] - start['position']['east']
            if east < self.minimum_east or sample['position']['vy'] < 9 or \
                    not 48 <= sample['altitude']['relative_m'] <= 75 or \
                    int(status.get('nav_tx_accepted', 0)) - nav_before < 25:
                return None
            return sample

        end = self.wait(19, traveled, 'no sustained eastward fixed-wing motion with valid airspeed and height')
        rows = [row for row in self.rows if start['position']['mono'] <= row['mono'] <= end['position']['mono']]
        airspeeds = [row['airspeed'] for row in rows if row.get('message') == 'VFR_HUD']
        groundspeeds = [math.hypot(row['vx'], row['vy']) for row in rows
                        if row.get('message') == 'LOCAL_POSITION_NED']
        altitudes = [row['relative_m'] for row in rows if row.get('message') == 'GLOBAL_POSITION_INT']
        for samples, low, high, key in ((airspeeds, AIR_MIN, AIR_MAX, 'airspeed'),
                                        (groundspeeds, GROUND_MIN, GROUND_MAX, 'ground speed'),
                                        (altitudes, 20, 130, 'above-home altitude')):
            need(len(samples) >= 5 and all(isinstance(sample, (float, int)) and low <= sample <= high
                                           for sample in samples), f'independent {key} left the flight envelope')
        return {**entry, 'travel': {
                    'start': start, 'end': end, 'east_m': end['position']['east'] - start['position']['east'],
                    'airspeed_range_mps': [min(airspeeds), max(airspeeds)],
                    'groundspeed_range_mps': [min(groundspeeds), max(groundspeeds)],
                    'altitude_range_m': [min(altitudes), max(altitudes)],
                    'air_speed_samples': len(airspeeds), 'position_samples': len(groundspeeds),
                    'altitude_samples': len(altitudes)}}

    def normal_handoff(self):
        status = fields(self.runtime())
        holds = int(status['px4_hold_tx_accepted'])
        nav = int(status['nav_tx_accepted'])
        pause_at = self.poke('DEPLOY', 'false')

        def paused():
            row = self.runtime(pause_at, permission='false', reason='normal_handoff')
            stop = self.latest('mail', 'IVPHELM_ALLSTOP', pause_at, age=.6)
            if (row and stop and stop['source'] == 'pHelmIvP' and
                    stop['value'] in ('NothingToDo', 'NoGoalBehavior') and
                    int(fields(row)['px4_hold_tx_accepted']) > holds and self.air_mode(OFFBOARD)):
                return {'status': row, 'stock_allstop': stop, 'offboard': self.air_mode(OFFBOARD)}
            return None

        pause = self.wait(1.1, paused, 'stock helm all-stop did not enter the bounded fixed-wing handoff')
        resume_at = self.poke('DEPLOY', 'true')

        def resumed():
            row = self.runtime(resume_at, permission='true', px4_offboard_confirmed=1)
            if row and int(fields(row)['nav_tx_accepted']) > nav and self.stock_inputs(resume_at):
                return row
            return None

        resume = self.wait(1.6, resumed, 'fresh stock helm output did not resume after normal handoff')
        need(not any(row.get('key') == 'MAVMOOS_NAV_CONTROL' and row.get('kind') == 'request' and
                     pause_at <= row['mono'] <= resume['mono'] for row in self.rows),
             'normal handoff used an extra acquire')
        return {'pause': pause, 'resumed': resume, 'pause_request_at': pause_at, 'resume_request_at': resume_at}

    def site_home_altitude(self):
        report = self.cli('listener', 'home_position', '-n', '1')
        value = re.search(r'^\s*alt:\s*([-+0-9.]+)\s*$', report, re.M)
        need(value and re.search(r'^\s*valid_alt:\s*True\s*$', report, re.M) and
             math.isclose(float(value[1]), SITE_ALTITUDE_M, abs_tol=5),
             'independent PX4 HOME altitude does not match the nonzero SIH site')
        return float(value[1])

    def run(self):
        self.boot()
        setting = re.search(r'\bSIH_LOC_H0\b[^\r\n]*?:\s*([-+0-9.]+)',
                            self.cli('param', 'show', 'SIH_LOC_H0'))
        need(setting and math.isclose(float(setting[1]), SITE_ALTITUDE_M, abs_tol=.001),
             'stock SIH startup did not select the configured nonzero site')
        self.report['simulator_site'] = {'configured_msl_m': float(setting[1]),
                                         'home_before_takeoff_msl_m': self.site_home_altitude()}
        for stream, rate in (('VFR_HUD', '5'), ('LOCAL_POSITION_NED', '10'), ('GLOBAL_POSITION_INT', '5')):
            self.cli('mavlink', 'stream', '-u', '18570', '-s', stream, '-r', rate)
        takeoff = self.event('fixture_takeoff')
        self.cli('commander', 'takeoff')

        def airborne():
            sample = self.flight_sample(takeoff)
            return sample if sample and self.air_mode(HOLD, takeoff) and 35 <= sample['altitude']['relative_m'] <= 75 else None

        self.report['native_airborne_hold'] = self.wait(48, airborne, 'runway takeoff did not reach bounded native Hold')
        self.settle_stable_home(takeoff)
        self.report['simulator_site']['home_after_settle_msl_m'] = self.site_home_altitude()
        deploy = self.poke('DEPLOY', 'true')

        def prepared():
            helm = self.latest('mail', 'MAVMOOS_HELM_STATUS', age=.5)
            if (helm and helm['source'] == 'iMAVLink' and fields(helm).get('acquire_preparation') == 'READY' and
                    self.flight_sample() and self.runtime(permission='false', nav_tx_accepted=0,
                                                          px4_hold_tx_accepted=0, px4_native_hold_ready=1)):
                return self.stock_inputs(deploy)
            return None

        self.report['stock_inputs'] = self.wait(8, prepared, 'fixed-wing stock helm never showed valid zero-output readiness')
        self.report['first_acquisition'] = self.acquire_and_travel(0, 0)
        self.report['normal_handoff'] = self.normal_handoff()
        need(self.stock_helm_process(self.out / 'targ_px4.moos') == self.helm_record, 'stock helm identity changed')
        paused_at = self.event('stock_helm_paused', pid=self.helm_record['pid'])
        os.kill(self.helm_record['pid'], signal.SIGSTOP)
        self.paused = True
        revoked = self.wait(3, lambda: self.runtime(paused_at, permission='false'),
                            'stock helm silence did not retire fixed-wing permission')
        counts = fields(revoked)
        nav, holds = int(counts['nav_tx_accepted']), int(counts['px4_hold_tx_accepted'])
        self.report.update(retired_nav_count=nav, retired_hold_count=holds, retired_at=revoked['mono'])
        fallback = self.wait(7, lambda: self.air_mode(HOLD, paused_at),
                             'fixed-wing stream loss did not produce independently observed native Hold')
        self.report['independent_loss_hold'] = fallback
        self.report['native_hold_flight'] = self.wait(
            5, lambda: self.flight_sample(fallback['mono']) if time.monotonic() >= fallback['mono'] + 2 else None,
            'native Hold after stream loss did not retain bounded forward flight')
        self.resume_helm()
        resumed = self.event('stock_helm_resumed')
        self.wait(5, lambda: self.stock_inputs(resumed), 'stock helm did not resume publishing')
        self.wait(5, lambda: time.monotonic() >= resumed + 2 and self.air_mode(HOLD, resumed) and
                  self.runtime(resumed, permission='false', nav_tx_accepted=nav, px4_hold_tx_accepted=holds),
                  'new helm output automatically resumed without reacquisition')
        post_revoke = [fields(row) for row in self.rows if row.get('key') == 'MAVMOOS_RUNTIME_STATUS' and
                       row['mono'] >= revoked['mono']]
        need(all(int(row['nav_tx_accepted']) == nav and int(row['px4_hold_tx_accepted']) == holds
                 for row in post_revoke), 'new setpoint handoff occurred after retirement')
        self.reacquire_after_native_hold(paused_at, nav, holds)

        land = self.event('fixture_land')
        self.cli('commander', 'land')

        def in_land_mode():
            row = self.latest('wire', 'HEARTBEAT', land)
            if row and row['mav_type'] == self.mav_type and row['autopilot'] == 12 and row['armed'] and row['mode'] == LAND:
                return row
            return None

        self.report['independent_land_mode'] = self.wait(
            8, in_land_mode, 'native fixed-wing Land was not independently observed')
        # The pinned SIH airplane descends in a circle after native Land.
        # CPU-limited lockstep can leave it still descending at 80 wall seconds.
        self.report['landed_disarmed'] = self.wait(
            125, lambda: (self.latest('wire', 'EXTENDED_SYS_STATE', land) or {}).get('landed_state') == 1 and
            (self.latest('wire', 'HEARTBEAT', land) or {}).get('armed') is False,
            'owned SIH airplane did not land and disarm')


    def release_after_reacquisition(self):
        active = self.runtime(permission='true', fc_mode=OFFBOARD, px4_offboard_confirmed=1)
        need(active and self.air_mode(OFFBOARD), 'native takeover must start with an active fixed-wing grant')
        taken_at = self.event('fixture_native_hold_takeover')
        self.cli('commander', 'mode', 'auto:loiter')

        def takeover_observed():
            row = self.runtime(taken_at, permission='false', fc_mode=HOLD, reason='external_mode_change',
                               px4_native_hold_ready=1)
            mode = self.current_mode(HOLD, HOLD, taken_at)
            heartbeat = self.air_mode(HOLD, taken_at)
            helm = self.latest('mail', 'MAVMOOS_HELM_STATUS', taken_at, age=.5)
            if (row and mode and heartbeat and helm and helm['source'] == 'iMAVLink' and
                    fields(helm).get('state') == 'LATCHED'):
                return {'runtime': row, 'current_mode': mode, 'heartbeat': heartbeat, 'helm': helm}
            return None

        taken = self.wait(7, takeover_observed, 'native mode takeover did not independently revoke the active grant')
        counts = fields(taken['runtime'])
        nav, holds = int(counts['nav_tx_accepted']), int(counts['px4_hold_tx_accepted'])

        def fenced():
            row = self.runtime(taken['runtime']['mono'], permission='false', fc_mode=HOLD,
                               nav_tx_accepted=nav, px4_hold_tx_accepted=holds, px4_native_hold_ready=1)
            if row and time.monotonic() >= taken['runtime']['mono'] + 2 and self.stock_inputs(taken_at):
                return row
            return None

        observed = self.wait(5, fenced, 'post-takeover stock inputs did not leave authority latched')
        sequence = [fields(row) for row in self.rows if row.get('key') == 'MAVMOOS_RUNTIME_STATUS' and
                    taken['runtime']['mono'] <= row['mono'] <= observed['mono']]
        need(sequence and all(int(row['nav_tx_accepted']) == nav and int(row['px4_hold_tx_accepted']) == holds
                              for row in sequence), 'takeover left outgoing setpoints')
        self.report['native_takeover'] = {'before': active, 'requested_at': taken_at, 'observed': taken,
                                          'fence': observed}
        entry = self.enter_offboard(holds)

        def new_motion():
            row = self.runtime(entry['acquire_at'], permission='true', fc_mode=OFFBOARD,
                               px4_offboard_confirmed=1, reason='px4_offboard_motion')
            sample = self.flight_sample(entry['independent_offboard']['mono'])
            if row and sample and int(fields(row)['nav_tx_accepted']) > nav:
                return {'runtime': row, 'flight': sample}
            return None

        self.report['third_acquisition'] = {
            **entry, 'first_motion': self.wait(4, new_motion, 'fresh acquire after takeover did not admit motion'),
        }
        super().release_after_reacquisition()


def run_candidate(vehicle, revision, output, build_manifest):
    pin = json.loads(STABLE_PIN.read_text())
    need(pin['revision'] == STABLE_PX4_REVISION, 'stable candidate source revision differs from the upstream tag')
    workflow = FreshStableWorkflow
    if vehicle == 'fixed-wing':
        pin['files'].update(AIRFRAME_PINS)
        workflow = FixedWingWorkflow
    args = argparse.Namespace(
        output=output, source_revision=revision, db_port=15300, px4_root=Path('/opt/px4'),
        moos_root=Path('/work/moos-ivp'), behavior_dir=Path('/work/altitude/build/lib'),
        build=Path('/candidate'), probe=None, nsplug=None, stable_pin=pin, build_manifest=build_manifest,
    )
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(SystemExit(143)))
    return main(args, workflow)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-revision', required=True)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--build-manifest', required=True, type=Path)
    parser.add_argument('--vehicle', choices=('fixed-wing', 'quadrotor'), default='fixed-wing')
    options = parser.parse_args()
    if not re.fullmatch(r'[0-9a-f]{40}', options.source_revision):
        parser.error('exact source revision required')
    raise SystemExit(run_candidate(options.vehicle, options.source_revision, options.output.resolve(),
                                   options.build_manifest.resolve()))
