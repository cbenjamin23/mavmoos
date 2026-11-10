#!/usr/bin/env python3
"""Observe Rover mission effects using separate MOOS and read-only native links."""
import argparse
import errno
import fcntl
import json
import math
from pathlib import Path
import random
import signal
import socket
import subprocess
import sys
import tempfile
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from owned_processes import owned_records, stop_process_tree
from rover.run_rover import Pilot, port, sha, sockets
from rover_expanded.run_native import (
    ARDUPILOT, DEFAULTS, HOME, MOOS, PROFILES, build_source, endpoint_cleanup,
    env_for, evidence_files, exact_peer, generate, git, launcher_macros, native_replay_config, require,
    start, tracked_pin, wait_endpoints,
)
from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink2
from confirmation_loss import FAULTS, Route, audit

ROOT = Path(__file__).resolve().parents[2]
NATIVE = 'MAVMOOS_COMMAND'
MISSION = 'MAVMOOS_MISSION_CONTROL'
RECOVERY = 'MAVMOOS_AUTO_RECOVERY_REQUEST'
HELM = 'MAVMOOS_NAV_CONTROL'
TERMINAL_FAILURES = {'REJECTED', 'FAILED', 'TIMED_OUT', 'LOST', 'CANCELLED'}


def fields(value):
    return dict(row.split('=', 1) for row in value.split(',') if '=' in row)


def distance(a, b):
    return 6371000 * math.hypot(math.radians(b[0] - a[0]),
                                math.radians(b[1] - a[1]) * math.cos(math.radians(a[0])))


def position(row):
    return row['lat'] * 1e-7, row['lon'] * 1e-7


class MissionPilot(Pilot):
    def requested_mode(self):
        if (self.control / 'auto-reentry.request').is_file():
            return 1800, 'pilot AUTO reentry'
        return super().requested_mode()


class Bench:
    def __init__(self, out, profile, native, pantler, probe, monitor, pilot, result, *, expected_type=None):
        self.out, self.profile = out, profile
        self.native, self.pantler, self.probe = native, pantler, probe
        self.monitor, self.pilot, self.result = monitor, pilot, result
        self.expected_type = PROFILES[profile][1] if expected_type is None else expected_type
        self.decoder = mavlink2.MAVLink(None)
        self.decoder.robust_parsing = True
        self.endpoint = None
        self.wire, self.mail = [], []
        self.latest = {}
        self.receipts, self.helm_outputs = {}, {}
        self.readout = (out / 'moos.jsonl').open('r')
        self.raw = (out / 'fc.mavlink').open('wb')
        self.trace = (out / 'fc.jsonl').open('w')
        self.stop_mode = 5 if profile == 'boat' else 4
        self.events = result['phases'] = {}

    def close(self):
        for handle in (self.readout, self.raw, self.trace):
            handle.close()

    def pump(self):
        require(all(p.poll() is None for p in (self.native, self.pantler, self.probe)),
                'owned native, stock community or MOOS probe exited')
        if self.pilot:
            self.pilot.update()
        for _ in range(128):
            try:
                packet, address = self.monitor.recvfrom(65535)
            except BlockingIOError:
                break
            if self.endpoint is None:
                inventory = sockets(self.native)
                require(address[0] == '127.0.0.1' and
                        exact_peer(inventory, address[1], self.monitor.getsockname()[1]),
                        'independent route is not owned by the selected simulator')
                self.endpoint = address
                self.result['ports']['monitor_peer'] = address[1]
                self.result['independent_socket_proof'] = inventory
            require(address == self.endpoint, 'unexpected independent sender')
            self.raw.write(packet)
            for message in self.decoder.parse_buffer(packet) or []:
                if message.get_srcSystem() != 1 or message.get_srcComponent() != 1:
                    continue
                kind = message.get_type()
                if kind not in {'HEARTBEAT', 'GLOBAL_POSITION_INT', 'MISSION_CURRENT',
                                'MISSION_ITEM_REACHED', 'HOME_POSITION', 'STATUSTEXT', 'EXTENDED_SYS_STATE'}:
                    continue
                if kind == 'HEARTBEAT':
                    require(message.type == self.expected_type and message.autopilot == 3,
                            'independent heartbeat identity changed')
                row = {'at': time.monotonic(), **message.to_dict()}
                self.wire.append(row)
                self.trace.write(json.dumps(row) + '\n')
        while True:
            offset = self.readout.tell()
            line = self.readout.readline()
            if not line.endswith('\n'):
                self.readout.seek(offset)
                break
            row = json.loads(line)
            self.mail.append(row)
            if row['kind'] == 'mail':
                self.latest[row['key']] = row
                if row['key'].endswith('_RESULT') and not row['value'].startswith('{'):
                    value = fields(row['value'])
                    self.receipts.setdefault(value.get('id'), []).append(value)
            elif row['kind'] == 'helm':
                self.helm_outputs[row['key']] = row

    def wait(self, predicate, label, timeout=12):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.pump()
            result = predicate()
            if result:
                return result
            time.sleep(.025)
        raise RuntimeError(label)

    def state(self, key):
        row = self.latest.get(key)
        return fields(row['value']) if row else {}

    def last(self, kind, after=0):
        for row in reversed(self.wire):
            if row['at'] <= after:
                break
            if row['mavpackettype'] == kind and time.monotonic() - row['at'] < 2.5:
                return row
        return None

    def mode(self, value, after=0, armed=True):
        row = self.last('HEARTBEAT', after)
        return row if (row and row['custom_mode'] == value and bool(row['base_mode'] & 128) == armed) else None

    def cursor(self, after, seq, state, mode):
        row = self.last('MISSION_CURRENT', after)
        return row if (row and row['seq'] == seq and row['total'] == 2 and
                       row['mission_state'] == state and row['mission_mode'] == mode) else None

    def settled(self, after, radius=None):
        points = [row for row in self.wire if row['mavpackettype'] == 'GLOBAL_POSITION_INT' and row['at'] > after]
        if len(points) < 5 or time.monotonic() - points[-1]['at'] > .4:
            return None
        start = next((row for row in reversed(points[:-1]) if points[-1]['at'] - row['at'] >= 1), None)
        if start is None:
            return None
        suffix = [row for row in points if row['at'] >= start['at']]
        radius = radius or (1.5 if self.profile == 'boat' else .6)
        ceiling = 45 if self.profile == 'boat' else 25
        excursion = max(distance(position(start), position(row)) for row in suffix)
        speed = max(math.hypot(row['vx'], row['vy']) for row in suffix)
        if excursion > radius or speed > ceiling or suffix[-1]['time_boot_ms'] - start['time_boot_ms'] < 1000:
            return None
        return {'samples': len(suffix), 'seconds': suffix[-1]['at'] - start['at'],
                'excursion_m': excursion, 'maximum_speed_m_s': speed / 100}

    def post(self, key, value):
        at = time.monotonic()
        self.probe.stdin.write(f'{key}\t{value}\n')
        self.probe.stdin.flush()
        return at

    def receipt(self, request, expected='COMPLETED', timeout=14):
        def observed():
            for item in self.receipts.get(request, []):
                if item['status'] == expected:
                    return item
                if item['status'] in TERMINAL_FAILURES:
                    raise RuntimeError(f'{request} {item["status"]} {item.get("reason")}')
            return None
        return self.wait(observed, f'missing {request} {expected}', timeout)

    def native_command(self, request, action, timeout=14):
        self.post(NATIVE, f'id={request},action={action}')
        return self.receipt(request, timeout=timeout)

    def upload(self, distances=(12, 32)):
        self.wait(lambda: self.mode(4, armed=False) and
                  self.state('MAVMOOS_RUNTIME_STATUS').get('position_valid') == '1' and
                  self.state('MAVMOOS_HOME_POSITION').get('encoding_valid') == 'true',
                  'disarmed exact Hold, position and home required', 50)
        began = time.monotonic()
        self.wait(lambda: self.settled(began) and
                  self.state('MAVMOOS_MISSION_EXECUTION_STATUS').get('upload_ready') == 'true',
                  'both bridge and independent link need a settled, estimator-ready Hold', 60)
        home = self.state('MAVMOOS_HOME_POSITION')
        latitude, longitude, msl = (float(home[key]) for key in ('lat_deg', 'lon_deg', 'alt_msl_m'))
        points = [(latitude, longitude), *((latitude + offset / 111195., longitude) for offset in distances)]
        rows = [f'{seq}:16:{0 if seq == 0 else 3}:{1 if seq == 1 else 0}:1:0:0:0:0:'
                f'{round(lat * 1e7)}:{round(lon * 1e7)}:{msl if seq == 0 else 0}'
                for seq, (lat, lon) in enumerate(points)]
        self.post('MAVMOOS_MISSION_REQUEST', 'id=upload,action=upload,timeout=30,items=' + '|'.join(rows))
        receipt = self.receipt('upload', timeout=30)
        proof = self.wait(lambda: (value if (value := self.state('MAVMOOS_STORED_MISSION_PROOF')).get('valid') == 'true' else None),
                          'missing verified stored mission object')
        require(proof['verification_id'] == 'upload' and proof['count'] == '3', 'wrong verified mission identity')
        self.result['verified_upload'] = {'result': receipt, 'proof': proof, 'points': points}
        return proof, points

    def mission_start(self, request, proof):
        self.wait(lambda: self.state('MAVMOOS_MISSION_EXECUTION_STATUS').get('start_ready') == 'true',
                  f'{request} did not become start-ready', 20)
        at = self.request_mission_start(request, proof)
        self.receipt(request, 'ACTIVE')
        cursor = self.wait(lambda: self.mode(10, at) and self.cursor(at, 1, 3, 1),
                           'independent AUTO and active first cursor required')
        self.events[request] = {'at': at, 'cursor': cursor}

    def request_mission_start(self, request, proof):
        return self.post(MISSION, f'schema=1,id={request},action=start,timeout=100,'
                         f'verification_id={proof["verification_id"]},content_id={proof["content_id"]},'
                         f'proof_revision={proof["revision"]}')

    def transition(self, request, action, target):
        return self.post(MISSION, f'schema=1,id={request},action={action},target={target},timeout=14')

    def movement(self, origin, minimum, timeout=18):
        return self.wait(lambda: (value if (value := self.last('GLOBAL_POSITION_INT')) and
                                  distance(origin, position(value)) >= minimum else None),
                         'independent position did not show requested travel', timeout)

    def handback(self, at, request):
        self.receipt(request)
        evidence = self.wait(lambda: self.mode(self.stop_mode, at) and self.settled(at),
                             'independent settled handback not observed')
        state = self.state('MAVMOOS_RUNTIME_STATUS')
        require(state.get('permission') in ('0', 'false'), 'handback granted helm without an explicit request')
        return evidence

    def pilot_manual(self):
        marker = self.out / 'control/takeover.request'
        at = time.monotonic()
        marker.touch(exist_ok=False)
        self.wait(lambda: self.mode(0, at), 'actual pilot Manual takeover')
        return marker

    def verify_pilot_auto_reentry(self):
        marker = self.out / 'control/auto-reentry.request'
        at = time.monotonic()
        marker.touch(exist_ok=False)
        try:
            entered = self.wait(lambda: self.mode(10, at), 'pilot did not reenter native AUTO', 6)
            request = self.transition('pilot-auto-abort', 'abort', 'takeover-run')
            rejected = self.receipt('pilot-auto-abort', 'REJECTED')
            unchanged = self.wait(lambda: self.mode(10, max(request, entered['at'])),
                                  'retained abort displaced pilot-selected AUTO', 4)
            require(not any(row['at'] >= request and row.get('key') == 'MAVMOOS_TX_RESULT' and
                            fields(row['value']).get('id', '').startswith('armx.') for row in self.mail),
                    'retired execution reached command handoff after pilot AUTO reentry')
            return {'entry': entered, 'rejected_abort': rejected, 'retained_mode': unchanged}
        finally:
            marker.unlink()

    def fresh_helm(self, after):
        return (all(self.helm_outputs.get(key, {}).get('at', 0) > after
                    for key in ('DESIRED_HEADING', 'DESIRED_SPEED')) and
                self.helm_outputs['DESIRED_SPEED']['value'] > 0)

    def observe_paused_fence(self, after, last_request, cursor, recovery_state):
        """Require both independent FC stationarity and no bridge command/navigation handoff."""
        def complete_window():
            dwell = self.settled(after)
            if time.monotonic() - last_request < 2 or not dwell or not self.fresh_helm(last_request):
                return None
            return (dwell, self.mode(self.stop_mode, after), self.cursor(after, cursor, 2, 0))

        dwell, mode, cursor_report = self.wait(complete_window, 'paused restart did not physically settle')
        require(mode and cursor_report, 'pause did not retain the native mode and stopped cursor')
        heartbeats = [row for row in self.wire if row['at'] > after and row['mavpackettype'] == 'HEARTBEAT']
        cursors = [row for row in self.wire if row['at'] > after and row['mavpackettype'] == 'MISSION_CURRENT']
        positions = [row for row in self.wire if row['at'] > after and row['mavpackettype'] == 'GLOBAL_POSITION_INT']
        boot_times = [row['time_boot_ms'] for row in positions]
        require(len(positions) >= 10 and positions[0]['at'] - after <= .4 and boot_times[-1] - boot_times[0] >= 1500
                and all(0 <= b - a <= 400 for a, b in zip(boot_times, boot_times[1:])),
                'paused restart lost independent position coverage')
        speed = max(math.hypot(row['vx'], row['vy']) / 100 for row in positions)
        excursion = max(distance(position(positions[0]), position(row)) for row in positions)
        max_speed, max_excursion = (.45, 1.5) if self.profile == 'boat' else (.25, .6)
        require(speed <= max_speed and excursion <= max_excursion, 'fenced vehicle moved beyond its pause envelope')
        require(len(heartbeats) >= 2 and all(row['custom_mode'] == self.stop_mode and row['base_mode'] & 128
                                             for row in heartbeats), 'a fenced request changed the native mode')
        require(len(cursors) >= 2 and all(row['seq'] == cursor and row['mission_state'] == 2 and row['mission_mode'] == 0
                                         for row in cursors), 'a fenced request moved the paused cursor')
        rows = [row for row in self.mail if row['at'] > after and row['kind'] == 'mail']
        status = [fields(row['value']) for row in rows if row['key'] == 'MAVMOOS_RUNTIME_STATUS']
        require(len(status) >= 2 and all(row.get('permission') == 'false' and
                                         row.get('nav_tx_accepted') == '0' and
                                         row.get('auto_recovery_state') == recovery_state for row in status),
                'recovery or stock output unexpectedly granted navigation')
        require(not any(row['key'] == 'MAVMOOS_TX_RESULT' and fields(row['value']).get('status') == 'LOCAL_ACCEPTED'
                        for row in rows), 'fenced command reached local MAVLink handoff')
        return {'from': after, 'to': time.monotonic(), 'settled': dwell, 'mode': mode,
                'cursor': cursor_report, 'heartbeats': len(heartbeats), 'cursor_samples': len(cursors),
                'status_samples': len(status), 'stock_outputs': self.helm_outputs.copy(),
                'position_samples': len(positions), 'maximum_speed_m_s': speed, 'excursion_m': excursion}

    def run_paused_restart(self, restart):
        proof, points = self.upload()
        self.native_command('arm', 'arm')
        run = 'paused-restart-run'
        self.mission_start(run, proof)
        self.movement(points[0], 3)
        pause_at = self.transition('pause-before-restart', 'pause', run)
        paused = self.handback(pause_at, 'pause-before-restart')
        cursor = self.wait(lambda: self.cursor(pause_at, 1, 2, 0), 'missing stopped cursor before restart')
        self.events['pause_before_restart'] = {'dwell': paused, 'cursor': cursor}
        self.post('MOOS_MANUAL_OVERRIDE', 'false')
        deployed = self.post('DEPLOY', 'true')
        self.wait(lambda: self.fresh_helm(deployed), 'stock helm did not offer fresh guidance in pause')
        previous = self.state('MAVMOOS_AUTO_RECOVERY_STATUS')['session']
        restart_at = time.monotonic()
        restart()

        def restarted():
            status = self.state('MAVMOOS_AUTO_RECOVERY_STATUS')
            if status.get('session') != previous and status.get('state') == 'UNRESOLVED':
                return status
            return None

        recovery = self.wait(restarted, 'new bridge did not retain the paused mission uncertainty marker')
        require(recovery['effects_blocked'] == 'true', 'fresh paused evidence silently cleared recovery')
        require(recovery['ready_for_handback'] == 'false', 'the previous bridge stop evidence survived restart')
        self.post(RECOVERY, f'schema=1,id=early-handback,action=handback,session={recovery["session"]}')
        early = self.receipt('early-handback', 'REJECTED')
        require(early.get('reason') == 'fresh_prior_non_auto_observation_required',
                'current-session handback failed the wrong check before fresh settlement')
        self.wait(lambda: self.state('MAVMOOS_AUTO_RECOVERY_STATUS').get('ready_for_handback') == 'true' and
                  self.mode(self.stop_mode, restart_at) and self.fresh_helm(restart_at),
                  'restarted bridge did not observe fresh native pause evidence')
        fenced_at = self.post(RECOVERY, f'schema=1,id=old-session,action=handback,session={previous}')
        old_session = self.receipt('old-session', 'REJECTED')
        require(old_session.get('reason') == 'wrong_process_session', 'old handback failed the wrong check')
        self.transition('stale-resume-before-handback', 'resume', run)
        stale = self.receipt('stale-resume-before-handback', 'REJECTED')
        require(stale.get('reason') == 'target_not_retained', 'restart unexpectedly retained a resumable mission')
        self.transition('stale-abort-before-handback', 'abort', run)
        abort = self.receipt('stale-abort-before-handback', 'REJECTED')
        require(abort.get('reason') == 'target_not_retained', 'an old mission still owned stop authority')
        self.post(NATIVE, 'id=fenced-guided,action=guided')
        guided = self.receipt('fenced-guided', 'REJECTED')
        require(guided.get('reason') == 'busy', 'competing Guided command failed the wrong check')
        helm_request = self.post(HELM, 'acquire')
        self.wait(lambda: any(row['kind'] == 'request' and row['at'] > helm_request and
                              row.get('key') == HELM and row.get('value') == 'acquire' for row in self.mail),
                  'MOOS operator did not submit the fenced helm request')
        self.events['paused_restart_fenced'] = {
            'old_session': previous, 'new_session': recovery['session'],
            'rejected_handback': old_session, 'rejected_resume': stale, 'rejected_guided': guided,
            'rejected_abort': abort, 'rejected_early_handback': early,
            **self.observe_paused_fence(fenced_at, helm_request, cursor['seq'], 'UNRESOLVED')}

        self.post(RECOVERY, f'schema=1,id=explicit-handback,action=handback,session={recovery["session"]}')
        handback = self.receipt('explicit-handback', 'RELEASED')
        self.wait(lambda: self.state('MAVMOOS_AUTO_RECOVERY_STATUS').get('state') == 'READY' and
                  self.state('MAVMOOS_HELM_STATUS').get('next_action') == 'RESYNCHRONIZE_HELM',
                  'handback did not retire the old stock helm state')
        unfenced_at = self.transition('stale-resume-after-handback', 'resume', run)
        stale = self.receipt('stale-resume-after-handback', 'REJECTED')
        require(stale.get('reason') == 'target_not_retained', 'old mission became resumable after handback')
        proof_request = self.request_mission_start('old-proof-start', proof)
        stale_proof = self.receipt('old-proof-start', 'REJECTED')
        require(stale_proof.get('reason') == 'idle_armed_stopped_verified_plan_required' and
                self.state('MAVMOOS_STORED_MISSION_PROOF').get('valid') == 'false',
                'old verified plan was available for a new start')
        self.events['handback_without_helm_grant'] = {
            'handback': handback, 'rejected_resume': stale, 'rejected_old_proof': stale_proof,
            **self.observe_paused_fence(unfenced_at, proof_request, cursor['seq'], 'READY')}

        # An ordinary mission transition makes the unchanged helm publish a new all-stop state.
        self.post('DEPLOY', 'false')
        self.wait(lambda: self.state('MAVMOOS_HELM_STATUS').get('allstop') in ('NothingToDo', 'NoGoalBehavior'),
                  'stock helm did not publish a fresh inactive all-stop state')
        stopped = self.state('MAVMOOS_HELM_STATUS')
        redeployed = self.post('DEPLOY', 'true')
        self.wait(lambda: self.state('MAVMOOS_HELM_STATUS').get('acquire_preparation') == 'READY' and
                  self.fresh_helm(redeployed), 'stock helm did not produce a fresh clear state')
        require(self.state('MAVMOOS_RUNTIME_STATUS').get('permission') == 'false',
                'producer resynchronization granted helm without acquire')
        self.events['stock_helm_resynchronized'] = {'stopped': stopped, 'ready': self.state('MAVMOOS_HELM_STATUS')}
        self.finish_with_helm()

    def run(self, restart):
        proof, points = self.upload()
        self.native_command('arm', 'arm')
        self.mission_start('run', proof)
        self.movement(points[0], 3)
        pause_at = self.transition('pause', 'pause', 'run')
        paused = self.handback(pause_at, 'pause')
        cursor = self.wait(lambda: self.cursor(pause_at, 1, 2, 0), 'missing independent stopped first cursor')
        self.post(NATIVE, 'id=competing-disarm,action=disarm')
        self.receipt('competing-disarm', 'REJECTED')
        nav_before = self.state('MAVMOOS_RUNTIME_STATUS').get('nav_tx_accepted')
        self.post(HELM, 'acquire')
        until = time.monotonic() + 1.2
        while time.monotonic() < until:
            self.pump()
            state = self.state('MAVMOOS_RUNTIME_STATUS')
            require(state.get('permission') == 'false' and state.get('nav_tx_accepted') == nav_before,
                    'paused AUTO allowed competing helm')
            time.sleep(.025)
        resume_at = self.transition('resume', 'resume', 'run')
        self.receipt('resume')
        resumed = self.wait(lambda: self.mode(10, resume_at) and self.cursor(resume_at, cursor['seq'], 3, 1),
                            'no independent same-cursor resume')
        self.receipt('run', 'COMPLETED', timeout=40)
        def arrival():
            reached = next((row for row in self.wire if row['at'] > resume_at and
                            row['mavpackettype'] == 'MISSION_ITEM_REACHED' and row['seq'] == 2), None)
            complete = self.cursor(resume_at, 2, 5, 1)
            end = self.last('GLOBAL_POSITION_INT', resume_at)
            return (reached, complete, end) if reached and complete and end and distance(position(end), points[-1]) <= 3 else None

        reached, complete, end = self.wait(arrival, 'independent final arrival or completed cursor missing')
        self.events['pause_resume_completion'] = {'pause': paused, 'paused_cursor': cursor, 'resumed_cursor': resumed,
                                                  'completed_cursor': complete, 'final_position': end, 'reached': reached}
        release_at = self.transition('release', 'release', 'run')
        self.events['release'] = self.handback(release_at, 'release')
        self.wait(lambda: self.state('MAVMOOS_MISSION_EXECUTION_LEASE').get('held') == 'false',
                  'lease survived explicit release')
        self.mission_start('abort-run', proof)
        self.movement(position(end), 2)
        at = self.transition('abort', 'abort', 'abort-run')
        self.events['abort'] = self.handback(at, 'abort')
        self.transition('retired-resume', 'resume', 'abort-run')
        self.receipt('retired-resume', 'REJECTED')
        self.mission_start('takeover-run', proof)
        marker = self.pilot_manual()
        self.receipt('takeover-run', 'LOST')
        self.transition('manual-abort', 'abort', 'takeover-run')
        self.receipt('manual-abort', 'REJECTED')
        marker.unlink()
        self.events['pilot_auto_reentry'] = self.verify_pilot_auto_reentry()
        at = time.monotonic()
        self.wait(lambda: self.mode(4, at) and self.settled(at), 'pilot-selected Hold did not settle')
        at = self.transition('takeover-release', 'release', 'takeover-run')
        self.events['takeover_handback'] = self.handback(at, 'takeover-release')
        self.mission_start('restart-run', proof)
        previous = self.state('MAVMOOS_AUTO_RECOVERY_STATUS')['session']
        at = time.monotonic()
        restart()
        recovery = self.wait(lambda: (value if (value := self.state('MAVMOOS_AUTO_RECOVERY_STATUS')).get('state') == 'UNRESOLVED' and
                                      value.get('session') != previous else None), 'fresh process did not retain the marker')
        require(recovery['effects_blocked'] == 'true', 'restart did not block competing effects')
        self.wait(lambda: self.mode(10, at), 'restart did not observe native AUTO')
        self.post(RECOVERY, f'schema=1,id=old-session,action=handback,session={previous}')
        self.receipt('old-session', 'REJECTED')
        marker = self.pilot_manual()
        self.post(RECOVERY, f'schema=1,id=manual-recovery,action=handback,session={recovery["session"]}')
        self.receipt('manual-recovery', 'REJECTED')
        marker.unlink()
        at = time.monotonic()
        dwell = self.wait(lambda: self.mode(4, at) and self.settled(at), 'restart Hold did not settle')
        self.wait(lambda: self.state('MAVMOOS_AUTO_RECOVERY_STATUS').get('ready_for_handback') == 'true',
                  'restart fence did not observe a settled Hold')
        self.post(RECOVERY, f'schema=1,id=restart-handback,action=handback,session={recovery["session"]}')
        self.receipt('restart-handback', 'RELEASED')
        self.events['restart_handback'] = {'old_session': previous, 'new_session': recovery['session'], 'dwell': dwell}
        self.wait(lambda: self.mode(4) and self.state('MAVMOOS_AUTO_RECOVERY_STATUS').get('state') == 'READY' and
                  self.state('MAVMOOS_RUNTIME_STATUS').get('heartbeat_valid') == '1',
                  'bridge failed to re-establish fresh recovery state')
        self.post('MOOS_MANUAL_OVERRIDE', 'false')
        deploy_at = self.post('DEPLOY', 'true')
        self.wait(lambda: self.state('MAVMOOS_HELM_STATUS').get('acquire_preparation') == 'READY' and
                  all(self.helm_outputs.get(key, {}).get('at', 0) > deploy_at
                      for key in ('DESIRED_HEADING', 'DESIRED_SPEED')),
                  'fresh stock-helm observations did not become ready after handback')
        self.finish_with_helm()

    def finish_with_helm(self):
        start = self.last('GLOBAL_POSITION_INT')
        require(start is not None, 'missing stock helm recovery origin')
        helm_at = self.post(HELM, 'acquire')
        self.wait(lambda: self.mode(15, helm_at) and self.state('MAVMOOS_HELM_STATUS').get('state') == 'ACTIVE' and
                  all(self.helm_outputs.get(key, {}).get('at', 0) > helm_at
                      for key in ('DESIRED_HEADING', 'DESIRED_SPEED')),
                  'stock helm did not explicitly reacquire after restart handback', 15)
        end = self.movement(position(start), 2)
        self.events['stock_helm_after_handback'] = {'mode': self.mode(15), 'origin': start, 'end': end,
                                                     'helm_outputs': self.helm_outputs.copy()}
        self.native_command('final-hold', 'hold', timeout=27 if self.profile == 'boat' else 14)
        self.post('DEPLOY', 'false')
        self.native_command('final-disarm', 'disarm')
        self.wait(lambda: self.mode(4, armed=False), 'independent final disarm missing')


def dataflash_proof(file, stop_mode, scenario):
    records = {'MODE': [], 'ARM': []}
    reader = mavutil.mavlink_connection(str(file))
    while message := reader.recv_match(type=list(records)):
        records[message.get_type()].append(message.to_dict())
    modes = [row['ModeNum'] for row in records['MODE']]
    if scenario == 'native-confirmation-loss':
        require(any(row['ModeNum'] == 0 and row['Rsn'] == 1 for row in records['MODE']) and
                modes[-1] == 4, 'onboard pilot Manual and final Hold evidence missing')
    elif scenario == 'confirmation-loss':
        require(10 in modes and stop_mode in modes[modes.index(10) + 1:] and modes[-1] == 4,
                'onboard modes do not corroborate AUTO, mission stop and final Hold')
    elif scenario == 'paused-restart':
        require(modes.count(10) == 1 and 15 in modes and modes[-1] == 4 and
                modes[modes.index(10) + 1:modes.index(15)] == [stop_mode],
                'onboard modes do not corroborate uninterrupted pause followed by fresh Guided')
    else:
        require(modes.count(10) >= 5 and stop_mode in modes and {0, 15}.issubset(modes) and modes[-1] == 4,
                'onboard modes do not corroborate AUTO, pause, abort, Manual and recovery')
        manual = next((row for row in records['MODE'] if row['ModeNum'] == 0 and row['Rsn'] == 1), None)
        require(manual and any(row['ModeNum'] == 10 and row['Rsn'] == 1 and row['TimeUS'] > manual['TimeUS']
                               for row in records['MODE']), 'onboard log did not corroborate pilot AUTO reentry')
    require(any(row['ArmState'] == 1 and not row.get('Forced', 0) for row in records['ARM']) and
            records['ARM'][-1]['ArmState'] == 0, 'onboard normal arming/final disarm missing')
    return {'file': str(file), 'sha256': sha(file), **records}


def bind_restart_route(endpoint):
    # Preserve a checked free loopback port across process replacement without
    # exposing it to the kernel's automatic ephemeral port allocation.
    for candidate in random.sample(range(20000, 30000), 32):
        try:
            endpoint.bind(('127.0.0.1', candidate))
        except OSError as error:
            if error.errno != errno.EADDRINUSE:
                raise
        else:
            return
    raise RuntimeError('no free non-ephemeral loopback port for the owned restart')


def main(args):
    ap, moos, build = (Path(getattr(args, key)).resolve(strict=True)
                       for key in ('ardupilot_root', 'moos_root', 'build'))
    tracked_pin(ap, ARDUPILOT)
    tracked_pin(moos, MOOS)
    source = build_source(build)
    require(not source['tracked_changes'], 'qualification requires a clean build source')
    profile, expected_type, model, defaults_names = PROFILES[args.profile]
    defaults = [ap / 'Tools/autotest/default_params' / name for name in defaults_names]
    require(all(sha(path) == DEFAULTS[path.name] for path in defaults), 'default model differs from pin')
    binary, app, helm = ap / 'build/sitl/bin/ardurover', build / 'bin/iMAVLink', moos / 'bin/pHelmIvP'
    probe_file = build / 'bin/rover_mission_probe'
    native_loss = args.scenario == 'native-confirmation-loss'
    modules = ('telemetry', 'ardurover_control', 'parameters' if native_loss else 'missions')
    artifacts = [binary, app, helm, probe_file, *(build / 'plugins' / f'mavmoos_{name}.so'
                 for name in modules), *defaults,
                 Path(__file__), Path(__file__).with_name('RoverMissionProbe.cpp'),
                 ROOT / 'tests/rover/run_rover.py', ROOT / 'tests/rover_expanded/run_native.py',
                 ROOT / 'tests/owned_processes.py', ROOT / 'missions/rover_expanded/meta_vehicle.moos',
                 ROOT / 'missions/rover_sitl/meta_vehicle.bhv']
    loss = args.scenario == 'confirmation-loss' or native_loss
    boat_rtl = native_loss and args.profile == 'boat' and args.transition == 'rtl'
    if loss:
        tap = Path(args.tap).resolve(strict=True)
        artifacts.extend((tap, Path(__file__).with_name('confirmation.py'), ROOT / 'tests/confirmation_loss.py',
                          ROOT / 'tests/plane_faults/fault_tap.py', ROOT / 'tests/plane_faults/OwnedPlaneFaultTap.cpp'))
    if native_loss:
        require(Path(source['path']) == ROOT and git(ROOT, 'rev-parse', 'HEAD') == source['revision'],
                'native confirmation loss requires matching runner and build source')
        artifacts.append(Path(__file__).with_name('native_confirmation.py'))
    digests = {str(path): sha(path) for path in artifacts}
    out = Path(args.output).resolve()
    out.mkdir(parents=True, exist_ok=False)
    (out / 'logs').mkdir()
    control = out / 'control'
    control.mkdir()
    recovery = out / 'recovery'
    recovery.mkdir(mode=0o700)
    bridge, monitor = (socket.socket(socket.AF_INET, socket.SOCK_DGRAM) for _ in range(2))
    if native_loss and args.fault == 'restart':
        bind_restart_route(bridge)
    else:
        bridge.bind(('127.0.0.1', 0))
    bridge.settimeout(.2)
    monitor.bind(('127.0.0.1', 0))
    monitor.setblocking(False)
    bridge_port, monitor_port = bridge.getsockname()[1], monitor.getsockname()[1]
    db, base, rc = port(socket.SOCK_STREAM), port(socket.SOCK_STREAM), port(socket.SOCK_DGRAM)
    fixture = out / 'fixture.parm'
    fixture.write_text('SERIAL1_PROTOCOL 2\nMAV2_POSITION 10\nMAV2_EXT_STAT 2\nMAV2_EXTRA3 2\n'
                       'LOG_DISARMED 1\nWP_RADIUS 1\nWP_SPEED 1.6\nMODE_CH 8\nMODE1 0\nMODE4 4\nMODE6 10\n'
                       'MIS_RESTART 0\nAUTO_KICKSTART 0\nAUTO_TRIGGER_PIN -1\n' +
                       ('MIS_DONE_BEHAVE 1\nSIM_WIND_SPD 0\nSIM_WAVE_ENABLE 0\nSIM_TIDE_SPEED 0.18\n'
                        'SIM_TIDE_DIR 90\nLOIT_RADIUS 1\n' if args.profile == 'boat' else 'MIS_DONE_BEHAVE 0\n'))
    for channel in (0, 1):
        (out / f'message-intervals-chan{channel}.txt').write_text('33 100\n42 300\n193 500\n242 500\n')
    heading = 90 if boat_rtl else 0
    argv = [str(binary), '--model', model, '--home', f'{HOME[0]},{HOME[1]},10,{heading}', '--speedup', '1',
            '--wipe', '--defaults', ','.join(map(str, defaults + [fixture])),
            '--serial0', f'udpclient:127.0.0.1:{bridge_port}', '--serial1', f'udpclient:127.0.0.1:{monitor_port}',
            '--serial2', 'none', '--serial5', 'none', '--serial6', 'none', '--serial7', 'none', '--serial8', 'none',
            '--base-port', str(base), '--rc-in-port', str(rc)]
    result = {'result': 'FAIL', 'profile': profile, 'model': model, 'scenario': args.scenario,
              'command': argv, 'hardware': False,
              'source': source, 'firmware': ARDUPILOT, 'stock_moos': MOOS, 'digests_before': digests,
              'ports': {'bridge': bridge_port, 'monitor': monitor_port, 'db': db, 'base': base, 'rc': rc},
              'monitor_writes': 0, 'bridge_pids': []}
    native = antler = app_proc = probe = pilot = bench = None
    try:
        env = env_for(moos, build)
        if loss:
            env = {key: value for key, value in env.items() if not key.startswith(('DYLD_', 'LD_PRELOAD', 'MAVMOOS_PLANE_TAP_'))}
        native = start(argv, out, env, 'sitl')
        peer, result['native_socket_proof'] = wait_endpoints(native, bridge, bridge_port, rc, expected_type)
        result['ports']['peer'] = peer[1]
        bridge.close()
        pilot = MissionPilot(native, rc, control, out / 'fixture-pilot.jsonl')
        pilot.update()
        macros = launcher_macros(db, build, moos, bridge_port, peer[1], args.profile)
        macros['SITL_PID'] = native.pid
        macros['NATIVE_REPLAY_CONFIG'] = native_replay_config(out)
        generate(moos, out, macros, env)
        mission = out / 'targ_rover.moos'
        text = mission.read_text()
        text = '\n'.join(row for row in text.splitlines() if '~ iMAVLink' not in row) + '\n'
        if native_loss:
            require('control_native_actions = true' in text and 'control_mission_execution' not in text,
                    'native confirmation loss must run outside managed missions')
        else:
            text = text.replace('plugins = telemetry,parameters,ardurover_control',
                                'plugins = telemetry,ardurover_control,missions')
            text = text.replace('control_native_actions = true', 'control_native_actions = true\n'
                                '  control_mission_execution = true\n'
                                f'  auto_recovery_directory = {recovery}')
            require('control_mission_execution = true' in text and 'ardurover_control,missions' in text,
                    'generated mission expansion mismatch')
        mission.write_text(text)
        antler = start([moos / 'bin/pAntler', mission], out, env, 'pantler')
        app_argv = [app, mission, 'iMAVLink']
        route = Route(out, tap, result['ports'], result) if loss else None
        app_proc = start(app_argv, out, env | route.environment('original') if route else env, 'bridge-1')
        result['bridge_pids'].append(app_proc.pid)
        trace_path = out / 'moos.jsonl'
        trace_path.touch()
        with (out / 'probe.log').open('w') as log:
            probe = subprocess.Popen([probe_file, str(db), trace_path], cwd=out, env=env, stdin=subprocess.PIPE,
                                     stdout=log, stderr=subprocess.STDOUT, text=True, start_new_session=True)
        bench = Bench(out, args.profile, native, antler, probe, monitor, pilot, result)

        def restart():
            nonlocal app_proc
            if route:
                app_proc.kill()
                require(app_proc.wait(timeout=4) == -signal.SIGKILL, 'original bridge did not exit under fault')
            require(not stop_process_tree(app_proc), 'first bridge did not close')
            if route:
                route.mark('stopped')
            restart_argv = app_argv
            if args.relocate_restart:
                regenerated = out / 'regenerated'
                regenerated.mkdir(mode=0o700)
                new_mission = regenerated / mission.name
                original = mission.read_text()
                journal = str((out / 'native-replay').resolve())
                legacy = f'plugin.ardurover_control.native_replay_directory = {journal}'
                require(original.count(legacy) == 1, 'original mission did not select its exact replay journal')
                new_mission.write_text(original.replace(legacy, f'control_replay_directory = {journal}'))
                restart_argv = [app, new_mission, 'iMAVLink']
                result['regenerated_mission'] = {'original': str(mission), 'new': str(new_mission), 'journal': journal,
                                                 'sha256': sha(new_mission)}
            app_proc = start(restart_argv, out, env | route.environment('restarted') if route else env, 'bridge-2')
            result['bridge_pids'].append(app_proc.pid)

        if native_loss:
            from rover_mission.native_confirmation import exercise_native
            exercise_native(bench, route, restart, args.fault, args.transition)
        elif loss:
            from rover_mission.confirmation import exercise
            exercise(bench, route, restart, args.fault, args.transition)
        elif args.scenario == 'paused-restart':
            bench.run_paused_restart(restart)
        else:
            bench.run(restart)
        records = owned_records(antler)
        require(any(str(helm) in str(row) for row in records), 'stock helm was not in the owned community')
        result['owned_community'] = records
        result['result'] = 'PASS'
    except BaseException as error:
        result['error'] = str(error) or type(error).__name__
        if bench:
            result['last_bridge_state'] = {key: bench.state(key) for key in (
                'MAVMOOS_RUNTIME_STATUS', 'MAVMOOS_MISSION_EXECUTION_STATUS', 'MAVMOOS_AUTO_RECOVERY_STATUS')}
            result['last_native_mode'] = bench.last('HEARTBEAT')
            result['last_native_cursor'] = bench.last('MISSION_CURRENT')
    finally:
        if bench:
            bench.close()
        if pilot:
            pilot.close()
        remaining = []
        for process in (probe, app_proc, antler, native):
            try:
                remaining.extend(stop_process_tree(process))
            except Exception as error:
                remaining.append(str(error))
        bridge.close()
        monitor.close()
        result['ports_clear'] = endpoint_cleanup(result['ports'])
        result['owned_processes_remaining'] = remaining
        if remaining or not all(result['ports_clear'].values()):
            result['result'], result['cleanup_error'] = 'FAIL', 'owned endpoints or descendants remain'
        try:
            tracked_pin(ap, ARDUPILOT)
            tracked_pin(moos, MOOS)
            require(build_source(build) == source, 'build source identity changed during native run')
            result['digests_after'] = {str(path): sha(path) for path in artifacts}
            require(result['digests_after'] == digests, 'pinned inputs changed during native run')
            if result['result'] == 'PASS':
                files = list((out / 'logs').glob('*.BIN'))
                require(len(files) == 1, 'native recorder did not produce one owned binary')
                result['independent_dataflash'] = dataflash_proof(files[0], bench.stop_mode, args.scenario)
                if native_loss:
                    from rover_mission.native_confirmation import audit_native, verify_modes
                    verify_modes(result)
                    result['wire_audit'] = audit_native(out, result)
                elif loss:
                    result['wire_audit'] = audit(out, result)
        except Exception as error:
            result['result'], result['verification_error'] = 'FAIL', str(error)
        result['files'] = evidence_files(out)
        (out / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({k: result[k] for k in ('result', 'profile', 'error', 'cleanup_error', 'verification_error')
                      if k in result}, indent=2), flush=True)
    return 0 if result['result'] == 'PASS' else 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for key in ('ardupilot-root', 'moos-root', 'build', 'output'):
        parser.add_argument('--' + key, required=True)
    parser.add_argument('--profile', choices=PROFILES, required=True)
    parser.add_argument('--scenario', choices=('mission', 'paused-restart', 'confirmation-loss', 'native-confirmation-loss'), default='mission')
    parser.add_argument('--relocate-restart', action='store_true',
                        help='regenerate the bridge mission in a new directory with the same replay journal')
    parser.add_argument('--fault', choices=FAULTS)
    parser.add_argument('--transition', choices=('pause', 'resume', 'hold', 'loiter', 'rtl'))
    parser.add_argument('--tap', help='test-only direct-route fault tap library')
    args = parser.parse_args()
    if args.relocate_restart and (args.scenario != 'native-confirmation-loss' or args.fault != 'restart'):
        parser.error('--relocate-restart requires native confirmation loss with a restart')
    transitions = {'confirmation-loss': ('pause', 'resume'),
                   'native-confirmation-loss': ('hold', 'loiter', 'rtl') if args.profile == 'boat' else ('hold', 'rtl')}
    if args.scenario in transitions and not (args.fault and args.tap and args.transition in transitions[args.scenario]):
        parser.error(f'{args.scenario} requires --fault, --tap and --transition from {transitions[args.scenario]}')
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(SystemExit(143)))
    with (Path(tempfile.gettempdir()) / 'mavmoos-native-sitl.lock').open('w') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        sys.exit(main(args))
