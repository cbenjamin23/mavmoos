"""The owned stock-helm workflows. Motion evidence comes from the separate FC channel."""
import json
import math
import os
from pathlib import Path
import re
import shlex
import signal
import socket
import struct
import subprocess
import sys

from flight_fixture import (FBWA, GUIDED, HOME, LOITER, RTL, exact_rc, ground_speed, inventory,
                            log, need, now, start)
from owned_processes import owned_records, snapshot, stop_process_tree

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tests/plane_faults'))
import fault_tap

POINT, SET_MODE = 192, 176
COURSE, HEIGHT = 60, 65
MODULES = ('telemetry', 'air_data', 'arduplane_control')
TARGET_MASK = 0xFDF8


def offset(origin_lat, origin_lon, lat, lon):
    north = math.radians((lat - origin_lat) / 1e7) * 6378137
    east = math.radians((lon - origin_lon) / 1e7) * math.cos(math.radians(origin_lat / 1e7)) * 6378137
    return north, east


def distance(a, b):
    return math.hypot(*offset(a['lat'], a['lon'], b['lat'], b['lon']))


def angle(north, east):
    return math.degrees(math.atan2(east, north)) % 360


def signed_turn(start, end):
    return (end - start + 180) % 360 - 180


def fields(row):
    value = row['value'] if row else None
    return dict(item.split('=', 1) for item in value.split(',') if '=' in item) if isinstance(value, str) else {}


def matches(fields_, **values):
    return all(fields_.get(name) == str(value) for name, value in values.items())


def generate(paths, out, flight, simulation=True, shadow=False, lifecycle=False):
    mission_dir = ROOT / 'missions/plane_sitl'
    settings = dict(MOOS_PORT=flight.ports['db'], BIND_PORT=flight.bridge.port, PEER_PORT=flight.bridge.peer[1],
                    SITL_PID=flight.simulator.pid, LAT_ORIGIN=HOME[0], LONG_ORIGIN=HOME[1],
                    MOOSDB_PATH=paths['db'], HELM_PATH=paths['helm'], LOGGER_PATH=paths['logger'],
                    SIMULATION_ONLY=str(simulation).lower(), SHADOW_MODE=str(shadow).lower(),
                    BEHAVIOR_DIRECTORY=paths['altitude_behavior'].parent, PLUGIN_DIRECTORY=paths['plugin_directory'])
    mission, behavior = out / 'targ_plane.moos', out / 'targ_plane.bhv'
    for source, target, variables in (
        (mission_dir / 'meta_vehicle.moos', mission, settings), (mission_dir / 'meta_vehicle.bhv', behavior, {}),
    ):
        build = subprocess.run([str(paths['nsplug']), str(source), str(target), '--strict', '--force', '-x',
                                *(f'{key}={value}' for key, value in variables.items())],
                               cwd=out, capture_output=True, text=True, timeout=5)
        need(build.returncode == 0 and target.is_file(), f'mission generation failed: {build.stdout} {build.stderr}')
    if lifecycle:
        recovery = out / 'recovery'
        recovery.mkdir(mode=0o700)
        baseline = 'plugins = telemetry,air_data,arduplane_control'
        text = mission.read_text()
        need(text.count(baseline) == 1, 'lifecycle could not bind the standard Plane mission')
        extra = '\n  plane_native_autoland = true' if paths.get('modern') else ''
        mission.write_text(text.replace(baseline, baseline + ',missions\n  control_mission_execution = true\n'
                                       f'  auto_recovery_directory = {recovery}' + extra))
    if paths.get('autonomy_behavior'):
        from autonomy_missions.stock_grade import with_evaluator
        behavior.write_bytes(paths['autonomy_behavior'].read_bytes())
        mission.write_text(with_evaluator(mission.read_text(), paths['evaluator']))
    if paths.get('helm_tasks'):
        text = mission.read_text()
        mission.write_text(text.replace('  helm_source = pHelmIvP',
                                       '  operation_helm_enabled = true\n'
                                       '  operation_helm_tasks = return,survey,voronoi\n'
                                       '  helm_source = pHelmIvP'))
        text = behavior.read_text().replace('  heading = 90',
                                           '  condition = MAVMOOS_HELM_OPERATION = none\n  heading = 90', 1)
        behavior.write_text(text + (ROOT / 'tests/plane_native/task_behaviors.bhv').read_text())
    return mission


def packet_frames(codec, records):
    for record in records:
        if record.disposition in 'Ri':
            continue
        decoder = codec.MAVLink(None)
        frames = decoder.parse_buffer(record.data) or []
        need(len(frames) == 1, 'owned application TX datagram does not contain one complete MAVLink frame')
        frame = frames[0]
        need((frame.get_srcSystem(), frame.get_srcComponent()) == (245, 191), 'unexpected bridge MAVLink sender identity')
        yield record, frame


class Mission:
    def __init__(self, flight, paths, out, stack, fault_library, simulation=True, shadow=False, lifecycle=False):
        self.flight, self.paths, self.out = flight, paths, out
        self.profile_parameters = 24 if paths.get('modern') else 17 if lifecycle else 16
        self.antler = self.app = self.probe = None
        self.traces = []
        self.moos_rows = []
        self.mail = {}
        self.probe_output = stack.enter_context((out / 'moos-probe.jsonl').open('x'))
        self.probe_readback = stack.enter_context((out / 'moos-probe.jsonl').open('r'))
        self.probe_errors = stack.enter_context((out / 'moos-probe.log').open('x'))
        self.last_wire_read = -1
        self.wire_cache = []
        self.fault_library = fault_library
        self.mission = generate(paths, out, flight, simulation, shadow, lifecycle)
        self.env = fault_tap.clean_environment()
        self.env['PATH'] = os.pathsep.join((str(paths['helm'].parent), str(paths['app'].parent), self.env.get('PATH', '')))
        self.env['IVP_BEHAVIOR_DIRS'] = str(paths['altitude_behavior'].parent)

    def launch(self):
        self.flight.db.close()
        self.antler = start([self.paths['antler'], self.mission], self.out, 'pantler.log', self.env)
        self.probe = subprocess.Popen([str(self.paths['probe']), str(self.flight.ports['db'])], cwd=self.out,
                                      stdin=subprocess.PIPE, stdout=self.probe_output, stderr=self.probe_errors,
                                      start_new_session=True, env=self.env, text=True)
        self.flight.report['owned_antler'] = self.flight.wait(
            8, self._antler_proof, 'pAntler did not start the exact stock helm and MOOSDB')
        # Transfer the serial0 receive port from the discovery socket to the app.
        # The peer remains the exact native connected socket, never a relay.
        self.flight.bridge.endpoint.close()
        self.launch_app()
        self.wait(10, lambda: any(row['kind'] == 'connected' for row in self.moos_rows), 'MOOS probe not connected')
        self.wait(10, lambda: self.latest('MAVMOOS_RUNTIME_STATUS'), 'runtime MOOS status absent')

    def _antler_proof(self):
        proof = {}
        for record in owned_records(self.antler):
            args = shlex.split(record['command'])
            for role in ('helm', 'db', 'logger'):
                if args and args[0] == str(self.paths[role]) and str(self.mission) in args:
                    need(role not in proof, f'owned mission launched duplicate {role}')
                    proof[role] = record
        return proof if set(proof) == {'helm', 'db', 'logger'} else None

    def launch_app(self):
        identity = len(self.traces)
        trace = self.out / f'app-wire-{identity}.bin'
        self.control = self.out / f'app-fault-{identity}.txt'
        tap = fault_tap.environment(self.fault_library, self.flight.bridge.port, self.flight.bridge.peer[1], trace, self.control)
        fault_tap.set_mode(self.control, 'pass')
        self.app = start([self.paths['app'], self.mission, 'iMAVLink'], self.out, f'bridge-{identity}.log', self.env | tap)
        self.traces.append(trace)
        self.wire_cache = []
        self.last_wire_read = -1
        if identity:
            self.flight.event('bridge_restarted', pid=self.app.pid)
        proof = self.wait(8, self.app_proof, 'owned bridge UDP socket did not bind')
        self.flight.report.setdefault('owned_bridges', []).append(proof)

    def app_proof(self):
        need(self.app.poll() is None, f'owned bridge exited ({self.app.returncode}); inspect bridge log')
        proof = subprocess.run(['lsof', '-nP', '-a', '-p', str(self.app.pid), '-iUDP'], capture_output=True, text=True, timeout=3)
        if proof.returncode == 0 and re.search(rf'UDP\s+127\.0\.0\.1:{self.flight.bridge.port}\b', proof.stdout):
            process = snapshot([self.app.pid]).get(self.app.pid)
            if process:
                args = shlex.split(process['command'])
                need(args == [str(self.paths['app']), str(self.mission), 'iMAVLink'], 'owned bridge executable or mission changed')
                return {'process': process, 'udp': proof.stdout}
        return None

    def wait(self, timeout, check, why):
        def checked():
            self.poll_moos()
            return check()

        return self.flight.wait(timeout, checked, why)

    def poll_moos(self):
        need(self.probe.poll() is None and self.antler.poll() is None, 'owned MOOS observer or pAntler exited')
        need(self.app is None or self.app.poll() is None, 'owned bridge exited unexpectedly; inspect bridge log')
        while True:
            at = self.probe_readback.tell()
            line = self.probe_readback.readline()
            if not line.endswith('\n'):
                self.probe_readback.seek(at)
                break
            if not line.startswith('{'):
                continue
            row = json.loads(line)
            self.moos_rows.append(row)
            if row['kind'] == 'mail':
                self.mail[row['key'], row['source']] = row

    def latest(self, key, source='iMAVLink', after=0, max_age=2):
        row = self.mail.get((key, source))
        return row if row and after <= row['mono'] and now() - row['mono'] <= max_age else None

    def notify(self, key, value):
        mark = now()
        self.probe.stdin.write(f'{key}\t{value}\n')
        self.probe.stdin.flush()

        def acknowledgement():
            return next((r for r in reversed(self.moos_rows) if r['mono'] >= mark and r['kind'] == 'request' and
                         r['key'] == key and r['value'] == value), None)

        return self.wait(2, acknowledgement, 'private probe did not acknowledge MOOS publication')['mono']

    def state(self, variable, after=0, **values):
        row = self.latest(variable, after=after)
        return row if row and matches(fields(row), **values) else None

    def helm(self, after=0, **values):
        return self.state('MAVMOOS_HELM_STATUS', after, **values)

    def runtime(self, after=0, **values):
        return self.state('MAVMOOS_RUNTIME_STATUS', after, **values)

    def producer(self, course, height, after=0, speed=22):
        rows = {}
        for key, expected in (('DESIRED_HEADING', course), ('DESIRED_SPEED', speed), ('DESIRED_ALTITUDE', height),
                              ('IVPHELM_STATE', 'DRIVE'), ('IVPHELM_ALLSTOP', 'clear')):
            status = key.startswith('IVPHELM_')
            row = self.latest(key, 'pHelmIvP', after=0 if status else after,
                              max_age=400 if key == 'IVPHELM_ALLSTOP' else 1.5)
            if not row or row['value'] != expected:
                return None
            rows[key] = row
        times = [r['published'] for k, r in rows.items() if k.startswith('DESIRED_')]
        return rows if max(times) - min(times) <= .35 else None

    def acquire(self, course=COURSE, height=HEIGHT, *, verify=True):
        ready = self.wait(22, lambda: self.helm(acquire_preparation='READY') and self.flight.trimmed(LOITER) and
                          self.runtime(plane_profile='VERIFIED', plane_parameters_verified=self.profile_parameters,
                                       plane_watchdog_started=1),
                          'bridge profile, watched heartbeat, stock helm, or native Loiter not ready')
        self.flight.event('acquire_ready', evidence=ready)
        mark = self.notify('MAVMOOS_NAV_CONTROL', 'acquire')
        self.flight.event('acquire', request_mono=mark, course=course, height=height)
        self.wait(6, lambda: self.flight.heartbeat(GUIDED, True, mark) and self.helm(mark, state='ACTIVE'),
                  'fresh Guided and active stock helm did not follow explicit acquisition')
        if verify:
            pair = self.wait(6, lambda: self.target(course, height, mark), 'no new paired target after stock helm acquire')
            self.flight.event('target_verified', evidence=pair)
        return mark

    def configure_leg(self, course, height=HEIGHT, speed=22):
        self.notify('PLANE_COURSE_UPDATE', f'heading={course}')
        self.notify('PLANE_HEIGHT_UPDATE', f'altitude={height}')
        self.notify('PLANE_SPEED_UPDATE', f'speed={speed}')
        mark = self.notify('DEPLOY', 'true')
        return self.wait(6, lambda: self.producer(course, height, mark, speed), 'stock pHelmIvP did not publish leg scalars')

    def trace_records(self, strict=False):
        if not strict and now() - self.last_wire_read < .25:
            return self.wire_cache
        trace = self.traces[-1]
        try:
            result = fault_tap.read_trace(trace, self.flight.bridge.port, self.flight.bridge.peer[1])
        except (FileNotFoundError, ValueError):
            if strict:
                raise
            return self.wire_cache
        self.last_wire_read = now()
        self.wire_cache = result
        return result

    def tx(self, after=0, kinds='T'):
        return ((r, m) for r, m in packet_frames(self.flight.codec, self.trace_records())
                if r.at >= after and r.disposition in kinds)

    def target(self, course, height, after):
        positions = self.flight.monitor.messages
        home = self.flight.latest('HOME_POSITION', max_age=10)
        if not home:
            return None
        for record, msg in self.tx(after):
            if msg.get_type() != 'COMMAND_INT' or msg.command != POINT or abs(msg.z - height) > .05:
                continue
            need((msg.frame, msg.target_system, msg.target_component, msg.param1, msg.param2, msg.param3, msg.param4) ==
                 (6, 1, 1, -1, 0, 0, 0), 'Plane target encoded unexpected frame, destination or mode-change flag')
            before = next((r for r in reversed(positions) if r['mavpackettype'] == 'GLOBAL_POSITION_INT' and
                           0 <= record.at - r['mono'] <= .25), None)
            paired = next((r for r in positions if r['mavpackettype'] == 'POSITION_TARGET_GLOBAL_INT' and
                           0 < r['mono'] - record.at <= 1.5 and r['lat_int'] == msg.x and r['lon_int'] == msg.y and
                           r['coordinate_frame'] == 0 and r['type_mask'] == TARGET_MASK and
                           abs(r['alt'] - height - home['altitude'] / 1000) <= .12), None)
            if paired and before:
                north, east = offset(before['lat'], before['lon'], msg.x, msg.y)
                home_range = math.hypot(*offset(home['latitude'], home['longitude'], msg.x, msg.y))
                lookahead, projected = math.hypot(north, east), angle(north, east)
                if 290 <= lookahead <= 310 and abs(signed_turn(course, projected)) <= 5 and home_range <= 1081:
                    return {'sent_mono': record.at, 'frame': msg.to_dict(), 'readback': paired, 'before': before,
                            'lookahead_m': lookahead, 'course_deg': projected, 'home_distance_m': home_range}
        return None

    def fault(self, mode):
        bracket = fault_tap.set_mode(self.control, mode)
        self.flight.event('fault_' + mode, switch_bracket=bracket)
        return bracket[1]

    def stop_app(self, kill=False):
        if not self.app:
            return
        if kill:
            proof = self.app_proof()
            need(proof is not None, 'app process loss injection lacks current ownership proof')
            self.flight.event('bridge_killed', pid=self.app.pid, proof=proof)
            self.app.send_signal(signal.SIGKILL)
            need(self.app.wait(timeout=5) == -signal.SIGKILL, 'bridge crash injection did not stop selected process')
        remaining = stop_process_tree(self.app)
        self.app = None
        need(not remaining, f'owned bridge process remains: {remaining}')

    def close(self):
        cleanup = self.flight.report.setdefault('owned_mission_processes_remaining', [])
        try:
            self.stop_app()
        except Exception as error:
            cleanup.append({'role': 'bridge', 'error': str(error)})
        for proc in (self.probe, self.antler):
            if proc is not None:
                if proc.stdin:
                    proc.stdin.close()
                try:
                    cleanup.extend(stop_process_tree(proc))
                except Exception as error:
                    cleanup.append({'pid': proc.pid, 'error': str(error)})
        self.flight.report['bridge_traces'] = [trace.name for trace in self.traces]


def positions(flight, after, before=None):
    return [row for row in flight.monitor.messages if row['mavpackettype'] == 'GLOBAL_POSITION_INT' and
            after <= row['mono'] <= (now() if before is None else before)]


def track(flight, course, height, after):
    def physical():
        samples = positions(flight, after)
        if not samples:
            return None
        end = samples[-1]
        leg = [p for p in samples if p['mono'] >= end['mono'] - 4.2]
        if len(leg) < 32 or leg[-1]['mono'] - leg[0]['mono'] < 3.8 or not flight.heartbeat(GUIDED, True):
            return None
        if any(abs(signed_turn(course, angle(p['vx'], p['vy']))) > 23 or not 16 <= ground_speed(p) <= 29 or
               abs(p['relative_alt'] / 1000 - height) > 11 for p in leg):
            return None
        air = [r for r in flight.monitor.messages if r['mavpackettype'] == 'AIRSPEED' and r['id'] == 0 and
               leg[0]['mono'] <= r['mono'] <= leg[-1]['mono']]
        travel = distance(leg[0], leg[-1])
        north, east = offset(leg[0]['lat'], leg[0]['lon'], leg[-1]['lat'], leg[-1]['lon'])
        if len(air) < 12 or any(r['flags'] & 3 != 2 or not 16 <= r['airspeed'] <= 30 for r in air) or travel < 68:
            return None
        return {'start_mono': leg[0]['mono'], 'end_mono': leg[-1]['mono'], 'position_samples': len(leg),
                'air_samples': len(air), 'airspeed_min_mps': min(r['airspeed'] for r in air),
                'airspeed_max_mps': max(r['airspeed'] for r in air), 'groundspeed_min_mps': min(map(ground_speed, leg)),
                'groundspeed_max_mps': max(map(ground_speed, leg)), 'height_min_m': min(r['relative_alt'] / 1000 for r in leg),
                'height_max_m': max(r['relative_alt'] / 1000 for r in leg), 'north_m': north, 'east_m': east,
                'displacement_m': travel}

    proof = flight.wait(19, physical, 'independent trajectory never reached declared course, altitude and speed window')
    flight.event('track_observed', course=course, height=height, evidence=proof)
    return proof


def orbit(flight, mode, after):
    def physical():
        heartbeat = flight.heartbeat(mode, True, after)
        target = flight.latest('POSITION_TARGET_GLOBAL_INT', after)
        if not heartbeat or not target or target['coordinate_frame'] != 0:
            return None
        recent = positions(flight, after)
        if not recent:
            return None
        last = recent[-1]
        window = [r for r in recent if r['mono'] >= last['mono'] - 11.2]
        if len(window) < 90 or last['mono'] - window[0]['mono'] < 10.5:
            return None
        polar = [offset(target['lat_int'], target['lon_int'], row['lat'], row['lon']) for row in window]
        radii = [math.hypot(*point) for point in polar]
        bearings = [angle(*point) for point in polar]
        arc = sum(signed_turn(a, b) for a, b in zip(bearings, bearings[1:]))
        if arc < 140 or any(not 49 <= radius <= 120 for radius in radii) or any(
                not 15 <= ground_speed(row) <= 30 or abs(row['alt'] / 1000 - target['alt']) > 20 for row in window):
            return None
        return {'start_mono': window[0]['mono'], 'end_mono': last['mono'], 'samples': len(window),
                'center': target, 'arc_deg': arc, 'minimum_radius_m': min(radii), 'maximum_radius_m': max(radii)}

    proof = flight.wait(43, physical, 'native Loiter/RTL did not demonstrate bounded clockwise flight around its reported center')
    flight.event('orbit_observed', mode=mode, evidence=proof)
    return proof


def actual_return(flight, after):
    home = flight.latest('HOME_POSITION', max_age=10)
    need(home is not None, 'HOME missing for independent return proof')
    origin = {'lat': home['latitude'], 'lon': home['longitude']}
    prior = positions(flight, after - 1, after)
    need(prior, 'no independent position immediately before RTL')
    start = prior[-1]
    start_range = distance(origin, start)

    def progress():
        position = flight.latest('GLOBAL_POSITION_INT', after)
        if flight.heartbeat(RTL, True, after) and position:
            current = distance(origin, position)
            if current <= 160 and start_range - current >= 70:
                return {'from_m': start_range, 'to_m': current, 'relative_height_m': position['relative_alt'] / 1000,
                        'ground_speed_mps': ground_speed(position)}
        return None

    result = flight.wait(43, progress, 'native RTL did not physically approach the bound home')
    flight.event('return_observed', evidence=result)
    return result


def pilot_switch(flight, mode):
    need(mode in (FBWA, LOITER), 'invalid fixture switch destination')
    proof = inventory(flight.simulator)
    need(exact_rc(proof, flight.ports['rc']), 'simulated RC destination is not the owned native UDP socket')
    channels = (1500, 1500, 1580, 1500, 1000, 1000, 1000, 1550 if mode == FBWA else 1430)
    at = now()
    payload = struct.pack('<8H', *channels)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as pilot:
        pilot.bind(('127.0.0.1', 0))
        need(pilot.sendto(payload, ('127.0.0.1', flight.ports['rc'])) == len(payload), 'private simulated RC write failed')
    flight.event('pilot_switch', mode=mode, tx_mono=at, pwm=channels, inventory=proof)
    return flight.wait(6, lambda: flight.heartbeat(mode, True, at), 'owned simulated RC switch did not change FC mode')


def watch_heartbeat(mission, after=0):
    return [(r, m) for r, m in mission.tx(after) if m.get_type() == 'HEARTBEAT']


def mode_frames(mission, mode, after=0, kinds='T'):
    return [(r, m) for r, m in mission.tx(after, kinds) if m.get_type() == 'COMMAND_LONG' and
            m.command == SET_MODE and m.param1 == 1 and m.param2 == mode]


def points(mission, after=0, kinds='T'):
    return [(r, m) for r, m in mission.tx(after, kinds) if m.get_type() == 'COMMAND_INT' and m.command == POINT]


def ensure_for(mission, seconds, predicate, error):
    end = now() + seconds
    while now() < end:
        mission.flight.pump(min(.04, max(0, end - now())))
        mission.poll_moos()
        need(predicate(), error)


def mode_receipt(flight, name, heartbeat, reason):
    at = heartbeat['mono']
    previous = positions(flight, at - .5, at)
    following = flight.wait(.6, lambda: flight.latest('GLOBAL_POSITION_INT', at), 'no FC position adjacent to mode observation')
    boot_before = previous[-1]['time_boot_ms'] if previous else following['time_boot_ms']
    prior_mode = next((row['custom_mode'] for row in reversed(flight.monitor.messages)
                       if row['mavpackettype'] == 'HEARTBEAT' and row['mono'] < at), None)
    need(prior_mode is not None and prior_mode != heartbeat['custom_mode'], 'no distinct prior independent mode')
    row = {'label': name, 'mode': heartbeat['custom_mode'], 'expected_reason': reason,
           'previous_mode': prior_mode, 'receipt_mono': at, 'boot_ms_before': boot_before,
           'boot_ms_after': following['time_boot_ms']}
    flight.report.setdefault('required_mode_reasons', []).append(row)
    flight.event('mode_receipt', **row)
    return row


def native_command(mission, action, mode, label):
    action_id = 'fixture-loiter' if action == 'loiter' else 'fixture-return'
    mark = mission.notify('MAVMOOS_COMMAND', f'id={action_id},action={action}')
    heartbeat = mission.wait(6, lambda: mission.flight.heartbeat(mode, True, mark), 'native action lacked observed FC mode')
    answer = mission.wait(5, lambda: mission.state('MAVMOOS_COMMAND_RESULT', mark, id=action_id, status='CONFIRMED'),
                          'native action lacked post-handoff heartbeat confirmation')
    mission.wait(3, lambda: mode_frames(mission, mode, mark), 'native action absent on owned wire')
    mode_receipt(mission.flight, label, heartbeat, 2)
    mission.flight.event(label, result=answer, request_mono=mark)
    return heartbeat['mono']


def released(mission, mark, label):
    heartbeat = mission.wait(7, lambda: mission.flight.heartbeat(LOITER, True, mark), 'bridge did not hand back to native Loiter')
    mission.wait(3, lambda: mode_frames(mission, LOITER, mark), 'no application Loiter handback on owned wire')
    mode_receipt(mission.flight, label, heartbeat, 2)
    mission.wait(3, lambda: mission.runtime(plane_handback='CONFIRMED'), 'host Loiter handback not confirmed')
    mission.flight.event(label, request_mono=mark, observed_mono=heartbeat['mono'])
    return heartbeat['mono']


def normal(mission):
    flight = mission.flight
    mission.configure_leg(COURSE)
    acquired = mission.acquire()
    first_track = track(flight, COURSE, HEIGHT, acquired)
    # A real NothingToDo/NoGoalBehavior from the unchanged producer opens the
    # short handoff window. Keep its end under the configured 1.0 s bound.
    def confirmed_pause_point():
        pair = mission.target(COURSE, HEIGHT, max(first_track['end_mono'], now() - .45))
        if pair and mission.runtime(pair['sent_mono'], permission='true', plane_target='VERIFIED'):
            return pair

    anchor = mission.wait(5, confirmed_pause_point, 'no fresh confirmed target before normal handoff')
    pause_request = mission.notify('DEPLOY', 'false')
    pause = mission.wait(.8, lambda: mission.helm(pause_request, state='PAUSED'), 'stock helm did not produce a normal handoff')
    off = mission.latest('IVPHELM_ALLSTOP', 'pHelmIvP', pause_request)
    need(off and off['value'] in ('NothingToDo', 'NoGoalBehavior'), 'stock all-stop reason is not a normal handoff')
    resume_request = mission.notify('DEPLOY', 'true')
    resume = mission.wait(1.0, lambda: mission.helm(resume_request, state='ACTIVE', reason='resumed_after_normal_handoff'),
                          'bounded normal handoff failed to resume')
    need(resume['mono'] - pause['mono'] < 1.0 and flight.heartbeat(GUIDED, True), 'handoff exceeded its window or left Guided')
    fresh = mission.wait(2, lambda: mission.producer(COURSE, HEIGHT, resume_request), 'resume lacked new stock helm scalars')
    mission.wait(5, lambda: mission.target(COURSE, HEIGHT, resume_request), 'fresh target after normal handoff not verified')
    need(not mode_frames(mission, GUIDED, pause_request), 'normal handoff unexpectedly sent a new mode request')
    sent_during_pause = [r.at for r, _ in points(mission, pause['mono']) if r.at < resume_request]
    need(not sent_during_pause, 'old navigation escaped during the confirmed normal pause')
    flight.event('normal_handoff', anchor=anchor, pause=pause, resume=resume, producer_after_resume=fresh)
    mark = mission.notify('MAVMOOS_NAV_CONTROL', 'release')
    handed = released(mission, mark, 'operator_release')
    loiter_orbit = orbit(flight, LOITER, handed)
    need(not points(mission, handed), 'released navigation leaked into native Loiter')

    # Explicit Loiter and RTL are the native-action surface in the same plugin.
    second_course, second_height = 180, 75
    mission.configure_leg(second_course, second_height)
    acquired = mission.acquire(second_course, second_height)
    second_track = track(flight, second_course, second_height, acquired)
    loiter = native_command(mission, 'loiter', LOITER, 'explicit_loiter')
    ensure_for(mission, 2, lambda: flight.heartbeat(LOITER, True) and not points(mission, loiter),
               'explicit native Loiter did not own the FC')
    third_course = 95
    mission.configure_leg(third_course)
    mission.acquire(third_course)
    returned = native_command(mission, 'rtl', RTL, 'explicit_rtl')
    approach = actual_return(flight, returned)
    home_orbit = orbit(flight, RTL, returned)
    need(not points(mission, returned), 'expired Guided navigation leaked into RTL')
    return {'first_track': first_track, 'second_track': second_track, 'normal_handoff_s': resume['mono'] - pause['mono'],
            'release_orbit': loiter_orbit, 'explicit_rtl_approach': approach, 'rtl_orbit': home_orbit}


def producer_loss(mission, invalid=False):
    flight = mission.flight
    mission.configure_leg(COURSE)
    mission.acquire()
    if invalid:
        mark = mission.notify('PLANE_SPEED_UPDATE', 'speed=19')
        mission.wait(4, lambda: mission.helm(mark, state='LATCHED', reason='input_out_of_envelope'),
                     'out-of-envelope stock speed did not latch authority')
        flight.event('out_of_envelope_speed', producer=mission.latest('DESIRED_SPEED', 'pHelmIvP', mark))
        handed = released(mission, mark, 'invalid_input_handback')
        mission.notify('PLANE_SPEED_UPDATE', 'speed=22')
    else:
        helm = mission._antler_proof()['helm']
        mark = flight.event('producer_stopped', process=helm)
        os.kill(helm['pid'], signal.SIGSTOP)
        try:
            handed = released(mission, mark, 'producer_silence_handback')
        finally:
            need(snapshot([helm['pid']]).get(helm['pid']) == helm, 'stopped stock producer identity changed')
            os.kill(helm['pid'], signal.SIGCONT)
            flight.event('producer_resumed', pid=helm['pid'])
    mission.wait(4, lambda: mission.producer(COURSE, HEIGHT, handed), 'producer did not resume normal fresh mail')
    ensure_for(mission, 2, lambda: flight.heartbeat(LOITER, True) and not points(mission, handed),
               'returning producer mail reacquired control without an operator request')
    resumed = mission.acquire()
    mark = mission.notify('MAVMOOS_NAV_CONTROL', 'release')
    released(mission, mark, 'post_reacquire_release')
    return {'handback_mono': handed, 'fresh_reacquire_mono': resumed}


def takeover(mission):
    flight = mission.flight
    mission.configure_leg(COURSE)
    mission.acquire()
    mission.fault('hold_point')
    held = mission.wait(4, lambda: points(mission, kinds='h'), 'next owned Guided target was not held by the wire fixture')
    held = held[0][1]
    heartbeat = pilot_switch(flight, FBWA)
    mode_receipt(flight, 'pilot_takeover', heartbeat, 1)
    mark = mission.fault('release_point')
    delivery = mission.wait(4, lambda: points(mission, mark, 'd'), 'held target was not delivered while external mode owned the FC')
    delivered = delivery[0][0].at
    ensure_for(mission, 7, lambda: flight.heartbeat(FBWA, True), 'healthy pilot takeover did not survive watchdog interval')
    need(not points(mission, heartbeat['mono']), 'old authority sent additional native navigation after pilot takeover')
    need(not any(r['mavpackettype'] == 'POSITION_TARGET_GLOBAL_INT' and r['mono'] >= delivered and
                 r['lat_int'] == held.x and r['lon_int'] == held.y for r in flight.monitor.messages),
         'prior Guided target changed FC target outside Guided mode')
    keepalive = [r.at for r, _ in watch_heartbeat(mission, heartbeat['mono'])]
    need(len(keepalive) >= 6 and max(b - a for a, b in zip(keepalive, keepalive[1:])) < 2,
         'healthy externally owned FC mode did not receive observed watchdog heartbeat')
    flight.event('late_point_outside_guided', delivered=delivered, held_target=held.to_dict())
    mission.fault('pass')
    hb = pilot_switch(flight, LOITER)
    mode_receipt(flight, 'pilot_loiter', hb, 1)
    mission.acquire()
    mark = mission.notify('MAVMOOS_NAV_CONTROL', 'release')
    released(mission, mark, 'post_takeover_release')
    return {'late_target_delivered_mono': delivered, 'healthy_pilot_seconds': hb['mono'] - heartbeat['mono'],
            'watchdog_heartbeats_in_fbwa': len(keepalive)}


def watchdog_loss(mission, kind):
    flight = mission.flight
    mission.configure_leg(COURSE)
    mission.acquire()
    track(flight, COURSE, HEIGHT, now())
    mission.wait(2, lambda: watch_heartbeat(mission, now() - 1.2), 'no current watched heartbeat before loss injection')
    before_trace = mission.traces[-1]
    if kind == 'process':
        mark = now()
        mission.stop_app(kill=True)
        mission.launch_app()
    else:
        mark = mission.fault({'rx': 'drop_rx', 'tx': 'drop_tx', 'handback': 'drop_loiter'}[kind])
        if kind == 'handback':
            mission.notify('MAVMOOS_NAV_CONTROL', 'release')
    rtl = mission.wait(13, lambda: flight.heartbeat(RTL, True, mark), 'independent FC watchdog RTL not observed')
    transition = mode_receipt(flight, 'watchdog_' + kind, rtl, 5)
    old_trace = fault_tap.read_trace(before_trace, flight.bridge.port, flight.bridge.peer[1])
    last = max(r.at for r, m in packet_frames(flight.codec, old_trace) if r.disposition == 'T' and
               r.at < rtl['mono'] and m.get_type() == 'HEARTBEAT')
    gap = rtl['mono'] - last
    need(4.7 <= gap <= 6.65, f'watchdog RTL lacked expected 5-second watched-heartbeat gap ({gap:.3f}s)')
    need(not points(mission, rtl['mono']), 'watchdog RTL did not fence old points')
    if kind != 'process':
        fault_code = {'rx': 'i', 'tx': 'o', 'handback': 'l'}[kind]
        need(any(r.disposition == fault_code and r.at >= mark for r in old_trace), 'requested directional fault was not witnessed')
        healthy_path = 'R' if kind in ('tx', 'handback') else 'i'
        need(sum(r.disposition == healthy_path and mark <= r.at <= rtl['mono'] for r in old_trace) > 10,
             'independent fault case lacked continued native→app input at the tap')
        mission.fault('pass')
    else:
        need(not any(r.at < rtl['mono'] for r, _ in watch_heartbeat(mission)),
             'restarted bridge renewed the watched heartbeat while prior Guided state was uncertain')
        need(not any(r.at < rtl['mono'] for r, _ in points(mission)),
             'restarted bridge inherited stale Guided targets')
    intermediate_loiter = transition['previous_mode'] == LOITER
    if intermediate_loiter:
        need(kind == 'rx' and any(mark <= r.at < rtl['mono'] for r, _ in mode_frames(mission, LOITER, mark)) and
             any(row['mavpackettype'] == 'HEARTBEAT' and row['custom_mode'] == LOITER and
                 mark <= row['mono'] < rtl['mono'] for row in flight.monitor.messages),
             'unexpected intermediate Loiter before watchdog RTL')
    else:
        need(transition['previous_mode'] == GUIDED, 'watchdog transition did not follow the owned flight mode')
    approach = actual_return(flight, rtl['mono'])
    ensure_for(mission, 2, lambda: flight.heartbeat(RTL, True) and not points(mission, rtl['mono']),
               'restored route reused old helm authority')
    flight.mode(LOITER)
    # The stock change-driven clear must be newly observed after an actual app
    # restart, rather than treating the retained all-stop as live authority.
    mission.notify('DEPLOY', 'false')
    mission.wait(3, lambda: (r := mission.latest('IVPHELM_ALLSTOP', 'pHelmIvP')) and r['value'] != 'clear',
                 'stock helm did not expose idle state for recovery')
    mission.configure_leg(COURSE)
    recovered = mission.acquire()
    mark_release = mission.notify('MAVMOOS_NAV_CONTROL', 'release')
    released(mission, mark_release, 'post_watchdog_release')
    return {'kind': kind, 'last_watched_heartbeat_mono': last, 'rtl_mono': rtl['mono'], 'gap_s': gap,
            'intermediate_loiter': intermediate_loiter, 'return': approach, 'fresh_reacquire_mono': recovered}


def passive(mission, shadow):
    mission.configure_leg(COURSE)
    if shadow:
        mark = mission.notify('MAVMOOS_NAV_CONTROL', 'acquire')
        mission.wait(5, lambda: mission.helm(mark, state='ACTIVE', reason='shadow_eligible'),
                     'stock helm did not reach observed shadow eligibility')
    ensure_for(mission, 4, lambda: mission.flight.heartbeat(LOITER, True) and not list(mission.tx(kinds='Teloqhde')),
               'passive/shadow fixture emitted a packet or changed native mode')
    records = mission.trace_records(strict=True)
    need(len(records) > 50 and all(r.disposition == 'R' for r in records), 'passive fixture lacked inbound-only trace')
    return {'egress_datagrams': 0, 'ingress_datagrams': len(records), 'shadow': shadow}


def exercise(mission, case):
    mission.launch()
    if case in ('lifecycle', 'native-autoland', 'approach-takeover'):
        from lifecycle import exercise_lifecycle
        return exercise_lifecycle(mission, approach_takeover=case == 'approach-takeover')
    if case == 'operator-nudges':
        from operator_nudges import exercise_operator_nudges
        return exercise_operator_nudges(mission)
    if case == 'helm-tasks':
        from helm_tasks import exercise_helm_tasks
        return exercise_helm_tasks(mission)
    if case == 'helm-mode-stress':
        from helm_mode_stress import run
        return run(mission)
    if case == 'autonomy-survey':
        from autonomy_survey import survey
        return survey(mission)
    if case == 'normal':
        return normal(mission)
    if case in ('silence', 'invalid'):
        return producer_loss(mission, case == 'invalid')
    if case == 'takeover':
        return takeover(mission)
    if case.startswith('loss-'):
        return watchdog_loss(mission, case.removeprefix('loss-'))
    return passive(mission, case == 'shadow')
