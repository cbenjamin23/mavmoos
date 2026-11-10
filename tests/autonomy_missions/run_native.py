#!/usr/bin/env python3
"""Run and grade stock IvP Helm autonomy on two pinned, owned SITL profiles."""
import argparse
import fcntl
import json
import math
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tests'))
from firmware_identity import load_selected_firmware
from owned_processes import stop_process_tree
from rover.run_rover import port, sha
from rover_expanded.run_native import (
    ARDUPILOT, DEFAULTS, HOME, MOOS, PROFILES, build_source, endpoint_cleanup,
    env_for, evidence_files, mission_inventory, require, start, tracked_pin, wait_endpoints,
)
from rover_mission.run_native import Bench
from confirmation_loss import Route

MISSION = ROOT / 'missions/autonomy_sitl'
COPTER_MANIFEST = ROOT / 'config/firmware/arducopter-4.6.3-quad.json'
STOCK_HELM = '5b3d2e705609c37b5b5000384655b4491010d6d797b94c43830d9972e684a876'
ROVER_BINARY = 'a859ab703395bbb04da523dae5414cc5fbc23b736a1c71639f2ef3e255df5fc9'
SURVEY_ROUTE = ((0, 7), (0, 15), (9, 15), (9, 23), (0, 23))
BOAT_FINISH = (0, 36)
COPTER_MISSIONS = ('copter-survey', 'copter-mode-stress')
BOAT_MISSIONS = ('boat-transect', 'boat-mode-stress')
STRESS_MISSIONS = ('copter-mode-stress', 'boat-mode-stress')


def xy(row):
    scale = math.pi * 6378137 / 180
    return ((row['lon'] * 1e-7 - HOME[1]) * scale * math.cos(math.radians(HOME[0])),
            (row['lat'] * 1e-7 - HOME[0]) * scale)


def speed(row):
    return math.hypot(row['vx'], row['vy']) / 100


def setup_inputs(args, copter):
    build, moos, ap = args.build, args.moos_root, args.ardupilot_root
    tracked_pin(moos, MOOS)
    source = build_source(build)
    require(source['path'] == str(ROOT) and not source['tracked_changes'], 'build source must be the clean mission checkout')
    require(sha(moos / 'bin/pHelmIvP') == STOCK_HELM, 'stock helm hash differs')
    if copter:
        firmware = load_selected_firmware(ROOT, ap, COPTER_MANIFEST, args.firmware_attestation)
        require(firmware and firmware['profile_id'] == 'arducopter-4.6.3-quad' and
                not firmware['fixture']['hardware'], 'expected the attested Copter SITL quad')
        binary = Path(firmware['native_path'])
        defaults = [ap / 'Tools/autotest/default_params/copter.parm']
        altitude = [p for p in args.behavior_dir.glob('libBHV_ConstantAltitude.*')
                    if p.suffix in ('.so', '.dylib') and p.is_file()]
        require(len(altitude) == 1, 'expected one built ConstantAltitude behavior')
        extra = [Path(firmware['manifest_path']), Path(firmware['attestation_path']), *altitude]
        model, vehicle_type, profile, firmware_revision = 'quad', 2, 'arducopter', firmware['source_revision']
    else:
        tracked_pin(ap, ARDUPILOT)
        profile, vehicle_type, model, names = PROFILES['boat']
        defaults = [ap / 'Tools/autotest/default_params' / name for name in names]
        require(all(sha(path) == DEFAULTS[path.name] for path in defaults), 'boat defaults differ from pin')
        binary = ap / 'build/sitl/bin/ardurover'
        require(sha(binary) == ROVER_BINARY, 'boat simulator executable differs from pin')
        extra, firmware_revision = [], ARDUPILOT
    witnesses = [binary, *defaults, *extra, build / 'CMakeCache.txt', build / 'bin/iMAVLink',
                 build / 'bin/autonomy_mission_probe', build / 'plugins/mavmoos_telemetry.so',
                 build / 'plugins' / ('mavmoos_arducopter_control.so' if copter else 'mavmoos_ardurover_control.so'),
                 *(moos / 'bin' / name for name in ('MOOSDB', 'pAntler', 'pHelmIvP', 'nsplug', 'pLogger', 'pMissionEval')),
                 Path(__file__), ROOT / 'tests/rover_mission/run_native.py',
                 ROOT / 'tests/rover_mission/RoverMissionProbe.cpp', ROOT / 'tests/rover_expanded/run_native.py',
                 MISSION / 'meta_vehicle.moos', MISSION / (args.mission.replace('-', '_') + '.bhv')]
    if args.mission in STRESS_MISSIONS:
        witnesses.extend((args.tap, ROOT / 'tests/confirmation_loss.py', ROOT / 'tests/plane_faults/fault_tap.py',
                          ROOT / 'tests/plane_faults/OwnedPlaneFaultTap.cpp',
                          Path(__file__).with_name('mode_stress.py')))
        if copter:
            witnesses.append(build / 'plugins/mavmoos_missions.so')
    return binary, defaults, model, vehicle_type, profile, firmware_revision, source, witnesses


def generate(args, copter, db, bind, peer, pid):
    replay = args.output / 'replay'
    replay.mkdir(mode=0o700)
    stress = args.mission in STRESS_MISSIONS
    managed = args.mission == 'copter-mode-stress'
    recovery = args.output / 'auto-recovery'
    if managed:
        recovery.mkdir(mode=0o700)
    macros = {
        'MOOS_PORT': db, 'BIND_PORT': bind, 'PEER_PORT': peer, 'SITL_PID': pid,
        'BRIDGE_LAUNCH': '// Supervisor launches iMAVLink with its owned wire tap' if stress else
                         'Run = iMAVLink @ NewConsole = false ~ iMAVLink',
        'HELM_PATH': args.moos_root / 'bin/pHelmIvP',
        'PLUGIN_DIRECTORY': args.build / 'plugins', 'REPLAY_DIRECTORY': replay,
        'COMMUNITY': args.mission.replace('-', '_'), 'EXPECTED_TYPE': 2 if copter else 11,
        'PROFILE': 'arducopter' if copter else 'ardurover_boat',
        'PLUGINS': ('telemetry,arducopter_control,missions' if managed else
                    'telemetry,arducopter_control' if copter else 'telemetry,ardurover_control'),
        'MANAGED_AUTO_CONFIG': 'control_mission_execution = true' if managed else '// No managed AUTO',
        'AUTO_RECOVERY_CONFIG': f'auto_recovery_directory = {recovery}' if managed else '// No AUTO state',
        'SPEED_DOMAIN': 'speed:0:5:51' if copter else 'speed:0:2:21',
        'ALTITUDE_CONFIG': 'domain = altitude:0:50:51' if copter else '// Planar helm',
        'REQUIRED_FLAG': 'SURVEY_DONE' if copter else 'CROSSING_ACTIVE',
    }
    for kind, template in (('moos', 'meta_vehicle.moos'), ('bhv', macros['COMMUNITY'] + '.bhv')):
        target = args.output / f'targ_vehicle.{kind}'
        subprocess.run([str(args.moos_root / 'bin/nsplug'), str(MISSION / template), str(target),
                        '--strict', '--force', '-x', *(f'{key}={value}' for key, value in macros.items())],
                       cwd=args.output, check=True, timeout=10)
        require(target.is_file() and '$(' not in target.read_text(), 'unresolved mission template')
    return args.output / 'targ_vehicle.moos'


class MissionBench(Bench):
    def __init__(self, out, profile, native, launch, probe, monitor, result, vehicle_type):
        super().__init__(out, profile, native, launch, probe, monitor, None, result, expected_type=vehicle_type)
        self.guided_mode = '15' if profile == 'boat' else '4'
        self.contact = None
        self.next_contact = 0
        self.contact_trace = (out / 'contact.jsonl').open('w') if profile == 'boat' else None

    def pump(self):
        super().pump()
        now = time.monotonic()
        if self.contact and now >= self.next_contact:
            x, y = contact_xy(self.contact, now)
            self.post('NODE_REPORT', f'NAME=crossing,TYPE=ship,X={x:.3f},Y={y:.3f},'
                      f'HDG=270,SPD={self.contact["speed_m_s"]},TIME={time.time():.6f}')
            self.contact_trace.write(json.dumps({'at': now, 'x': x, 'y': y}) + '\n')
            self.next_contact = now + .18

    def close(self):
        if self.contact_trace is not None:
            self.contact_trace.close()
        super().close()

    def helm_event(self, key, value, after):
        return next((row for row in reversed(self.mail) if row['at'] >= after and row['kind'] == 'event'
                     and row['key'] == key and row['source'] == 'pHelmIvP' and row['value'] == value), None)

    def fix(self, after=0):
        row = self.last('GLOBAL_POSITION_INT', after)
        return row if row and time.monotonic() - row['at'] < .5 else None

    def positions(self, start, end):
        return [row for row in self.wire if row['mavpackettype'] == 'GLOBAL_POSITION_INT' and start <= row['at'] <= end]

    def runtime(self, **wanted):
        require(not any(row['kind'] == 'event' and row['key'] == 'IVPHELM_STATE' and
                        row['source'] == 'pHelmIvP' and row['value'] == 'MALCONFIG' for row in self.mail[-80:]),
                'stock helm rejected the mission behavior configuration')
        data = self.state('MAVMOOS_RUNTIME_STATUS')
        return data if all(data.get(key) == value for key, value in wanted.items()) else None

    def command(self, action, timeout=12, **arguments):
        identity = 'mission-' + action
        payload = f'id={identity},action={action}' + ''.join(f',{k}={v}' for k, v in arguments.items())
        sent = self.post('MAVMOOS_COMMAND', payload)
        self.result.setdefault('native_commands', {})[action] = self.receipt(identity, timeout=timeout)
        return sent

    def deploy(self):
        self.post('DEPLOY', 'true')
        self.wait(lambda: self.runtime(permission='false', fc_mode=self.guided_mode) and
                  {'DESIRED_HEADING', 'DESIRED_SPEED'} <= self.helm_outputs.keys(),
                  'stock helm did not produce guidance before acquisition', 8)
        acquired = self.post('MAVMOOS_NAV_CONTROL', 'acquire')
        self.wait(lambda: self.runtime(permission='true', fc_mode=self.guided_mode), 'Guided helm grant absent', 8)
        return acquired

    def station(self, goal, after, radius):
        def dwell():
            row = self.fix(after)
            if not row:
                return None
            points = self.positions(row['at'] - 2.5, row['at'])
            if (len(points) >= 12 and points[0]['at'] >= after and points[-1]['at'] - points[0]['at'] >= 2.2
                    and max(math.dist(xy(p), goal) for p in points) <= radius and max(map(speed, points)) <= .55):
                return {'samples': len(points), 'seconds': points[-1]['at'] - points[0]['at'],
                        'max_error_m': max(math.dist(xy(p), goal) for p in points), 'fix': row}
            return None

        return self.wait(dwell, 'independent stationkeeping dwell absent', 23)


def survey(bench):
    takeoff = bench.command('takeoff', timeout=55, height=5)
    bench.wait(lambda: (row := bench.fix(takeoff)) and 4.4 <= row['relative_alt'] / 1000 <= 5.6,
               'independent takeoff height absent', 10)
    active = bench.deploy()

    def first_leg():
        fix = bench.fix(active)
        return fix if (fix and math.dist(xy(fix), SURVEY_ROUTE[0]) < 2.3 and
                       bench.helm_event('SURVEY_INDEX', 1, active)) else None

    first = bench.wait(first_leg, 'first survey waypoint and stock next-leg index not observed', 25)
    require(bench.runtime(permission='true', fc_mode='4'), 'route update requires active helm control')
    update = bench.post('SURVEY_UPDATE', 'xpoints=' + ':'.join(f'{x},{y}' for x, y in SURVEY_ROUTE))
    done = bench.wait(lambda: bench.helm_event('SURVEY_DONE', 'true', update), 'updated survey did not finish', 65)
    checkpoints = []
    samples = bench.positions(update, done['at'] + .25)
    cursor = update
    for index, point in enumerate(SURVEY_ROUTE[1:], start=1):
        hit = next((p for p in samples if p['at'] > cursor and math.dist(xy(p), point) <= 2.5), None)
        require(hit, f'independent track missed revised waypoint {index}: {point}')
        checkpoints.append({'goal': point, 'fix': hit, 'error_m': math.dist(xy(hit), point)})
        cursor = hit['at']
    require(all(4.2 <= p['relative_alt'] / 1000 <= 5.8 for p in samples), 'survey left the altitude band')
    returning = bench.wait(lambda: bench.helm_event('SURVEY_RETURNING', 'true', done['at']),
                           'stock helm did not select its return behavior', 5)
    finished = bench.wait(lambda: bench.helm_event('SITL_ROUTE_FINISHED', 'true', returning['at']),
                          'autonomous return did not reach home', 40)
    station = bench.station((0, 0), finished['at'], 2)
    land = bench.command('land', timeout=65)
    bench.wait(lambda: bench.runtime(permission='false'), 'land handoff left helm authority active', 5)
    bench.post('DEPLOY', 'false')

    def landed():
        fix = bench.fix(land)
        state = bench.last('EXTENDED_SYS_STATE', land)
        return (fix if fix and state and state['landed_state'] == 1 and
                fix['relative_alt'] <= 650 and math.dist(xy(fix), (0, 0)) <= 2.5 else None)

    touchdown = bench.wait(landed, 'independent touchdown near home absent', 10)
    if not bench.mode(9, land, armed=False):
        bench.command('disarm')
    bench.wait(lambda: (row := bench.last('HEARTBEAT', land)) and not row['base_mode'] & 128,
               'independent final disarm absent', 8)
    return {'first_leg': first, 'route_update_at': update, 'updated_checkpoints': checkpoints,
            'stock_survey_done': done, 'stock_return_started': returning, 'stock_return_done': finished,
            'home_station': station, 'touchdown': touchdown}


def contact_xy(contact, at):
    return contact['start_x_m'] - contact['speed_m_s'] * (at - contact['at']), contact['cross_y_m']


def transect(bench):
    active = bench.deploy()

    def underway():
        fix = bench.fix(active)
        return fix if fix and xy(fix)[1] >= 4 and abs(xy(fix)[0]) <= 1.3 and fix['vx'] >= 82 else None

    entry = bench.wait(underway, 'independent boat did not establish northbound transect', 24)
    x, y = xy(entry)
    north_speed = entry['vx'] / 100
    began = time.monotonic()
    crossing_time = 11
    cross_speed = 1.15
    # The contact starts to starboard. Its projected centerline intersection
    # uses only the pre-encounter ownship velocity and stays fixed thereafter.
    contact = {'at': began, 'start_x_m': x + crossing_time * cross_speed,
               'cross_y_m': y + crossing_time * north_speed, 'speed_m_s': cross_speed}
    bench.contact = contact
    bench.result['contact_model'] = contact
    avoid = bench.wait(lambda: bench.helm_event('CROSSING_ACTIVE', 'true', began),
                       'stock collision avoidance did not activate for the crossing contact', 10)
    finish = bench.wait(lambda: bench.helm_event('SITL_ROUTE_FINISHED', 'true', avoid['at']),
                        'stock boat did not resume and complete the transect', 85)
    points = bench.positions(began, finish['at'])
    separations = [(math.dist(xy(p), contact_xy(contact, p['at'])), p) for p in points]
    require(len(points) >= 30 and max(b['at'] - a['at'] for a, b in zip(points, points[1:])) <= .65,
            'independent encounter position coverage has a gap')
    minimum, closest = min(separations, key=lambda row: row[0])
    relative = (x - contact['start_x_m'], y - contact['cross_y_m'])
    velocity = (entry['vy'] / 100 + cross_speed, north_speed)
    projected_time = -sum(a * b for a, b in zip(relative, velocity)) / math.hypot(*velocity) ** 2
    projected_cpa = math.hypot(*(a + projected_time * b for a, b in zip(relative, velocity)))
    require(6 <= projected_time <= 16 and projected_cpa <= 1.5, 'contact was not on a converging approach')
    require(minimum >= 3.5 and closest['at'] > avoid['at'],
            f'independent closest approach was {minimum:.2f} m')
    detour = max(abs(xy(row)[0]) for row in points if row['at'] <= closest['at'] + 5)
    slowest = min(speed(row) for row in points if avoid['at'] + 2 <= row['at'] <= closest['at'] + 3)
    require(detour >= 2 or slowest <= .55, 'no physical steering or yielding during encounter')
    rejoined = next((p for p in points if p['at'] > closest['at'] and abs(xy(p)[0]) <= 1.8 and
                     xy(p)[1] >= contact['cross_y_m'] + 7 and speed(p) >= .55), None)
    require(rejoined, 'independent boat track did not rejoin the original transect')
    station = bench.station(BOAT_FINISH, finish['at'], 2)
    bench.command('hold', timeout=27)
    bench.wait(lambda: bench.runtime(permission='false') and bench.mode(4), 'native Hold did not take ownership', 6)
    bench.post('DEPLOY', 'false')
    bench.command('disarm')
    bench.wait(lambda: bench.mode(4, armed=False), 'independent boat disarm absent', 8)
    return {'entry': entry, 'avoidance': avoid, 'projected_no_change_cpa_m': projected_cpa,
            'minimum_separation_m': minimum, 'closest': closest, 'maximum_encounter_cross_track_m': detour,
            'minimum_encounter_speed_m_s': slowest, 'rejoined': rejoined, 'stock_finish': finish,
            'final_station': station, 'independent_encounter_samples': len(points)}


def run(args):
    copter = args.mission in COPTER_MISSIONS
    stress = args.mission in STRESS_MISSIONS
    out = args.output
    out.mkdir(mode=0o700, parents=True, exist_ok=False)
    if args.just_make:
        generate(args, copter, args.moos_port or 15312, 14540, 14541, 0)
        print(out)
        return 0
    result = {'result': 'FAIL', 'mission': args.mission, 'hardware': False, 'independent_monitor_writes': 0}
    native = launch = app = probe = bench = bridge = monitor = None
    hashes = {}
    ports = result['ports'] = {}
    try:
        binary, defaults, model, vehicle_type, profile, firmware, source, witnesses = setup_inputs(args, copter)
        hashes = {str(p): sha(p) for p in witnesses}
        result.update(firmware_revision=firmware, source=source, stock_moos_revision=MOOS, input_sha256=hashes)
        if stress:
            result['profile'] = 'arducopter-4.6.3-quad' if copter else profile
        out.joinpath('logs').mkdir()
        bridge, monitor = [socket.socket(socket.AF_INET, socket.SOCK_DGRAM) for _ in range(2)]
        for endpoint in (bridge, monitor):
            endpoint.bind(('127.0.0.1', 0))
        bridge.settimeout(.2)
        monitor.setblocking(False)
        ports.update(bridge=bridge.getsockname()[1], monitor=monitor.getsockname()[1],
                     db=args.moos_port or port(socket.SOCK_STREAM), base=port(socket.SOCK_STREAM),
                     rc=port(socket.SOCK_DGRAM))
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as test:
            test.bind(('127.0.0.1', ports['db']))
        fixture = out / 'fixture.parm'
        fixture.write_text('SERIAL1_PROTOCOL 2\nMAV2_POSITION 10\nMAV2_EXT_STAT 2\nMAV2_EXTRA3 2\n'
                           'LOG_DISARMED 1\nSIM_WIND_SPD 0\n' + ('' if copter else
                           'INITIAL_MODE 4\nMODE_CH 0\nWP_RADIUS 1\nWP_SPEED 2\nSIM_WAVE_ENABLE 0\nSIM_TIDE_SPEED 0\n'))
        intervals = '33 100\n245 500\n'
        if args.mission == 'copter-mode-stress':
            intervals += '42 250\n'
        (out / 'message-intervals-chan1.txt').write_text(intervals)
        command = [binary, '--model', model, '--home', f'{HOME[0]},{HOME[1]},10,0',
                   '--speedup', 1, '--wipe', '--defaults', ','.join(map(str, [*defaults, fixture])),
                   '--serial0', f'udpclient:127.0.0.1:{ports["bridge"]}',
                   '--serial1', f'udpclient:127.0.0.1:{ports["monitor"]}',
                   '--serial2', 'none', '--serial5', 'none', '--serial6', 'none', '--serial7', 'none',
                   '--serial8', 'none', '--base-port', ports['base'], '--rc-in-port', ports['rc']]
        result['native_command'] = list(map(str, command))
        env = env_for(args.moos_root, args.build)
        env = {k: v for k, v in env.items() if not k.startswith(
            ('MOOS', 'DYLD_', 'LD_PRELOAD', 'LD_LIBRARY_PATH', 'MAVMOOS_PLANE_TAP_'))}
        if copter:
            env['IVP_BEHAVIOR_DIRS'] = str(args.behavior_dir)
        native = start(command, out, env, 'simulator')
        peer, proof = wait_endpoints(native, bridge, ports['bridge'], ports['rc'], vehicle_type)
        result['simulator_socket_proof'] = proof
        ports['peer'] = peer[1]
        bridge.close()
        target = generate(args, copter, ports['db'], ports['bridge'], peer[1], native.pid)
        launch = start([args.moos_root / 'bin/pAntler', target], out, env, 'pantler')
        route = None
        if stress:
            route = Route(out, args.tap, ports, result)
            app = start([args.build / 'bin/iMAVLink', target, 'iMAVLink'], out,
                        env | route.environment('original'), 'bridge')
        trace = out / 'moos.jsonl'
        trace.touch()
        with (out / 'probe.log').open('w') as log:
            probe = subprocess.Popen([str(args.build / 'bin/autonomy_mission_probe'), str(ports['db']), str(trace),
                                      '--autonomy'], cwd=out, env=env, stdin=subprocess.PIPE, text=True,
                                     stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        bench = MissionBench(out, 'copter' if copter else 'boat', native, launch, probe, monitor, result, vehicle_type)
        bench.wait(lambda: bench.runtime(permission='false', armed='0', heartbeat_valid='1', home_valid='1',
                                         estimator_valid='1', position_valid='1') and
                   bench.helm_event('IVPHELM_STATE', 'DRIVE', 0) and
                   (row := bench.last('HEARTBEAT')) and not row['base_mode'] & 128 and
                   (copter or row['custom_mode'] == 4),
                   'exact owned simulator is not ready and disarmed', 65)
        result['owned_mission'] = mission_inventory(launch, target, ports['bridge'], args.moos_root / 'bin/pHelmIvP')
        if args.mission == 'copter-mode-stress':
            from mode_stress import upload_copter_plan
            upload_copter_plan(bench)
        bench.command('guided')
        bench.command('arm')
        if stress:
            from mode_stress import boat_stress, copter_stress
            require(app.poll() is None, 'owned bridge exited before stress phase')
            result['outcome'] = (copter_stress if copter else boat_stress)(bench, route)
        else:
            result['outcome'] = survey(bench) if copter else transect(bench)
        bench.post('SITL_INDEPENDENT_DONE', 'true')
        bench.wait(lambda: any(row['kind'] == 'event' and row['key'] == 'MISSION_EVALUATED' and
                              row['value'] == 'true' and row['source'] == 'pMissionEval' for row in bench.mail),
                   'stock pMissionEval did not publish a verdict', 6)
        grade = (out / 'results.txt').read_text().strip()
        result['mission_grade'] = grade
        require(grade == 'grade=pass', f'stock mission evaluator reported {grade}')
        result['result'] = 'PASS'
    except (Exception, SystemExit) as error:
        result['error'] = str(error)
    finally:
        if bench:
            bench.close()
        remaining = []
        for process in (probe, app, launch, native):
            if process:
                if process.stdin:
                    process.stdin.close()
                remaining.extend(stop_process_tree(process))
        for endpoint in (bridge, monitor):
            if endpoint:
                endpoint.close()
        result['ports_clear'] = endpoint_cleanup(ports)
        result['owned_processes_remaining'] = remaining
        result['input_drift'] = [name for name, digest in hashes.items() if sha(Path(name)) != digest]
        if remaining or not all(result['ports_clear'].values()) or result['input_drift']:
            result.update(result='FAIL', cleanup_error='owned ports, processes or protected input drift')
        if stress and result['result'] == 'PASS':
            from mode_stress import audit_stress_wire
            try:
                result['wire_audit'] = audit_stress_wire(out, result)
            except (RuntimeError, ValueError, OSError, KeyError) as error:
                result.update(result='FAIL', error=f'wire replay: {error}')
        result['files'] = evidence_files(out)
        (out / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    print(f"{result['result']} {args.mission} {out} {result.get('error', '')}", flush=True)
    return result['result'] != 'PASS'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mission', choices=COPTER_MISSIONS + BOAT_MISSIONS)
    for key in ('build', 'moos-root', 'output'):
        parser.add_argument('--' + key, required=True, type=lambda s: Path(s).resolve())
    for key in ('ardupilot-root', 'firmware-attestation', 'behavior-dir', 'tap'):
        parser.add_argument('--' + key, type=lambda s: Path(s).resolve())
    parser.add_argument('--just-make', action='store_true', help='generate inactive targets and exit')
    parser.add_argument('--moos-port', type=int, metavar='PORT')
    args = parser.parse_args()
    if args.moos_port is not None and not 1024 <= args.moos_port <= 65535:
        parser.error('--moos-port must be between 1024 and 65535')
    if not args.just_make and (not args.ardupilot_root or
            args.mission in COPTER_MISSIONS and not (args.firmware_attestation and args.behavior_dir)):
        parser.error('SITL requires a pinned ArduPilot root, and Copter also needs its attestation and altitude behavior')
    if not args.just_make and args.mission in STRESS_MISSIONS and not args.tap:
        parser.error('mode stress requires the owned loopback fault tap built for this host')
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(SystemExit(143)))
    if args.just_make:
        return run(args)
    with (Path(tempfile.gettempdir()) / 'mavmoos-native-sitl.lock').open('a+') as lease:
        fcntl.flock(lease, fcntl.LOCK_EX)
        return run(args)


if __name__ == '__main__':
    raise SystemExit(main())
