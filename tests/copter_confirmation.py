"""Owned Copter takeoff under selective ACK loss, receive loss, or bridge restart."""

import argparse
import fcntl
import json
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time

from confirmation_loss import FAULTS, Route, audit, frames, mav, need
from copter_frame_fixture import STANDARD_COPTER_FRAMES, bind_copter_frame, copter_default_paths, one_copter_frame_log
from owned_processes import native_firmware_profile, owned_records, stop_process_tree
from plane_native.flight_fixture import revision
from rover.run_rover import port, sha
from rover_expanded.run_native import build_source, endpoint_cleanup, env_for, evidence_files, start, wait_endpoints
from rover_mission.run_native import Bench
from rover_mission.confirmation import state_after
from run_sitl import select_run_dataflash_log
from pymavlink import mavutil


ROOT = Path(__file__).resolve().parents[1]
STATUS = 'MAVMOOS_RUNTIME_STATUS'
REQUEST = 'takeoff-loss'


def exercise(bench, route, restart, fault, result):
    b = bench
    b.wait(lambda: b.mode(armed=False, value=0) or b.mode(armed=False, value=4) or
           (b.last('HEARTBEAT') and not b.last('HEARTBEAT')['base_mode'] & 128), 'disarmed heartbeat unavailable', 45)
    b.wait(lambda: all(b.state(STATUS).get(k) == '1' for k in
                      ('heartbeat_valid', 'position_valid', 'home_valid', 'landed_valid', 'estimator_valid')),
           'stock estimator, home and ground evidence unavailable', 75)
    b.native_command('guided', 'guided')
    b.native_command('arm', 'arm')
    origin = b.wait(lambda: b.mode(4) and b.last('GLOBAL_POSITION_INT'), 'independent armed Guided absent')
    if fault == 'pre_restart':
        route.exit_before_takeoff()
        b.post('MAVMOOS_COMMAND', f'id={REQUEST},action=takeoff,height=5')
        restart(before_send=True)
        now = time.monotonic()
        b.wait(lambda: state_after(b, STATUS, now, fc_mode='4', heartbeat_valid='1', armed='1', in_air='0',
                                   landed_valid='1'), 'fresh armed ground state unavailable after restart', 8)
        count = len(b.receipts.get(REQUEST, []))
        b.post('MAVMOOS_COMMAND', f'id={REQUEST},action=takeoff,height=5')
        retry = b.wait(lambda: next(iter(b.receipts.get(REQUEST, [])[count:]), None), 'missing replay result')
        need((retry['status'], retry['reason']) == ('DUPLICATE', 'native_id_already_recorded'),
             'replay was not blocked while a new takeoff would otherwise be eligible')
        result['retry_same'] = retry
        route.mark('cleanup')
        b.native_command('disarm', 'disarm')
        result['final'] = b.wait(lambda: b.mode(4, armed=False), 'independent final disarm absent')
        return
    # The Copter takeoff state precedes liftoff and can outlast the link
    # watchdog. Keep the link alive until that first independent effect.
    route.drop(fault, mav.MAV_CMD_NAV_TAKEOFF, ack_only=fault == 'restart')
    sent = b.post('MAVMOOS_COMMAND', f'id={REQUEST},action=takeoff,height=5')
    if fault == 'restart':
        result['native_takeoff_state'] = b.wait(lambda: (row if (row := b.last('EXTENDED_SYS_STATE', sent)) and
                                                        row['landed_state'] == 3 else None),
                                                 'controller did not begin takeoff before restart', 4)
        statuses = [row['status'] for row in b.receipts.get(REQUEST, [])]
        need(statuses == ['ADMITTED', 'SUBMITTED', 'LOCAL_ACCEPTED'],
             f'takeoff already had a terminal bridge result: {statuses}')
        result['original_results'] = statuses
        restart()
    else:
        terminals = ('COMPLETED',) if fault == 'ack' else ('CANCELLED',)
        result['terminal'] = b.wait(lambda: next((row for row in b.receipts.get(REQUEST, []) if
                                                 row['status'] in terminals), None), 'takeoff terminal result missing', 12)
        route.restore()
    result['physical_liftoff'] = b.wait(lambda: next((row for row in b.wire if row['at'] > sent and
                                                     row['mavpackettype'] == 'GLOBAL_POSITION_INT' and
                                                     row['relative_alt'] - origin['relative_alt'] >= 150 and row['vz'] < -20), None),
                                       'independent observer saw no takeoff climb', 8)
    observed_at = time.monotonic()
    b.wait(lambda: state_after(b, STATUS, observed_at, fc_mode='4', heartbeat_valid='1', armed='1', in_air='1',
                               landed_valid='1', permission='false'), 'bridge lacks fresh airborne Guided after loss', 10)
    result['physical_height'] = b.wait(lambda: (row if (row := b.last('GLOBAL_POSITION_INT', sent)) and
                                             row['relative_alt'] >= 4500 else None), 'takeoff never reached 4.5 m', 12)

    def retry(id_, status, reason):
        count = len(b.receipts.get(id_, []))
        b.post('MAVMOOS_COMMAND', f'id={id_},action=takeoff,height=5')
        row = b.wait(lambda: next(iter(b.receipts.get(id_, [])[count:]), None), f'missing retry {id_}')
        need((row['status'], row['reason']) == (status, reason), f'unexpected takeoff retry outcome: {row}')
        return row

    result['retry_same'] = retry(REQUEST, 'DUPLICATE',
                                 'native_id_already_recorded' if fault == 'restart' else 'id_already_seen')
    result['retry_fresh'] = retry('retry-fresh', 'REJECTED', 'ground_state_required')
    route.mark('handback')
    route.data['handback_stop_mode'] = 5
    at = time.monotonic()
    result['native_handback'] = b.native_command('native-handback', 'hold')
    b.wait(lambda: b.mode(5, at) and state_after(b, STATUS, at, fc_mode='5', permission='false'),
           'independent native Loiter handback absent')
    route.mark('retry_after')
    result['post_handback'] = retry('after-handback', 'REJECTED', 'guided_required')
    until = time.monotonic() + 1.2
    b.wait(lambda: time.monotonic() >= until and b.state(STATUS).get('permission') == 'false', 'quiet retry window failed')
    route.mark('cleanup')
    b.native_command('land', 'land', timeout=45)
    b.wait(lambda: b.last('EXTENDED_SYS_STATE') and b.last('EXTENDED_SYS_STATE')['landed_state'] == 1,
           'independent touchdown absent', 15)
    if b.state(STATUS).get('armed') == '1':
        b.native_command('disarm', 'disarm')
    result['final'] = b.wait(lambda: b.mode(9, armed=False), 'independent final disarm absent', 12)


def main(args):
    build, moos, ap, behavior, tap, out = (Path(getattr(args, name)).resolve() for name in
                                          ('build', 'moos_root', 'ardupilot_root', 'behavior_dir', 'tap', 'output'))
    pin = json.loads((ROOT/'config/setup.json').read_text())['moos']['revision']
    need(revision(moos) == pin, 'stock MOOS-IvP pin differs')
    source = build_source(build)
    need(Path(source['path']) == ROOT and not source['tracked_changes'] and revision(ROOT) == source['revision'],
         'clean matching source build required')
    manifest = ROOT/'config/firmware/arducopter-4.6.3-quad.json'
    attestation = Path(args.firmware_attestation).resolve(strict=True)
    firmware = native_firmware_profile(ROOT, ap, manifest, attestation)
    app, probe_file = build/'bin/iMAVLink', build/'bin/rover_mission_probe'
    helm, binary = moos/'bin/pHelmIvP', Path(firmware['native_path'])
    defaults = copter_default_paths(ap, args.frame)
    expected_type = STANDARD_COPTER_FRAMES[args.frame][0]
    inputs = [app, probe_file, helm, binary, *defaults, tap, manifest, attestation, ROOT/'config/setup.json',
              *behavior.glob('libBHV_ConstantAltitude.*'),
              *(build/'plugins'/f'mavmoos_{name}.so' for name in ('telemetry', 'arducopter_control', 'parameters')),
              *(ROOT/'tests'/name for name in ('copter_confirmation.py','copter_frame_fixture.py','confirmation_loss.py','rover_mission/confirmation.py',
                  'rover_mission/run_native.py','rover_mission/RoverMissionProbe.cpp','plane_faults/fault_tap.py',
                  'plane_faults/OwnedPlaneFaultTap.cpp','owned_processes.py')),
              *(ROOT/f'missions/copter_sitl/meta_vehicle.{suffix}' for suffix in ('moos','bhv'))]
    hashes = {str(path): sha(path) for path in inputs}
    out.mkdir(parents=True, exist_ok=False)
    (out/'logs').mkdir()
    bridge, monitor = (socket.socket(socket.AF_INET, socket.SOCK_DGRAM) for _ in range(2))
    for endpoint in (bridge, monitor):
        endpoint.bind(('127.0.0.1', 0))
    bridge.settimeout(.2)
    monitor.setblocking(False)
    bind, observer = bridge.getsockname()[1], monitor.getsockname()[1]
    db, base, rc = port(socket.SOCK_STREAM), port(socket.SOCK_STREAM), port(socket.SOCK_DGRAM)
    fixture = out/'fixture.parm'
    fixture.write_text('SERIAL0_PROTOCOL 2\nSERIAL1_PROTOCOL 2\nLOG_DISARMED 1\nSIM_WIND_SPD 0\n')
    (out/'message-intervals-chan1.txt').write_text('0 250\n33 100\n245 100\n')
    argv = [binary, '--model', args.frame, '--home', '42.358456,-71.087589,10,0', '--speedup', '1', '--wipe',
            '--defaults', ','.join(map(str, [*defaults, fixture])), '--serial0', f'udpclient:127.0.0.1:{bind}',
            '--serial1', f'udpclient:127.0.0.1:{observer}', '--serial2', 'none', '--serial5', 'none',
            '--serial6', 'none', '--serial7', 'none', '--serial8', 'none', '--base-port', base, '--rc-in-port', rc]
    result = {'result':'FAIL', 'hardware':False, 'scenario':args.fault, 'profile':f'arducopter-4.6.3-{args.frame}', 'frame':args.frame,
              'source':source, 'firmware':firmware, 'stock_moos':pin, 'digests_before':hashes,
              'ports':{'bridge':bind,'monitor':observer,'db':db,'base':base,'rc':rc}, 'bridge_pids':[]}
    native = antler = probe = bridge_app = bench = None
    try:
        env = {key:value for key,value in env_for(moos,build).items() if not key.startswith(('DYLD_', 'LD_PRELOAD', 'MAVMOOS_PLANE_TAP_'))}
        native = start(argv, out, env, 'sitl')
        peer, result['native_socket_proof'] = wait_endpoints(native, bridge, bind, rc, expected_type)
        result['ports']['peer'] = peer[1]
        bridge.close()
        macros = dict(MOOS_PORT=db, APP_PATH=app, BIND_PORT=bind, PEER_PORT=peer[1], LAT_ORIGIN='42.358456',
                      LONG_ORIGIN='-71.087589', SIMULATION_ONLY='true', SHADOW_MODE='false', SITL_PID=native.pid,
                      PLUGIN_DIRECTORY=build/'plugins', BEHAVIOR_DIRECTORY=behavior, HEIGHT=5, SPEED=1, WAYPOINTS='0,8:8,8')
        for suffix in ('moos','bhv'):
            subprocess.run([str(moos/'bin/nsplug'), str(ROOT/f'missions/copter_sitl/meta_vehicle.{suffix}'),
                            str(out/f'targ_copter.{suffix}'), '--strict','--force','-x',
                            *(f'{k}={v}' for k,v in macros.items())], check=True, capture_output=True, timeout=10)
        mission = out/'targ_copter.moos'
        text = '\n'.join(line for line in mission.read_text().splitlines() if '~ iMAVLink' not in line)
        text = text.replace('Run = pHelmIvP @', f'Run = {helm} @')
        mission.write_text(bind_copter_frame(text+'\n', args.frame).replace('  AppTick = 30', '  firmware_requests = false\n  AppTick = 30', 1))
        route = Route(out, tap, result['ports'], result)
        antler = start([moos/'bin/pAntler', mission], out, env, 'pantler')

        def boot(instance):
            child = start([app, mission, 'iMAVLink'], out, env | route.environment(instance), f'bridge-{instance}')
            result['bridge_pids'].append(child.pid)
            return child

        bridge_app = boot('original')
        trace = out/'moos.jsonl'
        trace.touch()
        with (out/'probe.log').open('w') as log:
            probe = subprocess.Popen([probe_file, str(db), trace], cwd=out, env=env, stdin=subprocess.PIPE,
                                     stdout=log, stderr=subprocess.STDOUT, text=True, start_new_session=True)
        bench = Bench(out, args.frame, native, antler, probe, monitor, None, result, expected_type=expected_type)

        def restart(before_send=False):
            nonlocal bridge_app
            if not before_send:
                bridge_app.kill()
            expected_exit = 88 if before_send else -signal.SIGKILL
            need(bridge_app.wait(timeout=4) == expected_exit and not stop_process_tree(bridge_app), 'original app did not exit')
            route.mark('stopped')
            bridge_app = boot('restarted')

        exercise(bench, route, restart, args.fault, result)
        need(any(str(helm) in str(row) for row in owned_records(antler)), 'stock helm missing from owned mission')
        result['result'] = 'PASS'
    except BaseException as error:
        result['error'] = str(error) or type(error).__name__
        if bench:
            result['last_bridge_status'] = bench.state(STATUS)
    finally:
        if bench:
            bench.close()
        result['owned_processes_remaining'] = [item for process in (probe, bridge_app, antler, native)
                                               for item in stop_process_tree(process)]
        bridge.close()
        monitor.close()
        result['ports_clear'] = endpoint_cleanup(result['ports'])
        try:
            need(all(result['ports_clear'].values()) and not result['owned_processes_remaining'], 'owned cleanup failed')
            result['digests_after'] = {str(path): sha(path) for path in inputs}
            need(result['digests_after'] == hashes and revision(moos) == pin and build_source(build) == source and
                 native_firmware_profile(ROOT, ap, manifest, attestation) == firmware, 'pinned input drift')
            if result['result'] == 'PASS':
                log = select_run_dataflash_log(out/'logs', set())
                reader = mavutil.mavlink_connection(str(log))
                modes = []
                while (row := reader.recv_match(type='MODE')) is not None:
                    modes.append(row.ModeNum)
                reader.close()
                if args.fault != 'pre_restart':
                    need(4 in modes and 5 in modes and 9 in modes and modes.index(4) < modes.index(5) < modes.index(9),
                         'onboard Guided, Loiter and Land order missing')
                result['dataflash_modes'] = modes
                result['frame_parameters'] = one_copter_frame_log(out/'logs', args.frame)
                if args.fault == 'pre_restart':
                    takeoffs = [(instance, direction) for instance, _, direction, message in frames(out,result)
                                if direction in ('T','h') and message.get_type() == 'COMMAND_LONG' and
                                message.command == mav.MAV_CMD_NAV_TAKEOFF]
                    need(takeoffs == [('original','h')], 'takeoff reached the socket before or after restart')
                    result['wire_audit'] = {'intercepted_before_socket': 1, 'takeoff_sends': 0}
                else:
                    result['wire_audit'] = audit(out,result)
        except Exception as error:
            result.update(result='FAIL', verification_error=str(error))
        result['files'] = evidence_files(out)
        (out/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(result['result'], out/'result.json', result.get('error',result.get('verification_error','')), flush=True)
    return 0 if result['result'] == 'PASS' else 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for key in ('build','moos-root','ardupilot-root','behavior-dir','tap','output','firmware-attestation'):
        parser.add_argument('--'+key, required=True)
    parser.add_argument('--frame', choices=STANDARD_COPTER_FRAMES, default='quad')
    parser.add_argument('--fault', choices=FAULTS+('pre_restart',), required=True)
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(SystemExit(143)))
    with (Path(tempfile.gettempdir())/'mavmoos-native-sitl.lock').open('a+') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        raise SystemExit(main(parser.parse_args()))
