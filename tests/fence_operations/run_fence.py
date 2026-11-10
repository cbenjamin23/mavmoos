#!/usr/bin/env python3
"""Exercise typed fence storage through MOOS on pinned, owned ArduCopter SITL."""
import argparse
import fcntl
import json
import math
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tests' / 'missions'))
from run_inspection import (HOME, LOOPBACK, SEED, SEED_COMPONENT, SEED_SYSTEM, TYPES,
                            SeedPeer, discover, events, fields, mission_text, owned_endpoint,
                            port, ref, require, reserve, sha)
from owned_processes import native_firmware_profile, stop_process_tree


RESULT = 'MAVMOOS_MISSION_INSPECTION_RESULT'
SNAPSHOT = 'MAVMOOS_MISSION_INSPECTION_FENCE_SNAPSHOT'
TX = 'MAVMOOS_MISSION_TX_RESULT'
ORIGINAL = SEED['fence'][:3]
REPLACEMENT = [(5001, 0, HOME[0] + x, HOME[1] + y, 0, 4)
               for x, y in ((-8000, -4000), (8000, -4000), (8000, 4000), (-8000, 4000))]
INTERRUPTED = [(5001, 0, HOME[0] + round(8000 * math.cos(math.tau * i / 16)),
                HOME[1] + round(8000 * math.sin(math.tau * i / 16)), 0, 16) for i in range(16)]


def fence_rows(source):
    return [dict(seq=seq, command=command, frame=frame, current=0, autocontinue=0,
                 params=[p1, 0, 0, 0], x=x, y=y, z=z)
            for seq, (command, frame, x, y, z, p1) in enumerate(source)]


class Observer(SeedPeer):
    def fresh(self):
        deadline = time.monotonic() + 4
        while time.monotonic() - min(self.heartbeat_at, self.landed_at) >= .8:
            require(time.monotonic() < deadline, 'independent fresh disarmed ground evidence missing')
            self.receive()

    def wait_for(self, kind, matches):
        deadline = time.monotonic() + 4
        while time.monotonic() < deadline:
            for message in self.receive():
                if message.get_type() == kind and matches(message):
                    require(message.get_msgbuf()[0] == 0xFD, 'independent response was not MAVLink2')
                    return message
        raise RuntimeError(f'independent observer deadline for {kind}')

    def read(self, name):
        plan = TYPES[name]

        def addressed(message):
            return (message.target_system, message.target_component, message.mission_type) == (
                SEED_SYSTEM, SEED_COMPONENT, plan)

        self.fresh()
        self.outbound.mission_request_list_send(1, 1, plan)
        count = self.wait_for('MISSION_COUNT', addressed).count
        require(0 <= count <= 16, f'unexpected fixture plan size {name}={count}')
        rows = []
        for seq in range(count):
            self.outbound.mission_request_int_send(1, 1, seq, plan)
            item = self.wait_for('MISSION_ITEM_INT', lambda msg: addressed(msg) and msg.seq == seq)
            rows.append(dict(seq=seq, command=item.command, frame=item.frame, current=item.current,
                             autocontinue=item.autocontinue,
                             params=[item.param1, item.param2, item.param3, item.param4],
                             x=item.x, y=item.y, z=item.z))
        self.outbound.mission_ack_send(1, 1, 0, plan)
        return rows

    def fence_settings(self):
        result = {}
        for name in ('FENCE_ENABLE', 'FENCE_AUTOENABLE'):
            self.fresh()
            self.outbound.param_request_read_send(1, 1, name.encode('ascii'), -1)
            value = self.wait_for('PARAM_VALUE', lambda msg: msg.param_id == name)
            result[name] = value.param_value
        require(all(value == 0 for value in result.values()), 'fence enable policy changed')
        return result


def drive(observer, probe, out):
    probe_log = out / 'probe.jsonl'
    observer.fresh()
    observer.load('flight')
    observer.load('rally')
    before = {name: observer.read(name) for name in ('flight', 'rally', 'fence')}
    require(not before['fence'], 'fixture fence not initially empty')
    settings_before = observer.fence_settings()
    observed = {}
    tx = {}
    step = 0

    def operate(action, profile, rows, packets, restore_required, backup=None, interrupted=False):
        nonlocal step
        operation = f'{action}-{profile}'
        backup = backup or f'save-{profile}'
        request = f'schema=1,id={operation},action={action},type=fence,timeout=8'
        if action != 'save':
            request += f',backup={backup}'
        if action == 'replace':
            request += ',items=' + '|'.join(f'{row[2]}:{row[3]}' for row in rows)
        draft = out / 'operator.next'
        draft.write_text(request + '\n')
        draft.rename(out / f'step-{step}.request')
        step += 1
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            observer.receive()
            trace = events(probe_log)
            if any(row['key'] == 'complete' and row['value'] == operation for row in trace):
                break
            require(probe.poll() is None, f'operator exited before {operation} completed')
        else:
            raise RuntimeError(f'operator deadline for {operation}')
        result = [fields(row['value']) for row in trace if row['key'] == RESULT]
        expected = 'CANCELLED' if interrupted else 'COMPLETED'
        terminal = [row for row in result if row['id'] == operation and row['status'] == expected]
        verified = action != 'save' and not interrupted
        require(len(terminal) == 1 and terminal[0]['backup_id'] == backup and
                terminal[0]['restore_required'] == str(restore_required).lower() and
                terminal[0]['remote_accepted'] == str(verified).lower() and
                terminal[0]['readback_confirmed'] == str(verified).lower(),
                f'terminal proof differs for {operation}')
        sent = [fields(row['value']) for row in trace if row['key'] == TX]
        sent = [row for row in sent if row['operation_id'] == operation]
        wire = [int(row['msgid']) for row in sent]
        require(all(row['mission_type'] == '1' and row['status'] == 'LOCAL_ACCEPTED' and
                    row['late'] == 'false' and row['clock_valid'] == 'true' for row in sent),
                f'wire handoff differs for {operation}')
        if interrupted:
            require(terminal[0]['reason'] == 'operator_cancelled' and
                    terminal[0]['mutation_may_have_applied'] == 'true' and
                    wire[:len(packets)] == packets and
                    1 <= len(wire) - len(packets) < len(rows) and
                    all(msgid == 73 for msgid in wire[len(packets):]),
                    'cancellation did not interrupt a partial upload')
        else:
            require(wire == packets, f'wire phase differs for {operation}')
            snapshots = [json.loads(row['value']) for row in trace if row['key'] == SNAPSHOT]
            actual = observer.read('fence')
            require(any(row.get('id') == operation and row['items'] == actual for row in snapshots) and
                    actual == fence_rows(rows), f'raw independent readback differs for {operation}')
            observed[operation] = actual
        tx[operation] = wire
        return terminal[0]

    def read(count):
        return [43] + [51] * count + [47]

    def write(count):
        return [44] + [73] * count

    for profile, original in [('empty', []), ('seeded', ORIGINAL)]:
        if original:
            observer.fresh()
            observer.load('fence', original)
        operate('save', profile, original, read(len(original)), False)
        operate('replace', profile, REPLACEMENT,
                read(len(original)) + write(len(REPLACEMENT)) + read(len(REPLACEMENT)), True)
        operate('restore', profile, original,
                (write(len(original)) if original else [45]) + read(len(original)), False)
    interrupted = operate('replace', 'interrupted', INTERRUPTED, read(len(ORIGINAL)) + [44], True,
                          backup='save-seeded', interrupted=True)
    # Pinned FC retains its old receiving protocol for eight seconds after the
    # last item. Keep receiving observation data while the private fixture settles.
    settled = time.monotonic() + 9
    while time.monotonic() < settled:
        observer.receive()
    observed['after-interruption'] = observer.read('fence')
    require(observed['after-interruption'] == fence_rows(ORIGINAL),
            'pinned FC changed stored fence after a partial upload')
    operate('restore', 'interrupted', ORIGINAL, write(len(ORIGINAL)) + read(len(ORIGINAL)), False,
            backup='save-seeded')
    require(probe.wait(timeout=2) == 0, 'operator probe failed')
    old = [fields(row['value']) for row in events(probe_log) if row['key'] == TX]
    old = [row for row in old if row['operation_id'] == 'replace-interrupted']
    require(len(old) == len(tx['replace-interrupted']) and
            all(float(row['at']) < float(interrupted['at']) for row in old),
            'an interrupted operation accepted a new handoff after cancellation')
    after = {name: observer.read(name) for name in ('flight', 'rally')}
    # The FC synthesizes flight sequence zero from live HOME. Keep both raw
    # observations and compare the stored executable rows and entire rally plan.
    require(len(before['flight']) == len(after['flight']) == len(SEED['flight']) and
            before['flight'][1:] == after['flight'][1:] and before['rally'] == after['rally'],
            'fence edit touched flight or rally storage')
    settings_after = observer.fence_settings()
    observer.fresh()
    require(observer.hearts >= 2 and observer.landed >= 2, 'insufficient independent ground samples')
    return {'independent_fence_rows': observed, 'bridge_mission_tx_msgids': tx,
            'independent_flight_rally_before': {name: before[name] for name in after},
            'independent_flight_rally_after': after, 'flight_home_compared': False,
            'settings_before': settings_before,
            'settings_after': settings_after, 'heartbeat_samples': observer.hearts,
            'on_ground_samples': observer.landed, 'ever_armed': False, 'interrupted': interrupted}


def run(args):
    out, build, moos, ap = (item.resolve() for item in
                           (args.output, args.build, args.moos_root, args.ardupilot_root))
    out.mkdir(parents=True, exist_ok=False)
    processes, streams = [], []
    ports, protected = {}, {}
    report = {'result': 'FAIL', 'scope': 'owned ArduCopter quad SITL fence storage', 'hardware': False}

    def launch(argv, name, env=None):
        with (out / name).open('w') as stream:
            process = subprocess.Popen(argv, cwd=out, env=env, stdout=stream, stderr=subprocess.STDOUT,
                                       start_new_session=True)
        processes.append(process)
        return process

    def udp(name):
        stream = reserve(socket.SOCK_DGRAM)
        stream.settimeout(.08)
        streams.append(stream)
        ports[name] = stream.getsockname()[1]
        return stream

    try:
        from pymavlink.dialects.v20 import ardupilotmega as dialect
        identity = native_firmware_profile(ROOT, ap, args.manifest, args.attestation)
        require(identity and identity['profile_id'] == 'arducopter-4.6.3-quad',
                'exact pinned ArduCopter 4.6.3 quad manifest and native attestation required')
        executable = ap / 'build/sitl/bin/arducopter'
        inputs = [executable, build / 'bin/iMAVLink', build / 'plugins/mavmoos_missions.so', args.probe,
                  moos / 'bin/pAntler', moos / 'bin/MOOSDB', moos / 'bin/pHelmIvP',
                  Path(__file__), Path(__file__).with_name('FenceMoosProbe.cpp'),
                  ROOT / 'tests/missions/run_inspection.py',
                  ROOT / 'tests/owned_processes.py', ROOT / 'tests/firmware_identity.py',
                  ROOT / 'src/libmavmoos_runtime/FencePlan.h',
                  ROOT / 'src/libmavmoos_runtime/RuntimeMission.cpp', ROOT / 'src/plugins/MissionPlugin.cpp']
        for role in ('manifest', 'attestation'):
            source = Path(identity[role + '_path'])
            inputs.append(source)
            shutil.copyfile(source, out / ('firmware-' + role + '.json'))
        protected = {str(path): sha(path) for path in inputs}
        report.update(firmware_identity=identity, ardupilot_ref=ref(ap), moos_ref=ref(moos),
                      source_ref=ref(ROOT))
        primary, secondary = udp('bridge'), udp('observer')
        ports['db'], ports['base'], ports['rc'] = (port(socket.SOCK_STREAM), port(socket.SOCK_STREAM),
                                                  port(socket.SOCK_DGRAM))
        defaults = out / 'fence.parm'
        defaults.write_text('SERIAL1_PROTOCOL 2\nMAV2_POSITION 2\nMAV2_EXT_STAT 2\nMAV2_EXTRA3 2\n'
                            'FENCE_ENABLE 0\nFENCE_AUTOENABLE 0\n')
        (out / 'message-intervals-chan1.txt').write_text('33 500\n245 500\n')
        command = [str(executable), '--model', 'quad', '--home', '42.358456,-71.087589,10,0',
                   '--speedup', '1', '--wipe', '--defaults',
                   str(ap / 'Tools/autotest/default_params/copter.parm') + ',' + str(defaults),
                   '--serial0', f'udpclient:{LOOPBACK}:{ports["bridge"]}',
                   '--serial1', f'udpclient:{LOOPBACK}:{ports["observer"]}',
                   '--serial2', 'none', '--serial5', 'none', '--serial6', 'none',
                   '--serial7', 'none', '--serial8', 'none',
                   '--base-port', str(ports['base']), '--rc-in-port', str(ports['rc'])]
        sitl = launch(command, 'sitl.log')
        peer, observer_peer = discover(primary, sitl, dialect), discover(secondary, sitl, dialect)
        ports.update(peer=peer[1], observer_peer=observer_peer[1])
        report.update(command=command, sitl_pid=sitl.pid,
                      socket_inventory=owned_endpoint(sitl, ports['peer'], ports['bridge']))
        owned_endpoint(sitl, ports['observer_peer'], ports['observer'])
        primary.close()
        observer = Observer(dialect, secondary, observer_peer, sitl, out)
        mission = out / 'fence.moos'
        config = mission_text(ports['db'], ports['bridge'], ports['peer'], build / 'plugins', sitl.pid)
        mission.write_text(config.replace('  plugins = missions\n',
                                          '  plugins = missions\n  plugin.missions.fence_edit = true\n'))
        env = dict(os.environ)
        env['PATH'] = os.pathsep.join((str(build / 'bin'), str(moos / 'bin'), env.get('PATH', '')))
        antler = launch([str(moos / 'bin/pAntler'), str(mission)], 'pantler.log', env)
        probe = launch([str(args.probe), str(ports['db']), str(out), '8'], 'probe.jsonl', env)
        report.update(drive(observer, probe, out))
        require(antler.poll() is None, 'MOOS mission launcher exited')
        report['result'] = 'PASS'
    except Exception as error:
        report['error'] = str(error)
    finally:
        report['owned_processes_remaining'] = [row for process in reversed(processes)
                                                for row in stop_process_tree(process)]
        for stream in streams:
            stream.close()
        clear = {}
        for name, number in ports.items():
            kind = socket.SOCK_STREAM if name in ('db', 'base') else socket.SOCK_DGRAM
            with socket.socket(socket.AF_INET, kind) as stream:
                if kind == socket.SOCK_STREAM:
                    stream.settimeout(.2)
                    clear[name] = stream.connect_ex((LOOPBACK, number)) != 0
                else:
                    try:
                        stream.bind((LOOPBACK, number))
                        clear[name] = True
                    except OSError:
                        clear[name] = False
        report.update(ports=ports, ports_clear=clear, artifacts_before=protected,
                      artifact_drift=[path for path, digest in protected.items()
                                      if not Path(path).is_file() or sha(Path(path)) != digest])
        if report['owned_processes_remaining'] or not all(clear.values()) or report['artifact_drift']:
            report['result'] = 'FAIL'
            report['cleanup_error'] = 'owned resources or protected inputs drifted'
        report['files'] = {str(path.relative_to(out)): sha(path) for path in sorted(out.rglob('*'))
                           if path.is_file() and not path.is_symlink()}
        (out / 'result.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({key: report[key] for key in ('result', 'error', 'cleanup_error', 'ports_clear')
                      if key in report}, indent=2))
    if report['result'] != 'PASS':
        for name in ('probe.jsonl', 'pantler.log', 'sitl.log'):
            if (out / name).is_file():
                print(name, (out / name).read_text(errors='replace')[-2800:])
    return int(report['result'] != 'PASS')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    for option in ('output', 'build', 'moos-root', 'ardupilot-root', 'probe', 'manifest', 'attestation'):
        parser.add_argument('--' + option, type=Path, required=True)
    args = parser.parse_args()
    args.probe = args.probe.resolve()
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(SystemExit(143)))
    with (Path(tempfile.gettempdir()) / 'mavmoos-native-sitl.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        raise SystemExit(run(args))
