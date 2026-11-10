#!/usr/bin/env python3
"""Flight-log operator workflow against generated RX-only or exact owned Copter SITL."""
import argparse
import fcntl
import hashlib
import json
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
sys.path.insert(0, str(ROOT / 'tests'))
from control.catalog_evidence import loaded_module_hashes
from owned_processes import native_firmware_profile, stop_process_tree

RESULT = 'MAVMOOS_FLIGHT_LOG_RESULT'
CATALOG = 'MAVMOOS_FLIGHT_LOG_CATALOG'
TX = 'MAVMOOS_FLIGHT_LOG_TX_RESULT'
STATUS = 'MAVMOOS_RUNTIME_STATUS'
CAPS = 'MAVMOOS_CAPABILITIES'
LOG_LIMIT = 4 * 1024 * 1024


def require(condition, explanation):
    if not condition:
        raise ValueError(explanation)


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(65536), b''):
            digest.update(block)
    return digest.hexdigest()


def fnv(data):
    value = 14695981039346656037
    for byte in data:
        value = ((value ^ byte) * 1099511628211) & 0xffffffffffffffff
    return f'{value:016x}'


def fields(text):
    return dict(field.split('=', 1) for field in text.split(','))


def events(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line.startswith('{')]


def revision(path):
    return subprocess.check_output(['git', '-C', str(path), 'rev-parse', '--verify', 'HEAD'],
                                   text=True, timeout=3).strip()


def port(kind):
    with socket.socket(socket.AF_INET, kind) as sock:
        sock.bind(('127.0.0.1', 0))
        return sock.getsockname()[1]


def cleared(number, kind):
    with socket.socket(socket.AF_INET, kind) as sock:
        if kind == socket.SOCK_STREAM:
            sock.settimeout(.2)
            return sock.connect_ex(('127.0.0.1', number)) != 0
        try:
            sock.bind(('127.0.0.1', number))
            return True
        except OSError:
            return False


def start(command, cwd, env, log_path):
    with log_path.open('w') as output:
        return subprocess.Popen([str(part) for part in command], cwd=cwd, env=env,
                                stdout=output, stderr=subprocess.STDOUT, start_new_session=True)


def mission_text(ports, private, plugin_dir, pid):
    native = pid is not None
    sitl = f'  sitl_pid = {pid}\n' if native else ''
    return f"""ServerHost = 127.0.0.1
ServerPort = {ports['moos']}
Community = flight_logs_{ports['moos']}
MOOSTimeWarp = 1
TERM_REPORTING = false
ProcessConfig = ANTLER
{{
  MSBetweenLaunches = 50
  Run = MOOSDB @ NewConsole = false
  Run = iMAVLink @ NewConsole = false
}}
ProcessConfig = iMAVLink
{{
  AppTick = 50
  CommsTick = 50
  bind_address = 127.0.0.1
  bind_port = {ports['bridge']}
  peer_address = 127.0.0.1
  peer_port = {ports['peer']}
  target_system = 1
  target_component = 1
  expected_type = 2
  expected_autopilot = 3
  simulation_only = {'true' if native else 'false'}
  shadow_mode = false
{sitl}  plugins = flight_logs
  plugin_directory = {plugin_dir}
  max_datagrams_per_iter = 128
  plugin.flight_logs.directory = {private}
  plugin.flight_logs.timeout = 60
}}
"""


def start_bridge(args, out, private, ports, processes, native=None):
    app, moos = args.build / 'bin', args.moos_root / 'bin'
    env = dict(os.environ)
    env['PATH'] = os.pathsep.join((str(app), str(moos), env.get('PATH', '')))
    mission = out / 'flight-logs.moos'
    mission.write_text(mission_text(ports, private, args.build / 'plugins', native.pid if native else None))
    processes.append(start([moos / 'pAntler', mission], out, env, out / 'pantler.log'))
    probe = [args.probe, ports['moos'], args.scenario]
    if native is None:
        probe.extend((ports['peer'], ports['bridge']))
    processes.append(start(probe, out, env, out / 'probe.jsonl'))
    return processes[-1]


def run_native(args, out, private, report, ports, processes, kinds):
    from pymavlink import DFReader
    from pymavlink.dialects.v20 import ardupilotmega as mavlink

    def selected():
        firmware = native_firmware_profile(ROOT, args.ardupilot_root,
                                           args.firmware_manifest, args.firmware_attestation)
        require(firmware is not None and firmware['frame'] == 'quad',
                'native flight logs require a verified quad firmware manifest and attestation')
        return firmware

    firmware = selected()
    report.update(firmware_identity=firmware, ardupilot_ref=firmware['source_revision'],
                  native_sha256=firmware['native_sha256'], observer_writes=0,
                  arming_requested=False, rc_input_sent=False, relay_used=False)
    report['firmware_inputs'] = {}
    for key in ('manifest', 'attestation'):
        copy = out / f'firmware-{key}.json'
        shutil.copyfile(firmware[f'{key}_path'], copy)
        report['firmware_inputs'][key] = {'path': copy.name, 'sha256': sha(copy)}

    sim = out / 'sim'
    sim.mkdir(mode=0o700)
    native = Path(firmware['native_path'])
    stock_defaults = args.ardupilot_root / 'Tools/autotest/default_params/copter.parm'

    def launch(name, bridge, base, rc, monitor=None):
        config = out / f'{name}.parm'
        config.write_text(f"LOG_BACKEND_TYPE 1\nLOG_DISARMED {int(name == 'seed')}\nSERIAL0_PROTOCOL 2\n"
                          + ('SERIAL1_PROTOCOL 2\n' if monitor else ''))
        command = [native, '--model', firmware['frame'], '--home', '42.358456,-71.087589,10,0',
                   '--speedup', '1', '--wipe', '--defaults', f'{stock_defaults},{config}',
                   '--serial0', f'udpclient:127.0.0.1:{bridge}',
                   '--serial1', f'udpclient:127.0.0.1:{monitor}' if monitor else 'none',
                   '--serial2', 'none', '--serial5', 'none', '--serial6', 'none', '--serial7', 'none',
                   '--serial8', 'none', '--base-port', base, '--rc-in-port', rc]
        process = start(command, sim, os.environ, out / f'{name}.log')
        processes.append(process)
        report[f'{name}_command'] = [str(arg) for arg in command]
        report[f'{name}_pid'] = process.pid
        return process

    def discover(receiver, process):
        decoder = mavlink.MAVLink(None)
        decoder.robust_parsing = True
        until = time.monotonic() + 20
        while time.monotonic() < until:
            require(process.poll() is None, 'owned SITL exited before heartbeat')
            try:
                data, address = receiver.recvfrom(65535)
            except socket.timeout:
                continue
            for message in decoder.parse_buffer(data) or ():
                if message.get_type() != 'HEARTBEAT' or message.get_srcSystem() != 1 or message.get_srcComponent() != 1:
                    continue
                require(address[0] == '127.0.0.1' and message.type == 2 and message.autopilot == 3 and
                        not message.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED,
                        'expected exact disarmed Copter heartbeat on the owned loopback endpoint')
                return address
        raise ValueError('owned disarmed heartbeat missing before discovery deadline')

    source = sim / 'logs/00000001.BIN'
    ports['seed_base'] = port(socket.SOCK_STREAM)
    kinds['seed_base'] = socket.SOCK_STREAM
    ports['seed_rc'] = port(socket.SOCK_DGRAM)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
        receiver.bind(('127.0.0.1', 0))
        receiver.settimeout(.2)
        ports['seed_bridge'] = receiver.getsockname()[1]
        seed = launch('seed', ports['seed_bridge'], ports['seed_base'], ports['seed_rc'])
        ports['seed_peer'] = discover(receiver, seed)[1]
        until = time.monotonic() + 18
        inspected = 0
        while time.monotonic() < until:
            require(seed.poll() is None, 'seed simulator exited before writing its DataFlash log')
            if source.is_file():
                size = source.stat().st_size
                require(size <= LOG_LIMIT, 'seed simulator log exceeded the fixture bound')
                if size >= 32768 and size != inspected:
                    inspected = size
                    # Format records may fill several blocks before the
                    # firmware message reaches disk.
                    with source.open('rb') as stream:
                        if b'ArduCopter' in stream.read(size):
                            break
            time.sleep(.1)
        else:
            raise ValueError('seed simulator did not write its DataFlash firmware marker in time')
    require(not stop_process_tree(seed), 'seed simulator teardown failed')
    require(not source.is_symlink() and 32768 <= source.stat().st_size <= LOG_LIMIT,
            'seed log outside the declared closed-log bounds')
    closed_bytes = source.read_bytes()
    reader = DFReader.DFReader_binary(str(source))
    saw_firmware = False
    for _ in range(10000):
        message = reader.recv_match()
        if message is None:
            break
        if message.get_type() == 'MSG' and 'ArduCopter' in str(message.Message):
            saw_firmware = True
            break
    require(saw_firmware, 'seed data does not contain its ArduCopter DataFlash identity')
    report['closed_source'] = {'path': str(source.relative_to(out)), 'bytes': len(closed_bytes),
                               'sha256': sha(source), 'dataflash_arducopter_message': saw_firmware}

    ports['base'] = port(socket.SOCK_STREAM)
    kinds['base'] = socket.SOCK_STREAM
    ports['rc'] = port(socket.SOCK_DGRAM)
    (sim / 'message-intervals-chan1.txt').write_text('245 500\n')
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as monitor:
        monitor.bind(('127.0.0.1', 0))
        monitor.settimeout(.1)
        ports['monitor'] = monitor.getsockname()[1]
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
            receiver.bind(('127.0.0.1', 0))
            receiver.settimeout(.2)
            ports['bridge'] = receiver.getsockname()[1]
            copter = launch('download', ports['bridge'], ports['base'], ports['rc'], ports['monitor'])
            ports['peer'] = discover(receiver, copter)[1]
        sockets = subprocess.run(['lsof', '-nP', '-a', '-p', str(copter.pid), '-iUDP'],
                                 capture_output=True, text=True, timeout=3, check=True).stdout
        (out / 'native-udp-sockets.txt').write_text(sockets)
        require(f"127.0.0.1:{ports['bridge']}" in sockets and str(ports['peer']) in sockets,
                'SITL-owned bridge endpoint absent from PID/socket witness')
        probe = start_bridge(args, out, private, ports, processes, copter)
        decoder = mavlink.MAVLink(None)
        decoder.robust_parsing = True
        end = time.monotonic() + 85
        heartbeats = landed = 0
        drain_until = None
        with (out / 'native-monitor.mavlink').open('wb') as raw:
            while time.monotonic() < end:
                require(copter.poll() is None, 'owned SITL exited during app workflow')
                if probe.poll() is not None:
                    if drain_until is None:
                        drain_until = time.monotonic() + .3
                    if time.monotonic() >= drain_until:
                        break
                try:
                    data, address = monitor.recvfrom(65535)
                except socket.timeout:
                    continue
                require(address[0] == '127.0.0.1' and raw.tell()+len(data) <= 4*1024*1024,
                        'native independent capture source or byte limit')
                if 'monitor_peer' not in ports:
                    ports['monitor_peer'] = address[1]
                require(address[1] == ports['monitor_peer'], 'independent capture changed source port')
                raw.write(data)
                for message in decoder.parse_buffer(data) or ():
                    if message.get_srcSystem() != 1 or message.get_srcComponent() != 1:
                        continue
                    if message.get_type() == 'HEARTBEAT':
                        require(not message.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED,
                                'independent Copter heartbeat reported arming')
                        heartbeats += 1
                    if message.get_type() == 'EXTENDED_SYS_STATE':
                        require(message.landed_state in (0, 1), 'independent Copter reported non-ground state')
                        landed += message.landed_state == 1
        require(probe.poll() == 0 and heartbeats > 0 and landed > 0, 'missing app completion or independent disarmed ground evidence')
        report['observer'] = {'heartbeat_samples': heartbeats, 'ground_samples': landed, 'ever_armed': False}

    report['firmware_identity_after'] = selected()
    require(report['firmware_identity_after'] == firmware, 'firmware identity drift')
    require(source.read_bytes() == closed_bytes, 'simulator log changed while downloading')
    audit_native(out, report)


def audit_native(out, report):
    """Reconstruct operator completion from MOOS receipts and exact output/source bytes."""
    trace = events(out / 'probe.jsonl')
    rows = [json.loads(event['value']) for event in trace if event.get('key') == RESULT]
    catalogs = [json.loads(event['value']) for event in trace if event.get('key') == CATALOG]
    tx = [fields(event['value']) for event in trace if event.get('key') == TX]
    proof = {}
    for ident in ('initial', 'retrieve', 'after-end'):
        outcomes = [row for row in rows if row['id'] == ident]
        receipts = [row for row in tx if row['operation_id'] == ident]
        require([row['status'] for row in outcomes] == ['queued', 'complete'] and
                outcomes[0]['expires'] == outcomes[1]['expires'] > outcomes[0]['at'] and
                outcomes[1]['at'] <= outcomes[1]['expires']+2 and
                outcomes[1]['cleanup'] == 'local_end_accepted_unacknowledged',
                f'{ident}: invalid original deadline, operator result or cleanup witness')
        require(receipts and all(row['msgid'] in {'117', '119', '122'} and
                row['status'] == 'LOCAL_ACCEPTED' and row['late'] == 'false' and row['clock_valid'] == 'true'
                for row in receipts), f'{ident}: unexpected outbound message or unsuccessful handoff')
        ends = [row for row in receipts if row['msgid'] == '122']
        require(len(ends) == 1 and receipts[-1] == ends[0], f'{ident}: final accepted END missing')
        require(all(float(row['at']) < outcomes[1]['expires'] for row in receipts if row['msgid'] != '122'),
                f'{ident}: stale list/data handoff')
        proof[ident] = {'result': outcomes[1], 'requests': len(receipts), 'end': ends[0]}
    require(set(row['operation_id'] for row in tx) == set(proof), 'unexpected wire operation')
    listed = [row for row in catalogs if row['status'] == 'complete']
    require([row['id'] for row in listed] == ['initial', 'after-end'] and all(
        row['source_system'] == row['source_component'] == 1 and row['count'] == 1 and
        row['logs'][0]['log_id'] == 1 and row['logs'][0]['bytes_reported'] == report['closed_source']['bytes']
        for row in listed), 'missing known closed catalog or follow-up availability')
    require(float(proof['retrieve']['end']['at']) < proof['after-end']['result']['at'],
            'follow-up did not complete after the download END handoff')
    row = proof['retrieve']['result']
    require(row['file'] == 'log-1-retrieve.bin' and row['staging_removed'] is True and
            row['digest_algorithm'] == 'fnv1a64', 'unexpected download release or digest')
    release = out / 'downloads' / row['file']
    source = out / report['closed_source']['path']
    require(not release.is_symlink() and release.stat().st_mode & 0o077 == 0 and release.stat().st_nlink == 1
            and len(list(release.parent.iterdir())) == 1, 'download directory/file not private or staging remained')
    data = release.read_bytes()
    require(data == source.read_bytes() and len(data) == row['bytes'] and
            fnv(data) == row['digest'] and sha(release) == report['closed_source']['sha256'],
            'released file differs from known closed FC DataFlash bytes')
    report['verified_workflow'] = proof
    report['byte_comparison'] = {'source': str(source.relative_to(out)), 'released': str(release.relative_to(out)),
                                 'bytes': len(data), 'sha256': sha(release), 'fnv1a64': fnv(data),
                                 'byte_equal': True, 'follow_up_available': True,
                                 'remote_end_ack_or_logging_resume_proven': False}


def run_rx(args, out, private, report, ports, processes):
    ports['bridge'] = port(socket.SOCK_DGRAM)
    ports['peer'] = port(socket.SOCK_DGRAM)
    probe = start_bridge(args, out, private, ports, processes)
    require(probe.wait(timeout=15) == 0, 'generated RX-only operator probe failed')
    trace = events(out / 'probe.jsonl')
    results = [json.loads(event['value']) for event in trace if event.get('key') == RESULT]
    statuses = [fields(event['value']) for event in trace if event.get('key') == STATUS]
    peer = [event for event in trace if event.get('fixture') == 'generated_receive_only']
    require([row['status'] for row in results] == ['queued', 'rejected'] and
            results[-1]['id'] == 'initial' and results[-1]['reason'] == 'local_request_rejected',
            'generated disarmed peer bypassed RX-only request policy')
    require(statuses and all(row['mode'] == 'RX_ONLY' and row['permission'] == 'false' and
                            row['tx_attempts'] == row['tx_accepted'] == '0' for row in statuses),
            'runtime receive-only effect counters differ')
    require(len(peer) == 1 and peer[0]['sent'] > 0 and peer[0]['received_from_app'] == 0 and
            not any(row.get('key') in (CATALOG, TX) for row in trace) and not list(private.iterdir()),
            'RX-only probe observed outbound or partial/complete artifacts')
    report['generated_peer'] = peer[0]


def run(args):
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    private = out / 'downloads'
    private.mkdir(mode=0o700)
    paths = {'app_sha256': args.build / 'bin/iMAVLink', 'probe_sha256': args.probe,
             'module_sha256': args.build / 'plugins/mavmoos_flight_logs.so',
             'stock_helm_sha256': args.moos_root / 'bin/pHelmIvP'}
    report = {'schema': 1, 'result': 'FAIL', 'scenario': args.scenario, 'hardware': False, 'helm_started': False}
    report.update({key: sha(path) for key, path in paths.items()})
    ports = {}
    kinds = {'moos': socket.SOCK_STREAM}
    processes = []
    report['ports'] = ports
    try:
        ports['moos'] = port(socket.SOCK_STREAM)
        if args.scenario == 'native':
            report['mavmoos_ref'] = revision(ROOT)
            report['moos_ref'] = revision(args.moos_root)
            run_native(args, out, private, report, ports, processes, kinds)
        else:
            run_rx(args, out, private, report, ports, processes)
        observed = [json.loads(row['value']) for row in events(out / 'probe.jsonl') if row.get('key') == CAPS]
        require(observed and all(catalog == observed[0] for catalog in observed), 'missing/stale capability composition')
        require([module['name'] for module in observed[0]['composition']['modules']] == ['flight_logs'],
                'unexpected app module composition')
        report['capability_catalog'] = observed[0]
        report['loaded_modules_sha256'] = loaded_module_hashes(args.build / 'plugins', observed[0])
        report['result'] = 'PASS'
    except Exception as error:
        report['error'] = str(error)
    finally:
        remaining = []
        for process in reversed(processes):
            try:
                remaining.extend(stop_process_tree(process))
            except Exception as error:
                remaining.append({'pid': process.pid, 'error': str(error)})
        report['owned_processes_remaining'] = remaining
        report['ports_clear'] = {name: cleared(number, kinds.get(name, socket.SOCK_DGRAM))
                                 for name, number in ports.items()}
        report['artifact_drift'] = [key for key, path in paths.items() if sha(path) != report[key]]
        if args.scenario == 'native' and 'moos_ref' in report:
            for key, path in (('mavmoos_ref', ROOT), ('moos_ref', args.moos_root)):
                report[key+'_after'] = revision(path)
                if report[key+'_after'] != report[key]:
                    report['artifact_drift'].append(key)
            if 'closed_source' in report and sha(out / report['closed_source']['path']) != report['closed_source']['sha256']:
                report['artifact_drift'].append('closed_source')
        if report['artifact_drift'] or remaining or not all(report['ports_clear'].values()):
            report['result'] = 'FAIL'
            report.setdefault('error', 'artifact or scoped cleanup proof failed')
        report['files'] = {str(path.relative_to(out)): sha(path) for path in sorted(out.rglob('*'))
                           if path.is_file() and not path.is_symlink()}
        (out / 'result.json').write_text(json.dumps(report, indent=2, sort_keys=True) + '\n')
    print(json.dumps(report, indent=2, sort_keys=True))
    if report['result'] != 'PASS':
        for name in ('probe.jsonl', 'pantler.log', 'seed.log', 'download.log'):
            path = out / name
            if path.exists():
                print(name, path.read_text(errors='replace')[-4000:])
    return int(report['result'] != 'PASS')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario', choices=('native', 'rx'), required=True)
    for key in ('moos-root', 'build', 'probe'):
        parser.add_argument('--'+key, type=Path, required=True)
    for key in ('ardupilot-root', 'firmware-manifest', 'firmware-attestation'):
        parser.add_argument('--'+key, type=Path)
    output = parser.add_mutually_exclusive_group(required=True)
    output.add_argument('--output', type=Path)
    output.add_argument('--output-parent', type=Path)
    args = parser.parse_args()
    for key in ('moos_root', 'build', 'probe', 'ardupilot_root'):
        value = getattr(args, key)
        if value is not None:
            setattr(args, key, value.resolve())
    if args.output_parent is not None:
        args.output_parent.mkdir(parents=True, exist_ok=True)
        args.output = Path(tempfile.mkdtemp(prefix='run-', dir=args.output_parent)) / 'evidence'
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(SystemExit(143)))
    if args.scenario == 'native':
        if args.ardupilot_root is None:
            parser.error('--ardupilot-root required for native scenario')
        with (Path(tempfile.gettempdir()) / 'mavmoos-native-sitl.lock').open('a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            return run(args)
    return run(args)


if __name__ == '__main__':
    raise SystemExit(main())
