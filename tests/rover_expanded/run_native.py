#!/usr/bin/env python3
"""Qualify one ground or surface-boat profile against an owned native Rover."""
import argparse
import fcntl
import json
import math
import os
from pathlib import Path
import re
import signal
import shlex
import statistics
import stat
import socket
import subprocess
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tests'))
from owned_processes import owned_records, stop_process_tree
from control.catalog_evidence import loaded_module_hashes, observed_catalog
from rover.run_rover import Pilot, dataflash, owns_udp, port, sha, sockets
from pymavlink import mavutil


HOME = (42.358456, -71.087589)
ARDUPILOT = '04cf6486e8d42ce138de816e8466244346ed77cb'
MOOS = '023fb54e00692611ecee2cd6f1138d10eec2127e'
DEFAULTS = {
    'rover.parm': '6479879d3e625854a6c32cb4d3fe231bab81d935e07904eac8a411fd6f971bc4',
    'motorboat.parm': '55a816688adc1a7df897fdc56cbdd6bd5b43c354ee823891bff1e8a861861b01',
    'rover-skid.parm': '45a572363fc0bb1459ec676eafcf379f9f7a0823a3cf2be7a3d9f54c2e2c53a9',
}
SOURCE_WITNESSES = (
    'Tools/autotest/pysim/vehicleinfo.py', 'libraries/SITL/SIM_MotorBoat.h',
    'libraries/SITL/SIM_Sailboat.cpp', 'libraries/SITL/SITL.cpp',
    'Rover/mode_guided.cpp', 'Rover/mode_loiter.cpp', 'Rover/mode_rtl.cpp',
)
PROFILES = {
    'ground': ('ardurover', 10, 'rover', ('rover.parm',)),
    'boat': ('ardurover_boat', 11, 'motorboat-skid', tuple(DEFAULTS)),
}


def require(value, reason):
    if not value:
        raise RuntimeError(reason)


def git(path, *args):
    return subprocess.check_output(['git', '-C', str(path), *args], text=True, timeout=10).strip()


def build_source(build):
    cache = (build / 'CMakeCache.txt').read_text()
    selected = re.search(r'^CMAKE_HOME_DIRECTORY:INTERNAL=(.+)$', cache, flags=re.M)
    require(selected is not None, 'build cache does not identify its source checkout')
    path = Path(selected[1]).resolve(strict=True)
    return {'path': str(path), 'revision': git(path, 'rev-parse', 'HEAD'),
            'tracked_changes': git(path, 'diff', '--name-only', 'HEAD').splitlines()}


def evidence_files(out):
    return {str(file.relative_to(out)): sha(file) for file in sorted(out.rglob('*'))
            if file.name != 'result.json' and file.is_file() and not file.is_symlink()}


def tracked_pin(path, revision):
    require(git(path, 'rev-parse', 'HEAD') == revision, f'wrong source revision for {path}')
    require(not git(path, 'diff', '--name-only', 'HEAD'), f'tracked source differs from pin in {path}')


def exact_peer(text, source, destination):
    return bool(re.search(rf'UDP\s+127\.0\.0\.1:{source}->127\.0\.0\.1:{destination}(?:\s|$)', text))


def env_for(moos, build):
    env = dict(os.environ)
    env['PATH'] = os.pathsep.join([str(build / 'bin'), str(moos / 'bin'), env.get('PATH', '')])
    env.pop('IVP_BEHAVIOR_DIRS', None)
    return env


def start(command, out, env, name):
    with (out / f'{name}.log').open('w') as log:
        return subprocess.Popen([str(arg) for arg in command], cwd=out, env=env, stdout=log,
                                stderr=subprocess.STDOUT, start_new_session=True)


def generate(moos, out, macros, env):
    for suffix, source in (
        ('moos', ROOT / 'missions/rover_expanded/meta_vehicle.moos'),
        ('bhv', ROOT / 'missions/rover_sitl/meta_vehicle.bhv'),
    ):
        subprocess.run([str(moos / 'bin/nsplug'), str(source), str(out / f'targ_rover.{suffix}'),
                        '--strict', '--force', '-x', *[f'{key}={value}' for key, value in macros.items()]],
                       cwd=out, env=env, check=True, capture_output=True, timeout=8)


def native_replay_config(out):
    directory = out / 'native-replay'
    directory.mkdir(mode=0o700)
    return f'plugin.ardurover_control.native_replay_directory = {directory}'


def launcher_macros(db, build, moos, bridge, peer, kind):
    profile, expected_type, _, _ = PROFILES[kind]
    return {'MOOS_PORT': db, 'APP_PATH': build / 'bin/iMAVLink', 'BIND_PORT': bridge,
            'PEER_PORT': peer, 'LAT_ORIGIN': HOME[0], 'LONG_ORIGIN': HOME[1],
            'SIMULATION_ONLY': 'true', 'SHADOW_MODE': 'false', 'SITL_PID': 0,
            'HELM_ENABLED': 'true', 'NATIVE_ENABLED': 'true', 'HELM_PATH': moos / 'bin/pHelmIvP',
            'PLUGIN_DIRECTORY': build / 'plugins', 'SPEED': '1.2',
            'NATIVE_REPLAY_CONFIG': '// Native replay is disabled for passive tests',
            'WAYPOINTS': '0,7:8,7:8,15:17,15:17,21', 'PROFILE': profile, 'EXPECTED_TYPE': expected_type}


def wait_endpoints(process, bridge, bridge_port, rc_port, expected_type):
    decoder = mavutil.mavlink.MAVLink(None)
    decoder.robust_parsing = True
    endpoint = None
    deadline = time.monotonic() + 20
    while time.monotonic() < deadline and endpoint is None:
        require(process.poll() is None, 'owned native Rover exited during startup')
        try:
            packet, address = bridge.recvfrom(65535)
        except socket.timeout:
            continue
        for message in decoder.parse_buffer(packet) or []:
            if (message.get_type(), message.get_srcSystem(), message.get_srcComponent()) != ('HEARTBEAT', 1, 1):
                continue
            require(address[0] == '127.0.0.1' and message.type == expected_type and message.autopilot == 3,
                    f'wrong native heartbeat identity at {address} (type {message.type})')
            endpoint = address
    require(endpoint, 'native heartbeat not observed during startup')
    snapshots = []
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        inventory = sockets(process)
        snapshots.append({'observed_mono': time.monotonic(), 'inventory': inventory})
        if exact_peer(inventory, endpoint[1], bridge_port) and owns_udp(inventory, rc_port, allow_wildcard=True):
            return endpoint, snapshots
        time.sleep(.1)
    raise RuntimeError('exact connected native MAVLink route and RC endpoint were not owned')


def mission_inventory(launch, mission, bridge_port, helm):
    rows = owned_records(launch)
    by_mission = [(row, shlex.split(row['command'])) for row in rows if str(mission) in row['command']]
    matched = [row for row, tokens in by_mission if str(mission) in tokens]
    require(any(tokens[0] == str(helm) and str(mission) in tokens for _, tokens in by_mission),
            'pAntler did not launch the exact pinned stock pHelmIvP path')
    apps = [row for row in matched if Path(row['command'].split()[0]).name == 'iMAVLink']
    require(len(apps) == 1, 'missing uniquely owned iMAVLink process')
    inventory = subprocess.check_output(
        ['lsof', '-nP', '-a', '-p', str(apps[0]['pid']), '-iUDP'], text=True, timeout=3)
    require(owns_udp(inventory, bridge_port), 'owned app did not bind the selected loopback endpoint')
    return {'processes': matched, 'app_udp_inventory': inventory}


class LinkPause:
    """Pause and resume the same owned process to cause real FC telemetry loss."""
    def __init__(self, process, marker):
        self.process = process
        self.marker = marker
        self.events = []
        self.settlement = None
        self.request_observed = None

    def update(self):
        now = time.monotonic()
        if not self.events and self.marker.is_file():
            require(self.process.poll() is None, 'owned native Rover exited before link fault')
            os.kill(self.process.pid, signal.SIGSTOP)
            self.events.append({'event': 'SIGSTOP', 'mono': now, 'pid': self.process.pid})
        elif len(self.events) == 1 and now - self.events[0]['mono'] >= 3.4:
            require(self.process.poll() is None, 'owned native Rover exited before recovery')
            os.kill(self.process.pid, signal.SIGCONT)
            self.events.append({'event': 'SIGCONT', 'mono': now, 'pid': self.process.pid})
        return len(self.events) == 1

    def acknowledge_settlement(self, packets):
        if self.settlement is not None or len(self.events) != 2:
            return
        request = self.marker.with_name('link_settled.request')
        try:
            identity = request.lstat()
        except FileNotFoundError:
            return
        require(stat.S_ISREG(identity.st_mode) and identity.st_uid == os.getuid(),
                'unsafe independent link witness request marker')
        now = time.monotonic()
        if self.request_observed is None:
            self.request_observed = now
        released = [row for row in packets if row['mavpackettype'] == 'GLOBAL_POSITION_INT' and
                    row['received_mono'] >= self.events[1]['mono']]
        suffix = settled_suffix(released)
        if not suffix or now - suffix[-1]['received_mono'] > .4:
            return
        acknowledgement = self.marker.with_name('link_settled.verified')
        require(not acknowledgement.exists(), 'unexpected existing settlement acknowledgement')
        self.settlement = {'request_observed_mono': self.request_observed, 'ack_mono': now,
                           'first_boot_ms': suffix[0]['time_boot_ms'], 'last_boot_ms': suffix[-1]['time_boot_ms'],
                           'last_received_mono': suffix[-1]['received_mono'], 'samples': len(suffix)}
        temp = acknowledgement.with_suffix('.tmp')
        with temp.open('x') as stream:
            json.dump(self.settlement, stream)
        temp.rename(acknowledgement)


def monitor_run(sitl, monitor, launch, probe, pilot, fault, out, result, expected_type):
    decoder = mavutil.mavlink.MAVLink(None)
    decoder.robust_parsing = True
    endpoint, drain, packets = None, None, []
    deadline = time.monotonic() + 255
    with (out / 'fc-monitor.jsonl').open('w') as trace, (out / 'fc-monitor.mavlink').open('wb') as raw:
        while time.monotonic() < deadline:
            require(sitl.poll() is None and launch.poll() is None, 'owned native/mission process exited early')
            if probe.poll() is not None:
                drain = drain or time.monotonic() + 1.6
                if time.monotonic() >= drain:
                    break
            elif not fault.update():
                pilot.update()
                fault.acknowledge_settlement(packets)
            try:
                packet, address = monitor.recvfrom(65535)
            except BlockingIOError:
                time.sleep(.01)
                continue
            if endpoint is None:
                inventory = sockets(sitl)
                require(address[0] == '127.0.0.1' and
                        exact_peer(inventory, address[1], monitor.getsockname()[1]),
                        'second link is not the exact owned, connected monitor route')
                endpoint = address
                result['ports']['monitor_peer'] = endpoint[1]
                result['monitor_socket_proof'] = inventory
            require(address == endpoint, 'unowned independent telemetry sender')
            raw.write(packet)
            for message in decoder.parse_buffer(packet) or []:
                if message.get_srcSystem() != 1 or message.get_srcComponent() != 1:
                    continue
                if message.get_type() == 'HEARTBEAT':
                    require(message.type == expected_type and message.autopilot == 3,
                            'independent link received the wrong heartbeat identity')
                if message.get_type() in {'HEARTBEAT', 'GLOBAL_POSITION_INT', 'SERVO_OUTPUT_RAW', 'STATUSTEXT'}:
                    row = {'received_mono': time.monotonic(), **message.to_dict()}
                    trace.write(json.dumps(row) + '\n')
                    packets.append(row)
    result['probe_exit'] = probe.poll()
    result['link_pause'] = fault.events
    result['link_settlement'] = fault.settlement
    require(probe.returncode is not None, 'bounded 255 second wrapper deadline exceeded')
    return packets


def displacement(row, reference):
    lat = row['lat'] / 1e7
    lon = row['lon'] / 1e7
    return ((lon - reference[1]) * 111195 * math.cos(math.radians(reference[0])),
            (lat - reference[0]) * 111195)


def gps(row):
    return row['lat'] / 1e7, row['lon'] / 1e7


def speed(row):
    return math.hypot(row['vx'], row['vy']) / 100


def settled_suffix(positions):
    """Take the latest contiguous independent low-speed firmware-time sequence."""
    suffix = []
    for row in reversed(positions):
        if speed(row) > .45:
            break
        if suffix and not 0 < suffix[-1]['time_boot_ms'] - row['time_boot_ms'] <= 250:
            break
        suffix.append(row)
        if len(suffix) >= 5 and suffix[0]['time_boot_ms'] - row['time_boot_ms'] >= 800:
            return list(reversed(suffix))
    return []


def wire_proof(out, packets, kind, link_events, settlement):
    rows = [json.loads(line) for line in (out / 'workflow.jsonl').read_text().splitlines()]
    phases = {row['key']: row['mono'] for row in rows if row['type'] == 'phase'}
    heartbeats = [row for row in packets if row['mavpackettype'] == 'HEARTBEAT']
    positions = [row for row in packets if row['mavpackettype'] == 'GLOBAL_POSITION_INT'
                 and math.hypot(*displacement(row, HOME)) < 200]
    modes = {row['custom_mode'] for row in heartbeats}
    require((({0, 4, 5, 11, 15} if kind == 'boat' else {0, 4, 11, 15}).issubset(modes) and
             any(row['base_mode'] & 128 for row in heartbeats) and not (heartbeats[-1]['base_mode'] & 128)),
            'independent link did not see all required modes plus arm and final disarm')
    require(len(link_events) == 2 and link_events[0]['mono'] < phases['link_loss_detected'] < link_events[1]['mono'],
            'revocation was not observed inside the native link cut')
    inside_cut = [row for row in heartbeats
                  if link_events[0]['mono'] + .2 < row['received_mono'] < link_events[1]['mono']]
    require(not inside_cut, 'native independent heartbeat continued during injected loss')
    manual = [row for row in heartbeats if phases['pilot_takeover_begin'] <= row['received_mono']
              <= phases['pilot_takeover_done'] and row['custom_mode'] == 0]
    require(manual, 'independent link did not see pilot Manual during the takeover window')

    def samples(start, end):
        chosen = [row for row in positions if phases[start] - .2 <= row['received_mono'] <= phases[end] + .15]
        require(len(chosen) >= 4, f'insufficient independent position data from {start} to {end}')
        return chosen

    def travel(start, end):
        chosen = samples(start, end)
        return max(math.hypot(*displacement(row, gps(chosen[0]))) for row in chosen)

    stock = samples('stock_waypoint_begin', 'stock_waypoint_done')
    offsets = [displacement(row, gps(stock[0])) for row in stock]
    stock_axes = [max(row[axis] for row in offsets) for axis in (0, 1)]
    require(stock_axes[0] >= 2.6 and stock_axes[1] >= 3.6,
            'independent link did not see both axes of the stock waypoint route')
    recovered = {name: travel(name + '_begin', name + '_done') for name in
                 ('allstop_reacquired', 'manual_reacquired', 'link_reacquired')}
    require(all(distance >= .8 for distance in recovered.values()),
            'independent link did not observe fresh motion after each explicit reacquisition')
    expiry_witness = {}
    for name, start, end in (
        ('allstop', 'allstop_stopped', 'allstop_reacquired_begin'),
        ('link_recovery', 'link_fresh_observations', 'link_recovered'),
        ('guided_cancel', 'guided_cancel_stopped', 'guided_cancel_done'),
    ):
        chosen = [row for row in samples(start, end) if row['received_mono'] <= phases[end]]
        require(len(chosen) >= 4, f'insufficient independent released samples for {name}')
        elapsed = chosen[-1]['time_boot_ms'] - chosen[0]['time_boot_ms']
        require(elapsed > 3000 and speed(chosen[-1]) <= .45,
                f'{name} did not outlast the old Guided target in independent firmware time')
        expiry_witness[name] = {'firmware_elapsed_ms': elapsed, 'final_speed_m_s': speed(chosen[-1])}
        if name == 'link_recovery':
            require(settlement and phases['link_settlement_requested'] <= settlement['request_observed_mono']
                    <= settlement['ack_mono'] <= phases['link_settlement_acknowledged'] <= phases[end],
                    'independent settlement was not acknowledged before explicit reacquisition')
            dwell = settled_suffix([row for row in chosen if row['received_mono'] <= settlement['last_received_mono']])
            require(dwell and dwell[0]['time_boot_ms'] == settlement['first_boot_ms'] and
                    dwell[-1]['time_boot_ms'] == settlement['last_boot_ms'] and len(dwell) == settlement['samples'],
                    'independent FC did not corroborate the acknowledged low-speed sequence')
            timeout_messages = [row for row in packets if row['mavpackettype'] == 'STATUSTEXT' and
                                row.get('text') == 'target not received last 3secs, stopping' and
                                phases['link_loss_begin'] <= row['received_mono'] <= phases[end]]
            expiry_witness[name]['firmware_timeout_observed'] = bool(timeout_messages)
            expiry_witness[name]['settled_position_samples'] = len(dwell)
            expiry_witness[name]['settled_max_speed_m_s'] = max(map(speed, dwell))
            expiry_witness[name]['settled_firmware_ms'] = dwell[-1]['time_boot_ms'] - dwell[0]['time_boot_ms']
    allstop = samples('allstop_begin', 'allstop_stopped')
    native_hold = samples('native_hold_drift_begin', 'native_hold_done')
    require(speed(allstop[-1]) <= .45 and speed(native_hold[-1]) <= .45,
            'all-stop or native Hold did not physically settle')
    guided = samples('guided_goto_begin', 'guided_goto_done')
    require(travel('guided_goto_begin', 'guided_goto_done') >= 3 and
            math.hypot(*displacement(guided[-1], (42.358501, HOME[1]))) <= 2.65,
            'guided goto did not independently reach its target')
    held = samples('guided_hold_begin', 'guided_hold_done')
    held_drift = travel('guided_hold_begin', 'guided_hold_done')
    require(held_drift <= 2 and speed(held[-1]) <= .4, 'guided hold did not independently settle')
    cancelled = samples('guided_cancel_begin', 'guided_cancel_done')
    require(travel('guided_cancel_begin', 'guided_cancel_done') >= 1 and speed(cancelled[-1]) <= .45,
            'independent link did not see movement and a stop after Guided cancel')

    returned = {}
    for name in ('guided_return', 'native_rtl'):
        chosen = samples(name + '_begin', name + '_done')
        initial, final = (math.hypot(*displacement(row, HOME)) for row in (chosen[0], chosen[-1]))
        require(initial >= 3 and initial - final >= 2 and final <= 2.65,
                f'{name} did not independently close the range to home: {initial:.2f} to {final:.2f} m')
        returned[name] = {'initial_home_m': initial, 'final_home_m': final}
    require(speed(positions[-1]) <= .45, 'final disarmed stop missing on independent link')

    proof = {'observed_modes': sorted(modes), 'stock_east_north_m': stock_axes,
             'stock_position_samples': len(stock), 'reacquired_motion_m': recovered,
             'guided_expiry_witness': expiry_witness,
             'guided_hold_drift_m': held_drift, 'returns': returned, 'final_speed_m_s': speed(positions[-1]),
             'independent_position_samples': len(positions), 'independent_heartbeat_samples': len(heartbeats)}
    if kind == 'boat':
        proof['boat_native_loiter'] = boat_loiter_proof(packets, heartbeats, phases, samples, travel)
    return proof


def boat_loiter_proof(packets, heartbeats, phases, samples, travel):
    hold = samples('boat_passive_baseline_begin', 'boat_passive_baseline_done')
    loiter = samples('native_loiter_settled', 'native_loiter_done')
    during_loiter = [row for row in heartbeats if phases['native_loiter_settled'] <= row['received_mono']
                     <= phases['native_loiter_done']]
    require(len(during_loiter) >= 15 and all(row['custom_mode'] == 5 for row in during_loiter),
            'independent heartbeats did not retain native Loiter during the measured window')
    hold_east, _ = displacement(hold[-1], gps(hold[0]))
    hold_seconds = hold[-1]['received_mono'] - hold[0]['received_mono']
    loiter_seconds = loiter[-1]['received_mono'] - loiter[0]['received_mono']
    hold_west_rate = -hold_east / hold_seconds
    baseline = hold_west_rate * loiter_seconds
    offsets = [(row['received_mono'], *displacement(row, gps(loiter[0]))) for row in loiter]
    drift = max(math.hypot(east, north) for _, east, north in offsets)
    require(.10 <= hold_west_rate <= .28 and baseline >= 3.5 and drift <= min(3.2, .72 * baseline),
            f'Loiter did not reduce drift from measured passive Hold (baseline {baseline:.2f} m, Loiter {drift:.2f} m)')
    outside_radius = next((mono for mono, east, north in offsets if math.hypot(east, north) >= .9), None)
    require(outside_radius is not None, 'boat never reached the configured 1 m Loiter radius')
    servos = [row for row in packets if row['mavpackettype'] == 'SERVO_OUTPUT_RAW' and row['port'] == 0]
    neutral = [row for row in servos if phases['boat_passive_baseline_begin'] <= row['received_mono'] <= phases['boat_passive_baseline_done']]
    require(len(neutral) >= 8, 'missing independent passive-Hold differential motor outputs')
    channels = ('servo1_raw', 'servo3_raw')
    mid = {key: statistics.median(row[key] for row in neutral) for key in channels}
    require(all(abs(pwm - 1500) <= 40 for pwm in mid.values()), 'native Hold did not show neutral drive channels')
    corrections = [row for row in servos if outside_radius <= row['received_mono'] <= phases['native_loiter_done'] and
                   max(abs(row[key] - mid[key]) for key in channels) >= 25]
    require(len(corrections) >= 5 and corrections[-1]['received_mono'] - corrections[0]['received_mono'] >= .8,
            'no sustained differential drive correction after reaching the Loiter radius')
    require(speed(loiter[-1]) <= .4, 'native Loiter did not finish at a bounded speed')
    loiter_reacquired = travel('loiter_reacquired_begin', 'loiter_reacquired_done')
    require(loiter_reacquired >= .8 and any(
        phases['loiter_reacquired_begin'] <= row['received_mono'] <= phases['loiter_reacquired_done'] and row['custom_mode'] == 15
        for row in heartbeats), 'native Loiter did not transition to Guided travel after explicit helm acquire')
    return {'passive_hold_west_rate_m_s': hold_west_rate, 'unpowered_projection_m': baseline,
            'loiter_max_displacement_m': drift, 'loiter_samples': len(loiter), 'loiter_seconds': loiter_seconds,
            'loiter_final_speed_m_s': speed(loiter[-1]), 'neutral_motor_pwm': mid,
            'correction_servo_samples': len(corrections), 'explicit_helm_reacquire_m': loiter_reacquired}


def catalog_proof(out, build, profile, mode):
    catalog = observed_catalog(out / 'workflow.jsonl', ('telemetry', 'parameters', 'ardurover_control'),
                               native=mode == 'SITL_CONTROL', helm=mode != 'RX_ONLY', mode=mode,
                               control_profile=profile)
    return catalog, loaded_module_hashes(build / 'plugins', catalog)


def dataflash_proof(path, kind):
    proof = dataflash(path)
    if kind == 'boat':
        log = mavutil.mavlink_connection(str(path))
        loiter = []
        while (message := log.recv_match(type='MODE')) is not None:
            if message.ModeNum == 5:
                loiter.append(message.to_dict())
        require(loiter, 'DataFlash did not corroborate native boat Loiter mode')
        proof['native_loiter_modes'] = loiter
    return proof


def endpoint_cleanup(ports):
    clear = {}
    for name, value in ports.items():
        if name in {'db', 'base'}:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as stream:
                stream.settimeout(.2)
                clear[name] = stream.connect_ex(('127.0.0.1', value)) != 0
        else:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as stream:
                try:
                    stream.bind(('127.0.0.1', value))
                    clear[name] = True
                except OSError:
                    clear[name] = False
    return clear


def main(args):
    ap, moos, build = (Path(getattr(args, key)).resolve() for key in ('ardupilot_root', 'moos_root', 'build'))
    tracked_pin(ap, ARDUPILOT)
    tracked_pin(moos, MOOS)
    source = build_source(build)
    profile, expected_type, model, defaults_names = PROFILES[args.profile]
    defaults = [ap / 'Tools/autotest/default_params' / name for name in defaults_names]
    require(all(sha(path) == DEFAULTS[path.name] for path in defaults), 'canonical model defaults differ from pin')
    binary, helm, probe_path = ap / 'build/sitl/bin/ardurover', moos / 'bin/pHelmIvP', build / 'bin/rover_expanded_probe'
    artifacts = [binary, helm, probe_path, build / 'bin/iMAVLink', moos / 'bin/pAntler', moos / 'bin/MOOSDB',
                 moos / 'bin/nsplug', Path(__file__), Path(__file__).with_name('RoverExpandedProbe.cpp'),
                 ROOT / 'missions/rover_expanded/meta_vehicle.moos', ROOT / 'missions/rover_sitl/meta_vehicle.bhv',
                 *defaults, *(ap / source for source in SOURCE_WITNESSES)]
    for file in artifacts:
        require(file.is_file() and not file.is_symlink(), f'regular pinned input required: {file}')
    digests = {str(file): sha(file) for file in artifacts}
    before = time.monotonic()
    probe_mono = float(subprocess.check_output([str(probe_path), '--clock'], text=True, timeout=5))
    require(before - .005 <= probe_mono <= time.monotonic() + .005,
            'probe and independent monitor disagree on monotonic timebase')
    if args.native_sha256:
        require(digests[str(binary)] == args.native_sha256, 'native executable hash differs from selected artifact')
    if args.output:
        out = Path(args.output).resolve()
        out.mkdir(parents=True, exist_ok=False)
    else:
        parent = Path(args.output_parent).resolve()
        parent.mkdir(parents=True, exist_ok=True)
        out = Path(tempfile.mkdtemp(prefix=args.profile + '-', dir=parent))
    (out / 'logs').mkdir()
    control = out / 'control'
    control.mkdir()
    bridge, monitor = (socket.socket(socket.AF_INET, socket.SOCK_DGRAM) for _ in range(2))
    bridge.bind(('127.0.0.1', 0))
    bridge.settimeout(.2)
    monitor.bind(('127.0.0.1', 0))
    monitor.setblocking(False)
    bridge_port, monitor_port = bridge.getsockname()[1], monitor.getsockname()[1]
    db = port(socket.SOCK_STREAM)
    base = port(socket.SOCK_STREAM)
    rc = port(socket.SOCK_DGRAM)
    fixture = out / 'fixture.parm'
    fixture.write_text('SERIAL1_PROTOCOL 2\nMAV2_POSITION 10\nMAV2_EXT_STAT 2\nMAV2_EXTRA3 2\n'
                       'LOG_DISARMED 1\nWP_RADIUS 1\nWP_SPEED 2\nMODE_CH 8\nMODE1 0\nMODE4 4\n' +
                       ('SIM_WIND_SPD 0\nSIM_WAVE_ENABLE 0\nSIM_TIDE_SPEED 0.18\nSIM_TIDE_DIR 90\nLOIT_RADIUS 1\n'
                        if args.profile == 'boat' else ''))
    intervals = out / 'message-intervals-chan1.txt'
    intervals.write_text('33 100\n36 200\n193 500\n')
    command = [str(binary), '--model', model, '--home', f'{HOME[0]},{HOME[1]},10,0', '--speedup', '1', '--wipe',
               '--defaults', ','.join(map(str, defaults + [fixture])),
               '--serial0', f'udpclient:127.0.0.1:{bridge_port}', '--serial1', f'udpclient:127.0.0.1:{monitor_port}',
               '--serial2', 'none', '--serial5', 'none', '--serial6', 'none', '--serial7', 'none', '--serial8', 'none',
               '--base-port', str(base), '--rc-in-port', str(rc)]
    result = {'result': 'FAIL', 'profile': profile, 'vehicle_profile': profile, 'mav_type': expected_type,
              'model': model, 'hardware': False, 'ardupilot_revision': ARDUPILOT, 'moos_revision': MOOS,
              'native_sha256': digests[str(binary)], 'app_sha256': digests[str(build / 'bin/iMAVLink')],
              'probe_sha256': digests[str(probe_path)], 'helm_sha256': digests[str(helm)],
              'mavmoos_revision': source['revision'], 'mavmoos_cmake_source': source,
              'mavmoos_harness_revision': git(ROOT, 'rev-parse', 'HEAD'),
              'digests_before': digests, 'fixtures_sha256': {file.name: sha(file) for file in (fixture, intervals)},
              'command': command, 'ports': {'bridge': bridge_port, 'monitor': monitor_port, 'db': db, 'base': base, 'rc': rc},
              'monitor_writes': 0, 'moos_timewarp': 1, 'sitl_speedup': 1}
    sitl = launch = probe = pilot = None
    try:
        env = env_for(moos, build)
        sitl = start(command, out, env, 'sitl')
        result['sitl_pid'] = sitl.pid
        peer, proofs = wait_endpoints(sitl, bridge, bridge_port, rc, expected_type)
        result['ports']['peer'] = peer[1]
        result['native_socket_proofs'] = proofs
        bridge.close()
        pilot = Pilot(sitl, rc, control, out / 'fixture-pilot.jsonl')
        pilot.update()
        macros = launcher_macros(db, build, moos, bridge_port, peer[1], args.profile)
        macros['SITL_PID'] = sitl.pid
        macros['NATIVE_REPLAY_CONFIG'] = native_replay_config(out)
        generate(moos, out, macros, env)
        result['generated_sha256'] = {file.name: sha(file) for file in out.glob('targ_rover.*') if file.is_file()}
        mission = out / 'targ_rover.moos'
        launch = start([moos / 'bin/pAntler', mission], out, env, 'pantler')
        probe = start([probe_path, db, out / 'workflow.jsonl', control, args.profile], out, env, 'probe')
        fault = LinkPause(sitl, control / 'pause.request')
        packets = monitor_run(sitl, monitor, launch, probe, pilot, fault, out, result, expected_type)
        result['owned_mission_proof'] = mission_inventory(launch, mission, bridge_port, helm)
        require(probe.returncode == 0, 'MOOS workflow failed; inspect probe.log and workflow.jsonl')
        result['independent_monitor'] = wire_proof(out, packets, args.profile, fault.events, fault.settlement)
        result['capabilities_catalog'], result['loaded_modules_sha256'] = catalog_proof(out, build, profile, 'SITL_CONTROL')
        result['result'] = 'PASS'
    except BaseException as error:
        result['error'] = str(error) or type(error).__name__
    finally:
        if pilot is not None:
            pilot.close()
        remaining = []
        for process in (probe, launch, sitl):
            try:
                remaining.extend(stop_process_tree(process))
            except Exception as error:
                remaining.append({'pid': process.pid, 'error': str(error)})
        bridge.close()
        monitor.close()
        result['ports_clear'] = endpoint_cleanup(result['ports'])
        result['owned_processes_remaining'] = remaining
        if not all(result['ports_clear'].values()) or remaining:
            result['result'], result['cleanup_error'] = 'FAIL', 'owned endpoint or process remains'
        try:
            tracked_pin(ap, ARDUPILOT)
            tracked_pin(moos, MOOS)
            require(build_source(build) == source, 'configured MAVMOOS source changed during qualification')
            result['digests_after'] = {str(file): sha(file) for file in artifacts}
            require(result['digests_after'] == digests, 'source or executable changed during qualification')
            if result['result'] == 'PASS':
                bins = list((out / 'logs').glob('*.BIN'))
                require(len(bins) == 1, 'expected one DataFlash BIN in fresh output')
                before = sha(bins[0])
                result['independent_dataflash'] = {'path': str(bins[0]), 'sha256': before, **dataflash_proof(bins[0], args.profile)}
                require(sha(bins[0]) == before, 'DataFlash artifact changed during inspection')
        except Exception as error:
            result['result'], result['readback_error'] = 'FAIL', str(error)
        result['files'] = evidence_files(out)
        (out / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({'result': result['result'], 'profile': profile, 'output': str(out), 'error': result.get('error'),
                      'cleanup_error': result.get('cleanup_error'), 'readback_error': result.get('readback_error'),
                      'independent_monitor': result.get('independent_monitor'), 'ports_clear': result['ports_clear']}, indent=2))
    return 0 if result['result'] == 'PASS' else 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('ardupilot-root', 'moos-root', 'build'):
        parser.add_argument('--' + name, required=True)
    parser.add_argument('--profile', choices=PROFILES, required=True)
    output = parser.add_mutually_exclusive_group(required=True)
    output.add_argument('--output', help='exact fresh output directory')
    output.add_argument('--output-parent', help='allocate a fresh subdirectory on each run')
    parser.add_argument('--native-sha256', help='optionally require the exact previously qualified native artifact')
    arguments = parser.parse_args()
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(SystemExit(143)))
    with (Path(tempfile.gettempdir()) / 'mavmoos-native-sitl.lock').open('w') as lease:
        fcntl.flock(lease, fcntl.LOCK_EX)
        raise SystemExit(main(arguments))
