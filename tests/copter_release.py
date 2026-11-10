"""Additional live and archived witnesses for the owned Copter starter mission."""
from contextlib import ExitStack
import json
from pathlib import Path
import math
import platform
import re
import socket
import time

from plane_native.flight_fixture import Channel, exact_peer, inventory, log, need, now, reserve, revision, sha

PROFILE = 'arducopter-4.6.3-quad'
MODULES = ('telemetry', 'parameters', 'arducopter_control')
OPERATOR_SYSTEM, OPERATOR_COMPONENT = 249, 190


def frozen_inputs(root, moos, build, behavior_dir, firmware, expected, frame=None, *,
                  public_stock=False, installed=None):
    source = installed.manifest['source_revision'] if installed else revision(root)
    need(not expected or source == expected, 'MAVMOOS checkout differs from the selected candidate')
    need(firmware and firmware['profile_id'] == PROFILE and firmware['frame'] == 'quad' and
         not firmware['fixture']['hardware'], 'release demo needs the attested Copter 4.6.3 quad')
    setup = root/'config/setup.json'
    moos_pin = '023fb54e00692611ecee2cd6f1138d10eec2127e'
    if public_stock:
        need(frame is None, 'public stock mode is scoped to the quad starter')
        moos_pin = json.loads(setup.read_text())['moos']['revision']
    if installed:
        need(public_stock and build == moos == installed.root and behavior_dir == build / 'lib' and
             installed.manifest['moos_revision'] == moos_pin, 'installed stock MOOS provenance differs')
        paths = {'bundle_manifest': build / 'manifest.json', 'probe': installed.probe}
    else:
        need(revision(moos) == moos_pin, 'stock MOOS source pin differs')
        cache = build / 'CMakeCache.txt'
        settings = cache.read_text()
        for key, value in (('CMAKE_HOME_DIRECTORY', root), ('MOOS_IVP_ROOT', moos)):
            locations = re.findall(rf'^{key}:[^=]+=([^\n]+)$', settings, re.M)
            need(len(locations) == 1 and Path(locations[0]).resolve() == value,
                 f'configured build does not bind {key}')
        paths = {'cache': cache, 'probe': build / 'bin/sitl_workflow_probe'}
    behaviors = [path for path in behavior_dir.glob('libBHV_ConstantAltitude.*')
                 if path.suffix in {'.dylib', '.so'} and path.is_file()]
    need(len(behaviors) == 1, 'exactly one compiled BHV_ConstantAltitude is required')
    paths.update({'app': build/'bin/iMAVLink',
             'native': Path(firmware['native_path']), 'firmware_manifest': Path(firmware['manifest_path']),
             'firmware_attestation': Path(firmware['attestation_path']), 'altitude_behavior': behaviors[0]})
    for name in ('pAntler', 'MOOSDB', 'pHelmIvP', 'nsplug', 'pLogger', 'uProcessWatch'):
        paths[name] = moos/'bin'/name
    modules = MODULES if frame is None else ('telemetry', 'arducopter_control')
    for name in modules:
        paths[name] = build/'plugins'/f'mavmoos_{name}.so'
    for name in ('tests/run_sitl.py', 'tests/SitlWorkflowProbe.cpp', 'tests/copter_release.py',
                 'tests/owned_processes.py', 'tests/firmware_identity.py', 'scripts/firmware/manifest.py',
                 'tests/plane_native/flight_fixture.py', 'missions/copter_release/launch.sh',
                 'missions/copter_sitl/meta_vehicle.moos', 'missions/copter_release/meta_vehicle.bhv'):
        paths[name] = root/name
    if public_stock:
        paths['config/setup.json'] = setup
    if frame is not None:
        paths['missions/copter_sitl/meta_vehicle.bhv'] = root/'missions/copter_sitl/meta_vehicle.bhv'
        paths['tests/control/catalog_evidence.py'] = root/'tests/control/catalog_evidence.py'
        ap = Path(firmware['native_path']).parents[3]
        paths['frame_defaults'] = ap/'Tools/autotest/default_params'/f'copter-{frame}.parm'
        paths['base_defaults'] = ap/'Tools/autotest/default_params/copter.parm'
    need(all(path.is_file() for path in paths.values()), 'frozen release demo artifact is missing')
    hashes = {name: sha(path) for name, path in paths.items()}
    if installed:
        need(all(hashes[name] == installed.manifest['files'][path.relative_to(build).as_posix()]['sha256']
                 for name, path in paths.items() if path.is_relative_to(build) and name != 'bundle_manifest'),
             'runtime input differs from the installed archive manifest')
    need(hashes['native'] == firmware['native_sha256'] and
         hashes['firmware_manifest'] == firmware['manifest_sha256'] and
         hashes['firmware_attestation'] == firmware['attestation_sha256'],
         'firmware changed after manifest verification')
    if frame is not None:
        need(hashes['pHelmIvP'] == '5b3d2e705609c37b5b5000384655b4491010d6d797b94c43830d9972e684a876',
             'stock Copter frame helm executable differs')
    return source, paths, hashes


class PilotTakeover:
    """Mode requests over a separate serial2 route to the owned simulator."""
    def __init__(self, out, codec, *, guided_handback=False):
        self.stack = ExitStack()
        try:
            endpoint = self.stack.enter_context(reserve(socket.SOCK_DGRAM))
            self.channel = Channel('operator', endpoint, codec, self.stack, out)
            self.proofs = self.stack.enter_context((out/'operator-ownership.jsonl').open('x'))
            self.timeline = self.stack.enter_context((out/'release-phases.jsonl').open('x'))
        except BaseException:
            self.stack.close()
            raise
        self.codec = codec
        self.encoder = codec.MAVLink(None, srcSystem=OPERATOR_SYSTEM, srcComponent=OPERATOR_COMPONENT)
        self.phases = {}
        self.phase_raw = {}
        self.mode_changes = {'external_takeover': (4, 5)}
        if guided_handback:
            self.mode_changes['external_guided_handback'] = (5, 4)
        self.sent_at = {}
        self.sent_raw = {}
        self.phase = None

    @property
    def port(self):
        return self.channel.port

    def tick(self, pilot, observed):
        self.channel.read(pilot.sitl, self.proofs)
        observed_at = time.monotonic()
        if pilot.phase and self.phase != pilot.phase:
            need(pilot.phase not in self.phases, 'operator fixture reentered a release phase')
            self.phase = pilot.phase
            self.phases[self.phase] = observed_at
            self.phase_raw[self.phase] = now()
            log(self.timeline, {'phase': self.phase, 'observed_mono': observed_at,
                                'observed_ack_clock_mono': self.phase_raw[self.phase]})
            print(f'[copter release] {self.phase.replace("_", " ")}', flush=True)
        if self.phase not in self.mode_changes or self.phase in self.sent_at:
            return
        previous_mode, next_mode = self.mode_changes[self.phase]
        hb, pose, landed = (observed.get(name) for name in ('HEARTBEAT', 'GLOBAL_POSITION_INT', 'EXTENDED_SYS_STATE'))
        need(pilot.active and pilot.prepared and self.channel.peer and
             self.channel.latest.get('HEARTBEAT'), 'exact independent pilot channel unavailable')
        need(hb and pose and landed and observed_at-hb['received_mono']<=2 and observed_at-pose['received_mono']<=.5 and
             observed_at-landed['received_mono']<=2 and hb['base_mode'] & 128 and hb['custom_mode']==previous_mode and
             landed['landed_state']==2 and pose['relative_alt']>=4000,
             f'independent fresh airborne mode {previous_mode} required for pilot transition')
        proof = inventory(pilot.sitl)
        need(exact_peer(proof, self.channel.peer, self.port), 'pilot endpoint no longer belongs to owned SITL')
        message = self.codec.MAVLink_command_long_message(
            1, 1, self.codec.MAV_CMD_DO_SET_MODE, 0,
            self.codec.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, next_mode, 0, 0, 0, 0, 0)
        packet = message.pack(self.encoder)
        sent_at = time.monotonic()
        sent_raw = now()
        need(self.channel.endpoint.sendto(packet, self.channel.peer)==len(packet), 'pilot takeover send failed')
        self.sent_at[self.phase] = sent_at
        self.sent_raw[self.phase] = sent_raw
        log(self.timeline, {'event': 'pilot_mode_handoff', 'sent_mono': sent_at,
                            'sent_ack_clock_mono': sent_raw,
                            'system': OPERATOR_SYSTEM, 'component': OPERATOR_COMPONENT, 'mode': next_mode,
                            'peer': self.channel.peer, 'wire_hex': packet.hex(), 'owned_socket_snapshot': proof})

    def close(self):
        self.stack.close()


def fields(value):
    return dict(part.split('=', 1) for part in value.split(',') if '=' in part)


def parameter_read(rows):
    request = [row for row in rows if row.get('type')=='request' and row.get('key')=='MAVMOOS_PARAM_REQUEST']
    need(len(request)==1 and fields(request[0]['value'])=={'id':'release-wind','action':'read','name':'SIM_WIND_SPD'},
         'missing sole allowlisted parameter request')
    for key in ('MAVMOOS_PARAM_TX_RESULT', 'MAVMOOS_PARAM_VALUE', 'MAVMOOS_PARAM_RESULT'):
        need(all(row['elapsed']>=request[0]['elapsed'] for row in rows if row.get('key')==key),
             'parameter outcome precedes the request')
    tx = [fields(row['value']) for row in rows if row.get('key')=='MAVMOOS_PARAM_TX_RESULT']
    value = [fields(row['value']) for row in rows if row.get('key')=='MAVMOOS_PARAM_VALUE']
    replies = [fields(row['value']) for row in rows if row.get('key')=='MAVMOOS_PARAM_RESULT']
    need(len(tx)==1 and tx[0].get('id')=='release-wind/initial' and tx[0].get('status')=='LOCAL_ACCEPTED' and
         tx[0].get('late')=='false' and tx[0].get('clock_valid')=='true', 'missing bounded parameter read handoff')
    need(len(value)==1 and value[0].get('id')=='release-wind' and value[0].get('name')=='SIM_WIND_SPD' and
         value[0].get('phase')=='read' and float(value[0]['value'])==0, 'wrong simulator wind readback')
    need(replies and all(row.get('id')=='release-wind' and row.get('action')=='read' and
                         row.get('write_may_have_applied')=='false' for row in replies) and
         replies[-1].get('status')=='COMPLETED', 'parameter read terminal proof missing')
    return {'name':'SIM_WIND_SPD', 'value':0, 'local_handoffs':len(tx), 'terminal':'COMPLETED'}


def stationkeeping(packets, phases):
    start, end = phases['station_settle']+2, phases['prepare_hold']
    positions = [row for row in packets if row['mavpackettype']=='GLOBAL_POSITION_INT' and
                 start<=row['received_mono']<=end]
    need(len(positions)>=10 and positions[-1]['received_mono']-positions[0]['received_mono']>=1.5,
         'insufficient independent StationKeep position window')
    origin = positions[0]
    cosine = math.cos(math.radians(origin['lat']/1e7))
    drift = max(math.hypot((row['lat']-origin['lat'])*.01113195, (row['lon']-origin['lon'])*.01113195*cosine)
                for row in positions)
    speed = max(math.hypot(row['vx'],row['vy'])/100 for row in positions)
    height = [row['relative_alt']/1000 for row in positions]
    origin_lat, origin_lon = 423584560, -710875890
    cosine_origin = math.cos(math.radians(origin_lat/1e7))

    def waypoint_error(row, east, north):
        x = (row['lon']-origin_lon)*.01113195*cosine_origin
        y = (row['lat']-origin_lat)*.01113195
        return math.hypot(x-east,y-north)

    route = [row for row in packets if row['mavpackettype']=='GLOBAL_POSITION_INT' and
             phases['route']<=row['received_mono']<phases['station_settle']]
    turn = min((waypoint_error(row,0,7) for row in route), default=float('inf'))
    final = max(waypoint_error(row,5,7) for row in positions)
    need(turn<=2 and final<=1.6, f'route or final station position not corroborated ({turn:.3f}, {final:.3f} m)')
    need(drift<=1.5 and speed<=.65 and min(height)>=4 and max(height)-min(height)<=.5,
         f'independent StationKeep bounds exceeded (drift={drift:.3f}, speed={speed:.3f})')
    return {'stock_behavior':'route_finished_hold', 'samples':len(positions), 'drift_m':drift,
            'closest_first_waypoint_m':turn, 'maximum_final_station_error_m':final,
            'maximum_speed_m_s':speed, 'minimum_height_m':min(height), 'maximum_height_m':max(height)}


def logged_wind(path):
    from pymavlink import mavutil
    reader = mavutil.mavlink_connection(str(path))
    values = set()
    try:
        while (message := reader.recv_match(type='PARM')) is not None:
            name = message.Name.decode() if isinstance(message.Name, bytes) else message.Name
            if name=='SIM_WIND_SPD':
                values.add(float(message.Value))
    finally:
        reader.close()
    need(values=={0}, 'closed DataFlash did not corroborate the pinned simulator wind value')
    return sorted(values)


def audit(out, pilot, packets, result):
    from operations.run_operations import events
    rows = events(out/'workflow.log')
    assertions = {row['key']: row['value'] for row in rows if row.get('type')=='assertion'}
    read = parameter_read(rows)
    counts = fields(assertions['parameter_read_during_route'])
    read.update(nav_before=int(counts['nav_before']), nav_after=int(counts['nav_after']))
    need(0<read['nav_before']<read['nav_after'], 'active navigation counter did not progress beside read')
    need(any(row.get('type')=='stock_helm' and row.get('key')=='ROUTE_COMPLETE' and row.get('value')=='true'
             for row in rows) and 'stock_stationkeeping' in assertions, 'no stock route completion evidence')
    phases = pilot.phases
    station = stationkeeping(packets, phases)
    ack = [row for row in pilot.channel.messages if row['mavpackettype']=='COMMAND_ACK' and
           row['command']==pilot.codec.MAV_CMD_DO_SET_MODE]
    sent = list(pilot.sent_at.values())
    sent_raw = list(pilot.sent_raw.values())
    need(len(ack)==len(sent)==len(pilot.mode_changes) and all(
        reply['result']==pilot.codec.MAV_RESULT_ACCEPTED and
        (reply.get('target_system'),reply.get('target_component'))==(OPERATOR_SYSTEM,OPERATOR_COMPONENT) and
        sent_raw[index]<=reply['mono']<(sent_raw[index+1] if index+1<len(sent_raw) else pilot.phase_raw['reacquire'])
        for index,reply in enumerate(ack)), 'missing ordered addressed FC mode acknowledgements')
    hearts = [row for row in packets if row['mavpackettype']=='HEARTBEAT']
    loiter = [row for row in hearts if row['custom_mode']==5 and row['base_mode'] & 128 and
              sent[0]<=row['received_mono']<=phases['reacquire']]
    need(loiter and any(row['custom_mode']==4 and row['received_mono']>=phases['reacquire'] for row in hearts)
         and 'no_automatic_recapture' in assertions and
         not any(row.get('type')=='command' and fields(row['value']).get('id')=='workflow-hold' for row in rows),
         'pilot mode takeover or separate reacquisition not independently corroborated')
    needs_resume = ('normal_handoff_automatic_resume', 'expired_handoff_requires_acquire',
                    'expired_handoff_explicit_reacquire')
    need(all(key in assertions for key in needs_resume), 'required handoff and reacquisition proof missing')
    proofs = {'lifecycle': result['independent_monitor'],
            'short_handoff': {'automatic_resume': assertions[needs_resume[0]]},
            'expired_handoff': {'inhibition':assertions[needs_resume[1]], 'explicit_reacquire':assertions[needs_resume[2]]},
            'stationkeeping':station,
            'pilot_takeover':{'command':pilot.codec.MAV_CMD_DO_SET_MODE, 'mode':5, 'accepted_ack':ack[0],
                              'armed_loiter_heartbeats':len(loiter), 'stale_navigation_fenced':True,
                              'explicit_reacquire':True},
            'in_flight_parameter_read':read}
    if 'external_guided_handback' in pilot.mode_changes:
        guided = [row for row in hearts if row['custom_mode']==4 and row['base_mode'] & 128 and
                  sent[1]<=row['received_mono']<phases['reacquire']]
        need(guided and 'pilot_guided_handback_fenced' in assertions,
             'pilot Guided handback before explicit acquisition was not corroborated')
        proofs['pilot_guided_handback'] = {'accepted_ack':ack[1], 'armed_guided_heartbeats_before_acquire':len(guided),
                                           'fence_assertion':assertions['pilot_guided_handback_fenced']}
    return proofs


def profile_receipt(result, native):
    firmware = result['firmware_identity']
    return {'schema':1, 'case':result.get('release_case','copter_operator'), 'result':result['result'],
            'source_revision':result['source_revision'],
            'platform':f'{platform.system().lower()}-{platform.machine().lower().replace("aarch64", "arm64")}',
            'moos_revision':result['moos_revision'], 'helm_sha256':result['helm_sha256'],
            'app_sha256':result['app_sha256'],
            'firmware':{'revision':firmware['source_revision'], 'native_sha256':firmware['native_sha256'], 'frame':'quad'},
            'proofs':result.get('proofs',{}), 'cleanup_proven':result['cleanup_proven'],
            'native_receipt':{'path':native.name, 'sha256':sha(native)}}


def original_files(output):
    """Index only bounded, stable original files from the private run directory."""
    entries = {}
    total = 0
    for path in sorted(output.rglob('*')):
        name = path.relative_to(output).as_posix()
        need(not path.is_symlink(), f'unexpected symbolic link in release evidence: {name}')
        if not path.is_file() or name in ('result.json', 'profile-result.json'):
            continue
        info = path.stat()
        total += info.st_size
        need(len(entries)<256 and total<=64*1024*1024, 'release evidence index exceeds its bound')
        entries[name] = {'bytes':info.st_size, 'sha256':sha(path)}
        after = path.stat()
        need((info.st_size,info.st_mtime_ns,info.st_ino)==(after.st_size,after.st_mtime_ns,after.st_ino),
             f'original release evidence changed during indexing: {name}')
    return entries
