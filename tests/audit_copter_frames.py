#!/usr/bin/env python3
"""Replay one standard ArduCopter frame qualification without a simulator."""
import argparse
from hashlib import sha256
import json
from pathlib import Path, PurePosixPath
import re
import subprocess

from pymavlink import DFReader, mavutil
from run_sitl import COPTER_FRAMES, frame_flight_proof, pilot_rcin_evidence, sha
from control.catalog_evidence import observed_catalog


def need(condition, message):
    if not condition:
        raise ValueError(message)


def independent_rows(directory):
    decoder = mavutil.mavlink.MAVLink(None)
    decoder.robust_parsing = True
    selected = {'HEARTBEAT', 'GLOBAL_POSITION_INT', 'EXTENDED_SYS_STATE', 'STATUSTEXT'}
    raw = decoder.parse_buffer((directory/'fc-monitor.mavlink').read_bytes()) or []
    bad = b''.join(bytes(msg.data) for msg in raw if msg.get_type() == 'BAD_DATA')
    need(bad in (b'', b'0 '*3), 'unexpected invalid independent MAVLink bytes')
    decoded = [msg.to_dict() for msg in raw if (msg.get_srcSystem(), msg.get_srcComponent()) == (1, 1)
               and msg.get_type() in selected]
    with (directory/'fc-monitor.jsonl').open() as stream:
        rows = [json.loads(line) for line in stream]
    need(decoded == [{key: value for key, value in row.items() if key != 'received_mono'} for row in rows],
         'captured independent MAVLink frames differ from decoded rows')
    return rows


def read(directory):
    result = json.loads((directory/'result.json').read_text())
    frame = result['frame']
    mav_type, frame_class = COPTER_FRAMES[frame]
    need(result['result'] == 'PASS' and result['hardware'] is False and result['cleanup_proven'] and
         not result['owned_processes_remaining'] and all(result['ports_clear'].values()) and
         result['inputs_sha256'] == result['inputs_sha256_after'] and result['command'][1:3] == ['--model', frame],
         'frame scope, qualification, protected bytes or cleanup differs')
    source = result['source_revision']
    root = Path(__file__).resolve().parents[1]
    need(re.fullmatch(r'[0-9a-f]{40}', source), 'invalid immutable source revision')
    for name, expected in result['inputs_sha256'].items():
        if name.startswith(('tests/', 'missions/', 'scripts/')):
            blob = subprocess.check_output(['git', '-C', root, 'show', f'{source}:{name}'], timeout=15)
            need(sha256(blob).hexdigest() == expected, f'archived fixture source differs: {name}')
    for name, entry in result['files'].items():
        relative = PurePosixPath(name)
        need(not relative.is_absolute() and '..' not in relative.parts, 'unsafe evidence member')
        path = directory/relative
        need(path.is_file() and path.resolve() == path and path.stat().st_size == entry['bytes'] and
             sha(path) == entry['sha256'], f'changed evidence {name}')
    catalog = observed_catalog(directory/'workflow.log', {'telemetry', 'arducopter_control'},
                               native=True, helm=True, mode='SITL_CONTROL',
                               frame_lifecycle='frame_control_surface' not in result)
    need(catalog == result['capabilities_catalog'], 'observed composition differs')
    rows = independent_rows(directory)
    hearts, positions, landed = ([row for row in rows if row['mavpackettype'] == name]
                                 for name in ('HEARTBEAT', 'GLOBAL_POSITION_INT', 'EXTENDED_SYS_STATE'))
    need({(row['type'], row['autopilot']) for row in hearts} == {(mav_type, 3)} and
         not hearts[-1]['base_mode'] & 128 and landed[-1]['landed_state'] == 1, 'independent identity or final ground differs')
    flight = frame_flight_proof(hearts, positions, landed)
    need(flight == result['independent_frame_flight'], 'independent flight report differs')
    log, = sorted((directory/'logs').glob('*.BIN'))
    reader = DFReader.DFReader_binary(str(log))
    params, arms, modes = {}, [], []
    while (message := reader.recv_match(type=['PARM', 'ARM', 'MODE'])) is not None:
        name = message.get_type()
        if name == 'PARM' and message.Name in ('FRAME_CLASS', 'FRAME_TYPE'):
            params[message.Name] = message.Value
        elif name == 'ARM':
            arms.append(message.ArmState)
        elif name == 'MODE':
            modes.append(message.ModeNum)
    need(params == {'FRAME_CLASS': frame_class, 'FRAME_TYPE': 0} and
         arms.count(1) >= 3 and arms.count(0) >= 3 and {4, 5, 6, 9} <= set(modes),
         'independent DataFlash frame, native mode or arm cycles differ')
    expected = result['independent_pilot_rcin']
    need(expected['input_sha256'] == sha(log) and
         all(expected[key] == value for key, value in pilot_rcin_evidence(log).items()), 'DataFlash pilot witness differs')
    return {'frame': frame, 'mav_type': mav_type, 'observed_modes': sorted(set(modes)),
            'independent_frames': len(rows), 'dataflash_frame': params,
            'arm_observations': arms.count(1), 'disarm_observations': arms.count(0),
            'return_homeward_m': flight['return_homeward_m']}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    print(json.dumps(read(parser.parse_args().directory.resolve()), indent=2))
