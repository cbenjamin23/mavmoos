#!/usr/bin/env python3
"""Reconcile an advanced Copter frame receipt with its raw owned evidence."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

from pymavlink import DFReader
from pymavlink.dialects.v20 import ardupilotmega

from copter_frame_fixture import STANDARD_COPTER_FRAMES, one_copter_frame_log
from control.catalog_evidence import observed_catalog

sys.path.insert(0, str(Path(__file__).resolve().parent/'operations'))
from run_operations import (catalog_evidence, events, flight_evidence, helm_parity_evidence,
                            operation_evidence, parity_evidence, precision_motion_evidence,
                            PRECISION_BEACON)

sys.path.insert(0, str(Path(__file__).resolve().parent/'mission_execution'))
from execution_evidence import audit_typed_dataflash, audit_typed_fc, audit_typed_probe, parse
from run_native import safe


OPERATIONS_MESSAGES = {'HEARTBEAT', 'GLOBAL_POSITION_INT', 'EXTENDED_SYS_STATE', 'STATUSTEXT'}
TYPED_MESSAGES = OPERATIONS_MESSAGES | {'MISSION_CURRENT', 'MISSION_ITEM_REACHED',
                                       'COMMAND_ACK', 'HOME_POSITION', 'AUTOPILOT_VERSION'}


def require(condition, reason):
    if not condition:
        raise ValueError(reason)


def json_value(value):
    return json.loads(json.dumps(value))


def sha(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def independent_rows(directory, expected_type, typed):
    raw = directory/('fc-observer.mavlink' if typed else 'fc-monitor.mavlink')
    trace = raw.with_suffix('.jsonl')
    clock_key = 'mono' if typed else 'received_mono'
    decoder = ardupilotmega.MAVLink(None)
    decoder.robust_parsing = True
    allowed = TYPED_MESSAGES if typed else OPERATIONS_MESSAGES
    prefix, valid, selected = b'', 0, []
    with raw.open('rb') as stream:
        while block := stream.read(65536):
            for message in decoder.parse_buffer(block) or []:
                kind = message.get_type()
                if kind == 'BAD_DATA':
                    prefix += bytes(message.data)
                    require(not valid and (b'0 '*3).startswith(prefix), 'invalid independent MAVLink bytes')
                    continue
                require((message.get_srcSystem(), message.get_srcComponent()) == (1, 1),
                        'unexpected source on independently owned FC stream')
                valid += 1
                if kind == 'HEARTBEAT':
                    require((message.type, message.autopilot) == (expected_type, 3), 'wrong Copter heartbeat')
                if kind not in allowed:
                    continue
                row = message.to_dict()
                if typed:
                    wire = bytes(message.get_msgbuf())
                    row.update(system=1, component=1, wire_magic=wire[0], wire_payload_length=wire[1])
                selected.append(safe(row) if typed else row)
    require(valid > 0 and prefix in (b'', b'0 '*3) and decoder.buf_len() == 0,
            'incomplete independent MAVLink stream')
    rows = [json.loads(line) for line in trace.read_text().splitlines()]
    require(selected == [{key: value for key, value in row.items() if key != clock_key} for row in rows],
            'independent decoded MAVLink frames differ from saved trace')
    return rows, valid


def replay(directory):
    result = json.loads((directory/'result.json').read_text())
    frame = result['frame']
    expected_type, expected_class = STANDARD_COPTER_FRAMES[frame]
    typed = result.get('scenario') == 'auto_typed_lifecycle'
    require(frame in ('hexa', 'octa') and result['result'] == 'PASS' and result['hardware'] is False and
            result['command'][1:3] == ['--model', frame] and result['cleanup_proven'] and
            all(result['ports_clear'].values()) and result['owned_processes_remaining'] == [] and
            result['artifact_drift'] == [], 'owned frame, cleanup or custody receipt differs')
    parameters = one_copter_frame_log(directory/'logs', frame)
    require(result['frame_identity'] == {'expected_mav_type': expected_type, 'expected_frame_class': expected_class,
                                        'dataflash_parameters': parameters}, 'frame parameters differ')
    rows, total = independent_rows(directory, expected_type, typed)
    log, = sorted((directory/'logs').glob('*.BIN'))
    if typed:
        records = [parse(line) for line in (directory/'probe.jsonl').read_text().splitlines() if line.startswith('{')]
        probe = audit_typed_probe(records)
        require(probe == result['probe_evidence'], 'typed MOOS proof differs')
        catalog = observed_catalog(directory/'probe.jsonl', set(result['modules_requested']),
                                   native=True, helm=True, mode='SITL_CONTROL', operations_helm=True,
                                   mission_execution=True, mission_resume=True)
        require(catalog == result['catalog'], 'typed loaded capability catalog differs')
        times = {row['kind']: row['python_mono'] for row in result['events']}
        fc = audit_typed_fc(rows, times, result['monitor_finished_python_mono'], probe['stored_plan'])
        require(fc == result['independent_fc'], 'independent typed mission effect differs')
        reader = DFReader.DFReader_binary(str(log))
        arm, mode, commands, params = [], [], [], {}
        for row in iter(lambda: reader.recv_match(type=['ARM', 'MODE', 'CMD', 'PARM']), None):
            kind = row.get_type()
            if kind == 'PARM':
                name = row.Name.decode(errors='replace') if isinstance(row.Name, bytes) else row.Name
                if name in result['fixture_parameters']:
                    params[name] = float(row.Value)
            else:
                {'ARM': arm, 'MODE': mode, 'CMD': commands}[kind].append(row.to_dict())
        require(json_value(audit_typed_dataflash(arm, mode, commands, params, result['fixture_parameters'])) ==
                result['dataflash_evidence'], 'independent typed DataFlash effect differs')
    else:
        trace = events(directory/'workflow.log')
        # Older receipts used the accepted-ACK speed contract.
        speed_terminal = result['parity_evidence']['commands']['parity-reset-speed']['terminal']
        precision_terminal = result['parity_evidence']['commands']['parity-precision-on']['terminal']
        require(speed_terminal in ('ACCEPTED', 'UNCONFIRMED') and precision_terminal in ('ACCEPTED', 'UNCONFIRMED'),
                'unknown speed or precision result contract')
        require(catalog_evidence(trace, True) == result['capabilities_catalog'] and
                operation_evidence(trace, True) == result['operations'] and
                parity_evidence(trace, legacy_speed_ack=speed_terminal == 'ACCEPTED',
                                legacy_precision_ack=precision_terminal == 'ACCEPTED') == result['parity_evidence'] and
                helm_parity_evidence(trace) == result['parity_helm'], 'native operation or stock-helm receipt differs')
        fc = flight_evidence(rows, trace, result['monitor_finished_mono'], PRECISION_BEACON)
        require(fc == result['independent_fc'] and
                json_value(precision_motion_evidence(log)) == result['independent_precision_loiter'],
                'independent flight or Precision Loiter receipt differs')
    native_sha = (result['dataflash']['logs'][0]['sha256'] if typed else
                  result['independent_pilot_rcin']['sha256'])
    require(sha(log) == native_sha, 'onboard DataFlash bytes differ')
    return {'result': 'PASS', 'frame': frame, 'workflow': 'typed_auto' if typed else 'operator_parity',
            'source': result.get('source_ref', result.get('revision')), 'decoded_frames': total,
            'selected_fc_rows': len(rows), 'dataflash_sha256': native_sha}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    print(json.dumps(replay(parser.parse_args().directory), indent=2))
