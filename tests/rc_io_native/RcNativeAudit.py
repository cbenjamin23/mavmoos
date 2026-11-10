#!/usr/bin/env python3
"""Strict offline RC evidence audit. No process, network or control operations.

Consumer elapsed seconds only delimit consumer windows. FC boot stamps only
pair FC reports/DataFlash in these short owned runs; no clock mapping or source
age guarantee is inferred. RCOU and SERVO_OUTPUT_RAW are command observations.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path


def parse(text):
    def reject(value):raise ValueError('nonfinite JSON ' + value)
    def finite(value):
        result = float(value)
        if not math.isfinite(result):raise ValueError('nonfinite JSON float')
        return result
    def unique(pairs):
        result = {}
        for key, value in pairs:
            if key in result:raise ValueError('duplicate JSON key ' + key)
            result[key] = value
        return result
    return json.loads(text, parse_constant=reject, parse_float=finite, object_pairs_hook=unique)


def number(value):
    if type(value) not in (int, float):raise ValueError('expected finite numeric value')
    try:valid = math.isfinite(value)
    except OverflowError:valid = False
    if not valid:raise ValueError('expected finite numeric value')
    return value


def integer(value, low, high):
    if type(value) is not int or not low <= value <= high:raise ValueError('integer outside schema bounds')
    return value


def quantity(value, known, raw, unit):
    if not isinstance(value, dict) or value.get('known') is not known or value.get('unit') != unit:
        raise ValueError('contradictory quantity metadata')
    if known:
        if type(value.get('value')) is not int or value['value'] != raw:raise ValueError('invented or changed reported quantity')
    elif value.get('value') is not None:raise ValueError('unknown quantity has a value')
    if not isinstance(value.get('reason'), str) or not value['reason']:raise ValueError('quantity reason missing')


def sample(record, family, timeout, seen):
    if record.get('schema') != 1 or record.get('family') != family:raise ValueError('wrong observation schema/family')
    session = integer(record.get('session_epoch'), 1, 2**64 - 1)
    for key in ('present', 'encoding_valid', 'fresh', 'age_is_upper_bound'):
        if type(record.get(key)) is not bool:raise ValueError('observation boolean missing or invalid')
    if record.get('source_age') != 'unknown' or record.get('source_quality') != 'unassessed':raise ValueError('invented source age/quality')
    if not record['present']:
        if record['fresh'] or record['encoding_valid'] or record.get('data') is not None or record.get('observation_id') != 0:
            raise ValueError('absent observation carries data/freshness/identifier')
        if any(record.get(key) is not None for key in ('effective_at', 'read_at', 'age_s', 'age_basis')):
            raise ValueError('absent observation carries timestamps')
        return False
    if record['encoding_valid'] is not True:raise ValueError('present native sample not encoding-valid')
    effective, read, age = (number(record.get(key)) for key in ('effective_at', 'read_at', 'age_s'))
    if effective < 0 or read < effective or age < 0 or age + 1e-6 < read - effective:raise ValueError('contradictory original acquisition metadata')
    if record['fresh'] and age > timeout:raise ValueError('fresh sample exceeds configured timeout')
    if record.get('age_basis') == 'QUEUE_EMPTY_FLOOR':
        if record['age_is_upper_bound'] is not True:raise ValueError('conservative queue floor lost')
    elif record.get('age_basis') == 'READ_ACQUISITION':
        if effective != read or record['age_is_upper_bound'] is not False:raise ValueError('read-acquisition contradiction')
    else:raise ValueError('unknown acquisition basis')
    oid = integer(record.get('observation_id'), 1, 2**64 - 1)
    signature = json.dumps([family, effective, read, record['age_basis'], record['data']], sort_keys=True)
    key = (session, oid)
    if key in seen and seen[key] != signature:raise ValueError('cached identifier renewed or changed original data/stamps')
    seen[key] = signature
    return True


def channels(data, family):
    rows = data.get('channels')
    length = 18 if family == 'RC_CHANNELS' else 16
    if not isinstance(rows, list) or len(rows) != length:raise ValueError('wrong channel slot count')
    version = integer(data.get('wire_version'), 1, 2)
    if data.get('pulse_validity') != 'unassessed':raise ValueError('invented pulse validity')
    if family == 'RC_CHANNELS':
        count = integer(data.get('reported_channel_count'), 0, 255)
        visible = min(18, count)
        if type(data.get('visible_channel_count')) is not int or data['visible_channel_count'] != visible or data.get('channels_beyond_message') is not (count > 18):
            raise ValueError('channel count truncation ambiguity lost')
        if data.get('channel_roles') != 'unmapped' or data.get('remote_clock') != 'boot_ms_unmapped_uint32':raise ValueError('invented channel role or clock mapping')
        integer(data.get('time_boot_ms'), 0, 2**32 - 1)
        rssi = data.get('rssi', {});raw = integer(rssi.get('raw'), 0, 255)
        quantity(rssi.get('quantity'), raw != 255, raw, 'device_dependent')
    else:
        integer(data.get('port'), 0, 255);integer(data.get('time_usec'), 0, 2**32 - 1)
        if data.get('remote_clock') != 'unmapped_uint32_us_no_epoch_inference' or data.get('output_kind') != 'fc_command_observation' or data.get('physical_feedback') is not False or data.get('actuator_mapping') != 'unmapped_port_and_slot':
            raise ValueError('invented servo feedback/mapping/clock')
        if type(data.get('extension_bytes_observed')) is not bool or (version == 1 and data['extension_bytes_observed']):raise ValueError('invalid extension evidence')
    for index, row in enumerate(rows, 1):
        if type(row.get('slot')) is not int or row['slot'] != index:raise ValueError('slot identity changed/duplicated')
        if family == 'RC_CHANNELS':
            raw = integer(row.get('raw_us'), 0, 65535);reported = index <= visible
            if row.get('reported_by_count') is not reported:raise ValueError('reported channel count mismatch')
            quantity(row.get('pwm_us'), reported, raw, 'us')
        else:
            if row.get('extension') is not (index > 8):raise ValueError('extension slot identity lost')
            available = index <= 8 or version == 2
            raw = integer(row.get('raw_us'), 0, 65535) if available else row.get('raw_us')
            if not available and raw is not None:raise ValueError('fabricated MAVLink1 extension value')
            quantity(row.get('pwm_us'), available and raw != 0, raw, 'us')
    return rows


def servo_ports(record, timeout, seen):
    if record.get('schema') != 1 or record.get('atomic') is not False or record.get('cache_capacity') != 8 or type(record.get('present')) is not bool:
        raise ValueError('invalid non-atomic servo aggregate')
    ports = record.get('ports')
    if not isinstance(ports, list) or len(ports) > 8 or record['present'] is not bool(ports):raise ValueError('servo cache bound/presence mismatch')
    previous = -1
    for item in ports:
        if item.get('session_epoch') != record.get('session_epoch'):raise ValueError('servo aggregate session mismatch')
        if not sample(item, 'SERVO_OUTPUT_RAW', timeout, seen):raise ValueError('empty servo port retained')
        channels(item['data'], 'SERVO_OUTPUT_RAW')
        port = item['data']['port']
        if port <= previous:raise ValueError('servo ports not distinct/sorted')
        previous = port
    return ports


def catalog(record, mode):
    if record.get('schema') != 1 or record.get('execution_mode') != mode or record.get('composition', {}).get('plugin_abi') != 10:
        raise ValueError('wrong native catalog execution/schema/ABI')
    modules = record['composition']['modules'];names = [item.get('name') for item in modules]
    expected = {'telemetry', 'rc_io'} if mode == 'RX_ONLY' else {'telemetry', 'parameters', 'ardurover_control', 'rc_io'}
    if len(names) != len(set(names)) or set(names) != expected:raise ValueError('unexpected native module composition')
    rc = next(item for item in modules if item['name'] == 'rc_io')
    if rc.get('profile') != 'ardupilot_rc_observations' or rc.get('control_provider') is not False or rc.get('request_owners') != []:
        raise ValueError('RC observer has effects or wrong profile')
    caps = rc.get('capabilities', [])
    if len(caps) != 2 or {item.get('name') for item in caps} != {'rc.input', 'rc.servo_output'}:
        raise ValueError('unexpected RC capabilities')
    if any(item.get('access') != 'observation' or item.get('request_variable') for item in caps):raise ValueError('RC capability effect path exposed')


def audit(rows, timeout=1.0, mode='RX_ONLY', duration=36.0):
    if not .25 <= number(timeout) <= 10 or not 5 <= number(duration) <= 220:raise ValueError('invalid audit timing')
    seen = {};inputs = [];outputs = [];states = [];runtime = [];catalogs = [];phases = [];end = None;last = -1
    for event in rows:
        elapsed = number(event.get('elapsed'))
        if not 0 <= elapsed <= duration + 1 or elapsed < last:raise ValueError('invalid/nonmonotonic consumer elapsed time')
        last = elapsed
        if event.get('type') == 'phase':
            if event.get('phase') not in {'A', 'B', 'C', 'D', 'INITIAL_HOLD', 'MANUAL_SWITCH'}:raise ValueError('unknown native fixture phase')
            phases.append((event['phase'], elapsed));continue
        if event.get('type') == 'end':
            if end is not None:raise ValueError('duplicate end marker')
            end = elapsed;continue
        if event.get('source') != 'iMAVLink' or not isinstance(event.get('value'), str):raise ValueError('unexpected publisher/type')
        key = event.get('key')
        if key == 'MAVMOOS_RC_INPUT':
            item = parse(event['value'])
            if sample(item, 'RC_CHANNELS', timeout, seen):channels(item['data'], 'RC_CHANNELS')
            inputs.append((elapsed, item))
        elif key == 'MAVMOOS_SERVO_OUTPUT':
            item = parse(event['value']);servo_ports(item, timeout, seen);outputs.append((elapsed, item))
        elif key == 'MAVMOOS_RC_IO_STATUS':
            item = parse(event['value'])
            if item.get('schema') != 1 or item.get('read_only') is not True or item.get('authoritative_for_control') is not False or item.get('atomic') is not False:raise ValueError('RC status grants authority/atomicity')
            if item.get('state') == 'BOUND' and (item.get('system_id') != 1 or item.get('component_id') != 1):raise ValueError('wrong source identity')
            states.append((elapsed, item))
        elif key == 'MAVMOOS_RUNTIME_STATUS':
            item = dict(part.split('=', 1) for part in event['value'].split(',') if '=' in part)
            if item.get('mode') != mode:raise ValueError('wrong runtime execution mode')
            if mode == 'RX_ONLY':
                if item.get('permission') != 'false' or item.get('control_profile') != 'none':raise ValueError('RX observer has control')
                if any(item.get(name) != '0' for name in ('tx_attempts', 'tx_accepted', 'nav_tx_accepted')):raise ValueError('nonzero/missing read-only egress counter')
                if item.get('armed') not in ('0', 'false') or item.get('in_air') not in ('0', 'false'):raise ValueError('Copter armed/airborne')
            runtime.append((elapsed, item))
        elif key == 'MAVMOOS_CAPABILITIES':
            item = parse(event['value']);catalog(item, mode);catalogs.append(item)
        else:raise ValueError('unexpected probe variable')
    if end is None or not runtime or not catalogs or not inputs or not outputs:raise ValueError('missing subscriber evidence/end marker')
    if len({json.dumps(item, sort_keys=True) for item in catalogs}) != 1:raise ValueError('changing catalog')
    windows = []
    expected = ['A', 'B', 'C', 'D'] if mode == 'RX_ONLY' else ['INITIAL_HOLD', 'MANUAL_SWITCH']
    if [phase for phase, _ in phases] != expected:raise ValueError('missing/reordered native RC phases')
    for index, (phase, begin) in enumerate(phases):
        finish = phases[index + 1][1] if index + 1 < len(phases) else end
        # Consumer-only stable window: every actual source sample also needs an
        # independent raw-value match below. Early-only observations cannot pass.
        width = min(3.0, (finish - begin) / 2)
        low, high = max(begin + .2, finish - width), finish - .1
        if high - low < .3:raise ValueError('native RC stable window too short')
        selected = [record for elapsed, record in inputs if low <= elapsed <= high and record.get('present') and record.get('fresh')]
        if len({record['observation_id'] for record in selected}) < 2 or len({record['read_at'] for record in selected}) < 2 or len({record['data']['time_boot_ms'] for record in selected}) < 2:
            raise ValueError('no advancing late RC acquisition in phase ' + phase)
        for record in selected:
            values = [row['raw_us'] for row in record['data']['channels']]
            expected_values = {1: 1500, 3: 1000, 4: 1500, 5: 1500} if mode == 'RX_ONLY' else {1: 1500, 3: 1500, 8: 1500 if phase == 'INITIAL_HOLD' else 1000}
            if phase == 'B':expected_values.update({1: 1100, 4: 1700})
            if phase == 'C':expected_values.update({1: 1900, 4: 1300})
            if any(values[slot - 1] != value for slot, value in expected_values.items()):raise ValueError('late RC values contradict phase ' + phase)
            if record['data']['visible_channel_count'] < 8:raise ValueError('native fixture input slots not reported')
        ports = {}
        for elapsed, aggregate in outputs:
            if low <= elapsed <= high:
                for record in aggregate['ports']:
                    if record['fresh']:ports.setdefault(record['data']['port'], []).append(record)
        if not ports or any(len({record['observation_id'] for record in records}) < 2 or len({record['read_at'] for record in records}) < 2 or len({record['data']['time_usec'] for record in records}) < 2 for records in ports.values()):
            raise ValueError('no advancing late per-port servo reports')
        if not any(low <= elapsed <= high and item.get('state') == 'BOUND' for elapsed, item in states):raise ValueError('RC source not bound in stable window')
        if len({elapsed for elapsed, item in runtime if low <= elapsed <= high and item.get('heartbeat_valid') in ('true', '1')}) < 2:
            raise ValueError('no late fresh heartbeat runtime observations')
        windows.append({'phase': phase, 'consumer_elapsed_window_s': [low, high], 'input_observation_ids': sorted({r['observation_id'] for r in selected}),
                        'servo_ports': sorted(ports), 'input_samples': selected, 'servo_samples': [r for values in ports.values() for r in values]})
    return {'schema': 1, 'result': 'pass', 'mode': mode, 'probe_duration_s': end,
            'scope': 'consumer-window metadata and advancing FC observations; remote value corroboration checked separately',
            'clock_domains': 'consumer elapsed and app monotonic never subtracted; remote FC stamps only pair same FC reports in this bounded run',
            'record_counts': {'inputs': len(inputs), 'outputs': len(outputs), 'status': len(states), 'runtime': len(runtime)}, 'windows': windows}


def corroborate(proof, packets, dataflash):
    """Check source values, not plausible ranges or merely publication counts."""
    rc = [p for p in packets if p.get('mavpackettype') == 'RC_CHANNELS']
    servo = [p for p in packets if p.get('mavpackettype') == 'SERVO_OUTPUT_RAW']
    rcin, rcou = dataflash.get('RCIN', []), dataflash.get('RCOU', [])
    if not rc or not servo or not rcin or not rcou:raise ValueError('missing independent RC/servo or DataFlash families')
    result = []
    for window in proof['windows']:
        input_matches = [];servo_matches = [];df_input = [];df_output = []
        for record in window['input_samples']:
            data = record['data'];stamp = data['time_boot_ms'];values = [row['raw_us'] for row in data['channels']]
            candidates = [row for row in rc if abs(row['time_boot_ms'] - stamp) <= 350 and row['chancount'] == data['reported_channel_count'] and row['rssi'] == data['rssi']['raw'] and all(row.get(f'chan{i}_raw') == values[i - 1] for i in range(1, 19))]
            if not candidates:raise ValueError('RC raw source values not independently corroborated')
            match = min(candidates, key=lambda row: abs(row['time_boot_ms'] - stamp));input_matches.append(match['time_boot_ms'])
            candidates = [row for row in rcin if abs(row['TimeUS'] / 1000 - stamp) <= 350 and all(row.get(f'C{i}') == values[i - 1] for i in range(1, 9))]
            if not candidates:raise ValueError('RCIN values do not corroborate late MOOS input')
            df_input.append(min(candidates, key=lambda row: abs(row['TimeUS'] / 1000 - stamp))['TimeUS'])
        for record in window['servo_samples']:
            data = record['data'];stamp = data['time_usec'];port = data['port'];values = [row['raw_us'] for row in data['channels']]
            candidates = [row for row in servo if row['port'] == port and abs(row['time_usec'] - stamp) <= 350000 and all(values[i - 1] is None or row.get(f'servo{i}_raw', 0) == values[i - 1] for i in range(1, 17))]
            if not candidates:raise ValueError('per-port servo command values not independently corroborated')
            match = min(candidates, key=lambda row: abs(row['time_usec'] - stamp));servo_matches.append([port, match['time_usec']])
            # DataFlash RCOU names FC output indices; mapping is justified by the
            # pinned sender's16-channel port blocks, not a general MAVLink rule.
            keys = [(i, f'C{port * 16 + i}') for i in range(1, 17) if values[i - 1] is not None and any(f'C{port * 16 + i}' in row for row in rcou)]
            if keys:
                candidates = [row for row in rcou if abs(row['TimeUS'] - stamp) <= 350000 and all(row.get(key) == values[i - 1] for i, key in keys)]
                if not candidates:raise ValueError('RCOU values contradict late servo command observation')
                df_output.append({'port': port, 'TimeUS': min(candidates, key=lambda row: abs(row['TimeUS'] - stamp))['TimeUS'], 'slots_compared': [i for i, _ in keys]})
        if len(set(input_matches)) < 2 or len(set(df_input)) < 2 or len({tuple(row) for row in servo_matches}) < 2 or not df_output:
            raise ValueError('independent corroboration lacks advancing late values')
        result.append({'phase': window['phase'], 'input_fc_boot_ms': sorted(set(input_matches)), 'rcin_TimeUS': sorted(set(df_input)),
                       'servo_port_time_usec': sorted({tuple(row) for row in servo_matches}), 'rcou_comparisons': df_output})
    return {'result': 'pass', 'windows': result, 'source_pair_tolerance_ms': 350,
            'servo_scope': 'FC command observations only; RCOU coverage limited to logged output indices; no physical actuator feedback',
            'clock_scope': 'same owned FC boot clock in a short no-wrap run; no app/consumer/source clock synchronization claim'}


def summary(proof):
    return {**proof, 'windows': [{key: value for key, value in window.items() if key not in ('input_samples', 'servo_samples')} for window in proof['windows']]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trace', type=Path, required=True);parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--mode', choices=('RX_ONLY', 'SITL_CONTROL'), default='RX_ONLY');parser.add_argument('--duration', type=float, default=36)
    args = parser.parse_args()
    try:result = summary(audit([parse(line) for line in args.trace.read_text().splitlines() if line.startswith('{')], mode=args.mode, duration=args.duration))
    except Exception as error:result = {'schema': 1, 'result': 'fail', 'reason': str(error)}
    result['trace_sha256'] = hashlib.sha256(args.trace.read_bytes()).hexdigest()
    args.output.write_text(json.dumps(result, indent=2) + '\n');print(json.dumps(result));return int(result['result'] != 'pass')


if __name__ == '__main__':raise SystemExit(main())
