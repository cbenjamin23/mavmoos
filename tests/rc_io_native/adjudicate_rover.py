#!/usr/bin/env python3
"""Separate offline adjudication of the retained Rover sampling-comparison failure.

Never changes the original result or starts a simulator. The final stationary
window qualifies all output values; moving output matches are partial evidence.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import tempfile
from RcNativeAudit import audit, corroborate, parse
from replay_rc_run import load_closed_archive


def input_values(proof, packets, records):
    rc = [row for row in packets if row.get('mavpackettype') == 'RC_CHANNELS'];result = []
    for window in proof['windows']:
        matches = [];logs = []
        for sample in window['input_samples']:
            data = sample['data'];stamp = data['time_boot_ms'];values = [row['raw_us'] for row in data['channels']]
            near = [row for row in rc if abs(row['time_boot_ms'] - stamp) <= 350 and row['chancount'] == data['reported_channel_count'] and row['rssi'] == data['rssi']['raw'] and all(row.get(f'chan{i}_raw') == values[i - 1] for i in range(1, 19))]
            if not near:raise ValueError('RC input phase values not corroborated')
            matches.append(min(near, key=lambda row: abs(row['time_boot_ms'] - stamp))['time_boot_ms'])
            near = [row for row in records['RCIN'] if abs(row['TimeUS'] / 1000 - stamp) <= 350 and all(row.get(f'C{i}') == values[i - 1] for i in range(1, 9))]
            if not near:raise ValueError('DataFlash RCIN phase values not corroborated')
            logs.append(min(near, key=lambda row: abs(row['TimeUS'] / 1000 - stamp))['TimeUS'])
        if len(set(matches)) < 2 or len(set(logs)) < 2:raise ValueError('RC input corroboration did not advance')
        result.append({'fixture_input_phase': window['phase'], 'all_selected_input_values_match': True,
                       'unique_moos_observations': len({row['observation_id'] for row in window['input_samples']}),
                       'fc_boot_ms': sorted(set(matches)), 'RCIN_TimeUS': sorted(set(logs))})
    return result


def moving_coverage(window, packets, records):
    wire = [row for row in packets if row.get('mavpackettype') == 'SERVO_OUTPUT_RAW'];ports = {}
    unique = {(row['session_epoch'], row['observation_id']): row for row in window['servo_samples']}
    for sample in unique.values():
        data = sample['data'];stamp = data['time_usec'];port = data['port'];values = [row['raw_us'] for row in data['channels']]
        group = ports.setdefault(port, {'unique_observations': 0, 'exact_fc_ids': [], 'exact_RCOU_ids': [], 'exact_both_ids': [], 'mismatches': []})
        group['unique_observations'] += 1
        near = [row for row in wire if row['port'] == port and abs(row['time_usec'] - stamp) <= 350000]
        exact = [row for row in near if all(value is None or row.get(f'servo{i}_raw', 0) == value for i, value in enumerate(values, 1))]
        keys = [(i, f'C{port * 16 + i}') for i in range(1, 17) if values[i - 1] is not None and any(f'C{port * 16 + i}' in row for row in records['RCOU'])]
        logged = [row for row in records['RCOU'] if keys and abs(row['TimeUS'] - stamp) <= 350000 and all(row.get(key) == values[i - 1] for i, key in keys)]
        if exact:group['exact_fc_ids'].append(sample['observation_id'])
        if logged:group['exact_RCOU_ids'].append(sample['observation_id'])
        if exact and logged:group['exact_both_ids'].append(sample['observation_id'])
        if not exact:
            closest = min(near, key=lambda row: abs(row['time_usec'] - stamp)) if near else None
            group['mismatches'].append({'observation_id': sample['observation_id'], 'app_reported_fc_time_usec': stamp,
                'app_reported_raw_us': values, 'nearest_independent_time_usec': closest['time_usec'] if closest else None,
                'nearest_source_delta_us': closest['time_usec'] - stamp if closest else None,
                'nearest_independent_raw_us': [closest.get(f'servo{i}_raw', 0) for i in range(1, 17)] if closest else None})
    if not ports or any(len(group['exact_both_ids']) < 2 for group in ports.values()):raise ValueError('moving window lacks two independently exact source-value matches per port')
    for group in ports.values():group['full_cross_link_equality_qualified'] = False
    return {'fixture_input_phase': window['phase'], 'published_servo_samples': len(window['servo_samples']),
            'scope': 'Partial exact value corroboration only; independent GCS senders sample a changing FC output at different source times', 'ports': ports}


def stationary_disarmed(window, packets, records):
    samples = window['servo_samples'];stamps = sorted({row['data']['time_usec'] for row in samples})
    if len(stamps) < 3 or not 2 <= (stamps[-1] - stamps[0]) / 1e6 <= 10:raise ValueError('final servo source window missing/too short or wrapping')
    begin, end = stamps[0] / 1000, stamps[-1] / 1000
    positions = [row for row in packets if row.get('mavpackettype') == 'GLOBAL_POSITION_INT' and begin - 100 <= row['time_boot_ms'] <= end + 100]
    if len({row['time_boot_ms'] for row in positions}) < 10 or (max(row['time_boot_ms'] for row in positions) - min(row['time_boot_ms'] for row in positions)) < 2000:
        raise ValueError('final stationary window lacks advancing independent position data')
    if any(math.hypot(row['vx'], row['vy']) / 100 > .1 or abs(row['vz']) / 100 > .1 for row in positions):raise ValueError('final observed vehicle not stationary within0.1m/s')
    hearts = [row for row in packets if row.get('mavpackettype') == 'HEARTBEAT']
    paired = []
    for position in positions:
        prior = [row for row in hearts if 0 <= position['received_mono'] - row['received_mono'] <= 2]
        if not prior:raise ValueError('no fresh passive heartbeat for final position')
        heart = max(prior, key=lambda row: row['received_mono'])
        if heart['base_mode'] & 128 or heart['custom_mode'] != 4:raise ValueError('final independent heartbeat not disarmed native Hold')
        paired.append(heart['received_mono'])
    arms = [row for row in records['ARM'] if row['TimeUS'] <= stamps[0]]
    if not arms or max(arms, key=lambda row: row['TimeUS'])['ArmState'] != 0 or any(row['ArmState'] != 0 for row in records['ARM'] if stamps[0] <= row['TimeUS'] <= stamps[-1]):
        raise ValueError('DataFlash does not corroborate final disarmed window')
    modes = [row for row in records['MODE'] if row['TimeUS'] <= stamps[0]]
    if not modes or max(modes, key=lambda row: row['TimeUS'])['ModeNum'] != 4:raise ValueError('DataFlash final mode not native Hold')
    return {'source_boot_window_usec': [stamps[0], stamps[-1]], 'source_span_s': (stamps[-1] - stamps[0]) / 1e6,
            'unique_servo_observations': len({row['observation_id'] for row in samples}), 'independent_position_samples': len(positions),
            'independent_position_span_s': (max(row['time_boot_ms'] for row in positions) - min(row['time_boot_ms'] for row in positions)) / 1000,
            'maximum_horizontal_speed_m_s': max(math.hypot(row['vx'], row['vy']) / 100 for row in positions),
            'maximum_vertical_speed_m_s': max(abs(row['vz']) / 100 for row in positions),
            'distinct_fresh_disarmed_heartbeat_receipts': len(set(paired)), 'FC_mode': 4, 'DataFlash_disarmed': True,
            'clock_domains': 'FC boot stamps delimit source interval; heartbeat/GPI receipt ages use the same passive-Python monotonic clock; consumer/app clocks are not subtracted'}


def adjudicate(result, events, packets, records, workflow):
    if result.get('result') != 'FAIL' or result.get('error') != 'per-port servo command values not independently corroborated':raise ValueError('not the preserved sampling-comparison failure')
    if result.get('artifact_drift') != [] or result.get('owned_processes_remaining') != [] or not result.get('ports_clear') or any(value is not True for value in result['ports_clear'].values()):raise ValueError('original source/resource qualification failed')
    if result.get('artifacts_before') != result.get('artifacts_after'):raise ValueError('original protected source or artifact drift')
    if not any(row.get('type') == 'result' and row.get('key') == 'PASS' for row in workflow) or any(row.get('type') == 'result' and row.get('key') == 'FAIL' for row in workflow):raise ValueError('existing control workflow did not pass')
    independent = result['independent_monitor']
    if independent.get('final_disarmed') is not True or independent['maximum_home_distance_m'] < 4 or not 0 <= independent['final_home_distance_m'] <= 2.5 or not 0 <= independent['final_speed_m_s'] <= .4:raise ValueError('existing independent motion/return outcome not qualified')
    armed = next((row for row in records['ARM'] if row['ArmState'] == 1), None)
    if armed is None or armed.get('Forced', 0) != 0:raise ValueError('normal native arming not corroborated')
    manual = next((row for row in records['MODE'] if row['ModeNum'] == 0 and row['TimeUS'] > armed['TimeUS']), None)
    recovered = next((row for row in records['MODE'] if row['ModeNum'] == 15 and manual and row['TimeUS'] > manual['TimeUS']), None)
    if manual is None or recovered is None or manual.get('Rsn') != 1:raise ValueError('DataFlash native RC-reason Manual and explicit Guided recovery not corroborated')
    neutral = [row for row in records['RCIN'] if manual['TimeUS'] <= row['TimeUS'] < recovered['TimeUS']]
    if not neutral or any(row['C1'] != 1500 or row['C3'] != 1500 or row['C8'] != 1000 for row in neutral):raise ValueError('native RC Manual drive inputs not neutral/correct')
    proof = audit(events, mode='SITL_CONTROL', duration=210)
    final = proof['windows'][-1]
    stationary = stationary_disarmed(final, packets, records)
    all_inputs = input_values(proof, packets, records)
    final_values = corroborate({'windows': [final]}, packets, records)
    moving = [moving_coverage(window, packets, records) for window in proof['windows'][:-1]]
    return {'schema': 1, 'result': 'PASS_SCOPED_OBSERVATIONS', 'original_runner_result': 'FAIL', 'new_native_execution': False,
            'scope': 'Existing Rover control workflow coexistence, all discrete RC input windows, and all selected final stationary/disarmed servo values',
            'all_RC_input_windows': all_inputs, 'final_stationary_disarmed': stationary, 'final_all_servo_values': final_values,
            'moving_servo_partial_corroboration': moving,
            'control_workflow': {'unchanged_probe_result': 'PASS', 'maximum_home_distance_m': independent['maximum_home_distance_m'],
                                 'final_home_distance_m': independent['final_home_distance_m'], 'final_speed_m_s': independent['final_speed_m_s'],
                                 'manual_mode_reason': manual['Rsn'], 'neutral_manual_RCIN_samples': len(neutral), 'normal_arm_then_disarm': True},
            'limits': ['Fixture input phase labels do not identify FC mode.', 'Moving servo frames on separately scheduled GCS links are not the same source frame; full equality is not claimed.',
                       'RCOU and SERVO_OUTPUT_RAW are FC command observations, not physical actuator feedback.',
                       'Short-run FC boot stamps only; no remote clock synchronization, physical route or pilot/authority inference.']}


def extract_dataflash(content, result):
    # Only the selected verified BIN bytes are written to a private temporary
    # file. No archive paths are extracted and no native executable is started.
    from pymavlink import DFReader
    logs = [name for name in content if name.endswith('.BIN')]
    if len(logs) != 1:raise ValueError('expected one closed native DataFlash log')
    digest = hashlib.sha256(content[logs[0]]).hexdigest()
    if digest != result['independent_dataflash']['sha256']:raise ValueError('closed DataFlash hash differs from original result')
    records = {name: [] for name in ('MODE', 'ARM', 'RCIN', 'RCOU')}
    with tempfile.TemporaryDirectory(prefix='mavmoos-rc-offline-') as temporary:
        path = Path(temporary) / 'native.BIN';path.write_bytes(content[logs[0]])
        reader = DFReader.DFReader_binary(str(path))
        while True:
            message = reader.recv_match(type=list(records))
            if message is None:break
            records[message.get_type()].append(message.to_dict())
        if hashlib.sha256(path.read_bytes()).hexdigest() != digest:raise ValueError('offline DataFlash copy changed')
    return records, digest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--archive', type=Path, required=True);parser.add_argument('--expected-sha256', required=True);parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():raise RuntimeError('refuse to replace adjudication output')
    try:
        original, content, digest = load_closed_archive(args.archive, args.expected_sha256)
        records, bin_sha = extract_dataflash(content, original)
        events = [parse(line) for line in content['rc-probe.jsonl'].decode().splitlines() if line.startswith('{')]
        packets = [parse(line) for line in content['fc-monitor.jsonl'].decode().splitlines() if line]
        workflow = [parse(line) for line in content['workflow.jsonl'].decode().splitlines() if line.startswith('{')]
        result = adjudicate(original, events, packets, records, workflow)
        result.update(archive_sha256=digest, original_result_sha256=hashlib.sha256(content['result.json']).hexdigest(), closed_DataFlash_sha256=bin_sha)
    except Exception as error:result = {'schema': 1, 'result': 'FAIL', 'reason': str(error), 'new_native_execution': False}
    args.output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({key: value for key, value in result.items() if key not in ('all_RC_input_windows', 'final_all_servo_values', 'moving_servo_partial_corroboration')}, indent=2))
    return int(result['result'] != 'PASS_SCOPED_OBSERVATIONS')


if __name__ == '__main__':raise SystemExit(main())
