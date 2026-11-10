"""Read-only bridge/FC-channel metrics for an archived concurrent fixture."""
import argparse
import hashlib
import json
from pathlib import Path


def fields(text):
    return dict(piece.strip().split('=', 1) for piece in text.split(',') if '=' in piece)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--fixture', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    events = []
    for line in (args.fixture / 'probe.jsonl').read_text().splitlines():
        try:
            events.append(json.loads(line))
        except json.JSONDecodeError:
            pass
    failure = next(e for e in events if e.get('type') == 'fail')
    sent = next(fields(e['value']) for e in events if e.get('type') == 'parameter_result' and 'id=concurrent-list,' in e['value'] and 'status=SENT,' in e['value'])
    fail_at, sent_at = failure['mono'], float(sent['at'])
    positions, updates, statuses = {}, {}, []
    logs = list(args.fixture.glob('logs/**/*.alog'))
    for path in logs:
        for line in path.read_text(errors='replace').splitlines():
            parts = line.split(None, 3)
            if len(parts) != 4 or parts[1] not in ['MAVMOOS_GLOBAL_POSITION', 'MAVMOOS_NAV_STATUS', 'MAVMOOS_STATUS']:
                continue
            data = fields(parts[3])
            if parts[1] == 'MAVMOOS_GLOBAL_POSITION' and data.get('present') == 'true':
                read_at = float(data['read_mono_s'])
                positions[read_at] = {'read_at': read_at, 'effective_at': float(data['rx_mono_s']), 'boot_ms': int(data['source_time'])}
            elif parts[1] == 'MAVMOOS_NAV_STATUS' and float(data.get('read_at', 0)) > 0:
                read_at = float(data['read_at'])
                updates[read_at] = {'read_at': read_at, 'effective_at': float(data['received_at'])}
            elif parts[1] == 'MAVMOOS_STATUS' and sent_at - .3 <= float(data['last_read_at']) <= fail_at + .3:
                statuses.append({key: float(data[key]) for key in ['last_read_at', 'budget_hits', 'queue_stale_dropped', 'queue_clock_dropped', 'rx_errors', 'max_poll_gap_s', 'max_queue_age_upper_s']})
    # NAV_STATUS retains the last actual position read on every module iteration.
    # The raw diagnostic is published only every0.2s in these fixtures; its next
    # sample can skip an earlier returning position, so do not call that a wire gap.
    before = max((p for p in updates.values() if p['read_at'] <= fail_at), key=lambda p: p['read_at'])
    after = min((p for p in updates.values() if p['read_at'] > fail_at), key=lambda p: p['read_at'])
    raw_before = max((p for p in positions.values() if p['read_at'] <= before['read_at'] + 1e-6), key=lambda p: p['read_at'])
    raw_after = min((p for p in positions.values() if p['read_at'] >= after['read_at'] - 1e-6), key=lambda p: p['read_at'])
    observer = [json.loads(line) for line in (args.fixture / 'fc-observer.jsonl').read_text().splitlines()]
    window = [p for p in observer if p.get('mavpackettype') == 'GLOBAL_POSITION_INT' and raw_before['boot_ms'] <= p['time_boot_ms'] <= raw_after['boot_ms']]
    indices = {int(fields(e['value'])['index']) for e in events if e.get('type') == 'parameter_value' and 'id=concurrent-list,' in e['value'] and 'phase=list,' in e['value']}
    result = {'scope': 'software/telemetry failure reconstruction, not a physical incident',
              'fixture': str(args.fixture), 'local_list_sent_at_cpp_steady': sent_at, 'failure_at_cpp_steady': fail_at,
              'failure': failure['value'], 'last_position_before_failure': before, 'next_position_after_failure': after,
              'position_gap_basis': 'distinct original MAVMOOS_NAV_STATUS.read_at/received_at updates; raw diagnostics alone are subsampled',
              'bridge_position_read_gap_s': after['read_at'] - before['read_at'],
              'bridge_position_effective_gap_s': after['effective_at'] - before['effective_at'],
              'raw_diagnostic_bracket': {'before': raw_before, 'after': raw_after,
                                         'sample_read_gap_s': raw_after['read_at'] - raw_before['read_at'],
                                         'sample_effective_gap_s': raw_after['effective_at'] - raw_before['effective_at'],
                                         'sample_boot_gap_ms': raw_after['boot_ms'] - raw_before['boot_ms'],
                                         'scope': 'coarser raw diagnostic samples bracketing the observed read gap; not necessarily first next packet'},
              'client_logged_distinct_indices': len(indices), 'last_client_logged_index': max(indices),
              'status_near_failure': statuses,
              'independent_fc_channel': {'window_alignment': 'FC time_boot_ms interval from the enclosing raw diagnostic bracket, not subtracting Python from C++ clocks',
                                        'position_samples': len(window), 'boot_ms': [p['time_boot_ms'] for p in window],
                                        'max_observed_python_receipt_gap_s': max((b['mono']-a['mono'] for a,b in zip(window,window[1:])), default=None)},
              'input_hashes': [{'path': str(p), 'sha256': hashlib.sha256(p.read_bytes()).hexdigest()} for p in [args.fixture/'probe.jsonl', args.fixture/'fc-observer.jsonl', *logs]]}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({key: result[key] for key in ['bridge_position_read_gap_s', 'bridge_position_effective_gap_s', 'raw_diagnostic_bracket', 'client_logged_distinct_indices', 'last_client_logged_index', 'independent_fc_channel']}, indent=2))


if __name__ == '__main__':
    main()
