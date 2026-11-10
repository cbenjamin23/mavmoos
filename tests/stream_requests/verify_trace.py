#!/usr/bin/env python3
"""Strictly check the pure scheduler's JSONL results, including negative controls."""
import argparse
import copy
import json
import math
from pathlib import Path

EXPECTED_CASES = set('''idle_requires_admission empty_plan_is_once_only
duplicate_minimum_and_stable_order canonical_capacity_is_atomic
raw_capacity_bounds_duplicate_work raw_capacity_boundary_is_valid
invalid_demands_do_not_partially_admit valid_wire_boundaries
start_cannot_renew_lifetime start_rejection_still_advances_expiry
single_inflight_and_exact_copy copied_witness_cannot_be_tampered
local_rejection_retries_and_stops_on_acceptance retry_backoff_does_not_block_other_demand
global_spacing_uses_post_return_time one_success_never_retried attempt_budget_is_fixed
whole_plan_capacity_and_attempt_work_bound expiry_fences_dequeued_work
accepted_receipt_after_deadline_is_not_retried_or_hidden
expired_waiting_work_never_consumes_attempt reset_fences_queued_and_dequeued_work
new_plan_cannot_rebind_old_witness old_actual_acceptance_remains_event_only
duplicate_receipt_does_not_repeat_effect forged_receipt_cannot_release_current_attempt
backwards_clock_latches_and_retires_work nonfinite_clocks_fail_closed
explicit_valid_reset_recovers_clock_fault unrepresentable_deadline_has_no_work
unrepresentable_pacing_has_no_work diagnostic_reason_is_bounded_and_json_safe
demand_accessor_is_bounded'''.split())
EXPECTED_LABELS = set('''idle empty dedup capacity raw_capacity fixed_bound inflight
recovered_local_loss exhausted bounded_plan expired_inflight accepted_late_receipt
reset unmatched_actual_acceptance clock_fault unrepresentable unrepresentable_pacing escaped'''.split())


def require(condition, message):
    if not condition:
        raise ValueError(message)


def reject_constant(value):
    raise ValueError(f'non-JSON number: {value}')


def finite_tree(value):
    if isinstance(value, float):
        require(math.isfinite(value), 'nonfinite number')
    elif isinstance(value, dict):
        for item in value.values():
            finite_tree(item)
    elif isinstance(value, list):
        for item in value:
            finite_tree(item)


def parse(text):
    result = [json.loads(line, parse_constant=reject_constant) for line in text.splitlines() if line]
    finite_tree(result)
    return result


def verify(records):
    require(records and records[-1].get('type') == 'complete', 'missing final completion')
    kinds = [r.get('type') for r in records]
    require(all(k in {'state', 'pass', 'complete'} for k in kinds), 'unexpected record')
    require(kinds.count('complete') == 1, 'duplicate completion')
    complete = records[-1]
    require(complete == {'type': 'complete', 'groups': len(EXPECTED_CASES), 'io': 'none'}, 'bad completion')
    passes = [r['case'] for r in records if r['type'] == 'pass']
    require(set(passes) == EXPECTED_CASES and len(passes) == len(EXPECTED_CASES), 'missing/duplicate test cases')
    rows = [r for r in records if r['type'] == 'state']
    require({r['label'] for r in rows} == EXPECTED_LABELS and len(rows) == len(EXPECTED_LABELS), 'missing/duplicate states')
    states = {r['label']: r['value'] for r in rows}
    allowed = {'QUEUED', 'RETRY_PENDING', 'IN_FLIGHT', 'LOCAL_ACCEPTED', 'EXHAUSTED', 'EXPIRED', 'CANCELLED'}
    for label, state in states.items():
        require(state['schema'] == 1 and state['remote_application'] == 'unconfirmed'
                and state['ack_correlation'] == 'not_inferred', f'{label}: false remote claim')
        items = state['items']
        require(state['demands'] == len(items) <= 32, f'{label}: demand capacity')
        require(len({i['message_id'] for i in items}) == len(items), f'{label}: duplicate canonical demand')
        counts = {name: 0 for name in allowed}
        for item in items:
            require(item['state'] in allowed and 0 <= item['attempt'] <= 3, f'{label}: state/attempt bound')
            require(0 <= item['message_id'] <= 0xffffff and 10000 <= item['interval_us'] <= 60000000,
                    f'{label}: invalid wire demand')
            require(item['expires'] == state['expires'] == state['admitted_at'] + 10, f'{label}: deadline renewed')
            if item['state'] == 'RETRY_PENDING':
                require(item['attempt'] in {1, 2}, f'{label}: invalid retry')
            if item['state'] == 'EXHAUSTED':
                require(item['attempt'] == 3, f'{label}: premature exhaustion')
            if item['accepted_receipt_late']:
                require(item['state'] == 'LOCAL_ACCEPTED' and item['deadline_elapsed'], f'{label}: late acceptance hidden')
            counts[item['state']] += 1
        require(state['handoff_attempts'] == sum(i['attempt'] for i in items) <= 96, f'{label}: work bound')
        require(state['queued'] == counts['QUEUED'] + counts['RETRY_PENDING'], f'{label}: queue counter')
        for key, name in [('local_accepted', 'LOCAL_ACCEPTED'), ('exhausted', 'EXHAUSTED'),
                          ('expired', 'EXPIRED'), ('cancelled', 'CANCELLED')]:
            require(state[key] == counts[name], f'{label}: {key} counter')
        require(counts['IN_FLIGHT'] <= int(state['in_flight']) <= 1, f'{label}: multiple in-flight')
        require(state['accepted_receipt_late'] == sum(i['accepted_receipt_late'] for i in items), f'{label}: late counter')
    recovered = states['recovered_local_loss']['items'][0]
    require(recovered['state'] == 'LOCAL_ACCEPTED' and recovered['attempt'] == 2, 'local loss was not recovered')
    require(states['exhausted']['items'][0]['state'] == 'EXHAUSTED', 'exhaustion missing')
    require(states['dedup']['items'][0]['interval_us'] == 50000, 'minimum interval not retained')
    require(states['fixed_bound']['items'][0]['expires'] == 110 and states['fixed_bound']['expired'] == 1,
            'original lifetime not retained')
    require(states['expired_inflight']['items'][0]['state'] == 'EXPIRED', 'expired in-flight work escaped')
    require(states['accepted_late_receipt']['accepted_receipt_late'] == 1, 'actual late acceptance erased')
    require(states['reset']['last_reset_retired'] == 2 and not states['reset']['started'], 'reset did not retire work')
    require(states['unmatched_actual_acceptance']['handoff_attempts'] == 1
            and states['unmatched_actual_acceptance']['local_accepted'] == 0, 'old receipt changed new generation')
    require(states['clock_fault']['state'] == 'CLOCK_FAULT' and states['clock_fault']['cancelled'] == 1,
            'clock fault not latched')
    for label in ['capacity', 'raw_capacity', 'unrepresentable', 'unrepresentable_pacing']:
        require(states[label]['state'] == 'REJECTED' and states[label]['demands'] == 0, f'{label}: rejected work escaped')
    return {'groups': len(passes), 'state_traces': len(rows), 'result': 'pass', 'scope': 'pure fake-clock state only'}


def negative_controls(records):
    controls = []
    def rejected(name, mutate):
        bad = copy.deepcopy(records)
        mutate(bad)
        try:
            verify(bad)
        except (ValueError, KeyError, TypeError, IndexError):
            controls.append(name)
        else:
            raise ValueError(f'negative control accepted: {name}')
    def item(rows, label):
        return next(r['value']['items'][0] for r in rows if r.get('label') == label)
    rejected('missing_completion', lambda r: r.pop())
    rejected('missing_case', lambda r: r.pop(next(i for i, row in enumerate(r) if row['type'] == 'pass')))
    rejected('duplicate_case', lambda r: r.insert(-1, next(row for row in r if row['type'] == 'pass')))
    rejected('fabricated_group_count', lambda r: r[-1].update(groups=99))
    rejected('missing_state_trace', lambda r: r.pop(0))
    rejected('renewed_expiry', lambda r: item(r, 'fixed_bound').update(expires=111))
    rejected('unbounded_attempts', lambda r: item(r, 'exhausted').update(attempt=4))
    rejected('lost_minimum_interval', lambda r: item(r, 'dedup').update(interval_us=100000))
    rejected('erased_late_acceptance', lambda r: item(r, 'accepted_late_receipt').update(accepted_receipt_late=False))
    rejected('fabricated_remote_application', lambda r: next(row['value'] for row in r if row.get('label') == 'dedup').update(remote_application='confirmed'))
    for text in ['{"value":NaN}', '{"value":Infinity}', '{"value":1e309}']:
        try:
            parse(text)
        except ValueError:
            controls.append('invalid_number_' + text.split(':', 1)[1][:-1])
        else:
            raise ValueError('nonfinite JSON accepted')
    return controls


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trace', type=Path)
    parser.add_argument('--negative-controls', action='store_true')
    args = parser.parse_args()
    records = parse(args.trace.read_text())
    result = verify(records)
    if args.negative_controls:
        result['negative_controls_rejected'] = negative_controls(records)
    print(json.dumps(result, sort_keys=True))


if __name__ == '__main__':
    main()
