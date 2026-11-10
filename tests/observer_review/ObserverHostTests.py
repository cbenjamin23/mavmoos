#!/usr/bin/env python3
"""Validate traces from real Runtime/MODULE integration with fake I/O only."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess

GROUPS = {
    'coalesced_frames_and_actual_ingress_filtering',
    'payload_routes_preserve_fc_authority_and_observation_age',
    'host_interval_union_has_no_control_authority',
    'receive_only_projection_and_bounded_republication',
    'original_ages_and_family_expiry', 'independent_family_updates',
    'foreign_old_and_invalid_metadata_do_not_renew', 'post_reset_floor_must_follow_boundary',
    'new_heartbeat_cannot_hide_continuity_gap',
    'protocol_read_keeps_actual_receipt_during_observer_flood',
    'observers_do_not_change_target_schedule_or_geometry',
    'healthy_observers_cannot_mask_pose_loss', 'observer_updates_cannot_renew_resume_permit',
    'heartbeat_loss_cancels_old_protocol_despite_observers',
}
FAMILIES = ['MAVMOOS_SYS_STATUS', 'MAVMOOS_GPS_STATUS', 'MAVMOOS_ATTITUDE_EULER', 'MAVMOOS_ATTITUDE_QUATERNION']


def parse(text):
    def reject(value):
        raise ValueError('nonfinite JSON constant: ' + value)
    def floating(value):
        result = float(value)
        if not math.isfinite(result):
            raise ValueError('overflowed JSON float: ' + value)
        return result
    return json.loads(text, parse_constant=reject, parse_float=floating)


def check(condition, why):
    if not condition:
        raise ValueError(why)


def validate(rows, attitude):
    passes = [r['case'] for r in rows if r.get('type') == 'pass']
    check(len(passes) == len(GROUPS) and set(passes) == GROUPS, 'missing/duplicate actual host case result')
    ends = [r for r in rows if r.get('type') == 'complete']
    check(len(ends) == 1 and ends[0]['groups'] == len(GROUPS) and ends[0]['attitude_loaded'] is attitude,
          'incomplete or wrong-composition driver trace')
    check(ends[0]['io'] == 'synchronous_fake_only', 'unexpected I/O claim')
    snapshots = {}
    seen_samples = 0
    for row in rows:
        if row.get('type') != 'snapshot':
            continue
        key = (row['case'], row['stage'])
        check(key not in snapshots, 'duplicate snapshot stage')
        snapshots[key] = row
        row['decoded'] = {name: parse(value) for name, value in row['strings'].items()
                          if name in FAMILIES + ['MAVMOOS_SYSTEM_STATUS', 'MAVMOOS_ATTITUDE_STATUS']}
        catalog = parse(row['capabilities'])
        check(catalog['composition']['plugin_abi'] == 10, 'wrong module ABI')
        modules = {module['name']: module for module in catalog['composition']['modules']}
        for name in ('system_status', 'attitude', 'observer_review_intervals'):
            if name in modules:
                d = modules[name]
                check(d['control_provider'] is False and d['request_owners'] == [], 'observer owns effects/backend')
                check(all(c['access'] == 'observation' for c in d['capabilities']), 'observer advertises control')
        for name, sample in row['decoded'].items():
            check(sample['schema'] == 1, 'unsupported publication schema')
            if name not in FAMILIES:
                check(sample['read_only'] is True and sample['authoritative_for_control'] is False, 'observer asserts authority')
                if name == 'MAVMOOS_ATTITUDE_STATUS':
                    check(sample['atomic'] is False and sample['family_selection'] == 'independent_no_preference', 'invented fused generation/selection')
                continue
            check(type(sample['present']) is bool and type(sample['fresh']) is bool and type(sample['encoding_valid']) is bool,
                  'sample flags are not booleans')
            check(sample['source_age'] == 'unknown' and sample['source_quality'] == 'unassessed', 'invented generation age or quality')
            if not sample['present']:
                check(sample['data'] is None and sample['fresh'] is False and sample['encoding_valid'] is False,
                      'absent sample retains valid/fresh data')
                continue
            seen_samples += 1
            effective, read, age = [sample[k] for k in ('effective_at', 'read_at', 'age_s')]
            published = row['publication_times'][name]
            check(all(type(v) in (int, float) and math.isfinite(v) for v in (effective, read, age)), 'invalid sample timestamps')
            check(effective <= read + 1e-9 and read <= published + 1e-9 and published <= row['time'] + 1e-9
                  and age >= 0 and math.isclose(age, published - effective, abs_tol=1e-9),
                  'republication rewrote effective age')
            check(sample['age_basis'] == 'QUEUE_EMPTY_FLOOR' and sample['age_is_upper_bound'] is True, 'lost queue bound')
            if name.startswith('MAVMOOS_ATTITUDE_'):
                data = sample['data']
                check(data['reference_frame'] == 'LOCAL_NED' and data['body_frame'] == 'BODY_FRD', 'invented frame transform')
                check(data['estimate'] == 'fused_fc_attitude' and data['remote_clock'] == 'boot_ms_unmapped_uint32', 'sensor/clock relabelling')
                for axis in ('x', 'y', 'z'):
                    check(data['angular_velocity_body_frd_rad_s'][axis]['unit'] == 'rad/s', 'rate units changed')
    def sample(case, stage, name):
        return snapshots[(case, stage)]['decoded'][name]
    def unchanged(case, stages, names):
        for name in names:
            values = [sample(case, stage, name) for stage in stages]
            for field in ('effective_at', 'read_at', 'observation_id', 'data'):
                check(all(v[field] == values[0][field] for v in values[1:]), 'cache update/invalid input renewed ' + name + ' ' + field)
    active = FAMILIES if attitude else FAMILIES[:2]
    unchanged('original_ages_and_family_expiry', ['initial', 'republished', 'attitude_expired'], active)
    if attitude:
        for name in FAMILIES[2:]:
            check(sample('original_ages_and_family_expiry', 'republished', name)['fresh'] is True, 'premature attitude expiry')
            check(sample('original_ages_and_family_expiry', 'attitude_expired', name)['fresh'] is False, 'heartbeat renewed attitude age')
    check(sample('original_ages_and_family_expiry', 'attitude_expired', FAMILIES[0])['fresh'] is True, 'attitude expiry contaminated status')
    unchanged('foreign_old_and_invalid_metadata_do_not_renew', ['initial', 'rejected'], active)
    for name in active:
        for stage in ('reset', 'preboundary_heartbeat'):
            check(sample('post_reset_floor_must_follow_boundary', stage, name)['present'] is False, 'pre-boundary heartbeat resurrected observer cache')
        # GPS is deliberately not supplied after reset; it must remain absent.
        new = sample('post_reset_floor_must_follow_boundary', 'new_session', name)
        check(new['present'] is (name != 'MAVMOOS_GPS_STATUS'), 'wrong post-reset family presence')
        check(new['session_epoch'] > sample('post_reset_floor_must_follow_boundary', 'initial', name)['session_epoch'], 'session did not advance')
    for name in active:
        updated = name in ('MAVMOOS_SYS_STATUS', 'MAVMOOS_ATTITUDE_EULER')
        final = sample('independent_family_updates', 'one_each_updated', name)
        initial = sample('independent_family_updates', 'initial', name)
        check(final['fresh'] is updated, 'one family renewed another family freshness')
        check((final['observation_id'] != initial['observation_id']) is updated, 'one family changed another family identity')
    if attitude:
        coalesced = sample('coalesced_frames_and_actual_ingress_filtering', 'last_frame', 'MAVMOOS_ATTITUDE_EULER')
        check(coalesced['data']['raw_angles_rad'][2] == 1.5 and coalesced['data']['time_boot_ms'] == 101,
              'distinct frames sharing acquisition/read stamps lost the last attitude value')
        check(coalesced['effective_at'] == 100.15 and coalesced['read_at'] == 100.2,
              'coalesced frames invented a new acquisition/read time')
        unchanged('coalesced_frames_and_actual_ingress_filtering', ['last_frame', 'duplicates', 'filtered'], ['MAVMOOS_ATTITUDE_EULER'])
        invalid = snapshots[('invalid_observer_authority', 'with_observers')]
        for name in FAMILIES[2:]:
            value = invalid['decoded'][name]
            check(value['present'] is True and value['fresh'] is True and value['encoding_valid'] is False,
                  'fresh malformed attitude became a valid pose')
            check(value['data']['orientation_valid'] is False, 'invalid attitude retained old orientation validity')
    metric = [r for r in rows if r.get('type') == 'metric']
    check(len(metric) == 1 and metric[0]['native_wire_claim'] is False, 'interval evidence overstated as native wire')
    expected = {'33': 50000, '245': 250000, '193': 500000, '242': 500000, '1': 1000000, '147': 1000000, '24': 250000}
    if attitude:
        expected.update({'30': 200000, '31': 200000})
    check(metric[0]['intervals_us'] == expected, 'wrong captured interval union')
    ingress = [r for r in rows if r.get('type') == 'ingress_metric']
    check(len(ingress) == 1 and ingress[0] == {'type': 'ingress_metric', 'callbacks': 3, 'duplicates': 2,
          'wrong_target': 1, 'bad_frames': 1, 'socket_io': False}, 'missing/incorrect actual ingress evidence')
    return {'result': 'PASS', 'groups': len(GROUPS), 'attitude_loaded': attitude,
            'snapshots': len(snapshots), 'validated_present_samples': seen_samples, 'intervals_us': expected,
            'scope': 'real Runtime and compiled MODULEs with synchronous fake I/O; no sockets/MOOS/native/hardware'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('driver', 'telemetry', 'status', 'attitude', 'control', 'parameters', 'rate-fixture'):
        parser.add_argument('--' + name, required=True)
    parser.add_argument('--allow-status-only', action='store_true')
    parser.add_argument('--output-dir', type=Path)
    args = parser.parse_args()
    attitude = args.attitude != '-'
    if not attitude and not args.allow_status_only:
        parser.error('a missing attitude module must be explicitly labelled --allow-status-only')
    command = [str(Path(args.driver).resolve())] + [getattr(args, k) for k in ('telemetry', 'status', 'attitude', 'control', 'parameters', 'rate_fixture')]
    if args.output_dir:
        args.output_dir.mkdir(parents=True, exist_ok=False)
    before = {path: hashlib.sha256(Path(path).read_bytes()).hexdigest() for path in command if path != '-'}
    completed = subprocess.run(command, text=True, capture_output=True)
    result = {'result': 'FAIL', 'command': command, 'driver_exit': completed.returncode}
    try:
        check(completed.returncode == 0, 'driver failed: ' + completed.stderr.strip())
        result.update(validate([parse(line) for line in completed.stdout.splitlines() if line], attitude))
    except Exception as error:
        result['reason'] = str(error)
    result['trace_sha256'] = hashlib.sha256(completed.stdout.encode()).hexdigest()
    result['artifacts'] = {path: hashlib.sha256(Path(path).read_bytes()).hexdigest() for path in command if path != '-'}
    result['artifacts_before'] = before
    result['artifact_drift'] = [path for path, digest in before.items() if result['artifacts'][path] != digest]
    if result['artifact_drift']:
        result['result'] = 'FAIL'
        result['reason'] = 'driver/module bytes changed during execution'
    if args.output_dir:
        (args.output_dir / 'trace.jsonl').write_text(completed.stdout)
        (args.output_dir / 'stderr.log').write_text(completed.stderr)
        (args.output_dir / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))
    return int(result['result'] != 'PASS')


if __name__ == '__main__':
    raise SystemExit(main())
