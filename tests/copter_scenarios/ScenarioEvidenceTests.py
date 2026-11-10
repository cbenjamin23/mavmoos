#!/usr/bin/env python3
"""Synthetic acceptance-checker controls; no socket, simulator or test binary."""
import argparse
import copy
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys
import unittest

CHECKER = None


def firmware_fixture(version=None):
    """Deliberately synthetic source/build identity, never a native result."""
    manifest = {'schema': 1, 'kind': 'mavmoos.firmware_source', 'profile_id': 'synthetic-not-a-native-run',
                'vehicle_profile': 'arducopter', 'qualification': 'not_inferred',
                'target': {'system_id': 1, 'component_id': 1, 'mav_type': 2, 'autopilot': 3},
                'source': {'revision': '3' * 40, 'version': version or {'major': 4, 'minor': 7, 'patch': 0, 'release_type': 0}},
                'fixture': {'model': 'quad'}, 'artifact_path': 'build/sitl/bin/arducopter'}
    manifest_bytes = json.dumps(manifest, sort_keys=True).encode()
    identity = {'source_revision': manifest['source']['revision'], 'native_sha256': '2' * 64, 'frame': 'quad',
                'qualification': 'not_inferred', 'profile_id': manifest['profile_id'],
                'manifest_sha256': hashlib.sha256(manifest_bytes).hexdigest(), 'platform': {'system': 'Darwin', 'machine': 'arm64'}}
    attestation = {'schema': 1, 'kind': 'mavmoos.native_build_attestation', 'profile_id': manifest['profile_id'],
                   'qualification': 'not_inferred', 'manifest_sha256': identity['manifest_sha256'],
                   'source_identity': {'revision': identity['source_revision']},
                   'platform': identity['platform'], 'artifact': {'path': manifest['artifact_path'], 'sha256': identity['native_sha256']}}
    attestation_bytes = json.dumps(attestation, sort_keys=True).encode()
    return manifest_bytes, attestation_bytes, identity, CHECKER.firmware_inputs(manifest_bytes, attestation_bytes, identity)


def change_status(value, change, index=1):
    rows = [row for row in value[0] if row['type'] == 'firmware']
    row = rows[index];status = json.loads(row['value']);change(status);row['value'] = json.dumps(status)


def fixture(scenario='route_hold_return'):
    events, packets, times = [], [], {}
    cycles = CHECKER.SCENARIOS[scenario]['cycles']
    def event(at, kind, key, value, source='probe'):
        events.append({'elapsed': round(at, 6), 'type': kind, 'key': key, 'value': value, 'source': source})
    def kv(values):
        return ','.join(f'{k}={v}' for k, v in values.items())
    for cycle in range(1, cycles + 1):
        offset = (cycle - 1) * 60
        prefix, ident = f'cycle{cycle}', f'c{cycle}'
        schedule = {'ground.started': 0, 'ground.finished': 2, 'takeoff.started': 3, 'takeoff.finished': 8,
                    'route1.started': 9, 'route1.finished': 15, 'hold_helm.started': 16, 'hold_helm.finished': 20.02,
                    'cancel_helm.started': 21, 'cancel_helm.finished': 22.2, 'route2.started': 23, 'route2.finished': 29,
                    'hold_guided.started': 30, 'hold_guided.finished': 34.02, 'cancel_guided.started': 35, 'cancel_guided.finished': 36.2,
                    'return.started': 37, 'return.finished': 43, 'land.started': 44, 'land.finished': 49, 'finished': 51}
        def counter(t):
            return (cycle - 1) * 280 + int(10 * (max(0, min(t - 9, 11)) + max(0, min(t - 23, 11)) + max(0, min(t - 37, 6))))
        for step in CHECKER.STEPS:
            t = schedule[step]
            value = {'nav_tx_accepted': counter(t)}
            if step.startswith('hold_') and step.endswith('.started'):
                strategy = step[5:].split('.')[0]
                value.update(anchor_x=10 if strategy == 'helm' else 7, anchor_y=0 if strategy == 'helm' else 8, height=5, strategy=strategy)
            if step == 'return.started':
                value.update(home_lat=42.358456, home_lon=-71.087589)
            event(offset + t, 'phase', prefix + '.' + step, kv(value))
            if step.startswith('hold_'):
                key=prefix+'.'+step
                event(offset+t+.01,'boundary_ack',key,'hold:'+key+':1:1000.0\n')
                events[-1].update(wait_started_elapsed=offset+t-.001,deadline_elapsed=offset+t+2.999,ack_observed_elapsed=offset+t+.009)
            times[prefix + '.' + step] = 100 + offset + t
        requests = [
            (2.1, ident + '-guided', {'action': 'guided'}, 2.6),
            (2.7, ident + '-arm', {'action': 'arm'}, 2.9),
            (3, ident + '-takeoff', {'action': 'takeoff', 'height': 5}, 8),
            (9, ident + '-route1', {'action': 'goto', 'strategy': 'helm', 'lat': 42.358456, 'lon': -71.087468, 'height': 5}, 15),
            (15.1, prefix + '-hold-helm', {'action': 'hold', 'strategy': 'helm'}, 20.7),
            (20.5, prefix + '-cancel-helm', {'action': 'cancel', 'target': prefix + '-hold-helm'}, 20.7),
            (23, ident + '-route2', {'action': 'goto', 'strategy': 'helm', 'lat': 42.358528, 'lon': -71.087504, 'height': 5}, 29),
            (29.1, prefix + '-hold-guided', {'action': 'hold', 'strategy': 'guided'}, 34.7),
            (34.5, prefix + '-cancel-guided', {'action': 'cancel', 'target': prefix + '-hold-guided'}, 34.7),
            (37, ident + '-return', {'action': 'return', 'strategy': 'helm', 'height': 5}, 43),
            (44, ident + '-land', {'action': 'land'}, 49),
            (49.1, ident + '-disarm', {'action': 'disarm'}, 49.3),
        ]
        for at, opid, body, done in requests:
            request = {'schema': 1, 'id': opid, **body}
            event(offset + at, 'request', 'MAVMOOS_CONTROL_REQUEST', kv(request))
            expiry = 2000 + offset + at
            def result(when, status):
                event(offset + when, 'operation_result', 'MAVMOOS_CONTROL_RESULT', kv({'schema': 1, 'id': opid, 'status': status, 'expires': expiry, 'at': 1000 + offset + when}), 'iMAVLink')
            result(at + .01, 'ADMITTED')
            if '-hold-' in opid:
                result(at + .2, 'CONFIRMED')
                result(done, 'CANCELLED')
            else:
                result(done, 'COMPLETED')
        for tick in range(511):
            t = tick * .1
            in_hold1, in_hold2 = 16 <= t <= 20, 30 <= t <= 34
            airborne = 8 <= t < 49
            permission = in_hold1 or in_hold2
            runtime = {key: 'true' for key in ('heartbeat_valid', 'position_valid', 'home_valid', 'landed_valid', 'estimator_valid')}
            runtime.update(armed='true' if 3 <= t < 49.3 else 'false', in_air='true' if airborne else 'false',
                           permission='true' if permission else 'false', fc_mode='4' if t < 44 else '9', nav_tx_accepted=counter(t))
            event(offset + t, 'runtime', 'MAVMOOS_RUNTIME_STATUS', kv(runtime), 'iMAVLink')
            x, y = (10, 0) if 14 <= t <= 22.2 else (7, 8) if 28 <= t <= 36.2 else (0, 0)
            for key, value in {'NAV_X': x, 'NAV_Y': y, 'NAV_ALTITUDE': 5 if airborne else 0, 'NAV_SAMPLE_RECEIVED_AT': 1000 + offset + t}.items():
                event(offset + t, 'navigation', key, str(value), 'iMAVLink')
            for key, value in {'DESIRED_HEADING': 90, 'DESIRED_SPEED': .1, 'DESIRED_ALTITUDE': 5}.items():
                event(offset + t, 'helm_scalar', key, str(value), 'pHelmIvP')
            mono = 100 + offset + t
            position = {'mono': mono, 'system': 1, 'component': 1, 'mavpackettype': 'GLOBAL_POSITION_INT',
                        'time_boot_ms': int((offset + t + 1) * 1000),
                        'lat': round((42.358456 + y / 111195) * 1e7),
                        'lon': round((-71.087589 + x / (111195 * math.cos(math.radians(42.358456)))) * 1e7),
                        'relative_alt': 5000 if airborne else 0, 'vx': 3, 'vy': -2}
            packets.append(position)
            if tick % 5 == 0:
                packets.append({'mono': mono, 'system': 1, 'component': 1, 'mavpackettype': 'HEARTBEAT',
                                'base_mode': 128 if 3 <= t < 49.3 else 0, 'custom_mode': 4 if t < 44 else 9})
                packets.append({'mono': mono, 'system': 1, 'component': 1, 'mavpackettype': 'EXTENDED_SYS_STATE', 'landed_state': 2 if airborne else 1})
            if tick % 2 == 0:
                packets.append({'mono': mono, 'system': 1, 'component': 1, 'mavpackettype': 'POSITION_TARGET_LOCAL_NED'})
    events.sort(key=lambda row: row['elapsed'])
    event(cycles * 60 - 8.9, 'pass', scenario, 'synthetic checker fixture')
    # The native supervisor drains after probe completion; include that passive
    # tail without inventing further probe events after its terminal pass.
    for tick in range(1, 9):
        at = 100 + cycles * 60 - 9 + tick * .2
        packets.extend([{'mono': at, 'system': 1, 'component': 1, 'mavpackettype': 'HEARTBEAT', 'base_mode': 0, 'custom_mode': 9},
                        {'mono': at, 'system': 1, 'component': 1, 'mavpackettype': 'EXTENDED_SYS_STATE', 'landed_state': 1}])
    packets.sort(key=lambda row: row['mono'])
    arms = [{'ArmState': state, 'Forced': 0} for _ in range(cycles) for state in (1, 0)]
    modes = [{'ModeNum': 0}] + [{'ModeNum': mode} for _ in range(cycles) for mode in (4, 9)]
    parameters = {'SIM_WIND_SPD': CHECKER.SCENARIOS[scenario]['wind_speed_mps'], 'SIM_WIND_DIR': 270, 'SIM_WIND_TURB': 0,
                  'LOG_DISARMED': 1, 'SERIAL0_PROTOCOL': 2, 'SERIAL1_PROTOCOL': 2}
    _, _, _, firmware = firmware_fixture()
    status = {'schema': 1, 'epoch': 1, 'state': 'WAITING_VERSION', 'read_only': True, 'authoritative_for_control': False,
              'qualification': 'not_evaluated', 'source_age': 'unknown', 'system_id': 1, 'component_id': 1,
              'present': False, 'fresh': False, 'observation_id': 0, 'identity_revision': 0,
              'effective_at': None, 'read_at': None, 'age_s': None, 'age_basis': 'READ_ACQUISITION',
              'age_is_upper_bound': False, 'retained_until_session_reset': True, 'reported': None}
    event(0, 'firmware', 'MAVMOOS_FIRMWARE_STATUS', json.dumps(status), 'iMAVLink')
    prefix = firmware['source_revision'][:8]
    reported = {**firmware['expected_version'], 'version_known': True,
                'flight_custom_version_ascii': prefix, 'flight_custom_version_raw_hex': prefix.encode().hex(),
                'custom_version_semantics': 'ardupilot_ascii_prefix_or_opaque_bytes_not_full_git_revision',
                'capability_assessment': 'reported_bits_not_behavior_qualification'}
    for at in [.2, .8] + list(range(5, cycles * 60 - 8, 5)):
        status.update(state='OBSERVED' if at <= 30 else 'STALE', present=True, fresh=at <= 30, observation_id=1,
                      effective_at=1000, read_at=1000, age_s=at, reported=reported)
        event(at, 'firmware', 'MAVMOOS_FIRMWARE_STATUS', json.dumps(status), 'iMAVLink')
    events.sort(key=lambda row: row['elapsed'])
    return [events, packets, times, 100 + cycles * 60 - 7.4, arms, modes, parameters]


class EvidenceTests(unittest.TestCase):
    def rejects(self, change):
        value = copy.deepcopy(fixture())
        change(value)
        with self.assertRaises((ValueError, KeyError, TypeError)):
            self.audit(value)

    def audit(self, value, scenario='route_hold_return'):
        events, packets, times, end, arms, modes, parameters = value
        proof = CHECKER.audit_probe(events, scenario, firmware_fixture()[3])
        CHECKER.audit_fc(packets, times, end, proof)
        CHECKER.audit_dataflash(arms, modes, parameters, scenario)
        return proof

    def test_complete_calm_positive(self): self.audit(fixture())
    def test_two_wind_cycles_positive(self): self.audit(fixture('wind_repeated_cycle'), 'wind_repeated_cycle')
    def test_static_identity_can_age_to_stale_positive(self):
        proof = self.audit(fixture())
        self.assertEqual(proof['firmware_discovery']['unique_observations'], 1)
        self.assertGreater(proof['firmware_discovery']['stale_publications'], 0)
    def test_fresh_additional_version_packet_positive(self):
        value = fixture()
        for row in value[0]:
            if row['type'] == 'firmware' and row['elapsed'] >= 35:
                status = json.loads(row['value'])
                age = row['elapsed'] - 35
                status.update(observation_id=2, effective_at=1035, read_at=1035, age_s=age, fresh=True, state='OBSERVED')
                row['value'] = json.dumps(status)
        self.assertEqual(self.audit(value)['firmware_discovery']['unique_observations'], 2)
    def test_queue_floor_age_positive(self):
        value = fixture()
        for row in value[0]:
            if row['type'] == 'firmware':
                status = json.loads(row['value'])
                if status['present']:
                    status.update(age_basis='QUEUE_EMPTY_FLOOR', age_is_upper_bound=True, read_at=1000.1)
                    row['value'] = json.dumps(status)
        self.audit(value)
    def test_stable_release_version_positive(self):
        value = fixture();expected = firmware_fixture({'major': 4, 'minor': 6, 'patch': 3, 'release_type': 255})[3]
        for row in value[0]:
            if row['type'] == 'firmware':
                status = json.loads(row['value'])
                if status['present']:
                    status['reported'].update(expected['expected_version']);row['value'] = json.dumps(status)
        CHECKER.audit_probe(value[0], 'route_hold_return', expected)
    def test_firmware_missing(self): self.rejects(lambda v: v.__setitem__(0, [r for r in v[0] if r['type'] != 'firmware']))
    def test_firmware_not_early(self): self.rejects(lambda v: v.__setitem__(0, [r for r in v[0] if r['type'] != 'firmware' or r['elapsed'] > 3]))
    def test_firmware_unknown_version(self): self.rejects(lambda v: change_status(v, lambda s: s['reported'].update(version_known=False)))
    def test_firmware_wrong_version(self): self.rejects(lambda v: change_status(v, lambda s: s['reported'].update(minor=6)))
    def test_firmware_wrong_release(self): self.rejects(lambda v: change_status(v, lambda s: s['reported'].update(release_type_raw=255)))
    def test_firmware_wrong_raw_version(self): self.rejects(lambda v: change_status(v, lambda s: s['reported'].update(flight_sw_version_raw=0)))
    def test_firmware_short_prefix(self): self.rejects(lambda v: change_status(v, lambda s: s['reported'].update(flight_custom_version_ascii='deadbeef')))
    def test_firmware_publisher(self): self.rejects(lambda v: next(r for r in v[0] if r['type'] == 'firmware').update(source='another_app'))
    def test_firmware_target(self): self.rejects(lambda v: change_status(v, lambda s: s.update(component_id=2)))
    def test_firmware_authority_promotion(self): self.rejects(lambda v: change_status(v, lambda s: s.update(authoritative_for_control=True)))
    def test_firmware_qualification_promotion(self): self.rejects(lambda v: change_status(v, lambda s: s.update(qualification='supported')))
    def test_firmware_unknown_age(self): self.rejects(lambda v: change_status(v, lambda s: s.update(age_s=None)))
    def test_firmware_false_read_age_basis(self): self.rejects(lambda v: change_status(v, lambda s: s.update(read_at=1000.1)))
    def test_firmware_original_time_renewed(self): self.rejects(lambda v: change_status(v, lambda s: s.update(effective_at=1000.1, read_at=1000.1), 2))
    def test_firmware_age_backwards(self): self.rejects(lambda v: change_status(v, lambda s: s.update(age_s=.1), 2))
    def test_firmware_stale_labelled_fresh(self): self.rejects(lambda v: change_status(v, lambda s: s.update(fresh=True, state='OBSERVED'), 9))
    def test_firmware_epoch_regressed(self): self.rejects(lambda v: change_status(v, lambda s: s.update(epoch=0), 2))
    def test_firmware_reset_reuses_original_sample(self): self.rejects(lambda v: change_status(v, lambda s: s.update(epoch=2), 2))
    def test_firmware_cleared_epoch_retains_report(self): self.rejects(lambda v: change_status(v, lambda s: s.update(present=False, fresh=False, state='WAITING_VERSION')))
    def test_missing_phase(self): self.rejects(lambda v: v[0].remove(next(r for r in v[0] if r['type'] == 'phase')))
    def test_probe_failure(self): self.rejects(lambda v: v[0][-1].update(type='fail'))
    def test_publisher_identity(self): self.rejects(lambda v: next(r for r in v[0] if r['type'] == 'runtime').update(source='other'))
    def test_helm_source(self): self.rejects(lambda v: next(r for r in v[0] if r['type'] == 'helm_scalar').update(source='other'))
    def test_missing_helm_quantity(self): self.rejects(lambda v: v.__setitem__(0, [r for r in v[0] if r['key'] != 'DESIRED_ALTITUDE']))
    def test_original_deadline_renewed(self): self.rejects(lambda v: next(r for r in v[0] if r['type'] == 'operation_result').update(value=next(r for r in v[0] if r['type'] == 'operation_result')['value'].replace('expires=2002.1', 'expires=9999')))
    def test_wrong_request_strategy(self): self.rejects(lambda v: next(r for r in v[0] if r['type'] == 'request' and 'c1-route1,' in r['value']).update(value='schema=1,id=c1-route1,action=goto,strategy=guided'))
    def test_cancel_target(self): self.rejects(lambda v: next(r for r in v[0] if r['type'] == 'request' and 'cycle1-cancel-helm,' in r['value']).update(value='schema=1,id=cycle1-cancel-helm,action=cancel,target=other'))
    def test_effect_scope(self): self.rejects(lambda v: next(r for r in v[0] if r['type'] == 'request').update(key='MAVMOOS_PARAM_REQUEST'))
    def test_late_runtime_missing(self): self.rejects(lambda v: v.__setitem__(0, [r for r in v[0] if not (r['type'] == 'runtime' and 18 <= r['elapsed'] <= 20)]))
    def test_runtime_middle_gap(self): self.rejects(lambda v: v.__setitem__(0, [r for r in v[0] if not (r['type'] == 'runtime' and 17 <= r['elapsed'] <= 18)]))
    def test_held_permission(self): self.rejects(lambda v: next(r for r in v[0] if r['type'] == 'runtime' and r['elapsed'] == 19).update(value=next(r for r in v[0] if r['type'] == 'runtime' and r['elapsed'] == 19)['value'].replace('permission=true', 'permission=false')))
    def test_nav_counter_stopped(self): self.rejects(lambda v: [r.update(value=r['value'].rsplit('=', 1)[0] + '=90') for r in v[0] if r['type'] == 'runtime' and 16 <= r['elapsed'] <= 20])
    def test_position_clock_frozen(self): self.rejects(lambda v: [r.update(value='1000') for r in v[0] if r['key'] == 'NAV_SAMPLE_RECEIVED_AT'])
    def test_held_height(self): self.rejects(lambda v: [r.update(value='9') for r in v[0] if r['key'] == 'NAV_ALTITUDE' and 18 <= r['elapsed'] <= 20])
    def test_cancel_auto_reacquired(self): self.rejects(lambda v: [r.update(value=r['value'].replace('permission=false', 'permission=true')) for r in v[0] if r['type'] == 'runtime' and 21 <= r['elapsed'] <= 22.2])
    def test_route_arrival(self): self.rejects(lambda v: [r.update(value='100') for r in v[0] if r['key'] == 'NAV_X' and 14 <= r['elapsed'] <= 15])
    def test_fc_identity(self): self.rejects(lambda v: v[1][0].update(system=2))
    def test_fc_position_stamp_frozen(self): self.rejects(lambda v: [r.update(time_boot_ms=1) for r in v[1] if r['mavpackettype'] == 'GLOBAL_POSITION_INT'])
    def test_fc_mode(self): self.rejects(lambda v: [r.update(custom_mode=5) for r in v[1] if r['mavpackettype'] == 'HEARTBEAT' and 117 <= r['mono'] <= 120])
    def test_fc_airborne(self): self.rejects(lambda v: [r.update(landed_state=1) for r in v[1] if r['mavpackettype'] == 'EXTENDED_SYS_STATE' and 117 <= r['mono'] <= 120])
    def test_fc_position(self): self.rejects(lambda v: [r.update(relative_alt=9000) for r in v[1] if r['mavpackettype'] == 'GLOBAL_POSITION_INT' and 117 <= r['mono'] <= 120])
    def test_fc_speed(self): self.rejects(lambda v: [r.update(vx=400) for r in v[1] if r['mavpackettype'] == 'GLOBAL_POSITION_INT' and 117 <= r['mono'] <= 120])
    def test_fc_target_reports_missing(self): self.rejects(lambda v: v.__setitem__(1, [r for r in v[1] if r['mavpackettype'] != 'POSITION_TARGET_LOCAL_NED']))
    def test_final_fc_armed(self): self.rejects(lambda v: [r.update(base_mode=128) for r in v[1] if r['mavpackettype'] == 'HEARTBEAT' and r['mono'] >= 149])
    def test_final_runtime_airborne(self): self.rejects(lambda v: [r.update(value=r['value'].replace('in_air=false', 'in_air=true')) for r in v[0] if r['type'] == 'runtime' and r['elapsed'] >= 50])
    def test_closed_wind(self): self.rejects(lambda v: v[6].update(SIM_WIND_SPD=3))
    def test_closed_forced_arm(self): self.rejects(lambda v: v[4][0].update(Forced=1))
    def test_closed_incomplete_cycle(self): self.rejects(lambda v: v[4].pop())
    def test_closed_wrong_mode(self): self.rejects(lambda v: v[5].append({'ModeNum': 6}))
    def test_missing_hold_ack(self): self.rejects(lambda v: v[0].remove(next(r for r in v[0] if r['type']=='boundary_ack')))
    def test_wrong_hold_ack_token(self): self.rejects(lambda v: next(r for r in v[0] if r['type']=='boundary_ack').update(value='foreign'))
    def test_foreign_hold_ack_source(self): self.rejects(lambda v: next(r for r in v[0] if r['type']=='boundary_ack').update(source='another_probe'))
    def test_duplicate_hold_ack(self):
        def change(v):
            row=next(r for r in v[0] if r['type']=='boundary_ack');v[0].insert(v[0].index(row),copy.deepcopy(row))
        self.rejects(change)
    def test_original_wait_deadline_not_phase_log_delta(self):
        def change(v):
            row=next(r for r in v[0] if r['type']=='boundary_ack');row.update(wait_started_elapsed=12,deadline_elapsed=15,ack_observed_elapsed=16.009)
        self.rejects(change)
    def test_late_ack_read_not_saved_by_timely_marker(self):
        def change(v):
            row=next(r for r in v[0] if r['type']=='boundary_ack');row.update(ack_observed_elapsed=row['deadline_elapsed'],elapsed=row['deadline_elapsed']);v[0].sort(key=lambda r:r['elapsed'])
        self.rejects(change)
    def test_early_dwell_before_start_ack(self):
        def change(v):
            row=next(r for r in v[0] if r['type']=='boundary_ack');row.update(ack_observed_elapsed=16.2,elapsed=16.2);v[0].sort(key=lambda r:r['elapsed'])
        self.rejects(change)
    def test_cancel_before_finish_ack(self):
        def change(v):
            row=next(r for r in v[0] if r['type']=='request' and 'id=cycle1-cancel-helm,' in r['value']);row['elapsed']=20.025;v[0].sort(key=lambda r:r['elapsed'])
        self.rejects(change)
    def test_ack_cannot_renew_deadline(self): self.rejects(lambda v: next(r for r in v[0] if r['type']=='boundary_ack').update(deadline_elapsed=99))
    def test_delayed_marker_cannot_move_original_wait_floor(self):
        original={'elapsed':2.};ack={'wait_started_elapsed':1.,'deadline_elapsed':4.,'ack_observed_elapsed':4.5,'elapsed':4.6}
        self.assertLess(ack['elapsed']-original['elapsed'],3)
        with self.assertRaisesRegex(ValueError,'original deadline'):CHECKER.hold_ack_observation(original,ack)
    def test_timely_witness_can_be_logged_later_without_clock_reconstruction(self):
        original={'elapsed':2.};ack={'wait_started_elapsed':1.,'deadline_elapsed':4.,'ack_observed_elapsed':3.9,'elapsed':4.6}
        self.assertEqual(CHECKER.hold_ack_observation(original,ack),3.9)
    def test_duplicate_json_key(self):
        with self.assertRaises(ValueError): CHECKER.parse('{"x":1,"x":2}')
    def test_nonfinite_json(self):
        with self.assertRaises(ValueError): CHECKER.parse('{"x":NaN}')


def main():
    global CHECKER
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--checker', type=Path, default=Path(__file__).with_name('scenario_evidence.py'))
    args = parser.parse_args()
    spec = importlib.util.spec_from_file_location('_copter_scenario_checker', args.checker.resolve())
    CHECKER = importlib.util.module_from_spec(spec);spec.loader.exec_module(CHECKER)
    suite = unittest.defaultTestLoader.loadTestsFromTestCase(EvidenceTests)
    extra_path=Path(__file__).with_name('HoldBoundaryTests.py')
    spec=importlib.util.spec_from_file_location('_copter_hold_boundary_controls',extra_path)
    extra=importlib.util.module_from_spec(spec);sys.modules[spec.name]=extra;spec.loader.exec_module(extra)
    suite.addTests(unittest.defaultTestLoader.loadTestsFromTestCase(extra.BoundaryTests))
    result = unittest.TextTestRunner(verbosity=1).run(suite)
    print(json.dumps({'result': 'PASS' if result.wasSuccessful() else 'FAIL', 'tests_run': result.testsRun,
                      'scope': 'offline checker/files and inert owned UDP worker controls; no native vehicle/simulator execution'}, indent=2))
    return int(not result.wasSuccessful())


if __name__ == '__main__':
    raise SystemExit(main())
