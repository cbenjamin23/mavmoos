#!/usr/bin/env python3
"""Pure controls for the native observer evidence checks."""
import copy
import unittest

from native_audit import audit_forwarded, verify_family


def metadata(observation, timeout=1.0):
    return {'schema': 1, 'present': True, 'read_only': True, 'authoritative_for_control': False,
            'source_age': 'unknown', 'source_quality': 'unassessed', 'system_id': 1, 'component_id': 1,
            'autopilot': 3, 'effective_at': 10 + observation, 'read_at': 10 + observation,
            'age_s': .1, 'age_basis': 'READ_ACQUISITION', 'age_is_upper_bound': False,
            'sample_timeout_s': timeout, 'fresh': True, 'session_epoch': 1, 'observation_id': observation}


def frame(family, at, **data):
    return {'system': 1, 'component': 1, 'mavpackettype': family, 'mono': at, **data}


def reports(include_esc=True):
    wire, samples = [], {key: [] for key in
                        ('MAVMOOS_LOCAL_KINEMATICS', 'MAVMOOS_VFR_HUD', 'MAVMOOS_ESC_TELEMETRY')}
    for index, at in enumerate((18., 19.), 1):
        wire.extend([
            frame('HEARTBEAT', at, type=2, autopilot=3, base_mode=0),
            frame('EXTENDED_SYS_STATE', at, landed_state=1),
            frame('LOCAL_POSITION_NED', at, time_boot_ms=index,
                  x=1.100000023841858, y=-2.5, z=-3., vx=0., vy=.5, vz=-.25),
            frame('VFR_HUD', at, airspeed=3., groundspeed=4., heading=90, throttle=0, alt=10., climb=0.),
        ])
        samples['MAVMOOS_LOCAL_KINEMATICS'].append(
            {**metadata(index, .8), 'data': {'time_boot_ms': index, 'position_m': [1.1, -2.5, -3.],
                                            'velocity_m_s': [-0., .5, -.25]}})
        samples['MAVMOOS_VFR_HUD'].append(
            {**metadata(index), 'data': {'airspeed_reported_m_s': 3., 'groundspeed_m_s': 4.,
                                         'heading_deg': 90, 'throttle_pct': 0, 'alt_msl_m': 10., 'climb_m_s': 0.}})
        groups = []
        if include_esc:
            wire.append(frame('ESC_TELEMETRY_1_TO_4', at, voltage=[1680]*4, current=[80]*4,
                              totalcurrent=[1]*4, rpm=[0]*4, count=[index]*4, temperature=[32]*4))
            groups.append({'first_index': 0, 'message_id': 11030, 'metadata': metadata(index),
                           'entries': [{'index': slot, 'voltage_cv': 1680, 'current_ca': 80,
                                        'total_current_mah': 1, 'rpm_raw': 0, 'packet_count': index,
                                        'temperature_c': 32} for slot in range(4)]})
        samples['MAVMOOS_ESC_TELEMETRY'].append({'schema': 1, 'present': bool(groups),
                                                  'atomic': False, 'groups': groups})
    return wire, samples


class NativeAuditTests(unittest.TestCase):
    def test_witness_accepts_matching_forwarded_wire_and_moos(self):
        result = audit_forwarded(*reports(), 20.)
        self.assertFalse(result['ever_armed'])
        self.assertEqual(result['ground_observations'], 2)
        self.assertTrue(all(row['result'] == 'PASS' and row['fresh_observations'] == 2
                            for row in result['families'].values()))
        self.assertTrue(result['families']['ESC_TELEMETRY_1_TO_4']['simulated_payload_signature_seen'])

    def test_missing_producer_is_distinct_from_module_loss_or_invented_data(self):
        wire, samples = reports(False)
        result = audit_forwarded(wire, samples, 20.)
        self.assertEqual(result['families']['ESC_TELEMETRY_1_TO_4']['result'], 'PENDING_PRODUCER')
        _, invented = reports()
        with self.assertRaisesRegex(ValueError, 'with no forwarded native'):
            audit_forwarded(wire, invented, 20.)
        valid_wire, _ = reports()
        with self.assertRaisesRegex(ValueError, 'missing advancing fresh app'):
            audit_forwarded(valid_wire, samples, 20.)
        del samples['MAVMOOS_ESC_TELEMETRY']
        with self.assertRaisesRegex(ValueError, 'missing native MOOS publication'):
            audit_forwarded(wire, samples, 20.)

    def test_wire_delivery_with_no_simulated_producer_signature_stays_pending(self):
        wire, samples = reports()
        for row in wire:
            if row['mavpackettype'] == 'ESC_TELEMETRY_1_TO_4':
                row['temperature'] = [0] * 4
        for row in samples['MAVMOOS_ESC_TELEMETRY']:
            for entry in row['groups'][0]['entries']:
                entry['temperature_c'] = 0
        esc = audit_forwarded(wire, samples, 20.)['families']['ESC_TELEMETRY_1_TO_4']
        self.assertEqual(esc['result'], 'PENDING_PRODUCER')
        self.assertTrue(esc['wire_app_delivery_verified'])
        self.assertFalse(esc['simulated_payload_signature_seen'])

    def test_arm_source_or_landed_fault_rejects(self):
        wire, samples = reports()
        for index, key, value in ((0, 'base_mode', 128), (0, 'component', 2), (1, 'landed_state', 2)):
            with self.subTest(key=key):
                changed = copy.deepcopy(wire)
                changed[index][key] = value
                with self.assertRaises(ValueError):
                    audit_forwarded(changed, samples, 20.)
        with self.assertRaisesRegex(ValueError, 'insufficient recent'):
            audit_forwarded(wire, samples, 22.)

    def test_wire_value_and_age_tampering_rejects(self):
        wire, samples = reports()
        for field, change in (('position_m', [1.25, -2.5, -3.]), ('time_boot_ms', 20)):
            with self.subTest(field=field):
                changed = copy.deepcopy(samples)
                changed['MAVMOOS_LOCAL_KINEMATICS'][0]['data'][field] = change
                with self.assertRaisesRegex(ValueError, 'absent from forwarded native'):
                    audit_forwarded(wire, changed, 20.)
        samples['MAVMOOS_VFR_HUD'][0]['age_s'] = 2
        with self.assertRaisesRegex(ValueError, 'freshness differs'):
            audit_forwarded(wire, samples, 20.)

    def test_republication_cannot_change_timestamps(self):
        wire, samples = reports()
        rows = samples['MAVMOOS_LOCAL_KINEMATICS']
        renewed = copy.deepcopy(rows[0])
        renewed['effective_at'] = renewed['read_at'] = 50.
        with self.assertRaisesRegex(ValueError, 'republished observation changed'):
            verify_family('LOCAL_POSITION_NED', [*rows, renewed],
                          [row for row in wire if row['mavpackettype'] == 'LOCAL_POSITION_NED'])


if __name__ == '__main__':
    unittest.main(verbosity=2)
