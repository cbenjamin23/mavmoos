#!/usr/bin/env python3
"""Negative controls for the independent native-workflow acceptance checks."""
import unittest
from run_operations import flight_evidence, operation_evidence


def operation_trace():
    outcomes = {'ops-reject-ground': 'REJECTED', 'ops-guided': 'COMPLETED', 'ops-arm': 'COMPLETED',
                'ops-takeoff': 'COMPLETED', 'ops-guided-goto': 'COMPLETED',
                'ops-guided-hold': 'CANCELLED', 'ops-guided-cancel': 'COMPLETED',
                'ops-helm-goto': 'COMPLETED', 'ops-helm-hold': 'CANCELLED',
                'ops-helm-cancel': 'COMPLETED', 'ops-helm-return': 'COMPLETED',
                'ops-native-hold': 'COMPLETED', 'ops-rtl-stage': 'COMPLETED', 'ops-native-rtl': 'COMPLETED'}
    trace = [{'type': 'observation', 'key': 'MAVMOOS_CONTROL_RESULT', 'mono': 1,
              'value': f'schema=1,id={ident},status={status},expires=200,at=100'}
             for ident, status in outcomes.items()]
    for start, phase in enumerate(('helm_goto', 'helm_hold', 'helm_return'), 10):
        trace.append({'type': 'phase', 'key': phase, 'mono': start})
        trace += [{'type': 'stock_helm', 'key': key, 'mono': start+.1}
                  for key in ('DESIRED_HEADING', 'DESIRED_SPEED', 'DESIRED_ALTITUDE')]
    trace.append({'type': 'phase', 'key': 'prepare_hold', 'mono': 13})
    return trace


def flight_trace():
    trace = [{'type': 'observation', 'key': 'MAVMOOS_HOME_POSITION', 'mono': 0,
              'value': 'encoding_valid=true,lat_deg=42,lon_deg=-71'},
             {'type': 'phase', 'key': 'native_rtl', 'mono': 20}]
    def heartbeat(at, mode, armed=True):
        return {'received_mono': at, 'mavpackettype': 'HEARTBEAT', 'custom_mode': mode,
                'base_mode': 128 if armed else 0}
    def position(at, east, altitude=5):
        return {'received_mono': at, 'mavpackettype': 'GLOBAL_POSITION_INT',
                'lat': 420000000, 'lon': round((-71+east/82634)*1e7), 'relative_alt': altitude*1000}
    packets = [heartbeat(1, 4), position(1, 0), heartbeat(10, 5)]
    packets += [position(10+i*.2, 0) for i in range(9)]
    packets += [heartbeat(15, 4), position(19.9, -8), heartbeat(20.2, 6)]
    packets += [position(20.3+i*.2, -8+i*1.5) for i in range(6)]
    packets += [position(30.9, -.1, 0), heartbeat(31, 6, False),
                {'received_mono': 31, 'mavpackettype': 'EXTENDED_SYS_STATE', 'landed_state': 1}]
    return packets, trace


class EvidenceTests(unittest.TestCase):
    def test_complete_public_trace_is_accepted(self):
        self.assertEqual(operation_evidence(operation_trace())['ops-native-rtl']['terminal'], 'COMPLETED')

    def test_native_mode_ack_without_terminal_completion_is_rejected(self):
        trace = operation_trace()
        event = next(item for item in trace if 'id=ops-native-rtl,' in item.get('value', ''))
        event['value'] = event['value'].replace('status=COMPLETED', 'status=CONFIRMED')
        with self.assertRaisesRegex(RuntimeError, 'terminal'):
            operation_evidence(trace)

    def test_renewed_original_expiry_is_rejected(self):
        trace = operation_trace()
        trace.append({'type': 'observation', 'key': 'MAVMOOS_CONTROL_RESULT', 'mono': 1,
                      'value': 'schema=1,id=ops-guided-goto,status=COMPLETED,expires=201,at=100'})
        with self.assertRaisesRegex(RuntimeError, 'expiry changed'):
            operation_evidence(trace)

    def test_old_helm_outputs_do_not_qualify_a_new_phase(self):
        trace = [item for item in operation_trace() if not (item.get('type') == 'stock_helm'
                 and item.get('key') == 'DESIRED_ALTITUDE' and item['mono'] == 12.1)]
        with self.assertRaisesRegex(RuntimeError, 'helm_return'):
            operation_evidence(trace)

    def test_independent_loiter_and_away_home_rtl_are_accepted(self):
        packets, trace = flight_trace()
        proof = flight_evidence(packets, trace)
        self.assertGreater(proof['native_rtl']['before_mode6_max_home_distance_m'], 6)
        self.assertTrue(proof['final_on_ground'] and proof['final_disarmed'])

    def test_rtl_at_home_alone_does_not_qualify_trajectory(self):
        packets, trace = flight_trace()
        for item in packets:
            if item['mavpackettype'] == 'GLOBAL_POSITION_INT' and item['received_mono'] >= 19:
                item['lon'] = -710000000
        with self.assertRaisesRegex(RuntimeError, 'traverse'):
            flight_evidence(packets, trace)

    def test_telemetry_landing_claim_without_disarm_is_rejected(self):
        packets, trace = flight_trace()
        next(item for item in reversed(packets) if item['mavpackettype'] == 'HEARTBEAT')['base_mode'] = 128
        with self.assertRaisesRegex(RuntimeError, 'final state'):
            flight_evidence(packets, trace)

    def test_native_loiter_drift_is_rejected(self):
        packets, trace = flight_trace()
        next(item for item in packets if item['mavpackettype'] == 'GLOBAL_POSITION_INT'
             and item['received_mono'] == 10.8)['relative_alt'] = 6000
        with self.assertRaisesRegex(RuntimeError, 'Loiter'):
            flight_evidence(packets, trace)

    def test_probe_epoch_is_irrelevant_to_independent_fc_evidence(self):
        packets, trace = flight_trace()
        trace[-1]['mono'] += 1165976
        self.assertTrue(flight_evidence(packets, trace)['final_disarmed'])

    def test_away_home_only_far_before_rtl_is_insufficient(self):
        packets, trace = flight_trace()
        next(item for item in packets if item['mavpackettype'] == 'GLOBAL_POSITION_INT'
             and item['received_mono'] == 19.9)['received_mono'] = 17
        with self.assertRaisesRegex(RuntimeError, 'traverse'):
            flight_evidence(packets, trace)

    def test_missing_during_rtl_positions_is_insufficient(self):
        packets, trace = flight_trace()
        packets = [item for item in packets if not (item['mavpackettype'] == 'GLOBAL_POSITION_INT'
                   and 20 <= item['received_mono'] < 30)]
        with self.assertRaisesRegex(RuntimeError, 'traverse'):
            flight_evidence(packets, trace)

    def test_landing_away_from_home_after_approach_is_rejected(self):
        packets, trace = flight_trace()
        next(item for item in reversed(packets) if item['mavpackettype'] == 'GLOBAL_POSITION_INT')['lon'] -= 1000
        with self.assertRaisesRegex(RuntimeError, 'final state'):
            flight_evidence(packets, trace)

    def test_claimed_ground_with_airborne_height_is_rejected(self):
        packets, trace = flight_trace()
        next(item for item in reversed(packets) if item['mavpackettype'] == 'GLOBAL_POSITION_INT')['relative_alt'] = 5000
        with self.assertRaisesRegex(RuntimeError, 'final state'):
            flight_evidence(packets, trace)

    def test_later_status_traffic_cannot_make_old_final_state_fresh(self):
        packets, trace = flight_trace()
        packets.append({'mavpackettype': 'STATUSTEXT', 'received_mono': 1000, 'text': 'late FC traffic'})
        with self.assertRaisesRegex(RuntimeError, 'final state'):
            flight_evidence(packets, trace)

    def test_monitor_silence_does_not_freeze_final_freshness(self):
        packets, trace = flight_trace()
        with self.assertRaisesRegex(RuntimeError, 'final state'):
            flight_evidence(packets, trace, monitor_finished_mono=1000)

    def test_explicit_monitor_end_is_recorded_and_must_follow_receipts(self):
        packets, trace = flight_trace()
        proof = flight_evidence(packets, trace, monitor_finished_mono=31.2)
        self.assertEqual(proof['native_rtl']['freshness_scope'], 'monitor_finished_mono')
        self.assertAlmostEqual(proof['native_rtl']['final_observation_ages']['position_s'], .3)
        with self.assertRaisesRegex(RuntimeError, 'precedes'):
            flight_evidence(packets, trace, monitor_finished_mono=30.9)

    def test_nonfinite_public_expiry_and_result_time_are_rejected(self):
        for field in ('expires', 'at'):
            for invalid in ('nan', 'inf', '-inf'):
                with self.subTest(field=field, invalid=invalid):
                    trace = operation_trace()
                    event = next(item for item in trace if 'id=ops-guided-goto,' in item.get('value', ''))
                    event['value'] = event['value'].replace(field+'='+('200' if field == 'expires' else '100'), field+'='+invalid)
                    with self.assertRaisesRegex(RuntimeError, 'nonfinite'):
                        operation_evidence(trace)

    def test_nonfinite_phase_receipt_monitor_and_coordinate_evidence_is_rejected(self):
        trace = operation_trace()
        next(item for item in trace if item.get('type') == 'phase')['mono'] = float('nan')
        with self.assertRaisesRegex(RuntimeError, 'nonfinite'):
            operation_evidence(trace)
        for fault in ('receipt', 'monitor', 'latitude', 'height'):
            packets, trace = flight_trace(); end = 31.2
            if fault == 'receipt': packets[-1]['received_mono'] = float('inf')
            if fault == 'monitor': end = float('nan')
            if fault in ('latitude', 'height'):
                next(item for item in packets if item['mavpackettype'] == 'GLOBAL_POSITION_INT')['lat' if fault == 'latitude' else 'relative_alt'] = float('nan')
            with self.subTest(fault=fault), self.assertRaisesRegex(RuntimeError, 'nonfinite'):
                flight_evidence(packets, trace, monitor_finished_mono=end)

    def test_numeric_bounds_do_not_accept_negative_time_or_invalid_coordinates(self):
        for field, invalid in (('expires', '0'), ('expires', '-1'), ('at', '-1')):
            trace = operation_trace()
            event = next(item for item in trace if 'id=ops-guided-goto,' in item.get('value', ''))
            event['value'] = event['value'].replace(field+'='+('200' if field == 'expires' else '100'), field+'='+invalid)
            with self.subTest(field=field, invalid=invalid), self.assertRaises(RuntimeError):
                operation_evidence(trace)
        for field, value in (('received_mono', -1), ('lat', 900000001), ('lon', -1800000001)):
            packets, trace = flight_trace()
            next(item for item in packets if item['mavpackettype'] == 'GLOBAL_POSITION_INT')[field] = value
            with self.subTest(field=field), self.assertRaises(RuntimeError):
                flight_evidence(packets, trace, monitor_finished_mono=31.2)


if __name__ == '__main__':
    unittest.main()
