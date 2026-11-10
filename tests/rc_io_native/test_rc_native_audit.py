#!/usr/bin/env python3
"""Offline negative controls. No sockets, processes or native simulation."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch
import RcNativeAudit as audit
import native_support as support


def q(raw, known=True, unit='us'):
    return {'known': known, 'value': raw if known else None, 'unit': unit, 'reason': 'fixture'}


def observation(family, elapsed, oid, port=0, phase='A'):
    base = {'schema': 1, 'session_epoch': 1, 'family': family, 'present': True, 'encoding_valid': True, 'fresh': True,
            'effective_at': 1000 + elapsed - .01, 'read_at': 1000 + elapsed, 'age_s': .02, 'age_basis': 'QUEUE_EMPTY_FLOOR',
            'age_is_upper_bound': True, 'source_age': 'unknown', 'source_quality': 'unassessed', 'observation_id': oid}
    if family == 'RC_CHANNELS':
        raw = [1500, 1500, 1000, 1500, 1500, 1500, 1500, 1500] + [0] * 10
        if phase == 'B':raw[0], raw[3] = 1100, 1700
        if phase == 'C':raw[0], raw[3] = 1900, 1300
        base['data'] = {'time_boot_ms': int(100000 + elapsed * 1000), 'remote_clock': 'boot_ms_unmapped_uint32', 'wire_version': 2,
                        'reported_channel_count': 8, 'visible_channel_count': 8, 'channels_beyond_message': False,
                        'channel_roles': 'unmapped', 'pulse_validity': 'unassessed', 'rssi': {'raw': 255, 'quantity': q(255, False, 'device_dependent')},
                        'channels': [{'slot': i, 'raw_us': value, 'reported_by_count': i <= 8, 'pwm_us': q(value, i <= 8)} for i, value in enumerate(raw, 1)]}
    else:
        raw = [1000 + i for i in range(16)] if port == 0 else [0] * 16
        base['data'] = {'time_usec': int(100000000 + elapsed * 1000000), 'remote_clock': 'unmapped_uint32_us_no_epoch_inference',
                        'port': port, 'wire_version': 2, 'extension_bytes_observed': True, 'output_kind': 'fc_command_observation',
                        'physical_feedback': False, 'actuator_mapping': 'unmapped_port_and_slot', 'pulse_validity': 'unassessed',
                        'channels': [{'slot': i, 'raw_us': value, 'extension': i > 8, 'pwm_us': q(value, value != 0)} for i, value in enumerate(raw, 1)]}
    return base


def fixture():
    catalog = {'schema': 1, 'execution_mode': 'RX_ONLY', 'composition': {'plugin_abi': 10, 'modules': [
        {'name': 'telemetry', 'control_provider': False, 'request_owners': [], 'profile': 'telemetry', 'capabilities': []},
        {'name': 'rc_io', 'profile': 'ardupilot_rc_observations', 'control_provider': False, 'request_owners': [],
         'capabilities': [{'name': name, 'access': 'observation', 'request_variable': ''} for name in ('rc.input', 'rc.servo_output')]}]}}
    events = [];packets = [];df = {'RCIN': [], 'RCOU': []}
    def event(elapsed, key, value):
        events.append({'type': 'observation', 'elapsed': elapsed, 'source': 'iMAVLink', 'key': key, 'value': json.dumps(value) if isinstance(value, dict) else value})
    event(0, 'MAVMOOS_CAPABILITIES', catalog)
    for elapsed in range(36):
        phase = ('A', 'B', 'C', 'D')[min(3, elapsed // 8)]
        if elapsed in (0, 8, 16, 24):events.append({'type': 'phase', 'elapsed': elapsed, 'phase': phase})
        record = observation('RC_CHANNELS', elapsed + .2, 3 * elapsed + 1, phase=phase)
        event(elapsed + .2, 'MAVMOOS_RC_INPUT', record)
        servos = [observation('SERVO_OUTPUT_RAW', elapsed + .3, 3 * elapsed + 2 + port, port) for port in (0, 1)]
        event(elapsed + .3, 'MAVMOOS_SERVO_OUTPUT', {'schema': 1, 'session_epoch': 1, 'present': True, 'atomic': False, 'cache_capacity': 8, 'ports': servos})
        event(elapsed + .4, 'MAVMOOS_RC_IO_STATUS', {'schema': 1, 'read_only': True, 'authoritative_for_control': False, 'atomic': False,
                                                  'state': 'BOUND', 'system_id': 1, 'component_id': 1})
        event(elapsed + .5, 'MAVMOOS_RUNTIME_STATUS', 'mode=RX_ONLY,permission=false,control_profile=none,tx_attempts=0,tx_accepted=0,nav_tx_accepted=0,armed=0,in_air=0,heartbeat_valid=true')
        values = [row['raw_us'] for row in record['data']['channels']]
        packets.append({'mavpackettype': 'RC_CHANNELS', 'time_boot_ms': record['data']['time_boot_ms'], 'chancount': 8, 'rssi': 255,
                        **{f'chan{i}_raw': value for i, value in enumerate(values, 1)}})
        df['RCIN'].append({'TimeUS': record['data']['time_boot_ms'] * 1000, **{f'C{i}': value for i, value in enumerate(values[:8], 1)}})
        for servo in servos:
            values = [row['raw_us'] for row in servo['data']['channels']]
            packets.append({'mavpackettype': 'SERVO_OUTPUT_RAW', 'port': servo['data']['port'], 'time_usec': servo['data']['time_usec'],
                            **{f'servo{i}_raw': value for i, value in enumerate(values, 1)}})
            if servo['data']['port'] == 0:df['RCOU'].append({'TimeUS': servo['data']['time_usec'], **{f'C{i}': value for i, value in enumerate(values, 1)}})
    events.append({'type': 'end', 'elapsed': 36, 'records': len(events)})
    return events, packets, df


class AuditTest(unittest.TestCase):
    def setUp(self):self.events, self.packets, self.df = fixture()

    def change(self, key, mutate, index=-1):
        row = [row for row in self.events if row.get('key') == key][index]
        value = json.loads(row['value']);mutate(value);row['value'] = json.dumps(value)

    def reject(self):
        with self.assertRaises(ValueError):audit.audit(self.events)

    def test_actual_values_and_both_raw_ports(self):
        proof = audit.audit(self.events);result = audit.corroborate(proof, self.packets, self.df)
        self.assertEqual(len(result['windows']), 4)
        self.assertTrue(all(row['servo_ports'] == [0, 1] for row in proof['windows']))

    def test_rover_input_switch_observation_reuses_control_workflow(self):
        phases = []
        for event in self.events:
            if event.get('type') == 'phase':
                if event['elapsed'] in (0, 16):
                    event['phase'] = 'INITIAL_HOLD' if event['elapsed'] == 0 else 'MANUAL_SWITCH';phases.append(event)
                continue
            if event.get('key') == 'MAVMOOS_CAPABILITIES':
                item = json.loads(event['value']);item['execution_mode'] = 'SITL_CONTROL'
                item['composition']['modules'] += [{'name': 'parameters'}, {'name': 'ardurover_control'}]
                event['value'] = json.dumps(item)
            if event.get('key') == 'MAVMOOS_RUNTIME_STATUS':event['value'] = event['value'].replace('mode=RX_ONLY', 'mode=SITL_CONTROL')
            if event.get('key') == 'MAVMOOS_RC_INPUT':
                item = json.loads(event['value'])
                for slot, value in ((1, 1500), (3, 1500), (4, 1500), (8, 1500 if event['elapsed'] < 16 else 1000)):
                    item['data']['channels'][slot - 1].update(raw_us=value, pwm_us=q(value))
                event['value'] = json.dumps(item)
            phases.append(event)
        self.events = phases
        for row in self.packets:
            if row['mavpackettype'] == 'RC_CHANNELS':row.update(chan1_raw=1500, chan3_raw=1500, chan4_raw=1500, chan8_raw=1500 if row['time_boot_ms'] < 116000 else 1000)
        for row in self.df['RCIN']:row.update(C1=1500, C3=1500, C4=1500, C8=1500 if row['TimeUS'] < 116000000 else 1000)
        proof = audit.audit(self.events, mode='SITL_CONTROL')
        self.assertEqual([row['phase'] for row in proof['windows']], ['INITIAL_HOLD', 'MANUAL_SWITCH'])
        self.assertEqual(audit.corroborate(proof, self.packets, self.df)['result'], 'pass')

    def test_early_only_fresh_values_rejected(self):
        self.events = [row for row in self.events if row.get('key') != 'MAVMOOS_RC_INPUT' or row['elapsed'] < 2]
        self.reject()

    def test_late_republication_does_not_count_as_acquisition(self):
        saved = json.loads(next(row['value'] for row in self.events if row.get('key') == 'MAVMOOS_RC_INPUT'))
        for row in self.events:
            if row.get('key') == 'MAVMOOS_RC_INPUT':row['value'] = json.dumps(saved)
        self.reject()

    def test_wrong_late_phase_value(self):
        def mutation(v):v['data']['channels'][0].update(raw_us=1550, pwm_us=q(1550))
        self.change('MAVMOOS_RC_INPUT', mutation);self.reject()

    def test_quantity_mismatch(self):
        self.change('MAVMOOS_RC_INPUT', lambda v:v['data']['channels'][0]['pwm_us'].update(value=0));self.reject()

    def test_count_mismatch(self):
        self.change('MAVMOOS_RC_INPUT', lambda v:v['data'].update(visible_channel_count=9));self.reject()

    def test_unknown_rssi_cannot_be_known(self):
        self.change('MAVMOOS_RC_INPUT', lambda v:v['data']['rssi'].update(quantity=q(255, True, 'device_dependent')));self.reject()

    def test_zero_output_cannot_be_claimed_known(self):
        self.change('MAVMOOS_SERVO_OUTPUT', lambda v:v['ports'][1]['data']['channels'][0].update(pwm_us=q(0)));self.reject()

    def test_absent_mavlink1_extension_cannot_be_zero(self):
        self.change('MAVMOOS_SERVO_OUTPUT', lambda v:v['ports'][1]['data'].update(wire_version=1, extension_bytes_observed=False));self.reject()

    def test_port_identity_collision(self):
        self.change('MAVMOOS_SERVO_OUTPUT', lambda v:v['ports'][1]['data'].update(port=0));self.reject()

    def test_slot_identity_collision(self):
        self.change('MAVMOOS_SERVO_OUTPUT', lambda v:v['ports'][0]['data']['channels'][1].update(slot=1));self.reject()

    def test_invented_feedback(self):
        self.change('MAVMOOS_SERVO_OUTPUT', lambda v:v['ports'][0]['data'].update(physical_feedback=True));self.reject()

    def test_invented_source_clock(self):
        self.change('MAVMOOS_RC_INPUT', lambda v:v.update(source_age=0));self.reject()

    def test_fresh_over_age(self):
        self.change('MAVMOOS_RC_INPUT', lambda v:v.update(age_s=2));self.reject()

    def test_lost_queue_floor(self):
        self.change('MAVMOOS_RC_INPUT', lambda v:v.update(age_is_upper_bound=False));self.reject()

    def test_stamp_order(self):
        self.change('MAVMOOS_RC_INPUT', lambda v:v.update(effective_at=v['read_at'] + 1));self.reject()

    def test_same_identifier_cannot_renew(self):
        rows = [row for row in self.events if row.get('key') == 'MAVMOOS_RC_INPUT']
        first = json.loads(rows[-2]['value']);last = json.loads(rows[-1]['value']);last['observation_id'] = first['observation_id'];rows[-1]['value'] = json.dumps(last)
        self.reject()

    def test_wrong_publisher(self):
        next(row for row in self.events if row.get('key'))['source'] = 'anotherApp';self.reject()

    def test_missing_final_marker(self):
        self.events.pop();self.reject()

    def test_nonzero_read_only_send(self):
        row = next(row for row in self.events if row.get('key') == 'MAVMOOS_RUNTIME_STATUS');row['value'] = row['value'].replace('tx_attempts=0', 'tx_attempts=1');self.reject()

    def test_no_channel_authority(self):
        self.change('MAVMOOS_RC_IO_STATUS', lambda v:v.update(authoritative_for_control=True));self.reject()

    def test_no_plausible_range_substitute(self):
        proof = audit.audit(self.events)
        for row in self.packets:
            if row['mavpackettype'] == 'RC_CHANNELS':row['chan1_raw'] = 1501
        with self.assertRaises(ValueError):audit.corroborate(proof, self.packets, self.df)

    def test_no_early_only_fc_corroboration(self):
        proof = audit.audit(self.events)
        with self.assertRaises(ValueError):audit.corroborate(proof, self.packets[:8], self.df)

    def test_no_wrong_servo_port_pair(self):
        proof = audit.audit(self.events)
        self.packets = [row for row in self.packets if row.get('port') != 1]
        with self.assertRaises(ValueError):audit.corroborate(proof, self.packets, self.df)

    def test_dataflash_contradiction(self):
        proof = audit.audit(self.events)
        for row in self.df['RCOU']:row['C1'] = 1900
        with self.assertRaises(ValueError):audit.corroborate(proof, self.packets, self.df)

    def test_nonfinite_and_duplicate_json(self):
        for text in ('{"x":NaN}', '{"x":1e999}', '{"x":1,"x":2}'):
            with self.assertRaises(ValueError):audit.parse(text)


class SourceAndOwnershipTest(unittest.TestCase):
    def test_actual_source_root_helpers_ignore_cached_revision(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            for revision in ('one', 'two'):
                source = root / revision;(source / 'tests/control').mkdir(parents=True)
                (source / 'tests/owned_processes.py').write_text('revision = ' + repr(revision) + '\n')
                ((source / 'tests/control/catalog_evidence.py').resolve()).write_text('revision = ' + repr(revision) + '\n')
                modules = support.source_helpers(source)
                self.assertEqual(modules['catalog'].revision, revision)
                self.assertEqual(Path(modules['catalog'].__file__), (source / 'tests/control/catalog_evidence.py').resolve())

    def test_no_source_helper_symlink_escape(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary);(root / 'source/tests/control').mkdir(parents=True);(root / 'other.py').write_text('x=1\n')
            (root / 'source/tests/owned_processes.py').symlink_to(root / 'other.py')
            with self.assertRaises(RuntimeError):support.source_helpers(root / 'source')

    def test_port_prefix_and_wildcard_distinction(self):
        self.assertFalse(support.owns_udp('UDP 127.0.0.1:12345', 1234))
        self.assertFalse(support.owns_udp('UDP *:1234', 1234))
        self.assertTrue(support.owns_udp('UDP *:1234', 1234, True))
        self.assertFalse(support.exact_peer('UDP 127.0.0.1:1234->127.0.0.1:56789', 1234, 5678))

    @patch.object(support.subprocess, 'run')
    def test_app_identity_and_socket_must_remain_owned(self, run):
        ownership = Mock();ownership.owned_records.return_value = [{'pid': 2, 'command': '/bin/iMAVLink /owned/mission.moos'}]
        run.return_value = Mock(returncode=0, stdout='UDP 127.0.0.1:1234')
        launcher = Mock();mission = Path('/owned/mission.moos')
        result, _ = support.live_app(ownership, launcher, mission, 1234)
        with self.assertRaises(RuntimeError):support.live_app(ownership, launcher, mission, 1234, {'pid': 3, 'command': result['iMAVLink']['command']})
        ownership.owned_records.return_value = []
        with self.assertRaises(RuntimeError):support.live_app(ownership, launcher, mission, 1234)


if __name__ == '__main__':unittest.main()
