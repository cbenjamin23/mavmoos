"""Deterministic controls for the actual closed-case replay."""
import copy
import hashlib
import json
from pathlib import Path
import struct
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import evidence
import run_native


def checksum(data):
    # Independent bitwise form of CRC-16/MCRF4XX.
    value = 0xffff
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ (0x8408 if value & 1 else 0)
    return value


def frame(message_id, values, *, source=1, legacy=False):
    _, fmt, extra, _ = evidence.MESSAGE_FIELDS[message_id]
    payload = struct.pack(fmt, *values)
    if legacy:
        payload = payload[:3] if message_id == 77 else payload
        header = bytes([len(payload), 7, source, 1, message_id])
    else:
        payload = payload.rstrip(b'\0')
        header = bytes([len(payload), 0, 0, 7, source, 1]) + message_id.to_bytes(3, 'little')
    return bytes([0xfe if legacy else 0xfd]) + header + payload + struct.pack('<H', checksum(header + payload + bytes([extra])))


def lines(rows):
    return b''.join(json.dumps(row, allow_nan=False).encode() + b'\n' for row in rows)


def catalog():
    return {'schema': 1, 'execution_mode': 'SITL_CONTROL', 'composition': {'plugin_abi': 10, 'modules': [
        {'name': 'arducopter_control', 'profile': 'arducopter', 'control_provider': True, 'capabilities': [],
         'request_owners': ['MAVMOOS_COMMAND', 'MAVMOOS_NAV_CONTROL', 'MAVMOOS_CONTROL_REQUEST', 'MAVMOOS_MISSION_CONTROL']},
        {'name': 'example_disarmed_command', 'control_provider': False, 'request_owners': [evidence.REQUEST],
         'capabilities': [{'name': 'example.disarmed_command.disarm', 'request_variable': evidence.REQUEST,
             'request_value': 'schema=1,id=<id>,action=disarm,timeout=<seconds>', 'access': 'request'}]},
    ]}}


class Fixture:
    def __init__(self):
        self.probe = []
        self.packets = []
        self.raw = {
            'result': 'PASS', 'scenario': 'disarmed_command', 'profile': 'arducopter', 'hardware': False,
            'probe_exit': 0, 'ports_clear': {key: True for key in evidence.PORTS}, 'cleanup_proven': True,
            'owned_processes_remaining': [], 'artifact_drift': [], 'source_ref': 'f' * 40, 'source_ref_after': 'f' * 40,
            'moos_ref': 'a' * 40, 'moos_ref_after': 'a' * 40, 'rc_input_sent': False, 'relay_used': False,
            'firmware_identity': {'profile_id': 'test'}, 'firmware_identity_after': {'profile_id': 'test'},
            'capture': {'thread_retired': True, 'socket_closed': True, 'read_only': True, 'error': None},
            'chronology': {'probe_started': 12, 'probe_exit_observed': 14, 'capture_finished': 16},
        }
        self.mail('MAVMOOS_CAPABILITIES', json.dumps(catalog()))
        self.status()
        for index, (phase, source, request, age) in enumerate(evidence.REQUESTS):
            self.event('request', phase=phase, key=evidence.REQUEST, source=source, value=request,
                       moos_sample_at=1_789_000_000, moos_published_at=1_789_000_000 - age)
            if index != 4:
                request_id, reason = evidence.NEGATIVES[index if index < 4 else 4]
                self.report(id=request_id, status='REJECTED', reason=reason, observed_at=99 + index / 2,
                            remote_effect_uncertain=False)
                continue
            ack = frame(77, (400, 0, 0, 0, 245, 191)).hex()
            for n, (state, now) in enumerate(zip(evidence.PHASES, (100.1, 100.12, 100.14, 100.17, 100.9))):
                extras = {} if n < 3 else {'ack_effective_at': 100.12, 'ack_read_at': 100.17,
                    'ack_age_basis': 'QUEUE_EMPTY_FLOOR', 'ack_frame_hex': ack}
                if n == 2:
                    self.mail('MAVMOOS_TX_RESULT', 'id=example.disarm.positive,status=LOCAL_ACCEPTED')
                self.report(id='positive', wire_id='example.disarm.positive', status=state, observed_at=now,
                            published_at=100, operation_expires=105, packet_expires=101.2,
                            handoff_at=100.14 if n >= 2 else None, heartbeat_at=100.8 if n == 4 else 99.7,
                            landed_at=100.5 if n == 4 else 99.9, disarmed_ground_fresh=True,
                            remote_effect_uncertain=n in (2, 3), **extras)
        self.status()
        self.event('done')
        for n in range(11):
            at = 10.5 + n / 2
            self.packets.extend((self.sample(at, 0, (0, 2, 3, 0, 3, 3)),
                                 self.sample(at, 245, (0, 1)),
                                 self.sample(at, 33, (100 * n, 423584560, -710875890, 10000, 0, 0, 0, 0, 0))))
        self.files = self.streams()
        self.raw['files'] = self.hashes()
        self.raw['command_evidence'] = evidence.audit(self.raw, self.files.__getitem__)

    def event(self, kind, **fields):
        self.probe.append({'event': kind, 'elapsed': len(self.probe) / 25, **fields})

    def mail(self, key, value):
        self.event('mail', key=key, source='iMAVLink', value=value)

    def status(self):
        self.mail('MAVMOOS_RUNTIME_STATUS', 'mode=SITL_CONTROL,control_profile=arducopter,configuration_valid=true,'
                  'permission=false,armed=0,in_air=0,nav_tx_accepted=0,mission_execution_held=false,'
                  'mission_mutations=0,command_leases=0,heartbeat_valid=1,landed_valid=1')

    def report(self, **fields):
        self.mail(evidence.RESULT, json.dumps({'schema': 1, **fields}))

    def mutate_report(self, state, **fields):
        for event in self.probe:
            if event.get('key') != evidence.RESULT:
                continue
            payload = json.loads(event['value'])
            if payload.get('status') == state:
                payload.update(fields)
                event['value'] = json.dumps(payload)
                return
        raise AssertionError('mutation target not found')

    @staticmethod
    def sample(at, message_id, values, *, source=1):
        kind, _, _, keys = evidence.MESSAGE_FIELDS[message_id]
        return (frame(message_id, values, source=source),
                {'mono': at, 'kind': kind, 'system': source, 'component': 1, **dict(zip(keys, values))})

    def streams(self):
        return {'probe.jsonl': lines(self.probe), 'fc-observer.jsonl': lines(row for _, row in self.packets),
                'fc-observer.mavlink': b'0 0 0 ' + b''.join(raw for raw, _ in self.packets)}

    def hashes(self):
        return {name: hashlib.sha256(data).hexdigest() for name, data in self.files.items()}

    def replay(self, update=True):
        if update:
            self.files = self.streams()
            self.raw['files'] = self.hashes()
        return evidence.replay_archived(self.raw, self.files.__getitem__, expected_abi=10)


class ReplayTests(unittest.TestCase):
    def setUp(self):
        self.case = Fixture()

    def test_complete(self):
        proof = self.case.replay()
        self.assertEqual(proof['probe']['local_command_handoffs'], 1)
        self.assertEqual(proof['probe']['ack']['command'], 400)
        self.assertEqual(proof['passive_fc']['max_displacement_m'], 0)
        self.assertFalse(proof['passive_fc']['armed'])

    def test_actual_ack_frame_encoding_controls(self):
        legacy, v2 = (frame(77, (400, 0, 0, 0, 245, 191), legacy=old) for old in (True, False))
        self.assertEqual(evidence.wire_projection(legacy, ack=True)[0][0]['target_component'], 0)
        self.assertEqual(evidence.wire_projection(v2, ack=True)[0][0]['target_component'], 191)
        for changed in (v2[:-1], v2[:-1] + bytes([v2[-1] ^ 1]), v2 + v2, frame(77, (400, 0, 0, 0, 245, 191), source=2)):
            with self.subTest(changed=changed.hex()), self.assertRaises(ValueError):
                evidence.wire_projection(changed, ack=True)

    def test_ack_is_not_observed_completion(self):
        self.case.mutate_report('COMPLETED', heartbeat_at=100.14)
        with self.assertRaisesRegex(ValueError, 'pre-handoff'):
            self.case.replay()

    def test_original_deadline_cannot_renew(self):
        self.case.mutate_report('SENT', operation_expires=106)
        with self.assertRaisesRegex(ValueError, 'deadline drift'):
            self.case.replay()

    def test_pre_handoff_or_equal_read_is_not_ack(self):
        self.case.mutate_report('ACKNOWLEDGED', ack_read_at=100.14)
        with self.assertRaisesRegex(ValueError, 'handoff/ACK'):
            self.case.replay()

    def test_wrong_ack_target(self):
        changed = frame(77, (400, 0, 0, 0, 246, 191)).hex()
        self.case.mutate_report('ACKNOWLEDGED', ack_frame_hex=changed)
        self.case.mutate_report('COMPLETED', ack_frame_hex=changed)
        with self.assertRaisesRegex(ValueError, 'recipient differs'):
            self.case.replay()

    def test_wrong_source(self):
        event = next(row for row in self.case.probe if row.get('phase') == 'foreign')
        event['source'] = evidence.SOURCE
        with self.assertRaisesRegex(ValueError, 'payload or source'):
            self.case.replay()

    def test_extra_command(self):
        probe = self.case.probe
        index = next(i for i, row in enumerate(probe) if row.get('key') == 'MAVMOOS_TX_RESULT')
        probe.insert(index, copy.copy(probe[index]))
        with self.assertRaisesRegex(ValueError, 'exactly one'):
            self.case.replay()

    def test_wrong_abi(self):
        event = self.case.probe[0]
        composition = json.loads(event['value'])
        composition['composition']['plugin_abi'] = 8
        event['value'] = json.dumps(composition)
        with self.assertRaisesRegex(ValueError, 'wrong module ABI'):
            self.case.replay()

    def test_enabled_ordinary_owner(self):
        event = self.case.probe[0]
        composition = json.loads(event['value'])
        composition['composition']['modules'][0]['capabilities'] = [{'name': 'control.arm'}]
        event['value'] = json.dumps(composition)
        with self.assertRaisesRegex(ValueError, 'facets'):
            self.case.replay()

    def test_armed_passive_frame(self):
        self.case.packets[9] = self.case.sample(12, 0, (0, 2, 3, 128, 3, 3))
        with self.assertRaisesRegex(ValueError, 'arm, mode'):
            self.case.replay()

    def test_neutral_projection_at_zero_is_not_qualified_home(self):
        self.case.packets[11] = self.case.sample(12, 33, (300, 0, 0, 10000, 0, 0, 0, 0, 0))
        with self.assertRaisesRegex(ValueError, 'position unavailable'):
            self.case.replay()

    def test_motion(self):
        self.case.packets[11] = self.case.sample(12, 33, (300, 423584560, -710875890, 10000, 0, 81, 0, 0, 0))
        with self.assertRaisesRegex(ValueError, 'indicates movement'):
            self.case.replay()

    def test_missing_passive_post_window(self):
        self.case.raw['chronology']['probe_exit_observed'] = 15.75
        with self.assertRaisesRegex(ValueError, 'receive window'):
            self.case.replay()

    def test_passive_payload_mutation(self):
        self.case.packets[11][1]['lat'] += 1
        with self.assertRaisesRegex(ValueError, 'bytes differ'):
            self.case.replay()

    def test_unproven_cleanup(self):
        for key, value in (('owned_processes_remaining', None), ('cleanup_proven', False), ('ports_clear', {})):
            with self.subTest(key=key):
                case = Fixture()
                case.raw[key] = value
                with self.assertRaisesRegex(ValueError, 'cleanup unproven'):
                    case.replay()

    def test_changed_index(self):
        self.case.raw['files']['probe.jsonl'] = '0' * 64
        with self.assertRaisesRegex(ValueError, 'file hash'):
            self.case.replay(update=False)

    def test_asserted_pass_without_replay_binding(self):
        self.case.raw['command_evidence']['passive_fc']['max_speed_m_s'] = .1
        with self.assertRaisesRegex(ValueError, 'replayed command evidence differs'):
            self.case.replay()


class OwnershipTests(unittest.TestCase):
    def setUp(self):
        self.mission = Path('/fixture/targ_external_command.moos')
        self.moos = Path('/fixture/moos')
        self.app = Path('/fixture/build/bin/iMAVLink')
        self.antler = SimpleNamespace(pid=41)
        self.roster = [
            {'pid': 41, 'parent': 40, 'group': 41,
             'command': f'{self.moos}/bin/pAntler {self.mission}'},
            {'pid': 42, 'parent': 41, 'group': 42,
             'command': f'MOOSDB {self.mission} MOOSDB'},
            {'pid': 43, 'parent': 41, 'group': 43,
             'command': f'{self.app} {self.mission} iMAVLink'},
        ]

    def inspect(self, sockets=('UDP 127.0.0.1:14560', 'TCP *:19131 (LISTEN)')):
        ownership = SimpleNamespace(owned_records=lambda _: self.roster)
        with patch.object(run_native, 'inventory', side_effect=sockets) as inventory:
            apps, proof = run_native.owned_apps(ownership, self.antler, self.mission, self.app, self.moos, 14560, 19131)
        self.assertEqual([call.args[0] for call in inventory.call_args_list], [43, 42])
        return apps, proof

    def test_exact_launcher_and_both_app_sockets(self):
        apps, proof = self.inspect()
        self.assertEqual({name: row['pid'] for name, row in apps.items()}, {'MOOSDB': 42, 'iMAVLink': 43})
        self.assertEqual(set(proof), {'app_udp', 'db_tcp'})

    def test_another_launcher_is_not_the_owned_parent(self):
        self.roster.append({**self.roster[0], 'pid': 44})
        with self.assertRaisesRegex(RuntimeError, 'unexpected app'):
            self.inspect()

    def test_same_mission_arguments_do_not_prove_parentage(self):
        self.roster[1]['parent'] = 40
        with self.assertRaisesRegex(RuntimeError, 'wrong owned app'):
            self.inspect()

    def test_wrong_db_port_is_rejected(self):
        with self.assertRaisesRegex(RuntimeError, 'socket not ready'):
            self.inspect(('UDP 127.0.0.1:14560', 'TCP *:191310 (LISTEN)'))


if __name__ == '__main__':
    unittest.main(verbosity=2)
