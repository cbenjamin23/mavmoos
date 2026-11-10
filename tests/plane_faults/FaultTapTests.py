"""Test dynamic injection and exact loopback tuple behavior with no simulator."""

from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import unittest

from fault_tap import clean_environment, environment, read_trace, set_mode


PROBE, TAP = sys.argv[1:]
sys.argv[1:] = []


def receiver():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind(('127.0.0.1', 0))
    except OSError:
        sock.close()
        raise
    sock.settimeout(.15)
    return sock


class FaultTapTests(unittest.TestCase):
    def test_one_route_pass_drop_hold_release_and_fail_closed(self):
        with tempfile.TemporaryDirectory() as directory, receiver() as peer, receiver() as other:
            trace = Path(directory) / 'wire.bin'
            control = Path(directory) / 'fault.mode'
            set_mode(control, 'pass')
            with receiver() as reservation:
                app_port = reservation.getsockname()[1]
            peer_port, other_port = peer.getsockname()[1], other.getsockname()[1]
            env = clean_environment() | environment(TAP, app_port, peer_port, trace, control)
            with subprocess.Popen([PROBE, str(app_port)], env=env, stdin=subprocess.PIPE,
                                  stdout=subprocess.PIPE, text=True) as probe:
                try:
                    self.assertEqual(probe.stdout.readline().strip(), 'ready')
                    app = ('127.0.0.1', app_port)

                    def command(value, expected='accepted'):
                        probe.stdin.write(value + '\n')
                        probe.stdin.flush()
                        self.assertEqual(probe.stdout.readline().strip(), expected)

                    def send(kind='plain', expected='accepted'):
                        command(f'send {peer_port} {kind}', expected)

                    def empty(sock):
                        with self.assertRaises(socket.timeout):
                            sock.recvfrom(512)

                    send()
                    self.assertEqual(peer.recvfrom(512), (b'x', app))
                    peer.sendto(b'old telemetry', app)
                    command('receive', 'received')

                    set_mode(control, 'drop_rx')
                    peer.sendto(b'new telemetry', app)
                    peer.sendto(b'queued telemetry', app)
                    other.sendto(b'wrong peer', app)
                    command('receive', 'discarded')
                    command('receive', 'discarded')
                    command('receive', 'received')
                    send()
                    self.assertEqual(peer.recvfrom(512)[0], b'x')

                    set_mode(control, 'drop_mode_ack')
                    mode_ack = bytes.fromhex('fd0a00000001014d0000b000000000000000f5bf65f7')
                    interval_ack = bytes.fromhex('fd0a00000001014d0000ff01000000000000f5bf0b0a')
                    other_target = bytes.fromhex('fd0a00000001014d0000b000000000000000f9be1e4b')
                    heartbeat = bytes.fromhex('fd090000000101000000150000001603d90403a4ca')
                    peer.sendto(mode_ack, app)
                    command('receive', 'discarded')
                    for packet in (interval_ack, other_target, heartbeat, mode_ack[:-1] + b'\0'):
                        peer.sendto(packet, app)
                        command('receive', 'received')
                    other.sendto(mode_ack, app)
                    command('receive', 'received')

                    set_mode(control, 'drop_rtl_ack')
                    rtl_ack = bytes.fromhex('fd0a00000001014d00001400000000000000f5bfa1d9')
                    rtl_other_target = bytes.fromhex('fd0a00000001014d00001400000000000000f9beda65')
                    peer.sendto(rtl_ack, app)
                    command('receive', 'discarded')
                    for packet in (mode_ack, rtl_other_target, heartbeat):
                        peer.sendto(packet, app)
                        command('receive', 'received')

                    set_mode(control, 'drop_tx')
                    send()
                    empty(peer)
                    peer.sendto(b'last telemetry', app)
                    command('receive', 'received')
                    command(f'send {other_port} plain')
                    self.assertEqual(other.recvfrom(512), (b'x', app))
                    command(f'other {peer_port} plain')
                    self.assertNotEqual(peer.recvfrom(512)[1], app)

                    set_mode(control, 'drop_loiter')
                    send('loiter')
                    empty(peer)
                    send('rtl')
                    rtl, source = peer.recvfrom(512)
                    self.assertEqual(source, app)
                    send('badloiter')
                    bad, _ = peer.recvfrom(512)
                    self.assertNotEqual(rtl, bad)

                    set_mode(control, 'hold_point')
                    send('point')
                    send('otherpoint')
                    empty(peer)
                    send()
                    self.assertEqual(peer.recvfrom(512)[0], b'x')
                    set_mode(control, 'release_point')
                    # A wrong-tuple send cannot trigger release.
                    command(f'send {other_port} plain')
                    self.assertEqual(other.recvfrom(512)[0], b'x')
                    empty(peer)
                    send()
                    delayed, source = peer.recvfrom(512)
                    self.assertEqual(source, app)
                    self.assertEqual(peer.recvfrom(512)[0], b'x')
                    send()
                    self.assertEqual(peer.recvfrom(512)[0], b'x')

                    records = read_trace(trace, app_port, peer_port)
                    self.assertEqual(''.join(row.disposition for row in records), 'TRiiTiRRRRiRRRoRlTThqTdTT')
                    self.assertEqual(delayed, next(row.data for row in records if row.disposition == 'h'))
                    self.assertEqual(delayed, next(row.data for row in records if row.disposition == 'd'))
                    self.assertEqual([row.data for row in records if row.disposition == 'i'],
                                     [b'new telemetry', b'queued telemetry', mode_ack, rtl_ack])
                    with self.assertRaisesRegex(ValueError, 'tuple changed'):
                        read_trace(trace, other_port, peer_port)

                    control.unlink()
                    send(expected='error')
                    empty(peer)
                    peer.sendto(b'faulted trace', app)
                    command('receive', 'discarded')
                    with self.assertRaisesRegex(ValueError, 'invalid Plane fault capture'):
                        read_trace(trace, app_port, peer_port)
                finally:
                    probe.stdin.write('exit\n')
                    probe.stdin.flush()
                    self.assertEqual(probe.wait(timeout=3), 0)


if __name__ == '__main__':
    unittest.main()
