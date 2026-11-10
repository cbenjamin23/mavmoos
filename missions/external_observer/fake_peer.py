"""Local test source, not a vehicle backend. Standard unsigned MAVLink 1 HB.

The tiny fixture encoder is checked against a pinned golden packet and the
actual generated-MAVLink host decoder. It requires only Python's standard
library. The peer owns one loopback socket; capture runs independently of the
supervisor's process inventories. No serial, router, simulator or FC is used.
"""
import json
import math
import socket
import struct
import threading
import time


def heartbeat(sequence, *, system=1, component=1, custom_mode=42):
    for value in (sequence, system, component):
        if type(value) is not int or not 0 <= value <= 255:
            raise ValueError('invalid fixture header byte')
    if not system or not component or type(custom_mode) is not int or not 0 <= custom_mode <= 0xffffffff:
        raise ValueError('invalid fixture source/mode')
    payload = struct.pack('<IBBBBB', custom_mode, 2, 3, 1, 3, 3)
    body = bytes((len(payload), sequence, system, component, 0)) + payload
    checksum = 0xffff
    for byte in body + bytes((50,)):  # Standard HEARTBEAT CRC extra.
        temporary = byte ^ (checksum & 255)
        temporary ^= (temporary << 4) & 255
        checksum = ((checksum >> 8) ^ (temporary << 8) ^ (temporary << 3) ^ (temporary >> 4)) & 0xffff
    return bytes((0xfe,)) + body + struct.pack('<H', checksum)


def phase(elapsed):
    """Finite source schedule; silence crosses both example and host expiry."""
    if 0 <= elapsed < 1.5:
        return 'initial', 1, 42
    if 2.0 <= elapsed < 2.4:
        return 'foreign_target', 2, 99
    if 4.5 <= elapsed < 6.0:
        return 'recovered', 1, 84
    return 'silence', None, None


class HeartbeatPeer:
    def __init__(self, peer_port, bind_port, output, duration, clock=time.monotonic):
        if not math.isfinite(duration) or not 8 <= duration <= 30:
            raise ValueError('duration outside 8..30 seconds')
        self.clock, self.duration, self.output = clock, duration, output
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.socket.bind(('127.0.0.1', peer_port))
            self.bound_peer = self.socket.getsockname()
            self.socket.settimeout(.02)
        except BaseException:
            self.socket.close()
            raise
        self.destination = ('127.0.0.1', bind_port)
        self.stop = threading.Event()
        self.thread = None
        self.error = None
        self.sent, self.received, self.bytes_received = 0, 0, 0
        self.first_read = self.last_read = None
        self.started = None

    def start(self):
        if self.thread is not None:
            raise RuntimeError('peer already started')
        self.started = self.clock()
        self.thread = threading.Thread(target=self._run, name='owned-example-heartbeat-peer', daemon=True)
        self.thread.start()

    def _run(self):
        sequence, next_send, previous, wrong_sent = 0, 0., -1., False
        try:
            with (self.output/'peer-events.jsonl').open('x') as events, (self.output/'primary-egress.bin').open('xb') as raw:
                while not self.stop.is_set():
                    at = self.clock()
                    elapsed = at - self.started
                    if not math.isfinite(at) or elapsed < previous:
                        raise RuntimeError('peer clock invalid/regressed')
                    previous = elapsed
                    kind, source, mode = phase(elapsed)
                    if elapsed >= 2.7 and not wrong_sent:
                        with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as wrong:
                            wrong.bind(('127.0.0.1',0))
                            data = heartbeat(sequence,custom_mode=99)
                            wrong.sendto(data,self.destination)
                            events.write(json.dumps({'kind':'sent','mono':self.clock(),'elapsed':elapsed,
                                                     'phase':'wrong_peer','system_id':1,'custom_mode_raw':99,
                                                     'sender':list(wrong.getsockname()),'bytes_hex':data.hex()})+'\n')
                            events.flush()
                        wrong_sent = True
                        sequence = (sequence+1)%256
                    if elapsed < self.duration and source and elapsed >= next_send:
                        data = heartbeat(sequence, system=source, custom_mode=mode)
                        self.socket.sendto(data, self.destination)
                        sequence = (sequence + 1) % 256
                        self.sent += 1
                        events.write(json.dumps({'kind':'sent', 'mono':self.clock(), 'elapsed':elapsed,
                                                 'phase':kind, 'system_id':source, 'custom_mode_raw':mode,
                                                 'bytes_hex':data.hex()})+'\n')
                        events.flush()
                        next_send = elapsed + .1
                    try:
                        data, sender = self.socket.recvfrom(65535)
                    except socket.timeout:
                        continue
                    read = self.clock()
                    if not math.isfinite(read) or read < at:
                        raise RuntimeError('peer receive clock invalid/regressed')
                    self.received += 1
                    self.bytes_received += len(data)
                    if self.received > 2048 or self.bytes_received > 1024*1024:
                        raise RuntimeError('primary egress evidence bound exceeded')
                    self.first_read = read if self.first_read is None else self.first_read
                    self.last_read = read
                    raw.write(data)
                    raw.flush()
                    events.write(json.dumps({'kind':'received', 'mono':read, 'sender':list(sender),
                                             'size':len(data), 'bytes_hex':data.hex()})+'\n')
                    events.flush()
        except BaseException as error:
            self.error = f'{type(error).__name__}: {error}'

    def check(self):
        if self.error:
            raise RuntimeError('fake peer failed: '+self.error)
        if self.thread and not self.thread.is_alive() and not self.stop.is_set():
            raise RuntimeError('fake peer retired unexpectedly')

    def close(self):
        self.stop.set()
        if self.thread:
            self.thread.join(timeout=1)
        retired = self.thread is None or not self.thread.is_alive()
        self.socket.close()
        if not retired:
            raise RuntimeError('fake peer thread not retired')
        if self.error:
            raise RuntimeError('fake peer failed: '+self.error)

    def status(self):
        return {'read_only_fixture':True, 'peer':list(self.bound_peer),
                'sent_heartbeats':self.sent, 'received_datagrams':self.received,
                'received_bytes':self.bytes_received, 'first_read_at':self.first_read,
                'last_read_at':self.last_read, 'error':self.error,
                'thread_retired':self.thread is None or not self.thread.is_alive(),
                'socket_closed':self.socket.fileno()<0}
