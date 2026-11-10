"""Bounded, receive-only fixture capture, independent of coordinator polling.

Timestamps are sampled immediately after each actual recvfrom. They are not
kernel arrival times and are never reconstructed from FC boot or main-loop time.
The caller supplies the mandatory initial owned-peer proof and byte consumer.
"""
import math
import socket
import threading
import time


class PassiveReceiver:
    def __init__(self, stream, verify_peer, consume, *, max_datagrams=50000, clock=time.monotonic):
        if not callable(verify_peer) or not callable(consume) or not callable(clock):
            raise ValueError('passive capture requires peer proof, consumer and clock')
        if type(max_datagrams) is not int or max_datagrams <= 0:
            raise ValueError('passive datagram bound must be positive')
        if stream.family != socket.AF_INET or stream.type != socket.SOCK_DGRAM or stream.getblocking() or stream.getsockname()[0] != '127.0.0.1':
            raise ValueError('passive fixture requires an already bound nonblocking loopback UDP socket')
        self._stream,self._verify,self._consume,self._clock=stream,verify_peer,consume,clock
        self._maximum=max_datagrams
        self._stop=threading.Event()
        self._thread=None
        self._error=None
        self.peer=None
        self.peer_proof=None
        self.datagrams=0
        self.accepted=0
        self.first_read_at=None
        self.last_read_at=None
        self._closed=False

    def start(self):
        if self._thread is not None or self._closed:
            raise RuntimeError('passive receiver cannot be restarted')
        # Retirement is always explicitly joined. A failed, stuck callback is
        # reported as uncertain cleanup rather than hanging interpreter exit.
        self._thread=threading.Thread(target=self._run,name='mavmoos-passive-receive',daemon=True)
        self._thread.start()

    def _run(self):
        try:
            while not self._stop.is_set():
                try:
                    data,sender=self._stream.recvfrom(65535)
                except BlockingIOError:
                    self._stop.wait(.005)
                    continue
                at=self._clock()
                if type(at) not in (int,float) or not math.isfinite(at) or at < 0 or (self.last_read_at is not None and at < self.last_read_at):
                    raise RuntimeError('passive read clock invalid/regressed')
                self.datagrams+=1
                if self.datagrams > self._maximum:
                    raise RuntimeError('independent datagram bound exceeded')
                if self.peer is None:
                    if sender[0] != '127.0.0.1':
                        raise RuntimeError('passive sender is not loopback')
                    # This potentially blocking proof runs once at startup.
                    # Retain the preceding read timestamp across the proof.
                    proof=self._verify(sender)
                    if not isinstance(proof,str) or not proof:
                        raise RuntimeError('initial owned passive peer proof missing')
                    self.peer,self.peer_proof=sender,proof
                elif sender != self.peer:
                    raise RuntimeError('passive observer endpoint changed')
                if self._stop.is_set():
                    break
                if self.first_read_at is None:self.first_read_at=at
                self.last_read_at=at
                self._consume(data,sender,at)
                self.accepted+=1
        except BaseException as error:
            self._error=error

    def check(self):
        if self._error is not None:
            raise RuntimeError('passive receiver failed: '+str(self._error)) from self._error
        if self._thread is not None and not self._thread.is_alive() and not self._stop.is_set():
            raise RuntimeError('passive receiver exited unexpectedly')

    def close(self,timeout=4):
        if type(timeout) not in (int,float) or not math.isfinite(timeout) or timeout < 0:
            raise ValueError('passive retirement timeout invalid')
        self._stop.set()
        if self._thread is not None:self._thread.join(timeout)
        self._stream.close();self._closed=True
        if self._thread is not None and self._thread.is_alive():
            raise RuntimeError('passive receiver did not retire within bounded cleanup')
        self.check()

    def status(self):
        return {'read_only':True,'datagrams':self.datagrams,'accepted_datagrams':self.accepted,
                'maximum_datagrams':self._maximum,'peer':list(self.peer) if self.peer is not None else None,
                'first_read_at':self.first_read_at,'last_read_at':self.last_read_at,
                'error':str(self._error) if self._error is not None else None,
                'thread_retired':self._thread is None or not self._thread.is_alive(),'socket_closed':self._closed,
                'timestamp_scope':'actual recvfrom completion on Python monotonic clock; not kernel arrival or source age'}
