"""Inert owned-loopback capture controls; no MAVLink, FC, process or PID override."""
import socket
import threading
import unittest
from passive_receiver import PassiveReceiver


class PassiveReceiverTests(unittest.TestCase):
    def setUp(self):
        self.stream=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.stream.bind(('127.0.0.1',0));self.stream.setblocking(False)
        self.destination=self.stream.getsockname()
        self.sender=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);self.sender.bind(('127.0.0.1',0))
        self.rows=[];self.proofs=[];self.condition=threading.Condition();self.receiver=None;self.release=threading.Event()

    def tearDown(self):
        self.release.set()
        if self.receiver is not None:
            try:self.receiver.close(2)
            except RuntimeError:pass  # Expected sticky worker faults are asserted by their cases.
            self.assertTrue(self.receiver.status()['thread_retired'])
        self.stream.close();self.sender.close()

    def verify(self,sender):
        self.assertEqual(sender,self.sender.getsockname());self.proofs.append(sender)
        return 'inert test-owned loopback endpoint; no native process'

    def consume(self,data,sender,at):
        with self.condition:self.rows.append((data,sender,at));self.condition.notify_all()

    def start(self,**changes):
        arguments={'verify_peer':self.verify,'consume':self.consume};arguments.update(changes)
        self.receiver=PassiveReceiver(self.stream,**arguments);self.receiver.start();return self.receiver

    def send(self,value,sender=None):
        (sender or self.sender).sendto(str(value).encode(),self.destination)

    def count(self,value):
        with self.condition:self.assertTrue(self.condition.wait_for(lambda:len(self.rows)>=value,2),'collector did not progress')

    def fault(self,why):
        self.receiver._thread.join(2)
        self.assertFalse(self.receiver._thread.is_alive(),'fault did not terminate capture')
        with self.assertRaisesRegex(RuntimeError,why):self.receiver.check()

    def test_capture_progresses_while_main_identity_check_is_blocked(self):
        # Publisher advances a synthetic clock only after each actual UDP read
        # is acknowledged. The main coordinator is held inside an ownership
        # callback until all samples arrive: an inline reader cannot progress.
        # Every stored time is the clock sampled by recvfrom, never backdated.
        now=[10.0];finished=threading.Event();errors=[]
        self.start(clock=lambda:now[0]);self.send(0);self.count(1)
        def publisher():
            try:
                for index in range(1,7):
                    now[0]=10+index*.1;self.send(index);self.count(index+1)
            except BaseException as error:errors.append(error)
            finally:finished.set()
        thread=threading.Thread(target=publisher);thread.start()
        # This stands in for slow synchronous ps/lsof on the main thread.
        self.assertTrue(finished.wait(2),'blocked ownership check starved capture')
        thread.join(2);self.assertFalse(thread.is_alive());self.assertFalse(errors)
        self.receiver.close(2)
        self.assertEqual([row[0] for row in self.rows],[str(i).encode() for i in range(7)])
        self.assertEqual([row[2] for row in self.rows],[10+i*.1 for i in range(7)])
        self.assertLess(max(b[2]-a[2] for a,b in zip(self.rows,self.rows[1:])),.5)
        self.assertEqual(len(self.proofs),1)

    def test_initial_proof_cannot_renew_original_read_stamp(self):
        now=[10.0]
        def proof(sender):
            value=self.verify(sender);now[0]=20.0;return value
        self.start(verify_peer=proof,clock=lambda:now[0]);self.send('sample');self.count(1)
        self.assertEqual(self.rows[0][2],10.0)

    def test_missing_initial_proof_accepts_no_bytes(self):
        self.start(verify_peer=lambda _:None);self.send('unproven');self.fault('proof missing')
        self.assertEqual(self.rows,[])

    def test_rejected_identity_accepts_no_bytes(self):
        def reject(_):raise RuntimeError('inert identity rejection')
        self.start(verify_peer=reject);self.send('unproven');self.fault('identity rejection')
        self.assertEqual(self.rows,[])

    def test_sender_change_is_sticky_and_not_consumed(self):
        self.start();self.send(0);self.count(1)
        with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as other:
            other.bind(('127.0.0.1',0));self.send('foreign',other);self.fault('endpoint changed')
        self.assertEqual(len(self.rows),1);self.assertEqual(len(self.proofs),1)
        with self.assertRaisesRegex(RuntimeError,'endpoint changed'):self.receiver.close(2)

    def test_datagram_bound_is_not_extended(self):
        self.start(max_datagrams=2)
        for index in range(2):self.send(index);self.count(index+1)
        self.send(2);self.fault('datagram bound')
        self.assertEqual(len(self.rows),2);self.assertEqual(self.receiver.status()['datagrams'],3)

    def test_consumer_failure_propagates(self):
        def reject(*_):raise ValueError('inert parser/log failure')
        self.start(consume=reject);self.send('bad');self.fault('parser/log failure')
        self.assertEqual(self.receiver.status()['accepted_datagrams'],0)

    def test_nonfinite_read_clock_fails(self):
        values=iter((10,float('nan')))
        self.start(clock=lambda:next(values));self.send(0);self.count(1)
        self.send(1);self.fault('clock invalid')
        self.assertEqual(self.receiver.status()['last_read_at'],10)

    def test_regressed_read_clock_fails(self):
        values=iter((10,9))
        self.start(clock=lambda:next(values));self.send(0);self.count(1)
        self.send(1);self.fault('clock invalid/regressed')
        self.assertEqual(len(self.rows),1);self.assertEqual(self.receiver.status()['last_read_at'],10)

    def test_idle_shutdown_and_no_restart(self):
        self.start();self.receiver.close(2)
        self.assertTrue(self.receiver.status()['thread_retired']);self.assertEqual(self.stream.fileno(),-1)
        with self.assertRaises(RuntimeError):self.receiver.start()

    def test_unretired_initial_proof_is_failed_cleanup_not_acceptance(self):
        entered=threading.Event()
        def slow(sender):
            entered.set();self.release.wait(2);return self.verify(sender)
        self.start(verify_peer=slow);self.send('pending');self.assertTrue(entered.wait(2))
        with self.assertRaisesRegex(RuntimeError,'did not retire'):self.receiver.close(.01)
        self.assertFalse(self.receiver.status()['thread_retired']);self.assertTrue(self.receiver.status()['socket_closed'])
        self.release.set();self.receiver.close(2)
        self.assertEqual(self.rows,[])

    def test_proof_fault_after_stop_stays_failure(self):
        entered=threading.Event()
        def late_rejection(_):
            entered.set();self.release.wait(2);raise RuntimeError('inert rejection after stop')
        self.start(verify_peer=late_rejection);self.send('pending');self.assertTrue(entered.wait(2))
        with self.assertRaisesRegex(RuntimeError,'did not retire'):self.receiver.close(.01)
        self.release.set();self.fault('rejection after stop')
        with self.assertRaisesRegex(RuntimeError,'rejection after stop'):self.receiver.close(2)
        self.assertEqual(self.rows,[]);self.assertTrue(self.receiver.status()['thread_retired'])

    def test_required_validator_and_loopback_binding_cannot_be_omitted(self):
        with self.assertRaises(ValueError):PassiveReceiver(self.stream,None,self.consume)
        with self.assertRaises(ValueError):PassiveReceiver(self.stream,self.verify,self.consume,max_datagrams=0)
        with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as wildcard:
            wildcard.bind(('0.0.0.0',0));wildcard.setblocking(False)
            with self.assertRaises(ValueError):PassiveReceiver(wildcard,self.verify,self.consume)


if __name__=='__main__':unittest.main()
