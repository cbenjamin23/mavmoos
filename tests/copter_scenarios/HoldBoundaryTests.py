"""File/worker/inert-UDP controls, no native simulator or vehicle traffic."""
import importlib.util
import hashlib
import json
from pathlib import Path
import socket
import sys
import tempfile
import threading
import time
import unittest
from unittest import mock

def load(name,path):
    spec=importlib.util.spec_from_file_location(name,path);module=importlib.util.module_from_spec(spec);sys.modules[name]=module;spec.loader.exec_module(module);return module
HERE=Path(__file__).resolve().parent
BOUNDARY=load('_hold_boundary_tests_helper',HERE/'hold_boundary.py')
RECEIVER=load('_hold_receiver_tests_helper',HERE.parent/'observation_load/passive_receiver.py')
sys.path.insert(0,str(HERE))
try:RUNNER=load('_hold_boundary_runner',HERE/'run_scenarios.py')
finally:sys.path.pop(0)

class Healthy:
    def check(self):pass

class BoundaryTests(unittest.TestCase):
    key='cycle2.hold_helm.started'
    def prepared(self,folder):
        path=Path(folder);token=('hold:'+self.key+':1:1000.125\n').encode();(path/self.key).write_bytes(token);return path,token
    def test_exact_atomic_content_and_original_boundary_before_ack(self):
        with tempfile.TemporaryDirectory() as folder:
            p,token=self.prepared(folder);times={};events=[];clock=iter([10.,10.01]);BOUNDARY.record_marker(p,self.key,times,events,Healthy(),clock=lambda:next(clock))
            self.assertEqual((p/(self.key+'.ack')).read_bytes(),token);self.assertEqual(times,{self.key:10.});self.assertEqual([r['python_mono'] for r in events],[10.,10.01]);self.assertFalse((p/(self.key+'.ack.tmp')).exists())
            BOUNDARY.record_marker(p,self.key,times,events,Healthy(),clock=lambda:999.)
            self.assertEqual(times[self.key],10.);self.assertEqual(len(events),2)
    def test_missing_marker_creates_no_ack(self):
        with tempfile.TemporaryDirectory() as folder:
            times={};events=[];BOUNDARY.record_marker(Path(folder),self.key,times,events,Healthy());self.assertEqual(times,{});self.assertEqual(events,[])
    def test_foreign_key_or_token_rejected(self):
        for token in (b'hold:cycle2Xhold_helmXstarted:1:1000\n',b'hold:cycle1.hold_helm.started:1:1000\n',b'old',b'x'*129):
            with self.subTest(token=token),tempfile.TemporaryDirectory() as folder:
                p,_=self.prepared(folder);(p/self.key).write_bytes(token)
                with self.assertRaises(RuntimeError):BOUNDARY.record_marker(p,self.key,{},[],Healthy())
                self.assertFalse((p/(self.key+'.ack')).exists())
    def test_preexisting_ack_not_overwritten(self):
        with tempfile.TemporaryDirectory() as folder:
            p,_=self.prepared(folder);ack=p/(self.key+'.ack');ack.write_bytes(b'old')
            with self.assertRaises(RuntimeError):BOUNDARY.record_marker(p,self.key,{},[],Healthy())
            self.assertEqual(ack.read_bytes(),b'old')
    def test_racing_ack_publication_cannot_overwrite(self):
        with tempfile.TemporaryDirectory() as folder:
            p,_=self.prepared(folder);ack=p/(self.key+'.ack')
            def raced(source,destination):ack.write_bytes(b'racer');raise FileExistsError('racing ack')
            with mock.patch.object(BOUNDARY.os,'link',side_effect=raced):
                with self.assertRaises(FileExistsError):BOUNDARY.record_marker(p,self.key,{},[],Healthy())
            self.assertEqual(ack.read_bytes(),b'racer');self.assertFalse((p/(self.key+'.ack.tmp')).exists())
    def test_preexisting_temporary_ack_is_preserved(self):
        with tempfile.TemporaryDirectory() as folder:
            p,_=self.prepared(folder);temporary=p/(self.key+'.ack.tmp');temporary.write_bytes(b'sentinel')
            with self.assertRaises(FileExistsError):BOUNDARY.record_marker(p,self.key,{},[],Healthy())
            self.assertEqual(temporary.read_bytes(),b'sentinel');self.assertFalse((p/(self.key+'.ack')).exists())
    def test_receiver_fault_prevents_ack(self):
        with tempfile.TemporaryDirectory() as folder:
            p,_=self.prepared(folder);bad=mock.Mock();bad.check.side_effect=RuntimeError('receiver fault')
            with self.assertRaises(RuntimeError):BOUNDARY.record_marker(p,self.key,{},[],bad)
            self.assertFalse((p/(self.key+'.ack')).exists())
    def test_receiver_fault_after_temporary_write_prevents_ack(self):
        with tempfile.TemporaryDirectory() as folder:
            p,_=self.prepared(folder);bad=mock.Mock();bad.check.side_effect=[None,None,RuntimeError('late worker fault')]
            with self.assertRaises(RuntimeError):BOUNDARY.record_marker(p,self.key,{},[],bad)
            self.assertFalse((p/(self.key+'.ack')).exists());self.assertFalse((p/(self.key+'.ack.tmp')).exists())
    def test_regressed_ack_clock_stays_failure(self):
        with tempfile.TemporaryDirectory() as folder:
            p,_=self.prepared(folder);clock=iter([10.,9.])
            with self.assertRaises(RuntimeError):BOUNDARY.record_marker(p,self.key,{},[],Healthy(),clock=lambda:next(clock))
    def test_worker_receives_while_inventory_blocks_main(self):
        received=[];proofs=[];done=threading.Event()
        with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as stream,socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as sender:
            stream.bind(('127.0.0.1',0));stream.setblocking(False);sender.bind(('127.0.0.1',0));peer=sender.getsockname()
            def verify(actual):
                if actual!=peer:raise RuntimeError('inert owned endpoint differs')
                proofs.append(actual);return 'inert test-owned UDP endpoint only'
            def consume(data,actual,at):received.append((data,actual,at));done.set()
            collector=RECEIVER.PassiveReceiver(stream,verify,consume,max_datagrams=20);collector.start()
            try:
                sender.sendto(b'first',stream.getsockname());self.assertTrue(done.wait(.5));done.clear()
                def produce():
                    for _ in range(4):time.sleep(.02);sender.sendto(b'while-blocked',stream.getsockname())
                worker=threading.Thread(target=produce);worker.start();block_began=time.monotonic();time.sleep(.15);block_finished=time.monotonic();worker.join();collector.close()
                self.assertEqual(len(received),5);self.assertEqual(proofs,[peer]);self.assertTrue(all(block_began<=r[2]<block_finished for r in received[1:]));self.assertTrue(collector.status()['thread_retired']);self.assertTrue(collector.status()['socket_closed'])
            finally:collector.close()
    def synthetic_bytes(self):
        label='cycle1.hold_guided';data={};events=[];tokens={}
        for n,suffix in enumerate(('.started','.finished')):
            key=label+suffix;token=('hold:'+key+':1:1000.0\n').encode();data[key]=data[key+'.ack']=token;tokens[key]=token.decode()
            events.extend([{'kind':key,'python_mono':10.+n*4.1},{'kind':key+'.ack','python_mono':10.01+n*4.1,'token_sha256':hashlib.sha256(token).hexdigest()}])
        return data,events,{'windows':{label:{}},'boundary_tokens':tokens}
    def test_archive_callback_replays_complete_bytes_without_extraction(self):
        data,events,proof=self.synthetic_bytes();self.assertEqual(BOUNDARY.audit_boundary_bytes(events,proof,data.__getitem__)['result'],'PASS')
    def test_archive_rejects_ack_bytes_differing_from_probe_and_marker(self):
        data,events,proof=self.synthetic_bytes();data['cycle1.hold_guided.finished.ack']=b'foreign'
        with self.assertRaises(ValueError):BOUNDARY.audit_boundary_bytes(events,proof,data.__getitem__)
    def test_archive_rejects_extra_foreign_ack_receipt(self):
        data,events,proof=self.synthetic_bytes();events.append({'kind':'foreign.ack','python_mono':16})
        with self.assertRaises(ValueError):BOUNDARY.audit_boundary_bytes(events,proof,data.__getitem__)
    def test_archive_rejects_original_boundary_after_ack(self):
        data,events,proof=self.synthetic_bytes();events[0]['python_mono']=11
        with self.assertRaises(ValueError):BOUNDARY.audit_boundary_bytes(events,proof,data.__getitem__)
    def test_archive_rejects_ack_array_reordered_before_original(self):
        data,events,proof=self.synthetic_bytes();events[0],events[1]=events[1],events[0]
        with self.assertRaises(ValueError):BOUNDARY.audit_boundary_bytes(events,proof,data.__getitem__)
    def clean_result(self):
        ports={k:1 for k in ('bridge','monitor','rc','db','base')}
        return {'result':'PASS','ports':ports,'ports_clear':{k:True for k in ports},'passive_receiver':{'thread_retired':True,'socket_closed':True,'error':None}}
    def test_cleanup_exception_serializes_unknown_not_empty(self):
        ownership=mock.Mock();ownership.stop_process_tree.side_effect=RuntimeError('inventory failed');result=self.clean_result();remaining,known,errors=RUNNER.stop_owned_processes(ownership,[object()]);RUNNER.finalize_cleanup(result,remaining,known,errors)
        encoded=json.loads(json.dumps(result));self.assertIsNone(encoded['owned_processes_remaining']);self.assertFalse(encoded['cleanup_proven']);self.assertEqual(encoded['result'],'FAIL')
    def test_cleanup_nonlist_remains_unknown(self):
        ownership=mock.Mock();ownership.stop_process_tree.return_value=None;result=self.clean_result();remaining,known,errors=RUNNER.stop_owned_processes(ownership,[object()]);RUNNER.finalize_cleanup(result,remaining,known,errors)
        self.assertIsNone(result['owned_processes_remaining']);self.assertFalse(result['cleanup_proven'])
    def test_unretired_receiver_cannot_publish_cleanup_success(self):
        result=self.clean_result();result['passive_receiver']['thread_retired']=False;RUNNER.finalize_cleanup(result,[],True,[]);self.assertIsNone(result['owned_processes_remaining']);self.assertFalse(result['cleanup_proven'])
    def test_successful_cleanup_proof_preserves_empty_inventory(self):
        result=self.clean_result();RUNNER.finalize_cleanup(result,[],True,[]);self.assertEqual(result['owned_processes_remaining'],[]);self.assertTrue(result['cleanup_proven'])

if __name__=='__main__':unittest.main()
