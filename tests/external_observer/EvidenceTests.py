#!/usr/bin/env python3
"""Inert checker/configuration/encoder controls; no mission or socket launch."""
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import subprocess
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import evidence

ROOT=Path(__file__).resolve().parents[2]
def load(relative,name):
    spec=importlib.util.spec_from_file_location(name,ROOT/relative);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module
peer=load('missions/external_observer/fake_peer.py','_peer_control')
supervisor=load('missions/external_observer/supervise.py','_supervise_control')
driver=load('tests/external_observer/run_demo.py','_driver_control')


def fixture():
    report={'result':'PASS','cleanup_proven':True,'owned_processes_remaining':[],'cleanup_errors':[],
            'input_drift':[],'inputs_before':{'app':'unchanged'},'inputs_after':{'app':'unchanged'},
            'ports_clear_after':{'moos':True,'app':True,'peer':True},'ports':{'peer':14620},
            'owned_app_roster':{'MOOSDB':{'pid':1},'iMAVLink':{'pid':2}},
            'peer_capture':{'peer':['127.0.0.1',14620],'thread_retired':True,'socket_closed':True,'error':None,'sent_heartbeats':20,'received_datagrams':0,'received_bytes':0}}
    events=[]
    def add(key,value,at):events.append({'mono':at,'moos_time':at,'key':key,'source':'iMAVLink','value':value})
    status='state=RX_ONLY,tx_enabled=false,mavlink_tx_attempts=0,mavlink_tx_accepted=0,helm=disabled,wrong_target=4,wrong_peer=1,loaded_modules=1'
    add('MAVMOOS_STATUS',status,1)
    add('MAVMOOS_RUNTIME_STATUS','mode=RX_ONLY,permission=false,control_profile=none,tx_attempts=0,tx_accepted=0,nav_tx_accepted=0,mission_execution_held=false',1.01)
    add('MAVMOOS_CAPABILITIES',json.dumps({'schema':1,'execution_mode':'RX_ONLY','composition':{'plugin_abi':9,'modules':[
        {'name':'example_heartbeat_observer','control_provider':False,'request_owners':[],
         'capabilities':[{'name':'example.heartbeat.observation','request_variable':'','request_value':'','access':'observation'}]}]}}),1.02)
    def sample(at,mode,identity,age,epoch=1,present=True):
        value={'schema':1,'example':True,'read_only':True,'authoritative_for_control':False,'clock_fault':False,
               'sample_timeout_s':.5,'publish_interval_s':.1,'source_age':'unknown','source_quality':'unassessed',
               'present':present,'fresh':present and age<=.5,'session_epoch':epoch,'observation_id':identity,
               'effective_at':1.0 if epoch==1 else 4.0,'read_at':1.01 if epoch==1 else 4.01,
               'age_s':age,'age_basis':'QUEUE_EMPTY_FLOOR','system_id':1,'component_id':1,'custom_mode_raw':mode}
        if not present:
            for key in ('effective_at','read_at','age_s','system_id','component_id','custom_mode_raw'):value[key]=None
            value['age_basis']='UNKNOWN'
        add('EXAMPLE_HEARTBEAT_OBSERVATION',json.dumps(value),at)
    for i in range(8):sample(1.1+i*.1,42,1,.1+i*.1)
    sample(3.2,0,1,0,2,False)
    for i in range(4):sample(4.1+i*.1,84,2,.1+i*.1,2)
    events.sort(key=lambda v:v['mono'])
    return events,report


class EvidenceTests(unittest.TestCase):
    def test_normal(self):
        events,result=fixture();self.assertEqual(evidence.audit(events,result)['result'],'PASS')

    def test_egress_is_independent(self):
        events,result=fixture();result['peer_capture']['received_datagrams']=1
        with self.assertRaisesRegex(ValueError,'egress'):evidence.audit(events,result)

    def test_unknown_cleanup_is_not_empty(self):
        for key,value in (('cleanup_proven',False),('owned_processes_remaining',None),('cleanup_errors',['timeout'])):
            with self.subTest(key=key):
                events,result=fixture();result[key]=value
                with self.assertRaises(ValueError):evidence.audit(events,result)

    def test_foreign_output_rejected(self):
        events,result=fixture();index=next(i for i,v in enumerate(events) if v['key']=='EXAMPLE_HEARTBEAT_OBSERVATION')
        value=json.loads(events[index]['value']);value['system_id']=2;events[index]['value']=json.dumps(value)
        with self.assertRaisesRegex(ValueError,'foreign'):evidence.audit(events,result)

    def test_metadata_cannot_be_renewed(self):
        events,result=fixture();samples=[v for v in events if v['key']=='EXAMPLE_HEARTBEAT_OBSERVATION']
        value=json.loads(samples[6]['value']);value['effective_at']=1.005;samples[6]['value']=json.dumps(value)
        with self.assertRaisesRegex(ValueError,'renewed'):evidence.audit(events,result)

    def test_freshness_uses_original_age(self):
        events,result=fixture();row=next(v for v in events if v['key']=='EXAMPLE_HEARTBEAT_OBSERVATION' and json.loads(v['value'])['age_s']>.5)
        value=json.loads(row['value']);value['fresh']=True;row['value']=json.dumps(value)
        with self.assertRaisesRegex(ValueError,'freshness'):evidence.audit(events,result)

    def test_config_is_observed(self):
        events,result=fixture();row=next(v for v in events if v['key']=='EXAMPLE_HEARTBEAT_OBSERVATION')
        value=json.loads(row['value']);value['sample_timeout_s']=2;row['value']=json.dumps(value)
        with self.assertRaisesRegex(ValueError,'configuration'):evidence.audit(events,result)

    def test_actual_publication_interval(self):
        events,result=fixture()
        for row in events:row['moos_time']*=4
        with self.assertRaisesRegex(ValueError,'cadence'):evidence.audit(events,result)

    def test_reset_required_for_recovery(self):
        events,result=fixture()
        for row in events:
            if row['key']=='EXAMPLE_HEARTBEAT_OBSERVATION':
                value=json.loads(row['value']);value['session_epoch']=1;row['value']=json.dumps(value)
        with self.assertRaises(ValueError):evidence.audit(events,result)

    def test_control_and_catalog_abi(self):
        for key,value in (('plugin_abi',8),):
            events,result=fixture();row=next(v for v in events if v['key']=='MAVMOOS_CAPABILITIES');catalog=json.loads(row['value']);catalog['composition'][key]=value;row['value']=json.dumps(catalog)
            with self.assertRaises(ValueError):evidence.audit(events,result)
        events,result=fixture();row=next(v for v in events if v['key']=='MAVMOOS_RUNTIME_STATUS');row['value']=row['value'].replace('permission=false','permission=true')
        with self.assertRaisesRegex(ValueError,'authority'):evidence.audit(events,result)

    def test_filter_rejection_is_observed(self):
        events,result=fixture();row=next(v for v in events if v['key']=='MAVMOOS_STATUS');row['value']=row['value'].replace('wrong_target=4','wrong_target=0')
        with self.assertRaisesRegex(ValueError,'filter'):evidence.audit(events,result)

    def test_invalid_startup(self):
        _,result=fixture();events=[{'mono':1,'key':'MAVMOOS_STATUS','source':'iMAVLink','value':
            'state=CONFIG_ERROR,tx_enabled=false,mavlink_tx_attempts=0,mavlink_tx_accepted=0,helm=disabled,loaded_modules=0,rx_datagrams=0'}]
        events.append({'mono':2,'key':'APPCAST','value':'invalid sample_timeout finite seconds'})
        self.assertEqual(evidence.audit(events,result,scenario='invalid_config')['module_publications'],0)
        events[0]['value']=events[0]['value'].replace('rx_datagrams=0','rx_datagrams=1')
        with self.assertRaises(ValueError):evidence.audit(events,result,scenario='invalid_config')

    def test_input_drift_and_capture_retirement(self):
        for mutation in ('drift','thread'):
            events,result=fixture()
            if mutation=='drift':result['inputs_after']['app']='modified'
            else:result['peer_capture']['thread_retired']=False
            with self.assertRaises(ValueError):evidence.audit(events,result)

    def test_live_roster_and_capture_endpoint_required(self):
        for mutation in ('roster','endpoint'):
            events,result=fixture()
            if mutation=='roster':result['owned_app_roster'].pop('iMAVLink')
            else:result['peer_capture']['peer'][1]+=1
            with self.assertRaises(ValueError):evidence.audit(events,result)

    def test_log_order_and_source(self):
        for data in ('{"mono":1,"source":"other"}', '{"mono":2,"source":"iMAVLink"}\n{"mono":1,"source":"iMAVLink"}'):
            with self.assertRaises(ValueError):evidence.observations(data)


class MissionTests(unittest.TestCase):
    def test_standard_heartbeat_golden(self):
        # Independently produced by pinned pymavlink's generated dialect.
        self.assertEqual(peer.heartbeat(0).hex(),'fe09000101002a0000000203010303bc19')

    def test_source_schedule(self):
        self.assertEqual(peer.phase(1.49),('initial',1,42));self.assertEqual(peer.phase(2.1),('foreign_target',2,99))
        self.assertEqual(peer.phase(3),('silence',None,None));self.assertEqual(peer.phase(4.6),('recovered',1,84))

    def test_strict_ports(self):
        for value in ('0','65536','1;touch','nan','1.0'):
            with self.assertRaises(Exception):supervisor.port(value)
        self.assertEqual(supervisor.port('19121'),19121)

    def test_moos_injection_rejected(self):
        for value in ('x\nRun = other','$(APP)','a#b','a{b',''):
            with self.assertRaises(ValueError):supervisor.safe_value(value)

    def test_just_make_does_not_launch(self):
        args=supervisor.arguments(['--build=/b','--module=/m/mavmoos_example_heartbeat_observer.so','--moos-root=/m','--just_make'])
        with patch.object(supervisor,'arguments',return_value=args),patch.object(supervisor,'generate',return_value=(0,0,0,0,Path('/target'),[])),patch.object(supervisor,'supervise',side_effect=AssertionError('launched')):
            self.assertEqual(supervisor.main([]),0)

    def test_additional_settings_preserve_app_validation(self):
        args=supervisor.arguments(['--build=/b','--module=/m/mavmoos_example_heartbeat_observer.so','--moos-root=/m',
                                  '--setting=plugin.Example_heartbeat_observer.sample_timeout=1',
                                  '--setting=plugin.example_heartbeat_observer.sample_timeout=1'])
        self.assertEqual(len(args.setting),2)
        self.assertIn('plugin.Example_',args.setting[0])

    def test_roster_exact_identity(self):
        target=Path('/owned/mission.moos');app=Path('/build/iMAVLink')
        rows=[{'pid':1,'command':'MOOSDB /owned/mission.moos MOOSDB'},
              {'pid':2,'command':'/build/iMAVLink /owned/mission.moos iMAVLink'},
              {'pid':3,'command':'/foreign/iMAVLink /other/mission.moos iMAVLink'}]
        found=supervisor.app_roster(rows,target,app)
        self.assertEqual({k:v['pid'] for k,v in found.items()},{'MOOSDB':1,'iMAVLink':2})
        with self.assertRaises(RuntimeError):supervisor.app_roster(rows+[rows[1]],target,app)
        rows[1]['command']='/foreign/iMAVLink /owned/mission.moos iMAVLink'
        with self.assertRaises(RuntimeError):supervisor.app_roster(rows,target,app)


class PortablePreflightTests(unittest.TestCase):
    def setup_snapshot(self, folder):
        root=Path(folder);host=root/'host-source';runtime=host/'src/libmavmoos_runtime';runtime.mkdir(parents=True)
        identity={}
        for name,key in (('Plugin.h','api_sha256'),('PluginHost.cpp','loader_sha256')):
            data=(ROOT/'src/libmavmoos_runtime'/name).read_bytes();(runtime/name).write_bytes(data)
            identity[key]=hashlib.sha256(data).hexdigest()
        moos=root/'configured-moos';other=root/'other-moos'
        for tree in (moos,other):
            (tree/'bin').mkdir(parents=True)
            for name in ('MOOSDB','pAntler','nsplug'):(tree/'bin'/name).write_bytes(b'identical inert artifact')
        build=root/'host-build';(build/'bin').mkdir(parents=True);(build/'bin/iMAVLink').write_bytes(b'inert app artifact')
        (build/'CMakeCache.txt').write_text(f'CMAKE_HOME_DIRECTORY:INTERNAL={host}\nMOOS_IVP_ROOT:PATH={moos}\n')
        consumer=root/'consumer';(consumer/'plugins').mkdir(parents=True)
        module=consumer/'plugins/mavmoos_example_heartbeat_observer.so';module.write_bytes(b'inert module artifact')
        example=root/'copied-observer';example.mkdir()
        (consumer/'CMakeCache.txt').write_text(
            f'CMAKE_HOME_DIRECTORY:INTERNAL={example}\n'
            'CMAKE_PROJECT_NAME:STATIC=MAVMOOSExternalObserverExample\n'
        )
        (consumer/'example-inputs.json').write_text(json.dumps(identity))
        probe=root/'probe';probe.write_bytes(b'inert probe artifact')
        args=SimpleNamespace(source_root=str(ROOT),build=str(build),module=str(module),moos_root=str(moos),
                             probe=str(probe),scenario='normal',expected_app_sha256=None,expected_module_sha256=None)
        return args,other

    def test_no_git_configured_snapshot_allowed(self):
        with tempfile.TemporaryDirectory() as folder:
            args,_=self.setup_snapshot(folder)
            *_,metadata=driver.preflight(args)
            self.assertIsNone(metadata['host_source_ref']);self.assertIsNone(metadata['moos_ref'])
            self.assertEqual(metadata['module_build_kind'],'standalone_consumer')

    def test_distinct_dependency_path_rejected_despite_same_bytes(self):
        with tempfile.TemporaryDirectory() as folder:
            args,other=self.setup_snapshot(folder);args.moos_root=str(other)
            with self.assertRaisesRegex(ValueError,'configured host dependency'):driver.preflight(args)

    def test_sdk_input_copy_uses_original_bytes_and_rejects_drift(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);original=root/'example-inputs.json';copied=root/'sdk-inputs.json'
            data=b'{  "schema": 1 }\n';original.write_bytes(data);expected=hashlib.sha256(data).hexdigest()
            receipt=driver.copy_sdk_inputs(original,copied,expected)
            self.assertEqual(copied.read_bytes(),data);self.assertEqual(receipt['sha256'],expected)
            original.write_bytes(b'{"schema":1}\n');next_copy=root/'other.json'
            with self.assertRaisesRegex(ValueError,'changed before copy'):driver.copy_sdk_inputs(original,next_copy,expected)
            self.assertFalse(next_copy.exists())

    @unittest.skipUnless(shutil.which('git'),'Git unavailable for real enclosing-repository control')
    def test_nested_snapshot_does_not_inherit_git_identity(self):
        with tempfile.TemporaryDirectory() as folder:
            top=Path(folder);subprocess.run(['git','init','-q',str(top)],check=True,timeout=5)
            command=['git','-C',str(top),'-c','user.name=MAVMOOS fixture','-c','user.email=fixture@example.invalid',
                     '-c','commit.gpgsign=false','-c','core.hooksPath=/dev/null','commit','--allow-empty','-q','-m','fixture']
            subprocess.run(command,check=True,timeout=5)
            snapshot=top/'snapshot';snapshot.mkdir()
            self.assertRegex(driver.revision(top),r'^[0-9a-f]{40,64}$')
            self.assertIsNone(driver.revision(snapshot))


if __name__=='__main__':unittest.main(verbosity=2)
