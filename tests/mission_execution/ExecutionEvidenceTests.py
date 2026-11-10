#!/usr/bin/env python3
"""Synthetic checker/coordination negatives only; never runtime qualification."""
import copy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock
from execution_evidence import audit_probe,audit_fc,audit_boundary,audit_dataflash,markers,phase_ack_observation
from run_native import record_marker,stop_owned_processes,finalize_cleanup
import phase_boundary


def kv(**values):return ','.join(f'{key}={value}' for key,value in values.items())


def fixture(scenario='auto_complete_handback'):
    events=[]
    def event(at,kind,key,value,source='probe'):
        events.append({'elapsed':at,'type':kind,'key':key,'value':value,'source':source})
    all_times={'upload.started':1,'upload.finished':4,'takeoff.started':7,'takeoff.finished':12,'auto.started':15,'auto.completed':20,'auto.paused':20,'auto.finished':24,'abort.started':24.2,'abort.finished':27,'return.started':28,'return.arrived':38,'return.finished':41.2,'land.started':41.4,'land.finished':49,'finished':52}
    times={key:all_times[key] for key in markers(scenario)}
    for key,at in times.items():event(at,'phase',key,'nav_tx_accepted=5')
    plan=[{'seq':0,'command':16,'frame':0,'current':0,'autocontinue':1,'params':[0,0,0,0],'x':423584560,'y':-710875890,'z':10},
          {'seq':1,'command':16,'frame':3,'current':0,'autocontinue':1,'params':[0,0,0,0],'x':423587060,'y':-710875890,'z':5},
          {'seq':2,'command':16,'frame':3,'current':0,'autocontinue':1,'params':[0,0,0,0],'x':423587060,'y':-710874890,'z':5}]
    proof={'schema':'1','valid':'true','verification_id':'upload','content_id':'host.1.7','revision':'7','session_epoch':'1','count':'3','atomic':'false','external_writer_excluded_by_profile':'true'}
    rows='|'.join(f"{p['seq']}:16:{p['frame']}:0:1:0:0:0:0:{p['x']}:{p['y']}:{p['z']}" for p in plan)
    event(2,'request','MAVMOOS_MISSION_REQUEST','id=upload,action=upload,timeout=30,items='+rows)
    event(3.5,'snapshot','MAVMOOS_MISSION_SNAPSHOT',json.dumps({'id':'upload','status':'complete','count':3,'verification':'semantic_profile_match','items':plan}),'iMAVLink')
    event(3.6,'proof','MAVMOOS_STORED_MISSION_PROOF',kv(**proof),'iMAVLink')
    def result(at,key,status,kind='control_result',expiry=100):
        values={'schema':1,'id':key,'status':status,'reason':'fixture','expires':expiry}
        if kind=='execution_result' and key=='run':values.update(verification_id='upload',content_id='host.1.7',proof_revision=7,session_epoch=1,wire_operation_id='false')
        event(at,kind,{'control_result':'MAVMOOS_CONTROL_RESULT','mission_result':'MAVMOOS_MISSION_RESULT','execution_result':'MAVMOOS_MISSION_EXECUTION_RESULT'}[kind],kv(**values),'iMAVLink')
    result(3.7,'upload','COMPLETED','mission_result')
    for key,request,done,body in [('guided',5,5.5,'action=guided,timeout=10'),('arm',6,6.5,'action=arm,timeout=10'),('takeoff',7.1,11.9,'action=takeoff,height=5,timeout=45'),('return',28.2,37.9,'action=return,strategy=helm,height=5,speed=1.5,radius=1.5,timeout=45'),('land',41.5,48.9,'action=land,timeout=60')]:
        event(request,'request','MAVMOOS_CONTROL_REQUEST','schema=1,id='+key+','+body);result(done,key,'COMPLETED')
    event(12.5,'request','MAVMOOS_MISSION_CONTROL','schema=1,id=run,action=start,verification_id=upload,content_id=host.1.7,proof_revision=7,timeout=90')
    for at,state in [(12.6,'ADMITTED'),(12.61,'SUBMITTED'),(12.7,'SENT'),(12.8,'ACKNOWLEDGED'),(13,'SUBMITTED'),(13.1,'SENT'),(13.2,'ACKNOWLEDGED'),(14.5,'ACTIVE')]:result(at,'run',state,'execution_result')
    def progress(at,seq,state):event(at,'execution_progress','MAVMOOS_MISSION_EXECUTION_PROGRESS',kv(schema=1,id='run',seq=seq,total=2,mission_state=state,effective_at=300+at,read_at=300+at,source_age='unknown',wire_operation_id='false'),'iMAVLink')
    progress(14.6,1,3)
    if scenario=='auto_complete_handback':progress(18,2,3);progress(19.5,2,5);result(19.8,'run','COMPLETED','execution_result')
    else:
        event(15.1,'request','MAVMOOS_MISSION_CONTROL','schema=1,id=pause,action=pause,target=run,timeout=8')
        progress(19.5,1,4);result(19.8,'pause','COMPLETED','execution_result');result(19.81,'run','PAUSED','execution_result')
    for step in range(18):
        at=15.1+step*.5
        event(at,'runtime','MAVMOOS_RUNTIME_STATUS','fc_mode=3,armed=1,in_air=1,permission=false,nav_tx_accepted=5','iMAVLink')
        event(at+.001,'execution_status','MAVMOOS_MISSION_EXECUTION_STATUS','schema=1,id=run,lease_retained=true','iMAVLink')
    event(24.1,'assertion','auto_finish_ack','wait_s=0.1,timeout_s=3')
    event(24.3,'request','MAVMOOS_MISSION_CONTROL','schema=1,id=abort,action=abort,target=run,timeout=8')
    result(26.8,'abort','COMPLETED','execution_result')
    if scenario=='auto_pause_abort':result(26.8,'run','CANCELLED','execution_result')
    for e in events:
        if e['type']=='request' and e['key'] in ('MAVMOOS_MISSION_REQUEST','MAVMOOS_MISSION_CONTROL','MAVMOOS_CONTROL_REQUEST'):
            f=dict(item.split('=',1) for item in e['value'].split(','));published=e['elapsed']-.001
            e.update(local_publication_attempt_elapsed=published,local_original_expiry_elapsed=published+float(f['timeout']),local_clock_scope='probe_publication_attempt_not_host_expiry')
    origins={dict(item.split('=',1) for item in e['value'].split(','))['id']:e for e in events if e['type']=='request' and e['key'] in ('MAVMOOS_MISSION_REQUEST','MAVMOOS_MISSION_CONTROL','MAVMOOS_CONTROL_REQUEST')}
    for key in ('auto.completed' if scenario=='auto_complete_handback' else 'auto.paused','auto.finished','return.arrived','return.finished'):
        at=times[key];owner='run' if key.startswith('auto.') else 'return';origin=origins[owner];began=at-.001
        event(at+.01,'phase_ack',key,'phase:'+key+':1:1000.0\n')
        events[-1].update(original_request_id=owner,local_publication_attempt_elapsed=origin['local_publication_attempt_elapsed'],local_original_expiry_elapsed=origin['local_original_expiry_elapsed'],wait_started_elapsed=began,deadline_elapsed=min(began+3,origin['local_original_expiry_elapsed']),ack_observed_elapsed=at+.009)
    for tick in range(32):
        at=38.1+tick*.1
        event(at,'runtime','MAVMOOS_RUNTIME_STATUS','fc_mode=4,armed=1,in_air=1,permission=false,nav_tx_accepted=50,heartbeat_valid=true,position_valid=true,home_valid=true,landed_valid=true,estimator_valid=true','iMAVLink')
        event(at+.001,'execution_status','MAVMOOS_MISSION_EXECUTION_STATUS','schema=1,id=run,lease_retained=false','iMAVLink')
    event(29,'helm_scalar','DESIRED_HEADING','180','pHelmIvP')
    for at in (49.5,50.5,51.5):event(at,'runtime','MAVMOOS_RUNTIME_STATUS','fc_mode=9,armed=0,in_air=0,permission=false,nav_tx_accepted=50','iMAVLink')
    event(52.1,'pass',scenario,'synthetic checker fixture only')
    return sorted(events,key=lambda e:e['elapsed']),times,plan


def packets_fixture(times,plan,scenario='auto_complete_handback'):
    times={k:100+v for k,v in times.items()};packets=[]
    def row(at,kind,**values):packets.append({'mono':at,'system':1,'component':1,'mavpackettype':kind,**values})
    for step in range(91):
        at=115+step*.1;fraction=min(1,step/40)
        lat=round(plan[0]['x']+(plan[-1]['x']-plan[0]['x'])*fraction);lon=round(plan[0]['y']+(plan[-1]['y']-plan[0]['y'])*fraction)
        row(at,'GLOBAL_POSITION_INT',lat=lat,lon=lon,relative_alt=5000,vx=0,vy=0)
    for at in range(115,125):row(at,'HEARTBEAT',custom_mode=3,base_mode=217)
    row(116,'MISSION_CURRENT',seq=1,wire_magic=254,wire_payload_length=2,total=0,mission_state=0)
    if scenario=='auto_complete_handback':row(120,'MISSION_CURRENT',seq=2,wire_magic=254,wire_payload_length=2,total=0,mission_state=0);row(120.1,'MISSION_ITEM_REACHED',seq=2)
    for at in (127.5,139.5,140.5,141.1):row(at,'HEARTBEAT',custom_mode=4,base_mode=217)
    for step in range(53):row(136+step*.1,'GLOBAL_POSITION_INT',lat=plan[0]['x'],lon=plan[0]['y'],relative_alt=5000,vx=0,vy=0)
    for at in (149.5,150.5,151.5):row(at,'HEARTBEAT',custom_mode=9,base_mode=89);row(at,'EXTENDED_SYS_STATE',landed_state=1);row(at,'GLOBAL_POSITION_INT',lat=plan[0]['x'],lon=plan[0]['y'],relative_alt=0,vx=0,vy=0)
    return sorted(packets,key=lambda p:p['mono']),times


class EvidenceTests(unittest.TestCase):
    def reject_probe(self,mutate,scenario='auto_complete_handback'):
        events,_,_=fixture(scenario);mutate(events)
        with self.assertRaises((ValueError,KeyError)):audit_probe(events,scenario)
    def test_both_scenario_semantics(self):
        for scenario in ('auto_complete_handback','auto_pause_abort'):
            events,times,plan=fixture(scenario);proof=audit_probe(events,scenario);packets,times=packets_fixture(times,plan,scenario)
            self.assertEqual(audit_fc(packets,times,153,scenario,proof)['result'],'PASS')
            self.assertEqual(proof['mission_completed'],scenario=='auto_complete_handback')
    def test_result_expiry_cannot_renew(self):self.reject_probe(lambda es:next(e for e in es if e['type']=='execution_result' and 'status=ACTIVE,' in e['value']).__setitem__('value',next(e['value'] for e in es if e['type']=='execution_result' and 'status=ACTIVE,' in e['value']).replace('expires=100','expires=101')))
    def test_foreign_proof(self):self.reject_probe(lambda es:next(e for e in es if e['type']=='proof').__setitem__('source','other'))
    def test_wrong_start_selector(self):self.reject_probe(lambda es:next(e for e in es if e['type']=='request' and 'action=start,' in e['value']).__setitem__('value','schema=1,id=run,action=start,verification_id=upload,content_id=other,proof_revision=7'))
    def test_missing_separate_arm(self):self.reject_probe(lambda es:es.__setitem__(slice(None),[e for e in es if not(e['type']=='control_result' and ',id=arm,' in e['value'])]))
    def test_ack_only_abort(self):self.reject_probe(lambda es:next(e for e in es if e['type']=='execution_result' and ',id=abort,' in e['value']).__setitem__('value','schema=1,id=abort,status=ACKNOWLEDGED,reason=fixture,expires=100'))
    def test_completed_parent_not_cancelled(self):self.reject_probe(lambda es:next(e for e in es if e['type']=='execution_result' and ',id=run,status=COMPLETED,' in e['value']).__setitem__('value',next(e['value'] for e in es if e['type']=='execution_result' and ',id=run,status=COMPLETED,' in e['value']).replace('status=COMPLETED','status=CANCELLED')))
    def test_paused_parent_not_complete(self):self.reject_probe(lambda es:next(e for e in es if e['type']=='execution_result' and ',id=run,status=PAUSED,' in e['value']).__setitem__('value',next(e['value'] for e in es if e['type']=='execution_result' and ',id=run,status=PAUSED,' in e['value']).replace('status=PAUSED','status=COMPLETED')),'auto_pause_abort')
    def test_guided_stream_during_auto(self):self.reject_probe(lambda es:next(e for e in es if e['type']=='runtime').__setitem__('value','fc_mode=3,armed=1,in_air=1,permission=false,nav_tx_accepted=6'))
    def test_early_lease_release(self):self.reject_probe(lambda es:next(e for e in es if e['type']=='execution_status').__setitem__('value','schema=1,id=run,lease_retained=false'))
    def test_final_armed(self):self.reject_probe(lambda es:next(e for e in es if e['type']=='runtime' and e['elapsed']>49).__setitem__('value','fc_mode=9,armed=1,in_air=0,permission=false,nav_tx_accepted=50'))
    def test_early_abort_boundary(self):self.reject_probe(lambda es:next(e for e in es if e['key']=='auto_finish_ack').__setitem__('elapsed',24.3))
    def test_late_ack(self):self.reject_probe(lambda es:next(e for e in es if e['key']=='auto_finish_ack').__setitem__('value','wait_s=3,timeout_s=3'))
    def test_no_stock_helm(self):self.reject_probe(lambda es:es.__setitem__(slice(None),[e for e in es if e['type']!='helm_scalar']))
    def test_mutated_stored_row(self):
        def mutate(es):
            e=next(e for e in es if e['type']=='snapshot');s=json.loads(e['value']);s['items'][1]['x']+=100;e['value']=json.dumps(s)
        self.reject_probe(mutate)
    def test_forged_wire_operation_id(self):self.reject_probe(lambda es:next(e for e in es if e['type']=='execution_progress').__setitem__('value','schema=1,id=run,seq=1,total=2,mission_state=3,effective_at=314.6,read_at=314.6,source_age=unknown,wire_operation_id=true'))
    def reject_fc(self,mutate):
        es,times,plan=fixture();proof=audit_probe(es,'auto_complete_handback');packets,times=packets_fixture(times,plan);mutate(packets)
        with self.assertRaises(ValueError):audit_fc(packets,times,153,'auto_complete_handback',proof)
    def test_passive_mode_takeover(self):self.reject_fc(lambda ps:next(p for p in ps if p['mavpackettype']=='HEARTBEAT').__setitem__('custom_mode',5))
    def test_early_land_before_ack(self):self.reject_fc(lambda ps:next(p for p in ps if p['mavpackettype']=='HEARTBEAT' and p['mono']>122).__setitem__('custom_mode',9))
    def test_passive_foreign_source(self):self.reject_fc(lambda ps:ps[0].__setitem__('system',2))
    def test_position_reception_gap(self):self.reject_fc(lambda ps:ps.__setitem__(slice(None),[p for p in ps if not(p['mavpackettype']=='GLOBAL_POSITION_INT' and 117<p['mono']<118)]))
    def test_missing_final_reached(self):self.reject_fc(lambda ps:ps.__setitem__(slice(None),[p for p in ps if p['mavpackettype']!='MISSION_ITEM_REACHED']))
    def test_wrong_v2_extended_count(self):
        def mutate(ps):next(p for p in ps if p['mavpackettype']=='MISSION_CURRENT').update(wire_magic=253,wire_payload_length=5,total=9)
        self.reject_fc(mutate)
    def test_missing_actual_route(self):self.reject_fc(lambda ps:[p.update(lat=423587060,lon=-710874890) for p in ps if p['mavpackettype']=='GLOBAL_POSITION_INT' and 115<=p['mono']<=124])
    def test_moving_hold(self):self.reject_fc(lambda ps:next(p for p in ps if p['mavpackettype']=='GLOBAL_POSITION_INT' and p['mono']>122).__setitem__('vx',100))
    def test_wrong_return_geometry(self):self.reject_fc(lambda ps:next(p for p in ps if p['mavpackettype']=='GLOBAL_POSITION_INT' and 139.2<=p['mono']<=141.2).__setitem__('lat',423590000))
    def test_near_limit_return_outlier_is_not_filtered(self):self.reject_fc(lambda ps:next(p for p in ps if p['mavpackettype']=='GLOBAL_POSITION_INT' and 139.2<=p['mono']<=141.2).__setitem__('lat',423584788))
    def test_return_land_before_observer_boundary_is_rejected(self):self.reject_fc(lambda ps:next(p for p in ps if p['mavpackettype']=='HEARTBEAT' and 139.2<=p['mono']<=141.2).__setitem__('custom_mode',9))
    def test_ground_not_independent(self):self.reject_fc(lambda ps:next(p for p in ps if p['mavpackettype']=='EXTENDED_SYS_STATE').__setitem__('landed_state',2))
    def test_closed_records(self):
        arms=[{'ArmState':1},{'ArmState':0}];modes=[{'ModeNum':4},{'ModeNum':3},{'ModeNum':4},{'ModeNum':9}];cmd=[{'CTot':3,'CId':16,'CNum':n} for n in (1,2)]
        self.assertFalse(audit_dataflash(arms,modes,cmd,{'SIM_WIND_SPD':0},{'SIM_WIND_SPD':0})['whole_bin_framing_qualified'])
        for changed in [([],modes,cmd,{'SIM_WIND_SPD':0}),(arms,[{'ModeNum':3},{'ModeNum':9}],cmd,{'SIM_WIND_SPD':0}),(arms,modes,cmd[:1],{'SIM_WIND_SPD':0}),(arms,modes,cmd,{'SIM_WIND_SPD':1})]:
            with self.assertRaises(ValueError):audit_dataflash(*changed,{'SIM_WIND_SPD':0})
    def test_atomic_marker_ack(self):
        class Receiver:
            def check(self):pass
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory);token=b'phase:auto.finished:1:1000.0\n';(path/'auto.finished').write_bytes(token);times={};events=[]
            record_marker(path,'auto.finished',times,events,Receiver());self.assertEqual((path/'auto.finished.ack').read_bytes(),token)
            self.assertEqual(audit_boundary(events,token,token)['result'],'PASS')
            for bad in (b'other',b''):
                with self.assertRaises(ValueError):audit_boundary(events,token,bad)
            reordered=copy.deepcopy(events);reordered.reverse()
            with self.assertRaises(ValueError):audit_boundary(reordered,token,token)
    def test_preexisting_ack_fails(self):
        class Receiver:
            def check(self):pass
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory);(path/'auto.finished').write_bytes(b'phase:auto.finished:1:1000.0\n');(path/'auto.finished.ack').write_bytes(b'old')
            with self.assertRaises(RuntimeError):record_marker(path,'auto.finished',{},[],Receiver())
            self.assertEqual((path/'auto.finished.ack').read_bytes(),b'old')
    def test_receiver_fault_cannot_ack(self):
        class Receiver:
            def check(self):raise RuntimeError('sticky receive fault')
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory);(path/'auto.finished').write_bytes(b'phase:auto.finished:1:1000.0\n')
            with self.assertRaises(RuntimeError):record_marker(path,'auto.finished',{},[],Receiver())
            self.assertFalse((path/'auto.finished.ack').exists())
    def test_invalid_token_cannot_ack(self):
        class Receiver:
            def check(self):pass
        for token in (b'',b'x'*129):
            with tempfile.TemporaryDirectory() as directory:
                path=Path(directory);(path/'auto.finished').write_bytes(token)
                with self.assertRaises(RuntimeError):record_marker(path,'auto.finished',{},[],Receiver())
                self.assertFalse((path/'auto.finished.ack').exists())
    def test_ack_race_never_overwrites(self):
        class Receiver:
            def check(self):pass
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory);(path/'auto.finished').write_bytes(b'phase:auto.finished:1:1000.0\n');events=[]
            def raced_link(source,destination):
                self.assertEqual(Path(source).read_bytes(),b'phase:auto.finished:1:1000.0\n');Path(destination).write_bytes(b'foreign');raise FileExistsError('foreign ack raced publication')
            with mock.patch('run_native.os.link',side_effect=raced_link):
                with self.assertRaises(FileExistsError):record_marker(path,'auto.finished',{},events,Receiver())
            self.assertEqual((path/'auto.finished.ack').read_bytes(),b'foreign')
            self.assertFalse(any(e['kind']=='auto.finished.ack' for e in events))


class PhaseProtocolTests(unittest.TestCase):
    def reject(self,mutate):
        events,_,_=fixture();mutate(events);events.sort(key=lambda e:e['elapsed'])
        with self.assertRaises((ValueError,KeyError)):audit_probe(events,'auto_complete_handback')
    def test_all_four_acknowledgements_required(self):
        for key in ('auto.completed','auto.finished','return.arrived','return.finished'):
            with self.subTest(key=key):self.reject(lambda es:es.__setitem__(slice(None),[e for e in es if not(e['type']=='phase_ack' and e['key']==key)]))
    def test_missing_separate_return_stage_rejected(self):self.reject(lambda es:es.__setitem__(slice(None),[e for e in es if not(e['type']=='phase' and e['key']=='return.arrived')]))
    def test_return_dwell_starts_after_verified_start_ack(self):
        def mutate(es):next(e for e in es if e['type']=='phase_ack' and e['key']=='return.arrived').update(ack_observed_elapsed=38.21,elapsed=38.21)
        self.reject(mutate)
    def test_auto_dwell_starts_after_verified_start_ack(self):
        def mutate(es):next(e for e in es if e['type']=='phase_ack' and e['key']=='auto.completed').update(ack_observed_elapsed=21.1,elapsed=21.1)
        self.reject(mutate)
    def test_land_must_follow_finish_ack_trace(self):
        def mutate(es):next(e for e in es if e['type']=='phase_ack' and e['key']=='return.finished')['elapsed']=41.41
        self.reject(mutate)
    def test_return_observation_requires_fresh_guided_no_auto_owner(self):
        for change in ('fc_mode=9','home_valid=false','permission=true'):
            def mutate(es):
                row=next(e for e in es if e['type']=='runtime' and 38.1<=e['elapsed']<=41.2)
                before={'fc_mode=9':'fc_mode=4','home_valid=false':'home_valid=true','permission=true':'permission=false'}[change];row['value']=row['value'].replace(before,change)
            with self.subTest(change=change):self.reject(mutate)
        self.reject(lambda es:next(e for e in es if e['type']=='execution_status' and e['elapsed']>38).__setitem__('value','schema=1,id=run,lease_retained=true'))
    def test_phase_token_bound_to_exact_key(self):
        self.reject(lambda es:next(e for e in es if e['type']=='phase_ack' and e['key']=='return.finished').__setitem__('value','phase:return.arrived:1:1000.0\n'))
    def request(self):
        return {'elapsed':1.01,'value':'id=return,timeout=45','local_publication_attempt_elapsed':1.,'local_original_expiry_elapsed':46.,'local_clock_scope':'probe_publication_attempt_not_host_expiry'}
    def ack(self):
        return {'elapsed':45.8,'original_request_id':'return','local_publication_attempt_elapsed':1.,'local_original_expiry_elapsed':46.,'wait_started_elapsed':45.,'deadline_elapsed':46.,'ack_observed_elapsed':45.7}
    def test_original_request_expiry_clips_three_second_wait(self):
        self.assertEqual(phase_ack_observation({'elapsed':45.1},self.ack(),self.request(),'return'),45.7)
        bad=self.ack();bad['deadline_elapsed']=48.
        with self.assertRaises(ValueError):phase_ack_observation({'elapsed':45.1},bad,self.request(),'return')
    def test_late_ack_after_original_request_expiry_is_rejected(self):
        bad=self.ack();bad.update(ack_observed_elapsed=46.,elapsed=46.01)
        with self.assertRaises(ValueError):phase_ack_observation({'elapsed':45.1},bad,self.request(),'return')
    def test_marker_delay_cannot_renew_original_wait_start(self):
        req=self.request();bad=self.ack();bad.update(wait_started_elapsed=40.,deadline_elapsed=43.,ack_observed_elapsed=43.1,elapsed=43.2)
        with self.assertRaises(ValueError):phase_ack_observation({'elapsed':42.},bad,req,'return')
    def test_host_expiry_domain_cannot_be_used_as_probe_publication_time(self):
        req=self.request();req['local_clock_scope']='host_expiry'
        with self.assertRaises(ValueError):phase_ack_observation({'elapsed':45.1},self.ack(),req,'return')
    def callback_fixture(self,scenario='auto_complete_handback'):
        events,times,_=fixture(scenario);proof=audit_probe(events,scenario);data={};coordinator=[]
        for key,at in times.items():
            coordinator.append({'kind':key,'python_mono':100+at})
            if key in proof['boundary_tokens']:
                token=proof['boundary_tokens'][key].encode();data[key]=data[key+'.ack']=token
                coordinator.append({'kind':key+'.ack','python_mono':100+at+.002,'token_sha256':hashlib.sha256(token).hexdigest()})
        coordinator.sort(key=lambda row:row['python_mono']);return data,coordinator,proof
    def test_four_boundaries_replay_from_bytes_without_extraction(self):
        for scenario in ('auto_complete_handback','auto_pause_abort'):
            data,events,proof=self.callback_fixture(scenario);self.assertEqual(len(phase_boundary.audit_boundary_bytes(events,proof,data.__getitem__)['boundaries']),4)
    def test_reordered_or_foreign_ack_cannot_replay(self):
        data,events,proof=self.callback_fixture();first=next(i for i,row in enumerate(events) if row['kind']=='auto.completed');events[first],events[first+1]=events[first+1],events[first]
        with self.assertRaises(ValueError):phase_boundary.audit_boundary_bytes(events,proof,data.__getitem__)
        data,events,proof=self.callback_fixture();data['return.finished.ack']=b'foreign'
        with self.assertRaises(ValueError):phase_boundary.audit_boundary_bytes(events,proof,data.__getitem__)
    def test_preexisting_temporary_is_preserved(self):
        class Receiver:
            def check(self):pass
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder);(path/'return.finished').write_bytes(b'phase:return.finished:1:1000.0\n');temporary=path/'return.finished.ack.tmp';temporary.write_bytes(b'foreign')
            with self.assertRaises(FileExistsError):phase_boundary.record_marker(path,'return.finished',{},[],Receiver())
            self.assertEqual(temporary.read_bytes(),b'foreign');self.assertFalse((path/'return.finished.ack').exists())


class CleanupProofTests(unittest.TestCase):
    def initial(self):
        return {'result':'PASS','ports':dict(bridge=1,monitor=2,rc=3,db=4,base=5),
                'ports_clear':dict(bridge=True,monitor=True,rc=True,db=True,base=True),
                'passive_receiver':{'thread_retired':True,'socket_closed':True,'error':None}}
    def serialized(self,result,remaining,proven,errors):
        finalize_cleanup(result,remaining,proven,errors)
        return json.loads(json.dumps(result,allow_nan=False))
    def test_empty_is_proven_only_after_all_cleanup_gates(self):
        ownership=mock.Mock();ownership.stop_process_tree.return_value=[]
        remaining,proven,errors=stop_owned_processes(ownership,[object(),object()])
        result=self.serialized(self.initial(),remaining,proven,errors)
        self.assertTrue(result['cleanup_proven']);self.assertEqual(result['owned_processes_remaining'],[]);self.assertEqual(result['result'],'PASS')
        self.assertEqual(ownership.stop_process_tree.call_count,2)
    def test_inventory_exception_serializes_unknown_and_attempts_every_process(self):
        ownership=mock.Mock();ownership.stop_process_tree.side_effect=[[],RuntimeError('inventory failed'),[]]
        remaining,proven,errors=stop_owned_processes(ownership,[object(),object(),object()])
        result=self.serialized(self.initial(),remaining,proven,errors)
        self.assertFalse(result['cleanup_proven']);self.assertIsNone(result['owned_processes_remaining']);self.assertEqual(result['result'],'FAIL')
        self.assertEqual(result['known_owned_processes_remaining'],[]);self.assertEqual(result['cleanup_exceptions'],['inventory failed']);self.assertEqual(ownership.stop_process_tree.call_count,3)
    def test_non_list_cleanup_return_is_not_empty_inventory(self):
        for observed in (None,{},(),'',False):
            with self.subTest(observed=observed):
                ownership=mock.Mock();ownership.stop_process_tree.return_value=observed
                result=self.serialized(self.initial(),*stop_owned_processes(ownership,[object()]))
                self.assertFalse(result['cleanup_proven']);self.assertIsNone(result['owned_processes_remaining']);self.assertEqual(result['result'],'FAIL')
    def test_actual_remaining_process_is_preserved_as_remaining(self):
        ownership=mock.Mock();ownership.stop_process_tree.return_value=[{'pid':123,'command':'owned fixture'}]
        result=self.serialized(self.initial(),*stop_owned_processes(ownership,[object()]))
        self.assertFalse(result['cleanup_proven']);self.assertEqual(result['owned_processes_remaining'],[{'pid':123,'command':'owned fixture'}]);self.assertEqual(result['result'],'FAIL')
    def test_receiver_retirement_error_or_unknown_cannot_prove_cleanup(self):
        for status in (None,{},dict(thread_retired=False,socket_closed=True,error=None),dict(thread_retired=True,socket_closed=False,error=None),dict(thread_retired=True,socket_closed=True,error='sticky receive failure')):
            with self.subTest(status=status):
                result=self.initial();result['passive_receiver']=status;result=self.serialized(result,[],True,[])
                self.assertFalse(result['cleanup_proven']);self.assertIsNone(result['owned_processes_remaining']);self.assertEqual(result['result'],'FAIL')
    def test_missing_blocked_or_unknown_port_cannot_prove_cleanup(self):
        for value in (None,False,1,'true'):
            result=self.initial();result['ports_clear']['monitor']=value;result=self.serialized(result,[],True,[])
            self.assertFalse(result['cleanup_proven']);self.assertIsNone(result['owned_processes_remaining'])
        result=self.initial();del result['ports_clear']['monitor'];result=self.serialized(result,[],True,[]);self.assertFalse(result['cleanup_proven']);self.assertIsNone(result['owned_processes_remaining'])
    def test_close_exception_cannot_be_overridden_by_empty_inventory(self):
        result=self.serialized(self.initial(),[],True,['socket close failed'])
        self.assertFalse(result['cleanup_proven']);self.assertIsNone(result['owned_processes_remaining']);self.assertEqual(result['result'],'FAIL')
    def test_never_started_receiver_and_clean_cleanup_do_not_promote_prior_failure(self):
        result=self.initial();result['result']='FAIL';result['passive_receiver']['not_started']=True
        result=self.serialized(result,[],True,[])
        self.assertTrue(result['cleanup_proven']);self.assertEqual(result['owned_processes_remaining'],[]);self.assertEqual(result['result'],'FAIL')


if __name__=='__main__':unittest.main()
