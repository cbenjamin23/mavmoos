#!/usr/bin/env python3
"""Offline and inert owned-socket controls; never launch MOOS or native processes."""
import copy
import errno
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import socket
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
from observation_load_evidence import audit_probe as audit_selected_probe,audit_fc,audit_attitude_consistency,audit_finish_boundary,parse
import run_observation_load as runner

ROOT=Path(__file__).resolve().parents[2]
_spec=importlib.util.spec_from_file_location('_observation_fixture_test_selector',ROOT/'scripts/firmware/scenario_fixtures.py')
F=importlib.util.module_from_spec(_spec);_spec.loader.exec_module(F)
def selected(profile='arducopter-dev-04cf6486-quad'):
    pin=F.PROFILES[profile]
    identity={'schema':1,'profile_id':profile,'manifest_sha256':pin['firmware_manifest_sha256'],
              'source_revision':pin['source_revision'],'frame':'quad','native_sha256':'1'*64,
              'attestation_sha256':'2'*64,'qualification':'not_inferred'}
    return F.select(ROOT,identity,'armed_wiped_mission_read')
def audit_probe(events,profile='attitude',selection=None):
    return audit_selected_probe(events,profile,selected() if selection is None else selection)

def fixture():
    events=[]
    def add(kind,key,value,t=1,source=None):
        if not isinstance(value,str):value=json.dumps(value)
        events.append({'type':kind,'key':key,'value':value,'elapsed':t,'source':source or ('probe' if kind in ('request','phase','assertion','pass') else 'iMAVLink')})
    modules=[{'name':n,'control_provider':n=='arducopter_control','request_owners':[],
              'capabilities':[{'name':n+'.observation','access':'observation'}]} for n in ['telemetry','system_status','attitude','parameters','missions','arducopter_control']]
    add('catalog','MAVMOOS_CAPABILITIES',{'schema':1,'execution_mode':'SITL_CONTROL','composition':{'plugin_abi':10,'modules':modules}})
    add('assertion','mission_fixture_shape','bound_home_only')
    for key,t,n in [('load.started',10,20),('reads.completed',30,120),('load.finished',35,145)]:add('phase',key,'nav_tx_accepted='+str(n),t)
    for value in ['id=observation-guided,action=guided','id=observation-arm,action=arm','id=observation-takeoff,action=takeoff,height=5','id=observation-land,action=land']:
        add('request','MAVMOOS_COMMAND',value);ident=value.split(',')[0].split('=')[1];add('command_result','MAVMOOS_COMMAND_RESULT','id='+ident+',status=COMPLETED')
    for value in ['id=observation-before,action=read,name=SIM_WIND_SPD','id=observation-list,action=list','id=observation-after,action=read,name=SIM_WIND_SPD']:add('request','MAVMOOS_PARAM_REQUEST',value)
    add('request','MAVMOOS_MISSION_REQUEST','id=observation-mission,action=download,timeout=15')
    add('request','MAVMOOS_NAV_CONTROL','acquire')
    for i in range(125):
        t=10.05+i*.2;nav=21+i
        add('nav_counter','MAVMOOS_RUNTIME_STATUS','nav_tx_accepted='+str(nav),t)
        add('runtime','MAVMOOS_RUNTIME_STATUS','permission=true,heartbeat_valid=1,position_valid=1,landed_valid=1,estimator_valid=1,fc_mode=4,mission_mutations=0,control_profile=arducopter,nav_tx_accepted='+str(nav),t+1e-6)
        add('helm','MAVMOOS_HELM_STATUS','state=ACTIVE,reason=valid',t)
    add('assertion','load_continuity','nav_before=20,nav_after=145,max_nav_counter_update_gap_s=0.2,mission_mutations=0',35)
    for ident,at in [('observation-before',1005),('observation-after',1040)]:
        add('parameter_result','MAVMOOS_PARAM_RESULT',f'id={ident},action=read,status=COMPLETED,expires={at+5},at={at}')
        add('parameter_value','MAVMOOS_PARAM_VALUE',f'id={ident},name=SIM_WIND_SPD,value=0')
    for status,at in [('ADMITTED',1010),('COMPLETED',1030)]:add('parameter_result','MAVMOOS_PARAM_RESULT',f'id=observation-list,action=list,status={status},expires=1070,at={at},list_mode=paced,received=100,expected=100')
    table={'schema':1,'id':'observation-list','status':'complete','list_mode':'paced','completed_at':1030,'count':100,'values':[{'index':n,'name':'P'+str(n),'value':0} for n in range(100)]}
    add('parameter_snapshot','MAVMOOS_PARAM_SNAPSHOT',table,30)
    for status,at in [('ADMITTED',1010),('COMPLETED',1011)]:
        n=1 if status=='COMPLETED' else 0
        add('mission_result','MAVMOOS_MISSION_RESULT',f'id=observation-mission,action=download,status={status},expires=1025,at={at},mutation_may_have_applied=false,received={n},expected={n}')
    add('mission_snapshot','MAVMOOS_MISSION_SNAPSHOT',{'schema':1,'id':'observation-mission','status':'complete','mission_type':0,'count':1,'verification':'observation','atomic':False,'items':[{'seq':0,'command':16,'frame':0,'current':1,'autocontinue':1,'params':[0,0,0,0],'x':423584560,'y':-710875890,'z':10}]})
    add('home','MAVMOOS_HOME_POSITION','present=true,encoding_valid=true,freshness=REFERENCE_KNOWN,identity=EXPECTED,lat_deg=42.358456,lon_deg=-71.087589,alt_msl_m=10')
    for mid in (43,51,47):add('mission_tx','MAVMOOS_MISSION_TX_RESULT',f'operation_id=observation-mission,msgid={mid},status=LOCAL_ACCEPTED,late=false,clock_valid=true')
    for key in ['MAVMOOS_SYS_STATUS','MAVMOOS_GPS_STATUS','MAVMOOS_BATTERY_STATUS','MAVMOOS_ATTITUDE_EULER','MAVMOOS_ATTITUDE_QUATERNION']:
        for t in range(25,36):
            data={}
            if key=='MAVMOOS_BATTERY_STATUS':data={'origin':'BATTERY_STATUS','battery_id':0,'voltage_layout_valid':True,'reported_voltage_sum_v':{'known':True,'value':12.6,'unit':'V'}}
            if 'ATTITUDE' in key:
                data={'time_boot_ms':t*1000,'remote_clock':'boot_ms_unmapped_uint32','reference_frame':'LOCAL_NED','body_frame':'BODY_FRD','estimate':'fused_fc_attitude','orientation_valid':True,'angular_velocity_valid':True,
                      'angular_velocity_body_frd_rad_s':{axis:{'known':True,'value':0,'unit':'rad/s'} for axis in ('x','y','z')}}
                rpy={key:{'known':True,'value':0,'unit':'rad'} for key in ('roll_rad','pitch_rad','yaw_rad')};rpy.update(rpy_unique=True,source_yaw_heading_deg={'known':True,'value':0,'unit':'deg'})
                rpy['yaw_rad']['value']=math.pi/2;rpy['source_yaw_heading_deg']['value']=90
                if key.endswith('EULER'):data.update(raw_angles_rad=[0,0,math.pi/2],wire_declared_angle_range_conformant=True,profile_angle_range='plusminus_pi_with_float_tolerance',reported_rpy=rpy)
                else:data.update(quaternion_wxyz=[math.sqrt(.5),0,0,math.sqrt(.5)],quaternion_norm=1,unit_norm_tolerance=.001,quaternion_components_finite=True,derived_normalized_within_tolerance=True,derived_convention='hamilton_wxyz_zyx_in_source_ned_frd',derived_rpy=rpy,representation_offset={'raw_wxyz':[0,0,0,0],'status':'unsupported_or_omitted','encoding_valid':True,'extension_bytes_observed':False,'display_adjustment_applied':False})
            add('observation',key,{'schema':1,'present':True,'encoding_valid':True,'fresh':True,'effective_at':1000+t,'read_at':1000+t+.1,'age_s':.2,'age_basis':'QUEUE_EMPTY_FLOOR','age_is_upper_bound':True,'observation_id':t,'source_age':'unknown','source_quality':'unassessed','data':data},t)
    for key in ('MAVMOOS_SYSTEM_STATUS','MAVMOOS_ATTITUDE_STATUS'):add('observation',key,{'schema':1,'state':'BOUND','read_only':True,'authoritative_for_control':False,'atomic':False},34)
    add('pass','observation_load','complete',40)
    return events

def change(events,kind,key,mutate):
    for e in events:
        if e['type']==kind and e['key']==key:
            data=parse(e['value']);mutate(data);e['value']=json.dumps(data)

def fc_fixture():
    packets=[]
    for i in range(151):
        t=9+i*.2;base={'mono':t,'system':1,'component':1}
        for kind,values in [
            ('HEARTBEAT',{'custom_mode':4,'base_mode':128}),('GLOBAL_POSITION_INT',{'lat':423584560+int(t*100),'lon':-710875890,'relative_alt':5000,'vx':100,'vy':0}),
            ('EXTENDED_SYS_STATE',{'landed_state':2}),('POSITION_TARGET_LOCAL_NED',{'time_boot_ms':int(t*1000),'coordinate_frame':1,'type_mask':7|1024|2048,'vx':1,'vy':0,'vz':0}),
            ('SYS_STATUS',{}),('GPS_RAW_INT',{}),('BATTERY_STATUS',{'id':0,'voltages':[12600]+[65535]*9,'voltages_ext':[0]*4}),
            ('ATTITUDE',{'time_boot_ms':int(t*1000),'roll':0,'pitch':0,'yaw':math.pi/2,'rollspeed':0,'pitchspeed':0,'yawspeed':0}),
            ('ATTITUDE_QUATERNION',{'time_boot_ms':int(t*1000),'q1':math.sqrt(.5),'q2':0,'q3':0,'q4':math.sqrt(.5),'rollspeed':0,'pitchspeed':0,'yawspeed':0})]:packets.append({**base,'mavpackettype':kind,**values})
    for kind,values in [('HEARTBEAT',{'custom_mode':9,'base_mode':0}),('EXTENDED_SYS_STATE',{'landed_state':1})]:packets.append({'mono':40,'system':1,'component':1,'mavpackettype':kind,**values})
    return packets

class EvidenceTests(unittest.TestCase):
    def test_complete_profile(self):
        self.assertEqual(audit_probe(fixture())['table_count'],100)
        self.assertEqual(audit_fc(fc_fixture(),10,30,35,40)['result'],'PASS')
        self.assertEqual(audit_attitude_consistency(fixture(),fc_fixture(),(10,30,35))['result'],'PASS')
    def test_early_samples_do_not_qualify_during_or_after_reads(self):
        for cutoff in (26,31):
            e=fixture();e=[x for x in e if not (x['type']=='observation' and x['key']=='MAVMOOS_GPS_STATUS' and x['elapsed']>=cutoff)]
            with self.assertRaises(ValueError):audit_probe(e)
    def test_unknown_nonfinite_and_contradictory_metadata(self):
        for mutation in [lambda v:v.update(age_s=999),lambda v:v.update(read_at=0),lambda v:v.update(age_basis='UNKNOWN'),lambda v:v.update(age_is_upper_bound=False),lambda v:v.update(age_s=True),lambda v:v.update(read_at=math.inf),lambda v:v.update(observation_id=True),lambda v:v.update(fresh='true')]:
            e=fixture();change(e,'observation','MAVMOOS_ATTITUDE_EULER',mutation)
            with self.subTest(mutation=mutation),self.assertRaises(ValueError):audit_probe(e)
    def test_cached_republication_is_not_advancing_observation(self):
        for mutation in [lambda v:v.update(observation_id=1),lambda v:v.update(read_at=1100),lambda v:v.update(encoding_valid=False),lambda v:v.update(source_age='0')]:
            e=fixture();change(e,'observation','MAVMOOS_ATTITUDE_QUATERNION',mutation)
            with self.assertRaises(ValueError):audit_probe(e)
    def test_battery_voltage_identity_and_units(self):
        for mutation in [lambda v:v['data'].update(battery_id=1),lambda v:v['data']['reported_voltage_sum_v'].update(value=0),lambda v:v['data']['reported_voltage_sum_v'].update(known=False),lambda v:v['data']['reported_voltage_sum_v'].update(unit='mV')]:
            e=fixture();change(e,'observation','MAVMOOS_BATTERY_STATUS',mutation)
            with self.assertRaises(ValueError):audit_probe(e)
    def test_observers_cannot_own_control(self):
        for mutation in [lambda c:c['composition'].update(plugin_abi=6),lambda c:c['composition']['modules'][1].update(control_provider=True),lambda c:c['composition']['modules'][2].update(request_owners=['MAVMOOS_NAV_CONTROL'])]:
            e=fixture();change(e,'catalog','MAVMOOS_CAPABILITIES',mutation)
            with self.assertRaises(ValueError):audit_probe(e)
    def test_authority_loss_or_early_runtime_fails(self):
        for before,after in [('permission=true','permission=false'),('heartbeat_valid=1','heartbeat_valid=0'),('mission_mutations=0','mission_mutations=1')]:
            e=fixture();next(x for x in e if x['type']=='runtime' and x['elapsed']>27)['value']=next(x for x in e if x['type']=='runtime')['value'].replace(before,after)
            with self.assertRaises(ValueError):audit_probe(e)
        with self.assertRaises(ValueError):audit_probe([e for e in fixture() if not (e['type']=='runtime' and e['elapsed']>32)])
    def test_original_deadlines_snapshot_and_terminal_must_match(self):
        for before,after in [('expires=1070,at=1030','expires=1071,at=1030'),('status=COMPLETED,expires=1070','status=INCOMPLETE,expires=1070'),('expires=1070','expires=1200')]:
            e=fixture()
            for row in e:
                if row['type']=='parameter_result':row['value']=row['value'].replace(before,after)
            with self.assertRaises(ValueError):audit_probe(e)
        e=fixture();change(e,'parameter_snapshot','MAVMOOS_PARAM_SNAPSHOT',lambda v:v['values'].pop())
        with self.assertRaises(ValueError):audit_probe(e)
    def test_frozen_nav_counter_or_forged_summary_does_not_pass(self):
        e=fixture()
        for row in e:
            if row['type']=='runtime':row['value']=row['value'].rsplit('nav_tx_accepted=',1)[0]+'nav_tx_accepted=20'
        with self.assertRaises(ValueError):audit_probe(e)
        e=fixture();next(row for row in e if row['type']=='assertion' and row['key']=='load_continuity')['value']='nav_before=20,nav_after=999,max_nav_counter_update_gap_s=0.2,mission_mutations=0'
        with self.assertRaises(ValueError):audit_probe(e)
        with self.assertRaises(ValueError):audit_probe([row for row in fixture() if row['type']!='nav_counter'])
    def test_nav_tail_gap_cannot_hide_in_early_progress(self):
        e=fixture();e=[row for row in e if not (row['type']=='nav_counter' and row['elapsed']>33)]
        for row in e:
            if row['type']=='phase' and row['key']=='load.finished':row['value']='nav_tx_accepted=85'
        with self.assertRaises(ValueError):audit_probe(e)
    def test_no_mutations_or_reacquisition(self):
        for key,value in [('MAVMOOS_PARAM_REQUEST','id=x,action=write,name=SIM_WIND_SPD,value=1'),('MAVMOOS_MISSION_REQUEST','id=x,action=clear'),('MAVMOOS_NAV_CONTROL','acquire')]:
            e=fixture();e.append({'type':'request','key':key,'value':value,'elapsed':20,'source':'probe'})
            with self.assertRaises(ValueError):audit_probe(e)
    def test_mission_home_and_wire_read_only(self):
        e=fixture();change(e,'mission_snapshot','MAVMOOS_MISSION_SNAPSHOT',lambda v:v['items'][0].update(x=0))
        with self.assertRaises(ValueError):audit_probe(e)
        e=fixture();next(x for x in e if x['type']=='mission_tx')['value']='operation_id=observation-mission,msgid=44,status=LOCAL_ACCEPTED,late=false,clock_valid=true'
        with self.assertRaises(ValueError):audit_probe(e)
    def test_independent_fc_missing_target_gap_mode_or_motion(self):
        for mutate in [lambda p:[x for x in p if not (x['mavpackettype']=='POSITION_TARGET_LOCAL_NED' and x['mono']>30)],lambda p:[{**x,'base_mode':0} if x['mavpackettype']=='HEARTBEAT' and 10<x['mono']<35 else x for x in p],lambda p:[{**x,'lat':423584560} if x['mavpackettype']=='GLOBAL_POSITION_INT' else x for x in p]]:
            with self.assertRaises(ValueError):audit_fc(mutate(fc_fixture()),10,30,35,40)
    def test_target_frame_mask_and_finite_values(self):
        for mutation in ({'coordinate_frame':6},{'type_mask':7|8},{'vx':math.nan},{'vx':0,'vy':0}):
            p=fc_fixture();p=[{**x,**mutation} if x['mavpackettype']=='POSITION_TARGET_LOCAL_NED' else x for x in p]
            with self.assertRaises(ValueError):audit_fc(p,10,30,35,40)
    def test_fc_early_battery_or_stale_final_is_not_enough(self):
        p=[x for x in fc_fixture() if not (x['mavpackettype']=='BATTERY_STATUS' and x['mono']>26)]
        with self.assertRaises(ValueError):audit_fc(p,10,30,35,40)
        with self.assertRaises(ValueError):audit_fc(fc_fixture(),10,30,35,43)
    def test_separate_observer_clock_offset_is_irrelevant(self):
        p=fc_fixture()
        for x in p:x['mono']+=100000
        self.assertEqual(audit_fc(p,100010,100030,100035,100040)['result'],'PASS')
    def test_attitude_convention_units_and_timestamp_pairing(self):
        for mutation in [lambda v:v['data'].update(body_frame='BODY_FLU'),lambda v:v['data'].update(raw_angles_rad=[0,0,-math.pi/2]),lambda v:v['data'].update(time_boot_ms=4000000000),lambda v:v['data']['angular_velocity_body_frd_rad_s']['x'].update(unit='deg/s')]:
            e=fixture();change(e,'observation','MAVMOOS_ATTITUDE_EULER',mutation)
            with self.assertRaises(ValueError):audit_attitude_consistency(e,fc_fixture(),(10,30,35))
    def test_quaternion_sign_equivalence_does_not_mean_component_equality(self):
        e=fixture();change(e,'observation','MAVMOOS_ATTITUDE_QUATERNION',lambda v:v['data'].update(quaternion_wxyz=[-math.sqrt(.5),0,0,-math.sqrt(.5)]))
        self.assertEqual(audit_attitude_consistency(e,fc_fixture(),(10,30,35))['result'],'PASS')
    def test_wrapped_euler_comparison_cannot_hide_out_of_range_raw(self):
        e=fixture();change(e,'observation','MAVMOOS_ATTITUDE_EULER',lambda v:v['data'].update(raw_angles_rad=[0,0,2*math.pi]))
        with self.assertRaises(ValueError):audit_attitude_consistency(e,fc_fixture(),(10,30,35))
    def test_derived_orientation_norm_or_heading_must_be_consistent(self):
        for mutation in [lambda v:v['data']['derived_rpy']['yaw_rad'].update(value=999),lambda v:v['data']['derived_rpy']['source_yaw_heading_deg'].update(value=180),lambda v:v['data'].update(quaternion_norm=0),lambda v:v['data']['representation_offset'].update(raw_wxyz=[2,0,0,0]),lambda v:v['data'].update(derived_normalized_within_tolerance=False)]:
            e=fixture();change(e,'observation','MAVMOOS_ATTITUDE_QUATERNION',mutation)
            with self.subTest(mutation=mutation),self.assertRaises(ValueError):audit_attitude_consistency(e,fc_fixture(),(10,30,35))
    def test_wire_out_of_range_euler_and_impossible_offset_bytes_fail(self):
        p=fc_fixture()
        for row in p:
            if row['mavpackettype']=='ATTITUDE':row['yaw']=2*math.pi
        with self.assertRaises(ValueError):audit_attitude_consistency(fixture(),p,(10,30,35))
        e=fixture();change(e,'observation','MAVMOOS_ATTITUDE_QUATERNION',lambda v:v['data']['representation_offset'].update(raw_wxyz=[1,0,0,0],status='unit_offset_unapplied',extension_bytes_observed=False))
        with self.assertRaises(ValueError):audit_attitude_consistency(e,fc_fixture(),(10,30,35))
    def test_tiny_negative_yaw_normalizes_expected_heading_to_zero(self):
        e=fixture();p=fc_fixture()
        def euler(v):
            v['data']['raw_angles_rad']=[0,0,-1e-17];v['data']['reported_rpy']['yaw_rad']['value']=-1e-17;v['data']['reported_rpy']['source_yaw_heading_deg']['value']=0
        def quaternion(v):
            v['data']['quaternion_wxyz']=[1,0,0,-5e-18];v['data']['derived_rpy']['yaw_rad']['value']=-1e-17;v['data']['derived_rpy']['source_yaw_heading_deg']['value']=0
        change(e,'observation','MAVMOOS_ATTITUDE_EULER',euler);change(e,'observation','MAVMOOS_ATTITUDE_QUATERNION',quaternion)
        for row in p:
            if row['mavpackettype']=='ATTITUDE':row['yaw']=0
            elif row['mavpackettype']=='ATTITUDE_QUATERNION':row.update(q1=1,q2=0,q3=0,q4=0)
        self.assertEqual(audit_attitude_consistency(e,p,(10,30,35))['result'],'PASS')
        change(e,'observation','MAVMOOS_ATTITUDE_EULER',lambda v:v['data']['reported_rpy']['source_yaw_heading_deg'].update(value=360))
        with self.assertRaises(ValueError):audit_attitude_consistency(e,p,(10,30,35))
    def test_nan_literals_and_exponent_overflow_fail(self):
        for text in ('{"x":NaN}','{"x":1e999}','{"x":Infinity}'):
            with self.assertRaises(ValueError):parse(text)

class DeclaredMissionFixtureTests(unittest.TestCase):
    def empty(self):
        events=fixture()
        for row in events:
            if row['type']=='assertion' and row['key']=='mission_fixture_shape':row['value']='empty'
            if row['type']=='mission_snapshot':
                value=json.loads(row['value']);value.update(count=0,items=[]);row['value']=json.dumps(value)
            if row['type']=='mission_result':row['value']=row['value'].replace('received=1,expected=1','received=0,expected=0')
        return [row for row in events if not (row['type']=='mission_tx' and 'msgid=51,' in row['value'])]
    def test_empty_is_selected_before_observation_and_exercises_zero_items(self):
        proof=audit_probe(self.empty(),selection=selected('arducopter-4.6.3-quad'))
        self.assertEqual((proof['mission_count'],proof['mission_item_records_exercised'],proof['mission_shape']),(0,0,'empty'))
        self.assertEqual(proof['nav_counter_after'],145)
    def test_home_profile_cannot_accept_empty_even_with_matching_enum(self):
        events=self.empty();next(e for e in events if e['key']=='mission_fixture_shape')['value']='bound_home_only'
        with self.assertRaises(ValueError):audit_probe(events,selection=selected())
    def test_empty_profile_cannot_accept_home_even_with_matching_enum(self):
        events=fixture();next(e for e in events if e['key']=='mission_fixture_shape')['value']='empty'
        with self.assertRaises(ValueError):audit_probe(events,selection=selected('arducopter-4.6.3-quad'))
    def test_empty_forbids_item_request_and_requires_final_ack(self):
        for mode in ('item','no_ack'):
            events=self.empty()
            if mode=='item':events.append({'type':'mission_tx','key':'MAVMOOS_MISSION_TX_RESULT','source':'iMAVLink','elapsed':20,'value':'operation_id=observation-mission,msgid=51,status=LOCAL_ACCEPTED,late=false,clock_valid=true'})
            else:events=[e for e in events if not (e['type']=='mission_tx' and 'msgid=47,' in e['value'])]
            with self.assertRaises(ValueError):audit_probe(events,selection=selected('arducopter-4.6.3-quad'))
    def test_empty_requires_independently_bound_home(self):
        with self.assertRaisesRegex(ValueError,'HOME not independently bound'):
            audit_probe([e for e in self.empty() if e['type']!='home'],selection=selected('arducopter-4.6.3-quad'))
    def test_empty_snapshot_and_terminal_counts_cannot_disagree(self):
        for field in ('received','expected'):
            events=self.empty()
            for e in events:
                if e['type']=='mission_result' and 'status=COMPLETED' in e['value']:e['value']=e['value'].replace(field+'=0',field+'=1')
            with self.assertRaises(ValueError):audit_probe(events,selection=selected('arducopter-4.6.3-quad'))
    def test_no_declaration_cannot_qualify_probe(self):
        with self.assertRaisesRegex(ValueError,'source-bound mission fixture missing'):audit_selected_probe(fixture())

class OwnershipTests(unittest.TestCase):
    def records(self):return [{'pid':101,'command':'/owned/iMAVLink /owned/mission.moos'},{'pid':102,'command':'/owned/pHelmIvP /owned/mission.moos'}]
    def ownership(self,records=None):return SimpleNamespace(owned_records=lambda _:self.records() if records is None else records)
    def test_exact_live_apps_and_bound_socket(self):
        with patch.object(runner.subprocess,'run',return_value=SimpleNamespace(returncode=0,stdout='UDP 127.0.0.1:4242\n')):
            apps,_=runner.live_apps(self.ownership(),None,Path('/owned/mission.moos'),4242)
        self.assertEqual(apps['iMAVLink']['pid'],101)
    def test_missing_helm_or_bridge_cannot_use_retained_publications(self):
        for rows in (self.records()[:1],self.records()[1:],[]):
            with self.assertRaises(RuntimeError):runner.live_apps(self.ownership(rows),None,Path('/owned/mission.moos'),4242)
    def test_replaced_pid_or_command_rejects(self):
        rows=self.records();expected={Path(r['command'].split()[0]).name:r.copy() for r in rows}
        for field,value in (('pid',103),('command','/other/iMAVLink /owned/mission.moos')):
            changed=copy.deepcopy(rows);changed[0][field]=value
            with self.assertRaises(RuntimeError):runner.live_apps(self.ownership(changed),None,Path('/owned/mission.moos'),4242,expected)
    def test_missing_wrong_port_or_wildcard_bridge_socket_rejects(self):
        for result in (SimpleNamespace(returncode=1,stdout=''),SimpleNamespace(returncode=0,stdout='UDP 127.0.0.1:42420\n'),SimpleNamespace(returncode=0,stdout='UDP *:4242\n')):
            with patch.object(runner.subprocess,'run',return_value=result),self.assertRaises(RuntimeError):runner.live_apps(self.ownership(),None,Path('/owned/mission.moos'),4242)
    def test_duplicate_or_different_mission_app_rejects(self):
        for rows in (self.records()+self.records()[:1],[{**r,'command':r['command'].replace('/owned/mission.moos','/other/mission.moos')} for r in self.records()]):
            with self.assertRaises(RuntimeError):runner.live_apps(self.ownership(rows),None,Path('/owned/mission.moos'),4242)
    def test_explicit_source_root_beats_stale_cached_helpers(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder)
            for filename,marker in [('tests/owned_processes.py','fresh-ownership'),('tests/control/catalog_evidence.py','fresh-catalog'),
                                    ('tests/observation_load/passive_receiver.py','fresh-passive'),
                                    ('scripts/firmware/scenario_fixtures.py','fresh-scenario-fixtures')]:
                path=root/filename;path.parent.mkdir(parents=True,exist_ok=True);path.write_text('marker='+repr(marker)+'\n')
            with patch.dict(sys.modules,{'control.catalog_evidence':SimpleNamespace(marker='stale-ABI6')}):modules=runner.source_helpers(root)
            self.assertEqual(modules['catalog'].marker,'fresh-catalog');self.assertEqual(modules['ownership'].marker,'fresh-ownership')
            self.assertEqual(modules['passive_receiver'].marker,'fresh-passive')
            self.assertEqual(modules['scenario_fixtures'].marker,'fresh-scenario-fixtures')
            self.assertEqual(Path(modules['catalog'].__file__).resolve(),(root/'tests/control/catalog_evidence.py').resolve())
    def test_native_endpoint_requires_exact_both_loopback_ports(self):
        sample='arducopter 123 user UDP 127.0.0.1:52101->127.0.0.1:52102\n'
        self.assertTrue(runner.exact_peer(sample,52101,52102))
        for wrong in [('UDP *:52101->127.0.0.1:52102\n',52101,52102),(sample,52101,521020),(sample,521010,52102),('UDP 127.0.0.2:52101->127.0.0.1:52102\n',52101,52102)]:self.assertFalse(runner.exact_peer(*wrong))

class FinishBoundaryTests(unittest.TestCase):
    token=b'opaque producer clock bytes\n'
    def boundary(self):
        values='token_hex='+self.token.hex()+',wait_s=0.2,timeout_s=3,nav_tx_accepted=145,max_nav_counter_update_gap_s=0.2'
        events=[{'type':'phase','key':'load.finished','source':'probe','elapsed':35,'value':'nav_tx_accepted=145'},
                {'type':'assertion','key':'finish_boundary_ack','source':'probe','elapsed':35.2,'value':values}]
        for key,value,t in [('MAVMOOS_PARAM_REQUEST','id=observation-after,action=read,name=SIM_WIND_SPD',35.3),('DEPLOY','false',35.4),('MAVMOOS_COMMAND','id=observation-land,action=land',35.5)]:
            events.append({'type':'request','key':key,'source':'probe','elapsed':t,'value':value})
        coordinator=[{'kind':'load.finished','python_mono':100000000},
                     {'kind':'load.finished.ack','python_mono':100000000.5,'token_sha256':hashlib.sha256(self.token).hexdigest()}]
        return events,coordinator
    def test_content_bound_causal_barrier_uses_separate_clocks(self):
        events,coordinator=self.boundary()
        self.assertEqual(audit_finish_boundary(events,coordinator,self.token,self.token)['result'],'PASS')
    def test_finish_timestamp_precedes_complete_atomic_ack(self):
        with tempfile.TemporaryDirectory() as folder:
            control=Path(folder);(control/'load.finished').write_bytes(self.token);markers={};events=[];real_link=runner.os.link
            def checked_link(source,destination):
                self.assertEqual(markers['load.finished'],123);self.assertEqual(events[0]['python_mono'],123)
                self.assertFalse(destination.exists());self.assertEqual(source.read_bytes(),self.token);real_link(source,destination)
            with patch.object(runner.time,'monotonic',side_effect=[123,125]),patch.object(runner.os,'link',side_effect=checked_link):
                runner.record_marker(control,'load.finished',markers,events)
            self.assertEqual((control/'load.finished.ack').read_bytes(),self.token)
            self.assertEqual(markers['load.finished'],123);self.assertEqual(events[-1]['python_mono'],125)
            self.assertFalse((control/'load.finished.ack.tmp').exists())
    def test_other_or_missing_phase_does_not_publish_finish_ack(self):
        with tempfile.TemporaryDirectory() as folder:
            control=Path(folder);markers={};events=[]
            runner.record_marker(control,'load.finished',markers,events);self.assertEqual(events,[])
            (control/'load.started').write_text('opaque\n');runner.record_marker(control,'load.started',markers,events)
            self.assertEqual(set(markers),{'load.started'});self.assertFalse((control/'load.finished.ack').exists())
    def test_preexisting_ack_is_not_replaced(self):
        with tempfile.TemporaryDirectory() as folder:
            control=Path(folder);(control/'load.finished').write_bytes(self.token);(control/'load.finished.ack').write_bytes(b'foreign')
            with self.assertRaisesRegex(RuntimeError,'preexisting'):runner.record_marker(control,'load.finished',{},[])
            self.assertEqual((control/'load.finished.ack').read_bytes(),b'foreign')
    def test_racing_foreign_ack_cannot_be_overwritten(self):
        with tempfile.TemporaryDirectory() as folder:
            control=Path(folder);(control/'load.finished').write_bytes(self.token);events=[];real_link=runner.os.link
            def racing_link(source,destination):destination.write_bytes(b'foreign');real_link(source,destination)
            with patch.object(runner.os,'link',side_effect=racing_link),self.assertRaises(FileExistsError):runner.record_marker(control,'load.finished',{},events)
            self.assertEqual((control/'load.finished.ack').read_bytes(),b'foreign');self.assertEqual([e['kind'] for e in events],['load.finished'])
    def test_empty_or_oversized_token_cannot_be_acknowledged(self):
        for token in (b'',b'x'*129):
            with tempfile.TemporaryDirectory() as folder:
                control=Path(folder);(control/'load.finished').write_bytes(token)
                with self.assertRaisesRegex(RuntimeError,'token invalid'):runner.record_marker(control,'load.finished',{},[])
                self.assertFalse((control/'load.finished.ack').exists())
    def test_ack_write_or_link_failure_stays_failure(self):
        real_open=Path.open
        def fail_write(path,*args,**kwargs):
            if args and args[0]=='xb':raise OSError('inert acknowledgement write failure')
            return real_open(path,*args,**kwargs)
        for action in ('write','link'):
            with tempfile.TemporaryDirectory() as folder:
                control=Path(folder);(control/'load.finished').write_bytes(self.token);events=[]
                context=patch.object(Path,'open',new=fail_write) if action=='write' else patch.object(runner.os,'link',side_effect=OSError('inert acknowledgement link failure'))
                with context,self.assertRaisesRegex(OSError,'inert acknowledgement'):runner.record_marker(control,'load.finished',{},events)
                self.assertFalse((control/'load.finished.ack').exists());self.assertEqual([e['kind'] for e in events],['load.finished'])
    def test_missing_repeated_unbound_or_late_ack_fails(self):
        mutations=[lambda e:e.pop(1),lambda e:e.insert(1,copy.deepcopy(e[1])),
                   lambda e:e[1].update(value=e[1]['value'].replace('wait_s=0.2','wait_s=3')),
                   lambda e:e[1].update(source='iMAVLink'),lambda e:e[1].update(value=e[1]['value'].replace('timeout_s=3','timeout_s=4'))]
        for mutate in mutations:
            events,coordinator=self.boundary();mutate(events)
            with self.assertRaises(ValueError):audit_finish_boundary(events,coordinator,self.token,self.token)
        events,coordinator=self.boundary()
        with self.assertRaises(ValueError):audit_finish_boundary(events,coordinator,self.token,b'foreign')
    def test_postread_deploy_or_land_cannot_precede_ack(self):
        for index in (2,3,4):
            events,coordinator=self.boundary();events[index]['elapsed']=35.1
            with self.assertRaisesRegex(ValueError,'precedes finish acknowledgement'):audit_finish_boundary(events,coordinator,self.token,self.token)
    def test_coordinator_ack_cannot_precede_boundary_or_change_token(self):
        for mutation in (lambda e:e[1].update(python_mono=99999999),lambda e:e[1].update(token_sha256='0'*64),lambda e:e.reverse()):
            events,coordinator=self.boundary();mutation(coordinator)
            with self.assertRaises(ValueError):audit_finish_boundary(events,coordinator,self.token,self.token)
    def test_no_nav_updates_during_wait_cannot_hide_tail_gap(self):
        events,coordinator=self.boundary();events[1]['elapsed']=36;events[1]['value']=events[1]['value'].replace('wait_s=0.2','wait_s=1')
        with self.assertRaisesRegex(ValueError,'NAV tail gap'):audit_finish_boundary(events,coordinator,self.token,self.token)
    def test_early_land_remains_a_failure_at_actual_receipt_time(self):
        packets=fc_fixture()
        for row in packets:
            if row['mavpackettype']=='HEARTBEAT' and row['mono']==35:row.update(mono=34.999,custom_mode=9,base_mode=217)
        packets.sort(key=lambda p:p['mono'])
        with self.assertRaisesRegex(ValueError,'mode/arming continuity'):audit_fc(packets,10,30,35,40)

class RCReadinessTests(unittest.TestCase):
    def test_rc_availability_matches_native_wildcard_scope(self):
        with patch.object(runner,'free_port',return_value=52103) as allocator:
            self.assertEqual(runner.free_rc_port(),52103)
        allocator.assert_called_once_with(socket.SOCK_DGRAM,'0.0.0.0')
        with patch.object(runner.socket,'socket') as factory:
            stream=factory.return_value.__enter__.return_value;stream.getsockname.return_value=('0.0.0.0',52103)
            self.assertEqual(runner.free_rc_port(),52103)
        stream.bind.assert_called_once_with(('0.0.0.0',0))

    def test_wildcard_and_loopback_reservations_conflict(self):
        # These real test-owned binds prove conflict, not that a particular
        # production failure was a collision. The allocator's INADDR_ANY
        # call above independently checks its scope beyond loopback alone.
        for first,second in [('0.0.0.0','127.0.0.1'),('127.0.0.1','0.0.0.0')]:
            with self.subTest(first=first),socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as occupied,socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as candidate:
                occupied.bind((first,0));port=occupied.getsockname()[1]
                with self.assertRaises(OSError) as failure:candidate.bind((second,port))
                self.assertEqual(failure.exception.errno,errno.EADDRINUSE)

    def test_exact_rc_port_is_required(self):
        for row in ['arducopter 123 user IPv4 UDP *:52103\n','arducopter 123 user IPv4 UDP 0.0.0.0:52103\n']:
            self.assertTrue(runner.exact_rc_input(row,52103))
        for row in ['UDP *:*\n','UDP *:521030\n','UDP *:52104\n','UDP 127.0.0.1:52103\n',
                    'UDP 127.0.0.2:52103\n','UDP *:52103->127.0.0.1:44\n','TCP *:52103\n','']:
            self.assertFalse(runner.exact_rc_input(row,52103),row)

    def test_readiness_uses_exact_native_pid_and_original_deadline(self):
        process=SimpleNamespace(pid=123,poll=lambda:None);row='UDP *:52103\n'
        with patch.object(runner.subprocess,'run',return_value=SimpleNamespace(returncode=0,stdout=row)) as call,patch.object(runner.time,'monotonic',side_effect=[0,.1]):
            self.assertEqual(runner.wait_rc_input(process,52103,1),(row,.1))
        self.assertEqual(call.call_args.args[0],['lsof','-nP','-a','-p','123','-iUDP'])
        self.assertEqual(call.call_args.kwargs['timeout'],3)

    def test_unbound_input_fails_at_existing_deadline(self):
        with patch.object(runner,'inventory',return_value='UDP *:*\n') as inventory,patch.object(runner.time,'monotonic',side_effect=[0,.1,.2,1]),patch.object(runner.time,'sleep') as pause:
            with self.assertRaisesRegex(RuntimeError,'original startup deadline'):runner.wait_rc_input(None,52103,1)
        self.assertEqual(inventory.call_count,1);pause.assert_called_once_with(.02)

    def test_inventory_cannot_turn_a_late_ready_sample_into_success(self):
        with patch.object(runner,'inventory',return_value='UDP *:52103\n'),patch.object(runner.time,'monotonic',side_effect=[0,1]),patch.object(runner.time,'sleep') as pause:
            with self.assertRaisesRegex(RuntimeError,'original startup deadline'):runner.wait_rc_input(None,52103,1)
        pause.assert_not_called()

    def test_inventory_failure_is_not_a_ready_socket(self):
        with patch.object(runner,'inventory',side_effect=RuntimeError('inventory unavailable')),patch.object(runner.time,'monotonic',return_value=0):
            with self.assertRaisesRegex(RuntimeError,'inventory unavailable'):runner.wait_rc_input(None,52103,1)

from PassiveReceiverTests import PassiveReceiverTests

if __name__=='__main__':unittest.main()
