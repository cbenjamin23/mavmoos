#!/usr/bin/env python3
"""Independent synthetic acceptance/negative controls; no sockets or simulator.

Import the actual root checker explicitly. Fixtures model separately labelled
probe, host-acquisition, passive-receipt and FC-boot clock domains; the shared
constants here construct synthetic test data, not an observed clock mapping.
"""
import argparse
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import unittest

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--checker',type=Path,required=True)
parser.add_argument('--report',type=Path)
args,remaining=parser.parse_known_args()
checker_path=args.checker.resolve()
checker_bytes=checker_path.read_bytes()
checker_sha=hashlib.sha256(checker_bytes).hexdigest()
spec=importlib.util.spec_from_file_location('_mavmoos_distance_evidence_review_'+checker_sha,checker_path)
if spec is None or spec.loader is None:raise RuntimeError('cannot load explicit root checker')
checker=importlib.util.module_from_spec(spec);spec.loader.exec_module(checker)


def quantity(value,unit,reason,known=True):
    return {'known':known,'value':value,'unit':unit,'reason':reason}


def data(height,boot_ms):
    unknown_fov={'raw_rad':0,'quantity':quantity(None,'rad','unknown_unset_or_omitted',False)}
    return {'id':0,'time_boot_ms':boot_ms,'remote_clock':'boot_ms_unmapped_uint32',
        'timestamp_semantics':'ardupilot_message_send_time_not_certified_measurement_time',
        'wire_version':2,'payload_length':13,'extension_bytes_observed':False,
        'min_distance_cm':20,'max_distance_cm':2000,'current_distance_cm':int(height*100),
        'min_distance_m':quantity(.2,'m','reported_limit'),'max_distance_m':quantity(20,'m','reported_limit'),
        'current_distance_m':quantity(height,'m','reported_distance'),
        'bounds_consistent':True,'current_within_reported_limits':True,
        'type_raw':0,'type_known':True,'type_name':'LASER',
        'orientation_raw':25,'orientation_known':True,'orientation_name':'PITCH_270','orientation_frame':'BODY_FRD',
        'distance_interpretation':'sensor_report_no_vertical_or_terrain_projection','physical_accuracy':'unassessed',
        'covariance':{'raw_cm2':0,'quantity':quantity(None,'m^2','ardupilot_zero_unknown_or_invalid',False)},
        'signal_quality':{'raw':0,'extension_available':True,'quantity':quantity(None,'percent','unknown_unset_or_omitted',False)},
        'horizontal_fov':copy.deepcopy(unknown_fov),'vertical_fov':copy.deepcopy(unknown_fov),
        'quaternion':{'raw_wxyz':[0,0,0,0],'extension_available':True,'finite':True,'norm':0,'unit_norm':False,
            'required_for_custom_orientation':False,'usable_for_custom_orientation':False,'reason':'zero_omitted_or_nonunit'}}


def runtime(counter):
    return 'mode=SITL_CONTROL,permission=true,fc_mode=4,armed=1,in_air=1,heartbeat_valid=true,position_valid=true,home_valid=true,landed_valid=true,estimator_valid=true,nav_tx_accepted='+str(counter)


def fixture():
    events=[];packets=[];records=[];markers={};windows={}
    def event(kind,key,value,elapsed):
        events.append({'type':kind,'key':key,'value':value,'elapsed':elapsed,'source':'iMAVLink'})
    def packet(kind,elapsed,**values):
        packets.append({'mavpackettype':kind,'system':1,'component':1,'mono':5000+elapsed,**values})
    for index,(name,height) in enumerate((('height.low',5.),('height.high',8.),('height.return',5.))):
        begin=10+index*10;end=begin+4;base=index*100;windows[name]=(begin,end)
        event('phase',name+'.started','nav_tx_accepted='+str(base),begin)
        event('phase',name+'.finished','nav_tx_accepted='+str(base+40),end)
        markers[name+'.started']=5000+begin;markers[name+'.finished']=5000+end
        for step in range(41):event('runtime','MAVMOOS_RUNTIME_STATUS',runtime(base+step),begin+step*.1)
        for step in range(40):
            elapsed=begin+.05+step*.1;boot=round((elapsed+40)*1000);effective=100000+elapsed
            row={'schema':1,'session_epoch':1,'family':'DISTANCE_SENSOR','present':True,'encoding_valid':True,'fresh':True,
                'effective_at':effective,'read_at':effective+.002,'age_s':.003,'age_basis':'QUEUE_EMPTY_FLOOR','age_is_upper_bound':True,
                'source_age':'unknown','source_quality':'unassessed','observation_id':index*100+step+1,'data':data(height,boot)}
            aggregate={'schema':1,'session_epoch':1,'present':True,'atomic':False,'cache_capacity':8,'sensors':[row]}
            status={'schema':1,'session_epoch':1,'session_boundary':100000,'state':'BOUND','read_only':True,'authoritative_for_control':False,
                'atomic':False,'system_id':1,'component_id':1,'reset_reason':'startup','metadata_rejected':0,'source_rejected':0,
                'wire_shape_rejected':0,'out_of_order_rejected':0,'sensor_capacity_rejected':0,'sensors':aggregate}
            event('distance','MAVMOOS_DISTANCE_SENSORS',json.dumps(aggregate),elapsed)
            event('distance_status','MAVMOOS_DISTANCE_STATUS',json.dumps(status),elapsed)
            event('nav','NAV_ALTITUDE',str(height),elapsed)
            packet('DISTANCE_SENSOR',elapsed+.002,time_boot_ms=boot,min_distance=20,max_distance=2000,current_distance=int(height*100),
                type=0,id=0,orientation=25,covariance=0,horizontal_fov=0,vertical_fov=0,quaternion=[0,0,0,0],signal_quality=0)
            packet('GLOBAL_POSITION_INT',elapsed+.002,time_boot_ms=boot,relative_alt=int(height*1000),alt=int((height+10)*1000),
                lat=423584560,lon=-710875890,vx=0,vy=0,vz=0,hdg=9000)
            packet('POSITION_TARGET_LOCAL_NED',elapsed+.002,time_boot_ms=boot,coordinate_frame=1,type_mask=0,x=0,y=0,z=-height,vx=0,vy=0,vz=0,yaw=0)
            records.append({'mavpackettype':'RFND','Instance':0,'Orient':25,'TimeUS':boot*1000,'Dist':height})
        for step in range(4):packet('HEARTBEAT',begin+.15+step,type=2,autopilot=3,base_mode=128,custom_mode=4,system_status=4)
        for step in range(16):packet('EXTENDED_SYS_STATE',begin+.2+step*.25,landed_state=2,vtol_state=0)
    for name,value in [('ground.started',0),('ground.finished',4)]:
        event('phase',name,'nav_tx_accepted=0',value);markers[name]=5000+value
    event('request','ALTITUDE_UPDATE','altitude=8',17)
    event('request','ALTITUDE_UPDATE','altitude=5',27)
    event('pass','distance_flight','completed',37)
    packet('HEARTBEAT',37,type=2,autopilot=3,base_mode=0,custom_mode=4,system_status=3)
    packet('GLOBAL_POSITION_INT',37,time_boot_ms=77000,relative_alt=0,alt=10000,lat=423584560,lon=-710875890,vx=0,vy=0,vz=0,hdg=9000)
    packet('EXTENDED_SYS_STATE',37,landed_state=1,vtol_state=0)
    events.sort(key=lambda row:row['elapsed']);packets.sort(key=lambda row:row['mono'])
    return {'events':events,'packets':packets,'records':records,'markers':markers,'finished':5037.1,'windows':windows}


BASE=fixture()
executed=[]


def alter_sample(case,mutator):
    event=next(e for e in case['events'] if e['key']=='MAVMOOS_DISTANCE_SENSORS')
    value=json.loads(event['value']);mutator(value['sensors'][0]);event['value']=json.dumps(value)


def alter_data(case,mutator):alter_sample(case,lambda row:mutator(row['data']))


class DistanceEvidenceTests(unittest.TestCase):
    def setUp(self):self.case=copy.deepcopy(BASE)

    def probe(self):return checker.audit_probe(self.case['events'])

    def fc(self):return checker.audit_fc(self.case['packets'],self.case['markers'],self.case['finished'])

    def reject_probe(self,mutator,reason):
        mutator(self.case)
        with self.assertRaisesRegex(ValueError,reason):self.probe()
        executed.append(self.id().split('.')[-1])

    def reject_fc(self,mutator,reason):
        mutator(self.case)
        with self.assertRaisesRegex(ValueError,reason):self.fc()
        executed.append(self.id().split('.')[-1])

    def test_complete_positive_fixture(self):
        proof=self.probe();self.assertEqual(proof['result'],'PASS');self.assertEqual(self.fc()['result'],'PASS')
        paired=checker.audit_distance_consistency(self.case['events'],self.case['packets'],proof)
        logged=checker.audit_dataflash(self.case['events'],self.case['records'],proof)
        self.assertEqual(paired['result'],logged['result']);self.assertEqual(logged['result'],'PASS')
        for value in paired['windows'].values():self.assertEqual(value['matched'],value['selected'])

    def test_unknown_covariance_cannot_carry_a_numeric_value(self):
        self.reject_probe(lambda c:alter_data(c,lambda d:d['covariance']['quantity'].update(value=0)),'contradictory unknown')

    def test_unknown_quality_cannot_carry_a_numeric_value(self):
        self.reject_probe(lambda c:alter_data(c,lambda d:d['signal_quality']['quantity'].update(value=50)),'contradictory unknown')

    def test_covariance_units_cannot_change(self):
        self.reject_probe(lambda c:alter_data(c,lambda d:d['covariance']['quantity'].update(unit='m')),'contradictory unknown')

    def test_unknown_reason_cannot_claim_a_calibration(self):
        self.reject_probe(lambda c:alter_data(c,lambda d:d['signal_quality']['quantity'].update(reason='calibrated')),'contradictory unknown')

    def test_source_frame_cannot_be_silently_converted(self):
        self.reject_probe(lambda c:alter_data(c,lambda d:d.update(orientation_frame='BODY_FLU')),'frame/terrain/accuracy')

    def test_sensor_distance_cannot_become_terrain_clearance(self):
        self.reject_probe(lambda c:alter_data(c,lambda d:d.update(distance_interpretation='verified_vertical_clearance')),'frame/terrain/accuracy')

    def test_physical_accuracy_cannot_be_asserted(self):
        self.reject_probe(lambda c:alter_data(c,lambda d:d.update(physical_accuracy='verified')),'frame/terrain/accuracy')

    def test_omitted_fov_cannot_become_known(self):
        self.reject_probe(lambda c:alter_data(c,lambda d:d['horizontal_fov']['quantity'].update(known=True,value=.5)),'FOV extension')

    def test_zero_orientation_extension_cannot_be_applied(self):
        self.reject_probe(lambda c:alter_data(c,lambda d:d['quaternion'].update(raw_wxyz=[1,0,0,0],usable_for_custom_orientation=True)),'custom-orientation extension')

    def test_missing_airborne_heartbeats_is_not_accepted(self):
        self.reject_fc(lambda c:c.update(packets=[p for p in c['packets'] if p['mavpackettype']!='HEARTBEAT' or p['mono']>5035]),'armed Guided')

    def test_airborne_unarmed_heartbeat_is_not_accepted(self):
        self.reject_fc(lambda c:[p.update(base_mode=0) for p in c['packets'] if p['mavpackettype']=='HEARTBEAT' and p['mono']<5035],'armed Guided')

    def test_airborne_wrong_mode_is_not_accepted(self):
        self.reject_fc(lambda c:[p.update(custom_mode=5) for p in c['packets'] if p['mavpackettype']=='HEARTBEAT' and p['mono']<5035],'armed Guided')

    def test_airborne_ground_state_is_not_accepted(self):
        self.reject_fc(lambda c:[p.update(landed_state=1) for p in c['packets'] if p['mavpackettype']=='EXTENDED_SYS_STATE' and p['mono']<5035],'in-air')

    def test_missing_airborne_landed_state_is_not_accepted(self):
        self.reject_fc(lambda c:c.update(packets=[p for p in c['packets'] if p['mavpackettype']!='EXTENDED_SYS_STATE' or p['mono']>5035]),'in-air')

    def test_early_only_runtime_rows_do_not_prove_the_tail(self):
        begin,end=self.case['windows']['height.low']
        self.reject_probe(lambda c:c.update(events=[e for e in c['events'] if not(e['type']=='runtime' and begin+1.5<e['elapsed']<=end)]),'late Runtime')

    def test_full_runtime_rows_cannot_hide_a_navigation_terminal_gap(self):
        begin,end=self.case['windows']['height.low']
        def freeze(c):
            for e in c['events']:
                if e['type']=='runtime' and begin+2.8<e['elapsed']<=end:e['value']=runtime(28)
        self.reject_probe(freeze,'final-tail gap')

    def test_runtime_middle_gap_is_rejected(self):
        begin,end=self.case['windows']['height.low']
        self.reject_probe(lambda c:c.update(events=[e for e in c['events'] if not(e['type']=='runtime' and begin+1<e['elapsed']<begin+2)]),'Runtime evidence gap')

    def test_phase_counter_contradiction_is_rejected(self):
        self.reject_probe(lambda c:next(e for e in c['events'] if e['type']=='phase' and e['key']=='height.low.finished').update(value='nav_tx_accepted=0'),'phase NAV counters')

    def test_navigation_counter_regression_is_rejected(self):
        self.reject_probe(lambda c:next(e for e in c['events'] if e['type']=='runtime' and e['elapsed']>=10.8).update(value=runtime(-1)),'NAV progression')

    def test_missing_late_range_is_rejected(self):
        begin,end=self.case['windows']['height.low']
        self.reject_probe(lambda c:c.update(events=[e for e in c['events'] if not(e['key']=='MAVMOOS_DISTANCE_SENSORS' and end-2.5<=e['elapsed']<=end)]),'late range')

    def test_raw_distance_conversion_is_checked(self):
        self.reject_probe(lambda c:alter_data(c,lambda d:d['current_distance_m'].update(value=50)),'conversion/unknown')

    def test_cached_observation_cannot_renew_its_original_stamp(self):
        def renew(c):
            values=[e for e in c['events'] if e['key']=='MAVMOOS_DISTANCE_SENSORS'];old=json.loads(values[0]['value'])
            current=json.loads(values[1]['value']);current['sensors'][0]['observation_id']=old['sensors'][0]['observation_id'];values[1]['value']=json.dumps(current)
        self.reject_probe(renew,'cached sample renewed')

    def test_moos_source_identity_is_checked(self):
        self.reject_probe(lambda c:next(e for e in c['events'] if e['key']=='MAVMOOS_DISTANCE_SENSORS').update(source='cached_forwarder'),'MOOS source')

    def test_fixture_sensor_id_is_checked(self):
        self.reject_probe(lambda c:alter_data(c,lambda d:d.update(id=1)),'fixture ID')

    def test_fc_component_identity_is_checked(self):
        self.reject_fc(lambda c:c['packets'][0].update(component=200),'FC source')

    def test_constant_remote_stamp_is_not_advancing_evidence(self):
        begin,end=self.case['windows']['height.low']
        def constant(c):
            for e in c['events']:
                if e['key']=='MAVMOOS_DISTANCE_SENSORS' and begin<=e['elapsed']<=end:
                    value=json.loads(e['value']);value['sensors'][0]['data']['time_boot_ms']=1234;e['value']=json.dumps(value)
        self.reject_probe(constant,'constant remote time')

    def test_missing_phase_marker_is_rejected(self):
        self.reject_fc(lambda c:c['markers'].pop('height.high.finished'),'phase markers')

    def test_final_armed_state_is_rejected(self):
        self.reject_fc(lambda c:[p.update(base_mode=128) for p in c['packets'] if p['mavpackettype']=='HEARTBEAT' and p['mono']>5035],'remained armed')

    def test_final_airborne_state_is_rejected(self):
        self.reject_fc(lambda c:[p.update(landed_state=2) for p in c['packets'] if p['mavpackettype']=='EXTENDED_SYS_STATE' and p['mono']>5035],'not on ground')

    def test_stale_final_position_is_rejected(self):
        self.reject_fc(lambda c:c.update(packets=[p for p in c['packets'] if p['mavpackettype']!='GLOBAL_POSITION_INT' or p['mono']<5035]),'stale GLOBAL_POSITION_INT')

    def test_passive_fc_distance_disagreement_is_rejected(self):
        proof=self.probe()
        for row in self.case['packets']:
            if row['mavpackettype']=='DISTANCE_SENSOR':row['current_distance']+=30
        with self.assertRaisesRegex(ValueError,'paired source values'):checker.audit_distance_consistency(self.case['events'],self.case['packets'],proof)
        executed.append(self.id().split('.')[-1])

    def test_closed_dataflash_distance_disagreement_is_rejected(self):
        proof=self.probe()
        for row in self.case['records']:row['Dist']+=1
        with self.assertRaisesRegex(ValueError,'RFND source comparisons'):checker.audit_dataflash(self.case['events'],self.case['records'],proof)
        executed.append(self.id().split('.')[-1])


if __name__=='__main__':
    suite=unittest.defaultTestLoader.loadTestsFromTestCase(DistanceEvidenceTests)
    result=unittest.TextTestRunner(verbosity=2 if '-v' in remaining else 1).run(suite)
    report={'schema':1,'result':'PASS' if result.wasSuccessful() else 'FAIL','checker':str(checker_path),'checker_sha256':checker_sha,
        'test_source_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'tests_run':result.testsRun,
        'failures':len(result.failures),'errors':len(result.errors),'mutation_controls_rejected':sorted(executed),
        'scope':'Self-contained synthetic checker tests; no native execution, sockets, source-age mapping or vehicle-control evidence'}
    if args.report:
        if args.report.exists():raise RuntimeError('refuse to overwrite evidence report')
        args.report.parent.mkdir(parents=True,exist_ok=True);args.report.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2));raise SystemExit(0 if result.wasSuccessful() else 1)
