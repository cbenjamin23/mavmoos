#!/usr/bin/env python3
"""Pure declared mission-read branches; no SDK, process, socket or vehicle."""
import importlib.util
import json
from pathlib import Path
import unittest
from mission_read_evidence import mission_read_evidence

ROOT=Path(__file__).resolve().parents[2]
spec=importlib.util.spec_from_file_location('_mission_fixture_test_selector',ROOT/'scripts/firmware/scenario_fixtures.py')
F=importlib.util.module_from_spec(spec);spec.loader.exec_module(F)


def selected(profile):
    pin=F.PROFILES[profile]
    identity={'schema':1,'profile_id':profile,'manifest_sha256':pin['firmware_manifest_sha256'],
              'source_revision':pin['source_revision'],'frame':'quad','native_sha256':'1'*64,
              'attestation_sha256':'2'*64,'qualification':'not_inferred'}
    return F.select(ROOT,identity,'armed_wiped_mission_read')


def fixture(selection):
    expected=selection['expectation'];count=expected['expected_count'];events=[]
    def add(kind,key,value):
        events.append({'type':kind,'key':key,'value':json.dumps(value) if isinstance(value,dict) else value})
    add('fixture','mission_shape',expected['shape'])
    add('request','MAVMOOS_MISSION_REQUEST','id=concurrent-mission-read,action=download,timeout=15')
    add('request','MAVMOOS_PARAM_REQUEST','id=concurrent-parameter-read,action=read,name=SIM_WIND_SPD')
    home={'seq':0,'command':16,'frame':0,'current':1,'autocontinue':1,'params':[0,0,0,0],
          'x':423584560,'y':-710875890,'z':10}
    add('mission_snapshot','MAVMOOS_MISSION_SNAPSHOT',{'schema':1,'id':'concurrent-mission-read',
        'status':'complete','count':count,'items':[home] if count else [],'mission_type':0,
        'verification':'observation','atomic':False})
    add('home_observation','MAVMOOS_HOME_POSITION','present=true,encoding_valid=true,identity=EXPECTED,freshness=REFERENCE_KNOWN,lat_deg=42.358456,lon_deg=-71.087589,alt_msl_m=10')
    for status,at,n in [('ADMITTED',1000,0),('COMPLETED',1000.2,count)]:
        add('mission_result','MAVMOOS_MISSION_RESULT',f'id=concurrent-mission-read,action=download,status={status},at={at},expires=1015,received={n},expected={n},mutation_may_have_applied=false')
    for message in expected['required_host_message_ids']:
        add('mission_tx','MAVMOOS_MISSION_TX_RESULT',f'operation_id=concurrent-mission-read,msgid={message},status=LOCAL_ACCEPTED,late=false,clock_valid=true')
    add('parameter_result','MAVMOOS_PARAM_RESULT','id=concurrent-parameter-read,status=COMPLETED')
    add('assertion','read_services_complete','mission_status=COMPLETED,parameter_status=COMPLETED,mission_mutations=0,service_window_s=0.2')
    add('assertion','mission_read_continuity','post_read_observation_s=2,no_mutation=true')
    add('assertion','transfer_outcome','status=COMPLETED,loss_seen=false,nav_before=10,nav_after=20,max_observed_nav_counter_update_gap_s=0.1')
    return events


class MissionReadFixtureTests(unittest.TestCase):
    def test_both_exact_profiles_pass_only_their_declared_shape(self):
        for profile,count in [('arducopter-dev-04cf6486-quad',1),('arducopter-4.6.3-quad',0)]:
            expected=selected(profile);proof=mission_read_evidence(fixture(expected),expected)
            self.assertEqual(proof['snapshot_count'],count);self.assertEqual(proof['item_records_exercised'],count)
            self.assertEqual(proof['original_expires'],1015)

    def test_cross_profile_shape_is_not_inferred_from_reply(self):
        a,b=[selected(p) for p in F.PROFILES]
        for events,expected in [(fixture(a),b),(fixture(b),a)]:
            with self.assertRaises(RuntimeError):mission_read_evidence(events,expected)

    def test_empty_rejects_item_request_and_missing_final_ack(self):
        expected=selected('arducopter-4.6.3-quad')
        for mode in ('item','no_ack'):
            events=fixture(expected)
            if mode=='item':events.append({'type':'mission_tx','key':'MAVMOOS_MISSION_TX_RESULT','value':'operation_id=concurrent-mission-read,msgid=51,status=LOCAL_ACCEPTED,late=false,clock_valid=true'})
            else:events=[e for e in events if not (e['type']=='mission_tx' and 'msgid=47,' in e['value'])]
            with self.assertRaises(RuntimeError):mission_read_evidence(events,expected)

    def test_home_requires_bound_reference_and_wire_home_fields(self):
        expected=selected('arducopter-dev-04cf6486-quad')
        for mode in ('missing_home','wrong_item'):
            events=fixture(expected)
            if mode=='missing_home':events=[e for e in events if e['type']!='home_observation']
            else:
                row=next(e for e in events if e['type']=='mission_snapshot');value=json.loads(row['value']);value['items'][0]['x']=0;row['value']=json.dumps(value)
            with self.assertRaises(RuntimeError):mission_read_evidence(events,expected)

    def test_empty_still_requires_bound_home_reference(self):
        expected=selected('arducopter-4.6.3-quad')
        with self.assertRaises(RuntimeError):mission_read_evidence([e for e in fixture(expected) if e['type']!='home_observation'],expected)

    def test_terminal_count_original_expiry_and_late_completion_are_strict(self):
        expected=selected('arducopter-4.6.3-quad')
        for before,after in [('received=0','received=1'),('expected=0','expected=1'),('expires=1015','expires=1016'),('at=1000.2','at=1015')]:
            events=fixture(expected)
            row=next(e for e in events if e['type']=='mission_result' and 'status=COMPLETED' in e['value']);row['value']=row['value'].replace(before,after)
            with self.assertRaises(RuntimeError):mission_read_evidence(events,expected)

    def test_empty_does_not_permit_mutation_or_navigation_gap(self):
        expected=selected('arducopter-4.6.3-quad')
        for key,value in [('MAVMOOS_MISSION_REQUEST','id=concurrent-mission-read,action=clear,timeout=15'),
                          ('MAVMOOS_PARAM_REQUEST','id=concurrent-parameter-read,action=write,name=SIM_WIND_SPD')]:
            events=fixture(expected);next(e for e in events if e['key']==key)['value']=value
            with self.assertRaises(RuntimeError):mission_read_evidence(events,expected)
        events=fixture(expected);next(e for e in events if e['key']=='transfer_outcome')['value']=next(e for e in events if e['key']=='transfer_outcome')['value'].replace('gap_s=0.1','gap_s=0.8')
        with self.assertRaises(RuntimeError):mission_read_evidence(events,expected)


if __name__=='__main__':unittest.main()
