#!/usr/bin/env python3
"""Adversarial evidence-checker tests, without MOOS/network/simulation."""
import json
import unittest
from StatusObservationAudit import audit,parse

def evidence():
    def row(key,value,elapsed=21):return {'source':'iMAVLink','key':key,'value':json.dumps(value),'elapsed':elapsed}
    result=[row('MAVMOOS_CAPABILITIES',{'execution_mode':'RX_ONLY','composition':{'plugin_abi':10,'modules':[
        {'name':'telemetry','control_provider':False,'request_owners':[]},
        {'name':'system_status','control_provider':False,'request_owners':[]}]}}),
        {'source':'iMAVLink','key':'MAVMOOS_RUNTIME_STATUS','elapsed':21,'value':'mode=RX_ONLY,permission=false,armed=0,in_air=0,control_profile=none,heartbeat_valid=true,tx_attempts=0,tx_accepted=0,nav_tx_accepted=0'},
        row('MAVMOOS_SYSTEM_STATUS',{'schema':1,'state':'BOUND','read_only':True,'authoritative_for_control':False,'battery_ids':1,'text_assemblies':0})]
    result.append({**result[1],'elapsed':22})
    for key in ('MAVMOOS_SYS_STATUS','MAVMOOS_BATTERY_STATUS','MAVMOOS_GPS_STATUS'):
        for n in range(2):result.append(row(key,{'schema':1,'present':True,'fresh':True,'effective_at':100+n,'read_at':100.1+n,'age_s':.2,
            'age_basis':'QUEUE_EMPTY_FLOOR','age_is_upper_bound':True,'observation_id':n+1,'source_age':'unknown','source_quality':'unassessed',
            'data':{'origin':'BATTERY_STATUS','battery_id':0,'voltage_layout_valid':True,
                    'reported_voltage_sum_v':{'known':True,'value':12.6,'unit':'V'}} if key=='MAVMOOS_BATTERY_STATUS' else {}},elapsed=21+n))
    return result

def change(rows,key,mutate):
    for row in rows:
        if row['key']==key:
            value=json.loads(row['value']);mutate(value);row['value']=json.dumps(value)

class AuditTests(unittest.TestCase):
    def test_complete_observations_do_not_require_event_text(self):
        result=audit(evidence());self.assertEqual(result['result'],'pass');self.assertFalse(result['text_observed'])
        rows=evidence()
        for row in rows:
            if row['key']=='MAVMOOS_RUNTIME_STATUS':row['value']=row['value'].replace('heartbeat_valid=true','heartbeat_valid=1')
        self.assertEqual(audit(rows)['result'],'pass')

    def test_missing_families_runtime_catalog_are_not_qualification(self):
        for key in ('MAVMOOS_SYS_STATUS','MAVMOOS_BATTERY_STATUS','MAVMOOS_GPS_STATUS','MAVMOOS_SYSTEM_STATUS','MAVMOOS_RUNTIME_STATUS','MAVMOOS_CAPABILITIES'):
            with self.subTest(key=key),self.assertRaises(ValueError):audit([r for r in evidence() if r['key']!=key])

    def test_stale_and_republished_cache_cannot_substitute_new_observations(self):
        for mutate in (lambda v:v.update(fresh=False),lambda v:v.update(fresh='false'),lambda v:v.update(observation_id=1),lambda v:v.update(read_at=102)):
            rows=evidence();change(rows,'MAVMOOS_GPS_STATUS',mutate)
            with self.assertRaises(ValueError):audit(rows)

    def test_zero_send_claim_requires_all_counters_and_no_control(self):
        for before,after in [('tx_attempts=0','tx_attempts=1'),('nav_tx_accepted=0',''),('permission=false','permission=true'),('armed=0','armed=1'),('control_profile=none','control_profile=arducopter'),('mode=RX_ONLY','mode=SITL_CONTROL')]:
            rows=evidence();row=next(r for r in rows if r['key']=='MAVMOOS_RUNTIME_STATUS');row['value']=row['value'].replace(before,after)
            with self.subTest(after=after),self.assertRaises(ValueError):audit(rows)

    def test_unknown_clock_and_conservative_metadata_required(self):
        for mutate in (lambda v:v.update(age_s=-1),lambda v:v.update(read_at=0),lambda v:v.update(source_age='0'),lambda v:v.update(age_is_upper_bound=False)):
            rows=evidence();change(rows,'MAVMOOS_GPS_STATUS',mutate)
            with self.assertRaises(ValueError):audit(rows)

    def test_aggregate_cannot_grant_authority_or_assert_safety(self):
        for mutate in (lambda v:v.update(authoritative_for_control=True),lambda v:v.update(healthy=True),lambda v:v.update(battery_ids=9),lambda v:v.update(text_assemblies=5)):
            rows=evidence();change(rows,'MAVMOOS_SYSTEM_STATUS',mutate)
            with self.assertRaises(ValueError):audit(rows)

    def test_wrong_source_and_status_abi_fail(self):
        rows=evidence();rows[0]['source']='untrusted'
        with self.assertRaises(ValueError):audit(rows)
        rows=evidence();change(rows,'MAVMOOS_CAPABILITIES',lambda v:v['composition'].update(plugin_abi=6))
        with self.assertRaises(ValueError):audit(rows)

    def test_ambiguous_fallback_cannot_replace_identified_battery(self):
        rows=evidence();change(rows,'MAVMOOS_BATTERY_STATUS',lambda v:v.update(data={'origin':'SYS_STATUS','battery_id':None}))
        with self.assertRaises(ValueError):audit(rows)

    def test_fresh_expired_unknown_basis_and_read_contradictions_fail(self):
        cases=[{'age_s':999},{'age_basis':'UNKNOWN','age_is_upper_bound':False},
               {'age_basis':'READ_ACQUISITION'},
               {'age_basis':'READ_ACQUISITION','read_at':100,'effective_at':100,'age_is_upper_bound':True},
               {'age_s':0,'effective_at':100,'read_at':101}]
        for fields in cases:
            rows=evidence();change(rows,'MAVMOOS_GPS_STATUS',lambda v:v.update(fields))
            with self.subTest(fields=fields),self.assertRaises(ValueError):audit(rows)

    def test_nonfinite_overflow_and_boolean_metadata_fail(self):
        for value in ('1e1000','-1e1000','NaN','Infinity'):
            with self.subTest(value=value),self.assertRaises(ValueError):parse('{"age_s":'+value+'}')
        for field in ('effective_at','read_at','age_s','observation_id'):
            rows=evidence();change(rows,'MAVMOOS_GPS_STATUS',lambda v:v.update({field:True}))
            with self.subTest(field=field),self.assertRaises(ValueError):audit(rows)
        rows=evidence();rows[0]['elapsed']=True
        with self.assertRaises(ValueError):audit(rows)

    def test_read_acquisition_is_valid_only_with_matching_stamps_and_bound_flag(self):
        rows=evidence();change(rows,'MAVMOOS_GPS_STATUS',lambda v:v.update(age_basis='READ_ACQUISITION',effective_at=v['read_at'],age_is_upper_bound=False))
        self.assertEqual(audit(rows)['result'],'pass')

    def test_fixture_battery_zero_must_report_real_expected_voltage_in_volts(self):
        cases=[{'battery_id':4},{'battery_id':True},{'voltage_layout_valid':False},
               {'reported_voltage_sum_v':{'known':True,'value':0,'unit':'V'}},
               {'reported_voltage_sum_v':{'known':False,'value':None,'unit':'V'}},
               {'reported_voltage_sum_v':{'known':True,'value':12600,'unit':'mV'}},
               {'reported_voltage_sum_v':{'known':True,'value':True,'unit':'V'}}]
        for fields in cases:
            rows=evidence();change(rows,'MAVMOOS_BATTERY_STATUS',lambda v:v['data'].update(fields))
            with self.subTest(fields=fields),self.assertRaises(ValueError):audit(rows)

    def test_early_only_or_missing_late_family_and_runtime_evidence_fail(self):
        for key in (None,'MAVMOOS_GPS_STATUS','MAVMOOS_BATTERY_STATUS','MAVMOOS_SYS_STATUS','MAVMOOS_RUNTIME_STATUS','MAVMOOS_SYSTEM_STATUS'):
            rows=evidence()
            for row in rows:
                if key is None or row['key']==key:row['elapsed']=1
            with self.subTest(key=key),self.assertRaises(ValueError):audit(rows)

    def test_timing_configuration_and_cache_counts_reject_bad_types(self):
        for timeout in (float('inf'),True,0,31):
            with self.assertRaises(ValueError):audit(evidence(),status_timeout=timeout)
        rows=evidence();change(rows,'MAVMOOS_SYSTEM_STATUS',lambda v:v.update(battery_ids=True))
        with self.assertRaises(ValueError):audit(rows)

if __name__=='__main__':unittest.main()
