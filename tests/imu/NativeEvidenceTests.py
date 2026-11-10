#!/usr/bin/env python3
"""Pure offline negative controls. No process, socket or simulator fixture."""
import copy
import importlib.util
import json
from pathlib import Path
import tempfile
import types
import unittest
from unittest import mock
from audit_native import audit_probe,audit_fc,corroborate,parse


def record(epoch,oid,effective,age):
    return {'schema':1,'session_epoch':epoch,'present':True,'encoding_valid':True,'fresh':age<=1,
            'effective_at':effective,'read_at':effective+.05,'age_s':age,'age_basis':'QUEUE_EMPTY_FLOOR','age_is_upper_bound':True,
            'source_age':'unknown','source_quality':'unassessed','observation_id':oid,
            'data':{'message_id':27,'family':'RAW_IMU','wire_version':2,'payload_length':29,'sensor_instance':0,
                    'identity_basis':'zero_default_or_omitted_extension_id','physical_sensor_identity':'unverified',
                    'remote_timestamp':1234567,'remote_clock':'ardupilot_boot_us_unmapped_uint64','remote_timestamp_regressed':False,
                    'timestamp_semantics':'ardupilot_message_send_time_not_certified_measurement_time','body_frame':'BODY_FRD',
                    'estimate':'sensor_report_not_fused_attitude','unit_basis':'ardupilot_raw_imu_scaled_sender_convention',
                    'raw_units':{'acceleration':'milli-g','angular_velocity':'mrad/s','magnetic_field':'milligauss'},
                    'sensor_presence':'unknown','sensor_health':'unassessed','physical_accuracy':'unassessed','covariance':None,'gravity_removed':False,
                    'raw_acceleration':[1000,-2000,0],'raw_angular_velocity':[1000,-2000,125],'raw_magnetic_field':[1000,-250,0],
                    'acceleration':{'unit':'m/s^2','conversion_known':True,'value':[9.80665,-19.6133,0]},
                    'angular_velocity':{'unit':'rad/s','conversion_known':True,'value':[1,-2,.125]},
                    'magnetic_field':{'unit':'T','conversion_known':True,'value':[.0001,-.000025,0]},
                    'temperature':{'raw_cdegC':0,'known':False,'value':None,'unit':'degC','reason':'zero_unknown_or_omitted'}}}


def cache(epoch,values):return {'schema':1,'session_epoch':epoch,'present':bool(values),'atomic':False,'cache_capacity':8,'sensors':values}
def status(epoch,values,floor=100,state='BOUND'):
    return {'schema':1,'session_epoch':epoch,'session_boundary':floor,'state':state,'read_only':True,'authoritative_for_control':False,'atomic':False,
            'family_selection':'independent_no_precedence_or_fusion','system_id':1 if state=='BOUND' else None,'component_id':1 if state=='BOUND' else None,
            'metadata_rejected':0,'source_rejected':0,'wire_shape_rejected':0,'out_of_order_rejected':0,'sensor_capacity_rejected':0,'sensors':cache(epoch,values)}
def row(at,key,value):return {'elapsed':at,'key':key,'source':'iMAVLink','value':json.dumps(value) if isinstance(value,dict) else value}


def fixture():
    catalog={'schema':1,'execution_mode':'RX_ONLY','composition':{'plugin_abi':10,'modules':[{'name':'telemetry','control_provider':False,'request_owners':[],'capabilities':[]},
        {'name':'imu','control_provider':False,'request_owners':[],'capabilities':[{'name':name} for name in ('imu.acceleration','imu.angular_velocity','imu.magnetic_field','imu.temperature')]}]}}
    runtime='schema=1,mode=RX_ONLY,permission=false,control_profile=none,tx_attempts=0,tx_accepted=0,nav_tx_accepted=0,armed=0,in_air=0'
    app='schema=1,state=RX_ONLY,tx_enabled=false,helm=disabled,mavlink_tx_attempts=0,mavlink_tx_accepted=0'
    rows=[row(.1,'MAVMOOS_CAPABILITIES',catalog),row(.2,'MAVMOOS_RUNTIME_STATUS',runtime),row(.3,'MAVMOOS_STATUS',app)]
    values=[(1,record(1,1,100.2,.1)),(2,record(1,2,100.4,.1)),(3,record(1,2,100.4,1.3)),(20,record(2,3,104.2,.1)),(21,record(2,4,104.4,.1))]
    for at,sample in values:
        rows.append(row(at,'MAVMOOS_IMU_SENSORS',cache(sample['session_epoch'],[sample])))
        rows.append(row(at+.01,'MAVMOOS_IMU_STATUS',status(sample['session_epoch'],[sample],100 if sample['session_epoch']==1 else 103)))
    rows.append(row(4,'MAVMOOS_IMU_STATUS',status(2,[],103,'WAITING_HEARTBEAT')))
    return sorted(rows,key=lambda r:r['elapsed'])


def change_sample(rows,fn):
    for event in rows:
        if event['key'] in ('MAVMOOS_IMU_SENSORS','MAVMOOS_IMU_STATUS'):
            value=json.loads(event['value']);samples=value['sensors']['sensors'] if event['key']=='MAVMOOS_IMU_STATUS' else value['sensors']
            for sample in samples:fn(sample)
            event['value']=json.dumps(value)


def fc_fixture():
    rows=[]
    for at in (1,10,26):rows.append({'mono':at,'system':1,'component':1,'mavpackettype':'HEARTBEAT','type':2,'autopilot':3,'base_mode':0})
    rows.append({'mono':26,'system':1,'component':1,'mavpackettype':'EXTENDED_SYS_STATE','landed_state':1})
    for at in (1,25,26):rows.append({'mono':at,'system':1,'component':1,'mavpackettype':'RAW_IMU'})
    return sorted(rows,key=lambda r:r['mono'])


class NativeEvidenceTests(unittest.TestCase):
    def reject(self,mutate):
        rows=fixture();mutate(rows)
        with self.assertRaises((ValueError,KeyError,TypeError)):audit_probe(rows)

    def test_valid_bounded_loss_and_recovery_is_advisory_about_physical_sensors(self):
        report=audit_probe(fixture());self.assertEqual(report['result'],'PASS')
        self.assertEqual(report['physical_sensor_count'],'unknown');self.assertEqual(report['epochs'],[1,2])
        self.assertEqual(report['families_observed'],[27]);self.assertTrue(report['stale_then_cleared_then_new_epoch'])

    def test_current_vibration_catalog_is_explicit_and_does_not_extend_wire_evidence(self):
        rows = fixture()
        with self.assertRaisesRegex(ValueError, 'loaded IMU capability family'):
            audit_probe(rows, expect_vibration=True)
        event = next(row for row in rows if row['key'] == 'MAVMOOS_CAPABILITIES')
        catalog = json.loads(event['value'])
        imu = next(module for module in catalog['composition']['modules'] if module['name'] == 'imu')
        imu['capabilities'].append({'name': 'imu.vibration'})
        event['value'] = json.dumps(catalog)
        self.assertEqual(audit_probe(rows, expect_vibration=True)['families_observed'], [27])
        with self.assertRaisesRegex(ValueError, 'loaded IMU capability family'):
            audit_probe(rows)

    def test_altered_si_scaling_is_rejected(self):
        for field in ('acceleration','angular_velocity','magnetic_field'):
            with self.subTest(field=field):self.reject(lambda rows:change_sample(rows,lambda s:s['data'][field]['value'].__setitem__(0,1000)))

    def test_unknowns_cannot_be_relabelled_health_presence_accuracy(self):
        for field in ('physical_sensor_identity','sensor_presence','sensor_health','physical_accuracy'):
            with self.subTest(field=field):self.reject(lambda rows:change_sample(rows,lambda s:s['data'].__setitem__(field,'known_good')))

    def test_zero_temperature_and_mavlink1_extensions_stay_unknown(self):
        self.reject(lambda rows:change_sample(rows,lambda s:s['data']['temperature'].update(known=True,value=0)))
        self.reject(lambda rows:change_sample(rows,lambda s:s['data'].update(wire_version=1)))

    def test_cached_observation_stamps_cannot_be_renewed(self):
        def mutate(rows):
            event=next(r for r in rows if r['key']=='MAVMOOS_IMU_SENSORS' and r['elapsed']==3)
            value=json.loads(event['value']);value['sensors'][0]['effective_at']+=.01;event['value']=json.dumps(value)
        self.reject(mutate)

    def test_aged_sample_cannot_remain_fresh(self):
        self.reject(lambda rows:change_sample(rows,lambda s:s.update(fresh=True)))

    def test_pre_floor_data_and_missing_clear_or_recovery_are_rejected(self):
        self.reject(lambda rows:rows.__setitem__(slice(None),[r for r in rows if not(r['key']=='MAVMOOS_IMU_STATUS' and r['elapsed']==4)]))
        self.reject(lambda rows:change_sample(rows,lambda s:s.update(session_epoch=1)))
        self.reject(lambda rows:change_sample(rows,lambda s:s.update(effective_at=99,read_at=99.05)))

    def test_missing_stale_original_sample_is_rejected(self):
        self.reject(lambda rows:rows.__setitem__(slice(None),[r for r in rows if r['elapsed'] not in (3,3.01)]))

    def test_unknown_age_clock_and_remote_age_inference_are_rejected(self):
        self.reject(lambda rows:change_sample(rows,lambda s:s.update(age_basis='SOURCE_CLOCK')))
        self.reject(lambda rows:change_sample(rows,lambda s:s.update(source_age=0)))
        self.reject(lambda rows:change_sample(rows,lambda s:s['data'].update(remote_clock='unix_synchronized')))

    def test_empty_or_only_early_fresh_observations_are_rejected(self):
        with self.assertRaises(ValueError):audit_probe([])
        self.reject(lambda rows:rows.__setitem__(slice(None),[r for r in rows if r['elapsed']<20]))

    def test_foreign_source_or_regressed_consumer_clock_is_rejected(self):
        self.reject(lambda rows:rows[0].update(source='foreign'))
        self.reject(lambda rows:rows.reverse())

    def test_control_effects_or_control_module_are_rejected(self):
        for key,a,b in [('MAVMOOS_RUNTIME_STATUS','tx_attempts=0','tx_attempts=1'),('MAVMOOS_STATUS','tx_enabled=false','tx_enabled=true')]:
            self.reject(lambda rows:[r.update(value=r['value'].replace(a,b)) for r in rows if r['key']==key])
        def mutate(rows):
            r=next(r for r in rows if r['key']=='MAVMOOS_CAPABILITIES');v=json.loads(r['value']);v['composition']['modules'][1]['control_provider']=True;r['value']=json.dumps(v)
        self.reject(mutate)

    def test_duplicate_or_nonfinite_json_is_not_repaired(self):
        for text in ('{"x":1,"x":2}','{"x":NaN}','{"x":1e999}'):
            with self.assertRaises(ValueError):parse(text)

    def test_frame_and_family_logical_instance_cannot_change(self):
        self.reject(lambda rows:change_sample(rows,lambda s:s['data'].update(body_frame='LOCAL_NED')))
        self.reject(lambda rows:change_sample(rows,lambda s:s['data'].update(message_id=116,family='SCALED_IMU2',sensor_instance=0)))

    def test_passive_bound_disarmed_ground_family_corroboration(self):
        fc=audit_fc(fc_fixture(),27);report=corroborate(audit_probe(fixture()),fc)
        self.assertEqual(report['common_message_families'],['RAW_IMU']);self.assertFalse(report['same_samples_compared']);self.assertFalse(report['clock_mapping_inferred'])

    def test_passive_source_arming_ground_silence_or_no_sensor_rejected(self):
        for field,value in [('system',2),('base_mode',128),('type',10)]:
            rows=fc_fixture();next(r for r in rows if r['mavpackettype']=='HEARTBEAT')[field]=value
            with self.assertRaises(ValueError):audit_fc(rows,27)
        rows=fc_fixture();next(r for r in rows if r['mavpackettype']=='EXTENDED_SYS_STATE')['landed_state']=2
        with self.assertRaises(ValueError):audit_fc(rows,27)
        with self.assertRaises(ValueError):audit_fc(fc_fixture(),35)
        with self.assertRaises(ValueError):audit_fc([r for r in fc_fixture() if r['mavpackettype']!='RAW_IMU'],27)

    def test_independent_channels_do_not_satisfy_each_other_by_name_only(self):
        fc=audit_fc(fc_fixture(),27);fc['families']=['SCALED_IMU3']
        with self.assertRaises(ValueError):corroborate(audit_probe(fixture()),fc)

    def test_boolean_schema_duplicate_catalog_and_cross_session_cache_rejected(self):
        self.reject(lambda rows:change_sample(rows,lambda s:s.update(schema=True)))
        def duplicate(rows):
            event=next(r for r in rows if r['key']=='MAVMOOS_CAPABILITIES');v=json.loads(event['value']);v['composition']['modules'][1]['capabilities'].append({'name':'imu.temperature'});event['value']=json.dumps(v)
        self.reject(duplicate)
        def cross(rows):
            event=next(r for r in rows if r['key']=='MAVMOOS_IMU_STATUS');v=json.loads(event['value']);v['sensors']['session_epoch']=99;event['value']=json.dumps(v)
        self.reject(cross)


def runner_module():
    path=Path(__file__).with_name('run_native.py');spec=importlib.util.spec_from_file_location('_imu_preflight_control',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module


class PreflightTests(unittest.TestCase):
    def args(self,runner,root):
        return runner.parser().parse_args(['--scenario','rx_only','--source-root',str(runner.ROOT),'--ardupilot-root',str(root/'ap'),'--moos-root',str(root/'moos'),'--build',str(root/'build'),'--output',str(root/'output'),'--firmware-manifest',str(root/'source.json'),'--firmware-attestation',str(root/'attestation.json')])

    def test_missing_or_rejected_firmware_never_opens_socket_process_or_output(self):
        runner=runner_module()
        for returned in (None,{'qualification':'not_inferred','frame':'not_quad'}):
            with self.subTest(returned=returned),tempfile.TemporaryDirectory() as tmp:
                root=Path(tmp);ownership=types.SimpleNamespace(native_firmware_profile=mock.Mock(return_value=returned))
                with mock.patch.object(runner,'load_helpers',return_value={'ownership':ownership}),mock.patch.object(runner.socket,'socket') as opened,mock.patch.object(runner.subprocess,'Popen') as launched:
                    with self.assertRaises(ValueError):runner.run(self.args(runner,root))
                    opened.assert_not_called();launched.assert_not_called();self.assertFalse((root/'output').exists())
                    ownership.native_firmware_profile.assert_called_once()

    def test_wrong_selected_runner_source_is_rejected_before_helper_or_io(self):
        runner=runner_module()
        with tempfile.TemporaryDirectory() as tmp:
            args=self.args(runner,Path(tmp));args.source_root=str(Path(tmp)/'wrong')
            with mock.patch.object(runner,'load_helpers') as helpers,mock.patch.object(runner.socket,'socket') as opened,mock.patch.object(runner.subprocess,'Popen') as launched:
                with self.assertRaises(ValueError):runner.run(args)
                helpers.assert_not_called();opened.assert_not_called();launched.assert_not_called()

    def test_expected_hash_guard_fails_before_output_or_io(self):
        runner=runner_module()
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);args=self.args(runner,root);args.expected_app_sha256='a'*64
            paths=[root/'ap/build/sitl/bin/arducopter',root/'build/bin/iMAVLink',root/'build/bin/imu_moos_probe',root/'build/plugins/mavmoos_telemetry.so',root/'build/plugins/mavmoos_imu.so',root/'moos/bin/pAntler',root/'moos/bin/MOOSDB']
            for path in paths:path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(b'inert guard fixture')
            firmware={'qualification':'not_inferred','frame':'quad','native_path':str(paths[0])}
            ownership=types.SimpleNamespace(native_firmware_profile=mock.Mock(return_value=firmware))
            with mock.patch.object(runner,'load_helpers',return_value={'ownership':ownership}),mock.patch.object(runner.socket,'socket') as opened,mock.patch.object(runner.subprocess,'Popen') as launched:
                with self.assertRaisesRegex(ValueError,'SHA'):runner.run(args)
                opened.assert_not_called();launched.assert_not_called();self.assertFalse((root/'output').exists())

    def test_changed_manifest_copy_is_rejected_before_socket_or_process(self):
        runner=runner_module()
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);args=self.args(runner,root);manifest=root/'source.json';manifest.write_bytes(b'changed')
            firmware={'manifest_path':str(manifest),'manifest_sha256':'a'*64,'attestation_path':str(root/'attestation.json'),'attestation_sha256':'b'*64}
            fake_pymavlink=types.SimpleNamespace(mavutil=types.SimpleNamespace())
            pre=(runner.ROOT,root/'ap',root/'moos',root/'build',{},firmware,[root/str(n) for n in range(7)])
            with mock.patch.object(runner,'preflight',return_value=pre),mock.patch.dict('sys.modules',{'pymavlink':fake_pymavlink}),mock.patch.object(runner.socket,'socket') as opened,mock.patch.object(runner.subprocess,'Popen') as launched:
                with self.assertRaisesRegex(ValueError,'changed before exclusive copy'):runner.run(args)
                opened.assert_not_called();launched.assert_not_called();self.assertFalse((root/'output/firmware-source.json').exists())


if __name__=='__main__':unittest.main(verbosity=2)
