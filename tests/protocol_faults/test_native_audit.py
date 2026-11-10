#!/usr/bin/env python3
"""Synthetic positives/mutations for the native evidence checker; no network/FC."""
import copy
import struct
import unittest
from audit_native import audit
from verify_fake import CRC
from run_native import observer_message


def frame(mid,payload,source=(1,1)):
    payload=payload.rstrip(b'\0') or b'\0'
    head=bytes([len(payload),0,0,1,source[0],source[1],mid&255,(mid>>8)&255,(mid>>16)&255])
    crc=0xffff
    for byte in head+payload+bytes([CRC[mid]]):
        tmp=byte^(crc&255);tmp^=(tmp<<4)&255;crc=((crc>>8)^(tmp<<8)^(tmp<<3)^(tmp>>4))&65535
    return (b'\xfd'+head+payload+struct.pack('<H',crc)).hex()


def parameter(value):return struct.pack('<fHH16sB',value,1364,65535,b'SIM_WIND_SPD',9)
def setter(value):return struct.pack('<fBB16sB',value,1,1,b'SIM_WIND_SPD',9)
def item(seq):
    return struct.pack('<ffffiifHHBBBBBB',0,0,0,0,423584560+100*seq,-710875890,5 if seq else 10,seq,16,245,191,3 if seq else 0,0,1,0)


def fixture(scenario):
    rows=[{'type':'start','scenario':scenario}]
    def receive(mid,payload,phase,drop=False):
        rows.append({'type':'receive','phase':phase,'decision':'DROP_AT_PARSED_RECEIPT_BOUNDARY' if drop else 'DELIVER',
                     'msgid':mid,'wire_hex':frame(mid,payload),'effective_at':100,'read_at':100.01})
    def send(mid,payload,phase):
        rows.append({'type':'send','phase':phase,'receipt_at':100.01,'frame':{'msgid':mid,'wire_hex':frame(mid,payload,(245,191)),'at':100,'expires':101,'accepted':True}})
    def publication(key,op,status,flags):
        rows.append({'type':'publication','key':key,'phase':'fixture','text':'schema=1,id='+op+',status='+status+',expires=110,'+','.join(k+'='+v for k,v in flags.items()),'at':100.1})
    receive(0,struct.pack('<IBBBBB',0,2,3,0,4,3),'INITIAL')
    if scenario=='wind_readback_loss':
        send(23,setter(.25),'WRITE_WITH_READBACK_DROP');send(23,setter(0),'VERIFIED_WIND_RESTORE')
        receive(22,parameter(.25),'WRITE_WITH_READBACK_DROP',True)
        receive(22,parameter(.25),'FRESH_WIND_READ');receive(22,parameter(0),'VERIFIED_WIND_RESTORE')
        publication('MAVMOOS_PARAM_RESULT','wind-loss','TIMED_OUT',{'write_may_have_applied':'true','readback_confirmed':'false'})
        publication('MAVMOOS_PARAM_RESULT','wind-check','COMPLETED',{})
        publication('MAVMOOS_PARAM_RESULT','wind-restore','COMPLETED',{'readback_confirmed':'true'})
    else:
        send(44,struct.pack('<HBBB',2,1,1,0),'UPLOAD_WITH_ACK_DROP')
        send(73,item(1)[:32]+bytes([1,1])+item(1)[34:],'UPLOAD_WITH_ACK_DROP')
        receive(47,bytes([245,191,0,0]),'UPLOAD_WITH_ACK_DROP',True)
        receive(73,item(0),'FRESH_STORED_PLAN_READ');receive(73,item(1),'FRESH_STORED_PLAN_READ')
        receive(44,struct.pack('<HBBB',0,245,191,0),'VERIFIED_MISSION_CLEAR')
        publication('MAVMOOS_MISSION_RESULT','mission-loss','TIMED_OUT',{'mutation_may_have_applied':'true','remote_accepted':'false','readback_confirmed':'false'})
        publication('MAVMOOS_MISSION_RESULT','mission-check','COMPLETED',{})
        publication('MAVMOOS_MISSION_RESULT','mission-clear','COMPLETED',{'readback_confirmed':'true'})
    counts=[r for r in rows if r.get('type')=='receive'];drop=sum(r['decision'].startswith('DROP') for r in counts)
    rows.append({'type':'summary','result':'PASS','scenario':scenario,'never_armed':True,'navigation_granted':False,'physical':False,
                 'duration_s':5,'received':len(counts),'dropped':drop,'delivered':len(counts)-drop,'accepted_parameter_sets':2 if scenario=='wind_readback_loss' else 0,
                 'runtime':'permission=false,armed=0,in_air=0'})
    observer=[{'mavpackettype':'HEARTBEAT','system_id':1,'component_id':1,'base_mode':0,'custom_mode':0,'received_mono':t} for t in (8,9.8)]
    observer.append({'mavpackettype':'EXTENDED_SYS_STATE','landed_state':1,'received_mono':9.9})
    dataflash={'arms':[],'parameters':[{'Name':'SIM_WIND_SPD','Value':.25,'TimeUS':100},{'Name':'SIM_WIND_SPD','Value':0,'TimeUS':200}]}
    return rows,observer,dataflash,10


class NativeAudit(unittest.TestCase):
    def reject(self,mutate,scenario='wind_readback_loss'):
        values=copy.deepcopy(fixture(scenario));mutate(values)
        with self.assertRaises((ValueError,KeyError,TypeError,IndexError)):audit(*values)
    def test_wind_positive(self):self.assertEqual(audit(*fixture('wind_readback_loss'))['result'],'PASS')
    def test_mission_positive(self):self.assertEqual(audit(*fixture('mission_ack_loss'))['result'],'PASS')
    def test_original_failure_not_relabelled(self):self.reject(lambda v:v[0][-1].update(result='FAIL'))
    def test_missing_final_summary(self):self.reject(lambda v:v[0].pop())
    def test_drop_count_reconciled(self):self.reject(lambda v:v[0][-1].update(dropped=3))
    def test_observer_armed(self):self.reject(lambda v:v[1][0].update(base_mode=128))
    def test_observer_stale(self):self.reject(lambda v:v[1][-2].update(received_mono=1))
    def test_observer_no_late_progress(self):self.reject(lambda v:v[1][0].update(received_mono=9.8))
    def test_dataflash_armed(self):self.reject(lambda v:v[2]['arms'].append({'ArmState':1}))
    def test_missing_dataflash_restore(self):self.reject(lambda v:v[2]['parameters'].pop())
    def test_set_replay_rejected(self):self.reject(lambda v:v[0].insert(1,copy.deepcopy(next(r for r in v[0] if r.get('type')=='send'))))
    def test_wire_crc_change(self):self.reject(lambda v:next(r for r in v[0] if r.get('type')=='receive').update(wire_hex='fd010000010101000000000000'))
    def test_false_readback_confirmation(self):
        def mutate(v):
            r=next(r for r in v[0] if r.get('type')=='publication' and 'wind-loss' in r['text']);r['text']=r['text'].replace('readback_confirmed=false','readback_confirmed=true')
        self.reject(mutate)
    def test_missing_applied_wire_value(self):
        def mutate(v):
            r=next(r for r in v[0] if r.get('phase')=='FRESH_WIND_READ');r['wire_hex']=frame(22,parameter(.5))
        self.reject(mutate)
    def test_valid_crc_wrong_stored_item(self):
        def mutate(v):
            r=[r for r in v[0] if r.get('type')=='receive' and r.get('msgid')==73][-1]
            payload=bytearray(item(1));struct.pack_into('<f',payload,24,9);r['wire_hex']=frame(73,bytes(payload))
        self.reject(mutate,'mission_ack_loss')
    def test_nonempty_clear_wire(self):
        def mutate(v):
            r=next(r for r in v[0] if r.get('phase')=='VERIFIED_MISSION_CLEAR');r['wire_hex']=frame(44,struct.pack('<HBBB',2,245,191,0))
        self.reject(mutate,'mission_ack_loss')
    def test_missing_captured_remote_ack(self):
        def mutate(v):
            r=next(r for r in v[0] if r.get('type')=='receive' and r.get('msgid')==47);r['wire_hex']=frame(47,bytes([245,191,1,0]))
        self.reject(mutate,'mission_ack_loss')
    def test_original_clock_metadata_preserved(self):self.reject(lambda v:next(r for r in v[0] if r.get('type')=='receive').update(effective_at=999))
    def test_valid_crc_nonallowlisted_parameter_set(self):
        def mutate(v):
            r=next(r for r in v[0] if r.get('type')=='send')
            payload=struct.pack('<fBB16sB',.25,1,1,b'OTHER_PARAMETER',9);r['frame']['wire_hex']=frame(23,payload,(245,191))
        self.reject(mutate)
    def test_valid_crc_wrong_parameter_type(self):
        def mutate(v):
            r=next(r for r in v[0] if r.get('type')=='send')
            r['frame']['wire_hex']=frame(23,struct.pack('<fBB16sB',.25,1,1,b'SIM_WIND_SPD',6),(245,191))
        self.reject(mutate)
    def test_valid_crc_parameter_set_in_mission_scenario(self):
        def mutate(v):
            v[0].insert(1,{'type':'send','phase':'FRESH_STORED_PLAN_READ','receipt_at':100.01,
                'frame':{'msgid':23,'wire_hex':frame(23,setter(.25),(245,191)),'at':100,'expires':101,'accepted':True}})
        self.reject(mutate,'mission_ack_loss')


class NativeObserverStartup(unittest.TestCase):
    class Bad:
        def __init__(self,data=b'0 ',reason='Bad prefix'):self.data=data;self.reason=reason
        def get_type(self):return 'BAD_DATA'
    def test_exact_pinned_prefix_retained_before_mavlink(self):
        kind,row=observer_message(self.Bad(),12.5,0,0)
        self.assertEqual((kind,row['raw_hex'],row['received_mono']),('startup_diagnostic','3020',12.5))
    def test_same_prefix_after_valid_frame_rejected(self):
        with self.assertRaises(RuntimeError):observer_message(self.Bad(),12.5,1,0)
    def test_three_source_pinned_pairs_retained(self):
        for count in (0,2,4):self.assertEqual(observer_message(self.Bad(),12.5,0,count)[0],'startup_diagnostic')
    def test_coalesced_three_pairs_retained(self):
        self.assertEqual(observer_message(self.Bad(b'0 0 0 '),12.5,0,0)[1]['raw_hex'],'302030203020')
    def test_split_source_pinned_bytes_retained(self):
        for count in range(6):self.assertEqual(observer_message(self.Bad((b'0 '*3)[count:count+1]),12.5,0,count)[0],'startup_diagnostic')
    def test_fourth_prefix_rejected(self):
        with self.assertRaises(RuntimeError):observer_message(self.Bad(),12.5,0,6)
    def test_out_of_sequence_fragment_rejected(self):
        with self.assertRaises(RuntimeError):observer_message(self.Bad(b' '),12.5,0,0)
    def test_other_startup_bytes_rejected(self):
        with self.assertRaises(RuntimeError):observer_message(self.Bad(b'other'),12.5,0,0)
    def test_other_decoder_error_rejected(self):
        with self.assertRaises(RuntimeError):observer_message(self.Bad(reason='invalid CRC'),12.5,0,0)


if __name__=='__main__':unittest.main()
