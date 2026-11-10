#!/usr/bin/env python3
"""Independent standard-library audit of fake-I/O Runtime fault-suite evidence."""
import argparse
import copy
import json
import math
from pathlib import Path
import struct

CASES = set('''named_read_loss_original_deadline write_echo_loss_retains_uncertainty
write_readback_loss_never_replays_set write_mismatch_fails_without_retry
foreign_and_wrong_name_reply_cannot_complete_read paced_inventory_reorder_duplicate_and_unrequested
paced_inventory_changed_count_is_incomplete reset_during_parameter_readback_handoff_fences_old_work
mission_count_loss_has_bounded_recovery mission_reordered_future_item_does_not_advance
mission_final_ack_loss_retains_mutation_uncertainty mission_cancel_during_readback_keeps_remote_acceptance
mission_readback_cancel_at_encoded_handoff mission_readback_mismatch_is_not_verified
mission_reset_rejects_old_phase_and_id_replay untagged_same_name_reply_causality_is_unproven
protocol_timeout_preserves_independent_navigation missing_command_ack_and_state_times_out_without_reacquisition
lost_command_ack_can_complete_on_fresh_observed_outcome command_ack_alone_does_not_prove_observed_completion
receive_only_protocol_effects_denied shadow_protocol_effects_denied'''.split())
# CRC_EXTRA from pinned c_library_v2 a2d0ecf generated message definitions.
CRC = {0:50, 20:214, 22:220, 23:168, 40:230, 42:28, 43:132, 44:221,
       45:232, 47:153, 51:196, 73:38, 76:152, 77:143, 84:143, 86:5}


def require(value, why):
    if not value:
        raise ValueError(why)


def parse(text):
    def bad(value): raise ValueError('nonfinite JSON: ' + value)
    def pairs(items):
        result = {}
        for key, value in items:
            require(key not in result, 'duplicate JSON key')
            result[key] = value
        return result
    return json.loads(text, parse_constant=bad, object_pairs_hook=pairs)


def number(value):
    require(type(value) in (int, float) and math.isfinite(value), 'nonfinite/non-numeric time')
    return value


def fields(text):
    return dict(part.split('=', 1) for part in text.split(',') if '=' in part)


def packet(text, source=(245, 191), check_crc=True):
    require(isinstance(text, str), 'wire encoding not a string')
    data = bytes.fromhex(text)
    require(len(data) >= 12 and data[0] == 253 and len(data) == data[1] + 12,
            'not exactly one unsigned MAVLink2 frame')
    require(data[2] == 0 and data[3] == 0, 'unexpected incompat/compat flags')
    require(tuple(data[5:7]) == source, 'wire source identity changed')
    msgid = int.from_bytes(data[7:10], 'little')
    payload = data[10:-2]
    if check_crc:
        require(msgid in CRC, 'message outside explicit fault-suite wire family')
        crc = 0xffff
        for byte in data[1:-2] + bytes([CRC[msgid]]):
            tmp = byte ^ (crc & 0xff)
            tmp ^= (tmp << 4) & 0xff
            crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xffff
        require(crc == int.from_bytes(data[-2:], 'little'), 'wire CRC mismatch')
    return msgid, payload + bytes(255 - len(payload))


def result_rows(case, op):
    return [fields(row['text']) for row in case['publications']
            if row['key'] in ('MAVMOOS_PARAM_RESULT', 'MAVMOOS_MISSION_RESULT', 'MAVMOOS_COMMAND_RESULT')
            and fields(row['text']).get('id') == op]


def terminal(case, op, status, flags=None):
    rows = result_rows(case, op)
    require(any(row.get('status') == status for row in rows), 'expected terminal missing: ' + op)
    if status != 'COMPLETED':
        require(not any(row.get('status') == 'COMPLETED' for row in rows), 'false completion: ' + op)
    matches = [r for r in rows if r.get('status') == status]
    for key, value in (flags or {}).items():
        require(matches[-1].get(key) == value, 'terminal uncertainty/verification flag differs: ' + op + '/' + key)


def verify(records):
    cases = [row for row in records if row.get('type') == 'case']
    summaries = [row for row in records if row.get('type') == 'summary']
    require(len(cases) == len(CASES) and {r['name'] for r in cases} == CASES,
            'missing/duplicate fault case')
    require(len(records) == len(cases) + 1 and len(summaries) == 1 and summaries[0]['groups'] == len(CASES)
            and summaries[0]['result'] == 'PASS', 'wrong final summary')
    require('no network' in summaries[0]['scope'], 'fake scope absent')
    by_name, wire_counts = {}, {}
    for case in cases:
        name = case['name']; by_name[name] = case
        require(case['result'] == 'PASS' and 'no socket/FC' in case['scope'], 'case result/scope mismatch')
        counts = {}; accepted = []
        for frame in case['frames']:
            msgid, payload = packet(frame['wire_hex'])
            require(msgid == frame['msgid'], 'wire/summary message mismatch')
            at, expires = number(frame['at']), number(frame['expires'])
            require(type(frame['accepted']) is bool and expires > 0, 'invalid handoff result/expiry')
            if frame['accepted']:
                require(at < expires, 'accepted frame at/after expiry')
                counts[msgid] = counts.get(msgid, 0) + 1; accepted.append((msgid, payload))
            if msgid == 76:
                require(struct.unpack_from('<H', payload, 28)[0] in (176, 511), 'unexpected command in fault suite')
        for receipt in case['receipts']:
            msgid, _ = packet(receipt['wire_hex'], (1, 1) if name != 'foreign_and_wrong_name_reply_cannot_complete_read' else tuple(bytes.fromhex(receipt['wire_hex'])[5:7]))
            require(msgid == receipt['msgid'] and number(receipt['effective_at']) <= number(receipt['read_at']), 'receipt identity/time mismatch')
        expiries = {}
        for p in case['publications']:
            number(p['at'])
            if p['key'] not in ('MAVMOOS_PARAM_RESULT', 'MAVMOOS_MISSION_RESULT'): continue
            f = fields(p['text'])
            if 'at' in f:require(abs(number(float(f['at']))-p['at'])<=1e-6,'publication clock contradicts result clock')
            if f.get('status') in ('REJECTED', 'DUPLICATE') or 'expires' not in f: continue
            value = float(f['expires']); number(value)
            if value <= 0: continue
            key = (p['key'], f['id'])
            require(key not in expiries or value == expiries[key], 'original expiry renewed')
            expiries[key] = value
        wire_counts[name] = counts
        if name.startswith('receive_only') or name.startswith('shadow'):
            require(not case['frames'] and fields(case['runtime'])['permission'] == 'false', 'disabled mode sent/acquired')
    c=lambda name: by_name[name]
    n=lambda name, msg: wire_counts[name].get(msg, 0)
    terminal(c('named_read_loss_original_deadline'), 'lost', 'TIMED_OUT'); require(n('named_read_loss_original_deadline',20)==1,'named READ replayed')
    for name,op in [('write_echo_loss_retains_uncertainty','lost-echo'),('write_readback_loss_never_replays_set','lost-readback')]:
        terminal(c(name),op,'TIMED_OUT',{'write_may_have_applied':'true','readback_confirmed':'false'});require(n(name,23)==1,'SET replayed')
    require(n('write_echo_loss_retains_uncertainty',20)==0 and n('write_readback_loss_never_replays_set',20)==1,'write phase ordering changed')
    terminal(c('write_mismatch_fails_without_retry'),'mismatch','FAILED')
    foreign=c('foreign_and_wrong_name_reply_cannot_complete_read')
    terminal(foreign,'named','COMPLETED')
    inspected=[]
    for r in foreign['receipts']:
        source=tuple(bytes.fromhex(r['wire_hex'])[5:7]);mid,payload=packet(r['wire_hex'],source)
        require(mid==22,'foreign/name case receipt family changed')
        inspected.append((source,payload[8:24].split(b'\0')[0],r['read_at']))
    require([(s,n) for s,n,_ in inspected]==[((2,1),b'FRAME_CLASS'),((1,1),b'OTHER'),((1,1),b'FRAME_CLASS')],
            'foreign/name rejection sequence missing')
    completed=[p for p in foreign['publications'] if p['key']=='MAVMOOS_PARAM_RESULT'
               and fields(p['text']).get('id')=='named' and fields(p['text']).get('status')=='COMPLETED']
    require(len(completed)==1 and completed[0]['at']>=inspected[-1][2],
            'foreign or wrong-name receipt completed before correct value')
    terminal(c('paced_inventory_reorder_duplicate_and_unrequested'),'table','COMPLETED'); require(n('paced_inventory_reorder_duplicate_and_unrequested',20)==4,'duplicate inventory inflated/retried')
    terminal(c('paced_inventory_changed_count_is_incomplete'),'shape','INCOMPLETE')
    terminal(c('reset_during_parameter_readback_handoff_fences_old_work'),'reset-write','CANCELLED',{'write_may_have_applied':'true','readback_confirmed':'false'});require(n('reset_during_parameter_readback_handoff_fences_old_work',20)==0,'old encoded readback sent')
    terminal(c('mission_count_loss_has_bounded_recovery'),'no-count','TIMED_OUT');require(2<=n('mission_count_loss_has_bounded_recovery',43)<=6,'mission retry budget exceeded')
    ordered=c('mission_reordered_future_item_does_not_advance')
    terminal(ordered,'order','COMPLETED')
    items=[]
    for r in ordered['receipts']:
        mid,payload=packet(r['wire_hex'],(1,1))
        if mid==73:items.append((struct.unpack_from('<H',payload,28)[0],r['read_at']))
    require([seq for seq,_ in items]==[1,0,0,1],'reordered/duplicate mission receipt sequence missing')
    progress=[p for p in ordered['publications'] if p['key']=='MAVMOOS_MISSION_RESULT'
              and fields(p['text']).get('id')=='order' and fields(p['text']).get('status')=='ITEM_RECEIVED']
    require(len(progress)==2 and [fields(p['text'])['received'] for p in progress]==['1','2']
            and progress[0]['at']>=items[1][1] and progress[1]['at']>=items[3][1],
            'future/duplicate mission item advanced verified progress')
    require(n('mission_reordered_future_item_does_not_advance',51)==2
            and n('mission_reordered_future_item_does_not_advance',47)==1,'mission item/ACK request budget differs')
    requests=[];acks=[]
    for f in ordered['frames']:
        if not f['accepted']:continue
        mid,payload=packet(f['wire_hex'])
        if mid==51:requests.append((struct.unpack_from('<H',payload,0)[0],f['at']))
        if mid==47:acks.append(f['at'])
    require([seq for seq,_ in requests]==[0,1] and requests[0][1]<items[0][1]
            and requests[1][1]>=items[1][1],'mission requested indices/chronology differ')
    complete=[p for p in ordered['publications'] if p['key']=='MAVMOOS_MISSION_RESULT'
              and fields(p['text']).get('id')=='order' and fields(p['text']).get('status')=='COMPLETED']
    require(len(complete)==1 and items[-1][1]<=acks[0]<=complete[0]['at'],
            'mission completion precedes final item/local ACK')
    terminal(c('mission_final_ack_loss_retains_mutation_uncertainty'),'ack-loss','TIMED_OUT',{'mutation_may_have_applied':'true','remote_accepted':'false','readback_confirmed':'false'})
    terminal(c('mission_cancel_during_readback_keeps_remote_acceptance'),'cancel-readback','CANCELLED',{'mutation_may_have_applied':'true','remote_accepted':'true','readback_confirmed':'false'})
    terminal(c('mission_readback_cancel_at_encoded_handoff'),'cancel-final','CANCELLED');require(n('mission_readback_cancel_at_encoded_handoff',43)==0,'cancelled encoded mission request sent')
    terminal(c('mission_readback_mismatch_is_not_verified'),'bad-plan','FAILED',{'readback_confirmed':'false'})
    terminal(c('mission_reset_rejects_old_phase_and_id_replay'),'old','CANCELLED');require(n('mission_reset_rejects_old_phase_and_id_replay',51)==0,'old phase revived')
    amb=c('untagged_same_name_reply_causality_is_unproven')
    require(amb['ambiguity_observed'] is True and amb['remote_causality_proven'] is False and amb['wire_operation_id_present'] is False,'untagged ambiguity hidden')
    terminal(amb,'old-read','TIMED_OUT');terminal(amb,'new-read','COMPLETED')
    terminal(c('protocol_timeout_preserves_independent_navigation'),'nav-loss','TIMED_OUT');require(n('protocol_timeout_preserves_independent_navigation',84)>=10,'NAV starved')
    terminal(c('missing_command_ack_and_state_times_out_without_reacquisition'),'hold-lost','TIMED_OUT')
    terminal(c('lost_command_ack_can_complete_on_fresh_observed_outcome'),'hold-observed','COMPLETED')
    terminal(c('command_ack_alone_does_not_prove_observed_completion'),'hold-no-state','TIMED_OUT')
    return {'result':'PASS','groups':len(cases),'wire_frames':sum(len(c['frames']) for c in cases),'explicit_receipts':sum(len(c['receipts']) for c in cases),'scope':'offline fake-I/O evidence consistency; no new Runtime/native execution'}


def negative_controls(records):
    rejected=[]
    def reject(name, mutate):
        changed=copy.deepcopy(records);mutate(changed)
        try:verify(changed)
        except (ValueError,KeyError,TypeError,IndexError):rejected.append(name)
        else:raise ValueError('negative control accepted: '+name)
    reject('missing_case',lambda r:r.pop(0))
    reject('duplicate_case',lambda r:r.insert(0,copy.deepcopy(r[0])))
    reject('false_summary',lambda r:r[-1].update(groups=999))
    reject('bad_crc',lambda r:r[0]['frames'][0].update(wire_hex=r[0]['frames'][0]['wire_hex'][:-2]+'00'))
    reject('expired_accepted_frame',lambda r:r[0]['frames'][0].update(at=r[0]['frames'][0]['expires']))
    def row(r,name):return next(x for x in r if x.get('name')==name)
    reject('ambiguous_remote_causality_claim',lambda r:row(r,'untagged_same_name_reply_causality_is_unproven').update(remote_causality_proven=True))
    reject('accepted_set_replay',lambda r:row(r,'write_echo_loss_retains_uncertainty')['frames'].append(copy.deepcopy(next(x for x in row(r,'write_echo_loss_retains_uncertainty')['frames'] if x['msgid']==23))))
    def false_flag(r):
        p=next(p for p in row(r,'write_readback_loss_never_replays_set')['publications'] if 'status=TIMED_OUT' in p['text'])
        p['text']=p['text'].replace('readback_confirmed=false','readback_confirmed=true')
    reject('false_readback_confirmation',false_flag)
    def renewed(r):
        p=next(p for p in r[0]['publications'] if 'status=TIMED_OUT' in p['text']);f=fields(p['text']);p['text']=p['text'].replace('expires='+f['expires'],'expires='+str(float(f['expires'])+1))
    reject('renewed_original_deadline',renewed)
    reject('disabled_mode_effect',lambda r:row(r,'receive_only_protocol_effects_denied')['frames'].append(copy.deepcopy(r[0]['frames'][0])))
    def missing_named(r):
        case=row(r,'foreign_and_wrong_name_reply_cannot_complete_read')
        case['publications']=[p for p in case['publications'] if p['key']!='MAVMOOS_PARAM_RESULT']
    reject('missing_foreign_name_case_outcome',missing_named)
    def premature_item(r):
        case=row(r,'mission_reordered_future_item_does_not_advance')
        p=next(p for p in case['publications'] if fields(p['text']).get('status')=='ITEM_RECEIVED')
        early=next(q['read_at'] for q in case['receipts'] if q['msgid']==73)
        f=fields(p['text']);p['text']=p['text'].replace('at='+f['at'],'at='+str(early));p['at']=early
    reject('future_mission_item_false_progress',premature_item)
    def premature_completion(r):
        case=row(r,'mission_reordered_future_item_does_not_advance')
        p=next(p for p in case['publications'] if fields(p['text']).get('status')=='COMPLETED')
        early=next(q['read_at'] for q in case['receipts'] if q['msgid']==73)
        f=fields(p['text']);p['text']=p['text'].replace('at='+f['at'],'at='+str(early));p['at']=early
    reject('mission_completion_before_valid_final_item',premature_completion)
    def wrong_request_index(r):
        case=row(r,'mission_reordered_future_item_does_not_advance')
        f=[f for f in case['frames'] if f['msgid']==51][-1];data=bytearray.fromhex(f['wire_hex']);data[10:12]=b'\0\0';crc=0xffff
        for byte in data[1:-2]+bytes([CRC[51]]):
            tmp=byte^(crc&255);tmp^=(tmp<<4)&255;crc=((crc>>8)^(tmp<<8)^(tmp<<3)^(tmp>>4))&65535
        data[-2:]=struct.pack('<H',crc);f['wire_hex']=data.hex()
    reject('valid_crc_wrong_requested_mission_index',wrong_request_index)
    return rejected


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('trace',type=Path);parser.add_argument('--negative-controls',action='store_true');args=parser.parse_args()
    records=[parse(line) for line in args.trace.read_text().splitlines() if line]
    result=verify(records)
    if args.negative_controls:result['negative_controls_rejected']=negative_controls(records)
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
