#!/usr/bin/env python3
"""Offline service-bench audit: primary wire, actual module results and passive/DF state."""
import argparse
import json
from pathlib import Path
import struct
from verify_fake import fields, number, packet, parse, require


def value(payload):
    return {'value':struct.unpack_from('<f',payload,0)[0],
            'name':payload[8:24].split(b'\0')[0].decode('ascii'),'type':payload[24]}


def results(rows, key, op):
    return [fields(row['text']) for row in rows if row.get('type')=='publication' and row.get('key')==key
            and fields(row['text']).get('id')==op]


def terminal(rows, key, op, status, flags):
    r=results(rows,key,op)
    require(r and r[-1].get('status')==status,'wrong/missing native terminal: '+op)
    if status!='COMPLETED':require(all(p.get('status')!='COMPLETED' for p in r),'native false completion: '+op)
    require(all(r[-1].get(k)==v for k,v in flags.items()),'native uncertainty/verification flags differ: '+op)
    positive=[float(x['expires']) for x in r if float(x.get('expires','0'))>0]
    require(positive and len(set(positive))==1,'native original deadline renewed')
    return r[-1]


def audit(rows, observer, dataflash, monitor_finished):
    starts=[r for r in rows if r.get('type')=='start'];summaries=[r for r in rows if r.get('type')=='summary']
    require(len(starts)==len(summaries)==1 and summaries[0]['result']=='PASS','native driver start/summary missing or failed')
    start,summary=starts[0],summaries[0];scenario=start['scenario']
    require(scenario in ('wind_readback_loss','mission_ack_loss') and summary['scenario']==scenario,'scenario identity mismatch')
    require(summary['never_armed'] is True and summary['navigation_granted'] is False and summary['physical'] is False,'scope/state flags differ')
    require(0<number(summary['duration_s'])<90,'native duration outside bound')
    received=[r for r in rows if r.get('type')=='receive'];sends=[r for r in rows if r.get('type')=='send'];drops=[r for r in received if r['decision']=='DROP_AT_PARSED_RECEIPT_BOUNDARY']
    require(len(received)==summary['received'] and len(drops)==summary['dropped']>0
            and len(received)-len(drops)==summary['delivered'],'receipt/drop accounting differs')
    decoded=[]
    for r in received:
        require(r['decision'] in ('DELIVER','DROP_AT_PARSED_RECEIPT_BOUNDARY'),'unknown injection action')
        require(0<=number(r['effective_at'])<=number(r['read_at']),'original receive metadata invalid')
        msgid,payload=packet(r['wire_hex'],(1,1),check_crc=r['msgid'] in (0,20,22,23,40,42,43,44,45,47,51,73,76,77,84,86))
        require(msgid==r['msgid'],'primary wire/message identity differs');decoded.append((r,msgid,payload))
        if msgid==0:require(not payload[6]&128,'primary heartbeat armed')
    accepted=[];attempted_sets=[]
    for r in sends:
        f=r['frame'];msgid,payload=packet(f['wire_hex'])
        require(msgid==f['msgid'] and type(f['accepted']) is bool,'native send shape invalid')
        require(number(r['receipt_at'])>=number(f['at']) and number(f['expires'])>0,'send receipt clock/order invalid')
        if f['accepted']:require(f['at']<f['expires'],'accepted request starts at/after original expiry');accepted.append((r,msgid,payload))
        require(msgid in (0,20,23,43,44,45,47,51,73,76),'forbidden native effect message')
        if msgid==76:require(struct.unpack_from('<H',payload,28)[0]==511,'native control command emitted')
        if msgid==73:require(payload[35]!=2,'Guided/execute flag disguised as mission item')
        if msgid==23:
            require(scenario=='wind_readback_loss','mission scenario attempted a parameter SET')
            require(payload[4:6]==bytes([1,1]) and payload[6:22].split(b'\0')[0]==b'SIM_WIND_SPD'
                    and payload[22]==9 and struct.unpack_from('<f',payload,0)[0] in (.25,0),
                    'attempted SET outside exact wind target/name/type/value profile')
            attempted_sets.append((struct.unpack_from('<f',payload,0)[0],f['accepted']))
    require(not any(int(r.get('ArmState',0)) for r in dataflash.get('arms',[])),'DataFlash shows arming')
    hbs=[p for p in observer if p.get('mavpackettype')=='HEARTBEAT']
    require(len(hbs)>=2 and all(p.get('system_id')==1 and p.get('component_id')==1 and not p['base_mode']&128 for p in hbs),'passive disarmed heartbeat evidence absent/contradictory')
    require(hbs[-1]['custom_mode']==0,'passive final mode not unchanged STABILIZE')
    landed=[p for p in observer if p.get('mavpackettype')=='EXTENDED_SYS_STATE']
    require(landed and landed[-1]['landed_state']==1,'passive final on-ground observation absent')
    end=number(monitor_finished)
    require(0<=end-number(hbs[-1]['received_mono'])<=2 and 0<=end-number(landed[-1]['received_mono'])<=2,
            'passive final state is stale on its own receipt clock')
    late=[p for p in hbs if 0<=end-p['received_mono']<=5]
    require(len(late)>=2 and late[-1]['received_mono']-late[0]['received_mono']>=.5,
            'late advancing passive heartbeat evidence absent')
    state=fields(summary['runtime']);require(state['permission']=='false' and state['armed']=='0' and state['in_air']=='0','runtime acquired/armed/airborne')
    common={'result':'PASS','scenario':scenario,'raw_primary_receipts':len(received),'parsed_receipts_dropped':len(drops),
            'accepted_sends':len(accepted),'passive_heartbeats':len(hbs),'never_armed':True,
            'scope':'closed native service-bench evidence; parsed receipt omission, not a network delay/latency measurement; no physical route'}
    if scenario=='wind_readback_loss':
        terminal(rows,'MAVMOOS_PARAM_RESULT','wind-loss','TIMED_OUT',{'write_may_have_applied':'true','readback_confirmed':'false'})
        terminal(rows,'MAVMOOS_PARAM_RESULT','wind-check','COMPLETED',{})
        terminal(rows,'MAVMOOS_PARAM_RESULT','wind-restore','COMPLETED',{'readback_confirmed':'true'})
        setters=[payload for _,mid,payload in accepted if mid==23]
        require(len(setters)==summary['accepted_parameter_sets']==2,'SET replay or restore absent')
        require(attempted_sets==[(.25,True),(0.0,True)],'SET attempt sequence differs from one mutation and one verified restore')
        require([struct.unpack_from('<f',p,0)[0] for p in setters]==[.25,0.0]
                and all(p[6:22].split(b'\0')[0]==b'SIM_WIND_SPD' and p[4:6]==bytes([1,1]) for p in setters),'write expanded beyond fixed wind mutation/restore')
        require(all(r['msgid']==22 and value(packet(r['wire_hex'],(1,1))[1])['name']=='SIM_WIND_SPD' for r in drops),'wrong native receipt dropped')
        samples=[(r,value(p)) for r,mid,p in decoded if mid==22]
        require(any(r['phase']=='FRESH_WIND_READ' and v['name']=='SIM_WIND_SPD' and v['value']==.25 for r,v in samples),'fresh primary-wire applied value absent')
        require(any(r['phase']=='VERIFIED_WIND_RESTORE' and r['decision']=='DELIVER' and v['name']=='SIM_WIND_SPD' and v['value']==0 for r,v in samples),'fresh primary-wire restored value absent')
        parms=[p for p in dataflash.get('parameters',[]) if p['Name']=='SIM_WIND_SPD']
        changed=[p for p in parms if p['Value']==.25]
        require(changed and any(p['TimeUS']>changed[0]['TimeUS'] and p['Value']==0 for p in parms),'closed DataFlash does not corroborate wind mutation/restore')
        return {**common,'write_outcome':'TIMED_OUT; may have applied; not readback-confirmed',
                'fresh_wire_observed_mutation':.25,'fresh_wire_verified_restore':0,'dataflash_mutation_and_restore':True}
    terminal(rows,'MAVMOOS_MISSION_RESULT','mission-loss','TIMED_OUT',{'mutation_may_have_applied':'true','remote_accepted':'false','readback_confirmed':'false'})
    terminal(rows,'MAVMOOS_MISSION_RESULT','mission-check','COMPLETED',{})
    terminal(rows,'MAVMOOS_MISSION_RESULT','mission-clear','COMPLETED',{'readback_confirmed':'true'})
    require(all(r['msgid']==47 for r in drops),'wrong native receipt dropped')
    require(any(packet(r['wire_hex'],(1,1))[1][2]==0 for r in drops),'actual remote accepted ACK was not captured before omission')
    stored=[p for r,mid,p in decoded if mid==73 and r['phase']=='FRESH_STORED_PLAN_READ' and r['decision']=='DELIVER']
    require({struct.unpack_from('<H',p,28)[0] for p in stored}=={0,1},'fresh full stored-plan wire read absent')
    item=next(p for p in stored if struct.unpack_from('<H',p,28)[0]==1)
    require(struct.unpack_from('<i',item,16)[0]==423584660 and struct.unpack_from('<i',item,20)[0]==-710875890
            and abs(struct.unpack_from('<f',item,24)[0]-5)<.010001 and struct.unpack_from('<H',item,30)[0]==16
            and item[34]==3,'fresh non-HOME stored row differs')
    require(any(r['phase']=='VERIFIED_MISSION_CLEAR' and mid==44 and struct.unpack_from('<H',p,0)[0]==0 for r,mid,p in decoded),'fresh empty mission-count wire read absent')
    return {**common,'upload_outcome':'TIMED_OUT; mutation may have applied; ACK not delivered to module',
            'captured_remote_acceptance_before_drop':True,'fresh_wire_stored_rows':2,'fresh_wire_verified_empty_after_clear':True,
            'settlement_wait_s':8.5,'settlement_wait_scope':'fixed fixture wait beyond pinned AP timeout; no automatic product retry/settlement guarantee'}


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('output',type=Path);a=p.parse_args();base=a.output
    rows=[parse(line) for line in (base/'driver.jsonl').read_text().splitlines() if line]
    observer=[parse(line) for line in (base/'observer.jsonl').read_text().splitlines() if line]
    dataflash=parse((base/'dataflash.json').read_text())
    report=parse((base/'result.json').read_text())
    print(json.dumps(audit(rows,observer,dataflash,report['monitor_finished_python_mono']),indent=2))


if __name__=='__main__':main()
