"""Causal hold-window receipts. No sockets, process actions or clock mapping."""
import hashlib
import math
import os
from pathlib import Path
import re
import time


def is_hold_boundary(key):
    return re.fullmatch(r'cycle[12]\.hold_(helm|guided)\.(started|finished)',key) is not None


def valid_token(key,token):
    return (isinstance(token,bytes) and 0<len(token)<=128 and
            re.fullmatch(rb'hold:'+re.escape(key.encode('ascii'))+rb':[1-9][0-9]{0,3}:[0-9]+(?:\.[0-9]+)?\n',token) is not None)


def read_token(control,key):
    with (Path(control)/key).open('rb') as stream:token=stream.read(129)
    if not valid_token(key,token):raise RuntimeError('hold boundary token invalid')
    return token


def record_marker(control,key,times,events,receiver,*,clock=time.monotonic):
    control=Path(control)
    if key in times or not (control/key).exists():return
    receiver.check()
    observed=clock()
    if type(observed) not in (int,float) or not math.isfinite(observed) or observed<0:
        raise RuntimeError('hold marker observation clock invalid')
    times[key]=observed;events.append({'kind':key,'python_mono':observed})
    if not is_hold_boundary(key):return
    token=read_token(control,key)
    acknowledgement=control/(key+'.ack')
    if acknowledgement.exists():raise RuntimeError('preexisting hold acknowledgement')
    receiver.check()
    temporary=control/(key+'.ack.tmp')
    created=False
    try:
        with temporary.open('xb') as stream:created=True;stream.write(token)
        receiver.check()
        # A racing/preexisting acknowledgement must not be overwritten. The
        # complete file is visible atomically, after the boundary was sampled.
        os.link(temporary,acknowledgement)
    finally:
        if created and temporary.exists():temporary.unlink()
    acknowledged=clock()
    if type(acknowledged) not in (int,float) or not math.isfinite(acknowledged) or acknowledged<observed:
        raise RuntimeError('hold acknowledgement clock invalid/regressed')
    events.append({'kind':key+'.ack','python_mono':acknowledged,
                   'token_sha256':hashlib.sha256(token).hexdigest()})


def audit_boundary_bytes(events,proof,read):
    previous=-1
    for row in events:
        at=row.get('python_mono')
        if type(at) not in (int,float) or not math.isfinite(at) or at<previous:
            raise ValueError('hold receipt trace clock/order invalid')
        previous=at
    expected={label+suffix for label in proof['windows'] for suffix in ('.started','.finished')}
    seen=set();report={}
    for key in sorted(expected):
        originals=[row for row in events if row.get('kind')==key]
        acknowledgements=[row for row in events if row.get('kind')==key+'.ack']
        if len(originals)!=1 or len(acknowledgements)!=1:raise ValueError('missing/duplicate hold boundary receipt')
        if events.index(originals[0])>=events.index(acknowledgements[0]):raise ValueError('hold acknowledgement precedes boundary')
        before,after=originals[0]['python_mono'],acknowledgements[0]['python_mono']
        if (type(before) not in (int,float) or type(after) not in (int,float) or
            not math.isfinite(before) or not math.isfinite(after) or before<0 or after<before):
            raise ValueError('hold boundary receipt reordered/invalid')
        token=read(key);actual=read(key+'.ack')
        if not valid_token(key,token) or not isinstance(actual,bytes) or len(actual)>128:
            raise ValueError('hold boundary token bytes invalid')
        digest=hashlib.sha256(token).hexdigest()
        if actual!=token or acknowledgements[0].get('token_sha256')!=digest:
            raise ValueError('hold boundary acknowledgement differs')
        if proof['boundary_tokens'].get(key)!=token.decode('ascii'):
            raise ValueError('probe acknowledgement not bound to coordinator token')
        report[key]={'boundary_at':before,'acknowledged_at':after,'token_sha256':digest}
        seen.add(key+'.ack')
    if {row['kind'] for row in events if row.get('kind','').endswith('.ack')}!=seen:
        raise ValueError('foreign hold acknowledgement receipt')
    return {'result':'PASS','boundaries':report,'clock_scope':'each original local clock only; opaque content equality, no epoch conversion'}


def audit_boundaries(events,proof,control):
    def read(key):
        with (Path(control)/key).open('rb') as stream:return stream.read(129)
    return audit_boundary_bytes(events,proof,read)
