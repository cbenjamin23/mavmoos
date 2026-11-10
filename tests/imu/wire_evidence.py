"""Bounded passive byte decoder/replay; no sockets, processes or MAVLink writes."""
import math


def safe(value):
    if isinstance(value,float) and not math.isfinite(value):return {'nonfinite':str(value)}
    if isinstance(value,bytes):return {'raw_hex':value.hex()}
    if isinstance(value,dict):return {key:safe(item) for key,item in value.items()}
    if isinstance(value,(list,tuple)):return [safe(item) for item in value]
    return value


class FrameDecoder:
    def __init__(self,decoder,max_frames=20000,expected_type=2):
        if type(max_frames) is not int or not 1<=max_frames<=20000:raise ValueError('passive decoded-frame bound invalid')
        self.decoder=decoder;decoder.robust_parsing=True
        self.max_frames=max_frames;self.expected_type=expected_type
        self.startup_bytes=0;self.frames=0;self.input_bytes=0;self.consumed_bytes=0;self.last_read=-1

    def feed(self,data,at):
        if not isinstance(data,bytes) or not data:raise ValueError('empty/nonbyte passive datagram')
        if type(at) not in (int,float) or not math.isfinite(at) or at<0 or at<self.last_read:raise ValueError('passive actual read clock invalid/regressed')
        self.last_read=at;self.input_bytes+=len(data);rows=[];startup=[]
        for message in self.decoder.parse_buffer(data) or []:
            if message.get_type()=='BAD_DATA':
                raw=bytes(message.data);prefix=b'0 '*3
                if self.frames or not raw or self.startup_bytes+len(raw)>6 or raw!=prefix[self.startup_bytes:self.startup_bytes+len(raw)] or message.reason!='Bad prefix':raise ValueError('invalid passive startup/frame bytes')
                self.startup_bytes+=len(raw);self.consumed_bytes+=len(raw)
                startup.append({'mono':at,'raw_hex':raw.hex(),'reason':message.reason,'scope':'finite native SiK startup bytes, not observation evidence'})
                continue
            if self.startup_bytes not in (0,6):raise ValueError('incomplete passive startup prefix')
            if message.get_srcSystem()!=1 or message.get_srcComponent()!=1:raise ValueError('foreign passive native source')
            self.frames+=1
            if self.frames>self.max_frames:raise ValueError('passive decoded-frame bound exceeded')
            self.consumed_bytes+=len(message.get_msgbuf())
            row={'mono':at,'system':1,'component':1,**safe(message.to_dict())}
            if row.get('mavpackettype')=='HEARTBEAT' and (row.get('type')!=self.expected_type or row.get('autopilot')!=3 or row.get('base_mode',0)&128):raise ValueError('passive source armed/identity changed')
            rows.append(row)
        return rows,startup

    def finish(self):
        if not self.frames or self.startup_bytes not in (0,6) or self.input_bytes!=self.consumed_bytes:raise ValueError('passive raw bytes contain incomplete/unconsumed frame tail')
        return {'frames':self.frames,'startup_bytes':self.startup_bytes,'raw_bytes':self.input_bytes,'consumed_bytes':self.consumed_bytes}


def replay(raw,retained,decoder_factory,expected_type=2):
    """Compare decoded fields/order; only original local read stamps are external."""
    decoder=FrameDecoder(decoder_factory(),expected_type=expected_type);rows,startup=decoder.feed(raw,0)
    summary=decoder.finish()
    expected=[{k:v for k,v in row.items() if k!='mono'} for row in retained]
    actual=[{k:v for k,v in row.items() if k!='mono'} for row in rows]
    if actual!=expected:raise ValueError('raw MAVLink bytes differ from retained decoded fields/order')
    return {'schema':1,'result':'PASS',**summary,'same_message_fields_and_order':True,'local_read_timestamps_reconstructed':False,'scope':'fresh decoder replay of closed passive MAVLink bytes; no DataFlash whole-file or physical sensor claim'}
