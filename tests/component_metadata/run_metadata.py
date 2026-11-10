#!/usr/bin/env python3
"""Prove the selected component metadata mirror through real iMAVLink and MOOS."""
import argparse
import errno
import json
import os
from pathlib import Path
import socket
import struct
import subprocess
import sys
import tempfile
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from component_metadata.ComponentMetadataTests import crc
from external_command.evidence import crc_x25
from observer_matrix.run_loopback import audit, complete_capture, reserve, start, take_egress
from owned_processes import stop_process_tree
from plane_native.flight_fixture import need, sha

KEY = 'MAVMOOS_COMPONENT_METADATA'
COMPONENT = 196
URI = b'mftp:///component_metadata/general.json'
MODULES = ('telemetry', 'component_metadata')


class Witness:
    def __init__(self, out, peer, bridge, probe, log):
        self.out, self.peer, self.bridge, self.probe, self.log = out, peer, bridge, probe, log
        self.sequence = 0
        self.datagrams = 0
        self.egress = 0

    def send(self, mid, extra, data, component, stream):
        payload = data.rstrip(b'\0') or b'\0'
        header = bytes((len(payload), 0, 0, self.sequence, 1, component)) + mid.to_bytes(3, 'little')
        frame = b'\xfd' + header + payload + struct.pack('<H', crc_x25(header + payload + bytes((extra,))))
        self.sequence = (self.sequence + 1) % 256
        need(self.peer.sendto(frame, ('127.0.0.1', self.bridge)) == len(frame), 'incomplete fixture write')
        stream.write(frame)
        self.datagrams += 1

    def capture(self, phase, condition, *, heartbeat=True, boot=None, checksum=0, component=COMPONENT,
                hold=.25):
        prefix = b'CAPTURE\t' + KEY.encode() + b'\t'
        initial = len(complete_capture(self.log))
        with (self.out / 'input.mavlink').open('ab') as wire, (self.out / 'egress.mavlink').open('ab') as egress:
            started = time.monotonic()
            while time.monotonic()-started<hold:
                need(self.probe.poll() is None, 'observation probe exited')
                if heartbeat:
                    self.send(0, 50, struct.pack('<IBBBBB', 0, 2, 3, 1, 3, 3), 1, wire)
                if boot is not None:
                    self.send(397, 182, struct.pack('<II100s', boot, checksum, URI), component, wire)
                self.egress += take_egress(self.peer, self.bridge, egress)
                time.sleep(.075)
            deadline = time.monotonic()+1.2
            while time.monotonic()<deadline:
                lines = complete_capture(self.log)[initial:].splitlines()
                rows = [json.loads(line[len(prefix):]) for line in lines if line.startswith(prefix)]
                if rows and condition(rows[-1]):
                    (self.out / f'{phase}.json').write_text(json.dumps(rows[-1], indent=2)+'\n')
                    return rows[-1]
                self.egress += take_egress(self.peer, self.bridge, egress)
                time.sleep(.025)
        raise ValueError(f'{phase}: matching metadata observation missing')


def document(directory, version):
    value = {'version': 1, 'metadataTypes': [], 'vendorName': 'MAVMOOS Fixture',
             'modelName': 'Component Lab', 'firmwareVersion': version}
    body = json.dumps(value, separators=(',', ':')).encode()
    temp = directory/'general.next'
    temp.write_bytes(body)
    temp.chmod(0o600)
    temp.replace(directory/'general.json')
    return crc(body)


def run(args, out):
    build, moos = args.build.resolve(), args.moos_bin.resolve()
    report = {'result':'FAIL', 'scope':'owned generated SITL component through actual iMAVLink',
              'native_firmware':False}
    processes = []
    ports = {}
    try:
        with reserve(socket.SOCK_DGRAM) as peer, \
                tempfile.TemporaryDirectory(prefix='mavmoos-metadata-mirror.') as private:
            peer.setblocking(False)
            with reserve(socket.SOCK_STREAM) as tcp, reserve(socket.SOCK_DGRAM) as udp:
                ports = {'db':tcp.getsockname()[1], 'bridge':udp.getsockname()[1],
                         'peer':peer.getsockname()[1]}
            mirror = Path(private).resolve()
            a = document(mirror, '1.0.0')
            (out/'initial-general.json').write_bytes((mirror/'general.json').read_bytes())
            mission = out/'metadata.moos'
            mission.write_text(f'''ServerHost = 127.0.0.1
ServerPort = {ports['db']}
Community = component_metadata_{ports['db']}
MOOSTimeWarp = 1
TERM_REPORTING = true
ProcessConfig = ANTLER
{{
  MSBetweenLaunches = 1200
  Run = MOOSDB @ NewConsole = false
  Run = iMAVLink @ NewConsole = false
}}
ProcessConfig = iMAVLink
{{
  AppTick = 40
  CommsTick = 40
  bind_address = 127.0.0.1
  bind_port = {ports['bridge']}
  peer_address = 127.0.0.1
  peer_port = {ports['peer']}
  target_system = 1
  target_component = 1
  expected_type = 2
  expected_autopilot = 3
  plugins = telemetry,component_metadata
  component_metadata_component = {COMPONENT}
  plugin.component_metadata.mirror_directory = {mirror}
  plugin.component_metadata.sample_timeout = 0.55
}}
''')
            env = {k:v for k,v in os.environ.items() if not k.startswith(('LD_', 'DYLD_', 'MOOS'))}
            env['PATH'] = os.pathsep.join((str(build/'bin'), str(moos), '/usr/bin', '/bin'))
            antler = start([str(moos/'pAntler'), str(mission)], out, env, 'pantler.log')
            processes.append(antler)
            log = out/'probe.log'
            probe = start([str(args.probe), str(ports['db']), '20', '--watch='+KEY], out, env, 'probe.log')
            processes.append(probe)
            deadline = time.monotonic()+6
            while b'CAPTURE\tMAVMOOS_STATUS\t' not in complete_capture(log):
                need(time.monotonic()<deadline and antler.poll() is None and probe.poll() is None,
                     'actual app/probe startup deadline')
                time.sleep(.05)
            need(b"state=CONFIG_ERROR" not in complete_capture(log), 'metadata app startup rejected configuration')
            witness = Witness(out, peer, ports['bridge'], probe, log)
            wait = lambda row: row['verification']=='waiting_declaration' and row['document'] is None
            witness.capture('prebind', wait, heartbeat=False, boot=100, checksum=a)
            witness.capture('foreign_component', wait, boot=110, checksum=a, component=197)
            first = witness.capture('verified', lambda r: r['verification']=='verified' and
                r['document']['firmwareVersion']=='1.0.0', boot=120, checksum=a)
            stale = witness.capture('repeat_expired', lambda r: r['verification']=='stale_declaration' and
                r['document'] is None, boot=120, checksum=a, hold=.78)
            need(stale['metadata']['effective_at']==first['metadata']['effective_at'],
                 'duplicate producer stamp changed original age')
            b = document(mirror, '1.0.1')
            (out/'updated-general.json').write_bytes((mirror/'general.json').read_bytes())
            witness.capture('checksum_mismatch', lambda r: r['verification']=='checksum_mismatch' and
                r['document'] is None, boot=130, checksum=a)
            second = witness.capture('updated', lambda r: r['verification']=='verified' and
                r['document']['firmwareVersion']=='1.0.1', boot=140, checksum=b)
            witness.capture('cleared', wait, heartbeat=False, hold=2.25)
            samples, modules = audit(log, MODULES, build/'plugins')
            need(samples.get(KEY) and witness.egress==0, 'passive receipt or independent zero egress missing')
            report.update(result='PASS', modules=modules, component=COMPONENT, input_datagrams=witness.datagrams,
                          bridge_egress_datagrams=witness.egress, ports=ports,
                          app_sha256=sha(build/'bin/iMAVLink'), source_revision=subprocess.check_output(
                              ['git','-C',str(Path(__file__).resolve().parents[2]),'rev-parse','HEAD'], text=True).strip(),
                          initial_crc32=a, updated_crc32=b, first_receipt=first, second_receipt=second)
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as error:
        report.update(result='FAIL', error=str(error))
    finally:
        remaining=[]
        for process in reversed(processes):
            remaining.extend(stop_process_tree(process))
        report['owned_processes_remaining']=remaining
        clear={}
        for name, port in ports.items():
            kind=socket.SOCK_STREAM if name=='db' else socket.SOCK_DGRAM
            with socket.socket(socket.AF_INET,kind) as endpoint:
                if name=='db':
                    endpoint.settimeout(.2)
                    clear[name]=endpoint.connect_ex(('127.0.0.1',port))==errno.ECONNREFUSED
                else:
                    try:
                        endpoint.bind(('127.0.0.1',port))
                        clear[name]=True
                    except OSError:
                        clear[name]=False
        report['ports_clear']=clear
        if remaining or not all(clear.values()):
            report.update(result='FAIL', error='owned cleanup failed')
        (out/'result.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({'result':report['result'], 'output':str(out), 'error':report.get('error')}))
    return report['result']!='PASS'


def main():
    parser=argparse.ArgumentParser()
    for name in ('build','moos-bin','probe','output'):
        parser.add_argument('--'+name, type=Path, required=True)
    args=parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    return run(args, Path(tempfile.mkdtemp(prefix='metadata.', dir=args.output)).resolve())


if __name__=='__main__':
    raise SystemExit(main())
