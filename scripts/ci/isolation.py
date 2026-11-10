#!/usr/bin/env python3
"""Fixed owned-Linux execution domain: inert inspection and host lease only.

Docker State.Pid belongs to the Linux VM and is never a macOS signal target.
The namespace probe performs local ioctl/file reads and sends no packets.
"""
from datetime import datetime, timezone
import fcntl
import ipaddress
import json
import os
from pathlib import Path
import re
import stat
import sys
import tempfile

RESOURCE_ARGS = ['--cpus', '4', '--memory', '6g', '--memory-swap', '6g', '--pids-limit', '512']
SCOPE = 'clean-linux-20261103'
DOMAIN = 'linux-container-isolated'
LOCK_NAME = 'mavmoos-native-sitl-linux-container.lock'

PROBE = r'''from pathlib import Path
import errno,fcntl,json,os,socket,struct
addresses={}
with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as local:
 for _,name in socket.if_nameindex():
  try:address=socket.inet_ntoa(fcntl.ioctl(local.fileno(),0x8915,struct.pack('256s',name.encode()[:15]))[20:24]);error=None
  except OSError as exception:address=None;error=errno.errorcode.get(exception.errno,str(exception.errno))
  addresses[name]={'flags':Path('/sys/class/net',name,'flags').read_text().strip(),'ipv4':address,'address_error':error}
status={}
for line in Path('/proc/self/status').read_text().splitlines():
 if ':' in line:
  key,value=line.split(':',1);status[key]=value.strip()
print(json.dumps({'uid':os.getuid(),'gid':os.getgid(),'interface_details':addresses,
 'ipv6_addresses':Path('/proc/net/if_inet6').read_text().splitlines(),
 'cgroup':{n:Path('/sys/fs/cgroup',n).read_text().strip() for n in ('cpu.max','memory.max','memory.swap.max','pids.max')},
 'routes4':Path('/proc/net/route').read_text().splitlines()[1:],
 'routes6':Path('/proc/net/ipv6_route').read_text().splitlines(),
 'socket_rows':{n:Path('/proc/net',n).read_text().splitlines()[1:] for n in ('udp','udp6','tcp','tcp6')},
 'namespaces':{n:os.readlink('/proc/self/ns/'+n) for n in ('pid','net','mnt','ipc','uts','cgroup')},
 'pid1_cmdline':Path('/proc/1/cmdline').read_bytes().decode().strip('\0').split('\0'),
 'cap_eff':status.get('CapEff'),'no_new_privileges':status.get('NoNewPrivs'),
 'devices':sorted(p.name for p in Path('/dev').iterdir())},indent=2))'''


def require(value, why):
    if not value:
        raise ValueError(why)


def owned_container(owner, data):
    require(owner.get('schema') == 1 and re.fullmatch(r'mavmoos-linux-\d+', owner.get('name', '')) and
            owner.get('label_key') == 'mavmoos.ci.owner' and owner.get('label_value') == owner['name'],
            'invalid owned-container declaration')
    require(isinstance(data, list) and len(data) == 1, 'owned container inspect did not return one object')
    value = data[0]
    require(re.fullmatch(r'[0-9a-f]{64}', value.get('Id', '')) and value.get('Name') == '/' + owner['name'],
            'owned container ID/name differs')
    require(owner.get('container_id') is None or owner['container_id'] == value['Id'], 'owned container ID changed')
    labels = value.get('Config', {}).get('Labels', {})
    require(labels.get('mavmoos.ci.owner') == owner['label_value'] and
            labels.get('mavmoos.scope') == SCOPE, 'owned container labels differ')
    return value['Id']


def validate_container(owner, data, image, disconnected):
    identity = owned_container(owner, data)
    require(owner.get('container_id') == identity, 'isolation requires the original full container ID')
    value, host = data[0], data[0]['HostConfig']
    require(value['Image'] == image and re.fullmatch(r'sha256:[0-9a-f]{64}', image), 'container image changed')
    require(host['NetworkMode'] == 'bridge' and host['PidMode'] in ('', 'private') and
            host['IpcMode'] in ('', 'private') and host['UTSMode'] in ('', 'private') and
            host['CgroupnsMode'] == 'private', 'shared network/PID/IPC/UTS/cgroup mode is forbidden')
    require(value['Mounts'] == [] and all(not host.get(key) for key in
            ('Binds', 'VolumesFrom', 'Devices', 'DeviceRequests', 'DeviceCgroupRules', 'PortBindings')) and
            not value['NetworkSettings']['Ports'], 'host mounts/devices/published ports are forbidden')
    require(host['Privileged'] is False and not host.get('CapAdd') and
            set(host['CapDrop']) == {'ALL'} and 'no-new-privileges' in host['SecurityOpt'] and
            value['Config']['User'] == '65534:65534', 'container privilege/UID policy differs')
    require(host['NanoCpus'] == 4000000000 and host['Memory'] == 6442450944 and
            host['MemorySwap'] == 6442450944 and host['PidsLimit'] == 512, 'fixed container budget differs')
    networks = value['NetworkSettings']['Networks']
    require(networks == {} if disconnected else set(networks) == {'bridge'}, 'effective container network differs')
    return identity


def closed_tcp_rows(rows, ipv6):
    """Permit only well-formed loopback TCP TIME_WAIT, not active sockets.

    Linux /proc/net encodes each address as native-endian 32-bit hex words.
    Both qualified Linux architectures (ARM64 and x86_64) are little endian.
    Keep the complete original rows in the probe; this returns only counts.
    """
    require(isinstance(rows, list), 'TCP socket inventory is not a list')
    digits = 32 if ipv6 else 8
    for row in rows:
        require(isinstance(row, str), 'malformed TCP row')
        fields = row.split()
        require(len(fields) == 12 and re.fullmatch(r'[0-9]+:', fields[0]) and
                all(re.fullmatch(r'[0-9A-Fa-f]{' + str(digits) + r'}:[0-9A-Fa-f]{4}', address)
                    for address in fields[1:3]) and fields[3] == '06' and
                re.fullmatch(r'[0-9A-Fa-f]{8}:[0-9A-Fa-f]{8}', fields[4]) and
                re.fullmatch(r'[0-9A-Fa-f]{2}:[0-9A-Fa-f]{8}', fields[5]) and
                re.fullmatch(r'[0-9A-Fa-f]{8}', fields[6]) and
                all(re.fullmatch(r'[0-9]+', field) for field in fields[7:11]) and fields[9] == '0' and
                re.fullmatch(r'[0-9A-Fa-f]{16}', fields[11]),
                'active/listening or malformed TCP row')
        for endpoint in fields[1:3]:
            address, port = endpoint.split(':')
            require(int(port, 16) > 0, 'TIME_WAIT endpoint has no port')
            raw = b''.join(int(address[i:i+8], 16).to_bytes(4, 'little') for i in range(0, digits, 8))
            parsed = ipaddress.ip_address(raw)
            require(parsed.is_loopback or (ipv6 and parsed.ipv4_mapped is not None and
                                          parsed.ipv4_mapped.is_loopback), 'non-loopback TIME_WAIT endpoint')
    return len(rows)


def validate_namespace(value, previous=None):
    require(value['uid'] == value['gid'] == 65534 and value['pid1_cmdline'] == ['sleep', 'infinity'] and
            int(value['cap_eff'], 16) == 0 and value['no_new_privileges'] == '1', 'namespace identity/privilege differs')
    require(value['cgroup'] == {'cpu.max': '400000 100000', 'memory.max': '6442450944',
            'memory.swap.max': '0', 'pids.max': '512'}, 'actual cgroup budget differs')
    interfaces = value['interface_details']
    require(isinstance(interfaces, dict) and 'lo' in interfaces and
            {name for name, row in interfaces.items() if int(row['flags'], 16) & 1} == {'lo'} and
            interfaces['lo']['ipv4'] == '127.0.0.1', 'only loopback may be UP/addressed')
    require(all(row['ipv4'] is None and row['address_error'] == 'EADDRNOTAVAIL'
                for name, row in interfaces.items() if name != 'lo'), 'non-loopback interface has an address')
    require(value['ipv6_addresses'] and all(row.split()[-1] == 'lo' and
            row.split()[0] == '00000000000000000000000000000001' for row in value['ipv6_addresses']),
            'non-loopback IPv6 address')
    require(value['routes4'] == [] and all(row.split()[-1] == 'lo' for row in value['routes6']),
            'non-loopback route remains')
    sockets = value['socket_rows']
    require(isinstance(sockets, dict) and set(sockets) == {'udp', 'udp6', 'tcp', 'tcp6'} and
            sockets['udp'] == sockets['udp6'] == [], 'UDP socket remains or socket inventory missing')
    time_wait = {name: closed_tcp_rows(sockets[name], name == 'tcp6') for name in ('tcp', 'tcp6')}
    require(set(value['namespaces']) == {'pid', 'net', 'mnt', 'ipc', 'uts', 'cgroup'} and
            all(re.fullmatch(name + r':\[[0-9]+\]', identity) for name, identity in value['namespaces'].items()),
            'namespace inventory missing')
    require(set(value['devices']) <= {'core', 'fd', 'full', 'mqueue', 'null', 'ptmx', 'pts', 'random',
            'shm', 'stderr', 'stdin', 'stdout', 'tty', 'urandom', 'zero'}, 'unexpected /dev exposure')
    if previous is not None:
        require(value['namespaces'] == previous['namespaces'], 'container namespace changed')
    return {'result': 'PASS', 'domain': DOMAIN,
            'network_claim': 'effective disconnected bridge: only loopback UP/addressed; dormant unaddressed kernel tunnels DOWN; no non-loopback routes, UDP sockets, active TCP or listeners; closed loopback TCP TIME_WAIT may remain; not literal network-none creation',
            'closed_tcp_time_wait_counts': time_wait, 'resources': value['cgroup']}


class LinuxDomainLease:
    """One serial owned Linux campaign; independent of the host native domain.

    The two fixed host tempfile variants prevent split ownership caused by
    macOS tempfile-directory selection. Tests may inject private paths; the CLI
    never permits a lock-path override.
    """
    def __init__(self, path, source_ref, paths=None):
        self.path, self.handles = Path(path), []
        self.paths = list(dict.fromkeys(Path(p).parent.resolve() / Path(p).name for p in (paths or
            [Path('/tmp') / LOCK_NAME, Path(tempfile.gettempdir()) / LOCK_NAME])))
        self.record = {'schema': 1, 'domain': DOMAIN, 'pid': os.getpid(), 'argv': list(sys.argv),
                       'source_ref': source_ref, 'acquired': False, 'released': False,
                       'started_utc': datetime.now(timezone.utc).isoformat(), 'locks': []}

    def save(self):
        self.path.write_text(json.dumps(self.record, indent=2) + '\n')

    def acquire(self):
        try:
            for path in self.paths:
                descriptor = os.open(path, os.O_RDWR | os.O_CREAT | getattr(os, 'O_NOFOLLOW', 0), 0o600)
                handle = os.fdopen(descriptor, 'a+')
                try:
                    details = os.fstat(handle.fileno())
                    require(stat.S_ISREG(details.st_mode) and details.st_uid == os.getuid(), 'unowned/nonregular Linux-domain lock')
                    fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
                except BaseException:
                    handle.close()
                    raise
                self.handles.append(handle)
                self.record['locks'].append({'path': str(path), 'device': details.st_dev, 'inode': details.st_ino})
            self.record['acquired'] = True
            self.save()
        except BaseException as error:
            self.record['acquire_error'] = type(error).__name__ + ': ' + str(error)
            self.release()
            raise

    def release(self):
        errors = []
        while self.handles:
            handle = self.handles.pop()
            try:
                fcntl.flock(handle, fcntl.LOCK_UN)
            except OSError as error:
                errors.append(type(error).__name__ + ': ' + str(error))
            finally:
                handle.close()
        self.record['released'] = not errors
        self.record['release_errors'] = errors
        self.record['finished_utc'] = datetime.now(timezone.utc).isoformat()
        self.save()
        require(not errors, 'Linux-domain lease release failed')
