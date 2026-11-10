#!/usr/bin/env python3
"""Generate one ordinary MOOS community, then supervise only its children."""
import argparse
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import re
import shlex
import signal
import socket
import subprocess
import sys
import time

HERE = Path(__file__).resolve().parent
SOURCE = HERE.parents[1]


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise ValueError('helper unavailable: '+str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def port(value):
    if not re.fullmatch('[0-9]{1,5}', value) or not 1 <= int(value) <= 65535:
        raise argparse.ArgumentTypeError('port must be an integer in 1..65535')
    return int(value)


def safe_value(value):
    # These are nsplug values inside a MOOS config, not executable shell text.
    if not value or any(c in value for c in '\n\r\0{}#$'):
        raise ValueError('unsafe/empty MOOS substitution')
    return value


def arguments(argv=None):
    parser = argparse.ArgumentParser(description='Receive-only external observer; one local MOOSDB, no helm/GUI/FC. Fixed timewarp1.')
    parser.add_argument('--build', required=True, help='fresh matching host build containing bin/iMAVLink')
    parser.add_argument('--module', required=True, help='independently compiled mavmoos_example_heartbeat_observer.so/dylib')
    parser.add_argument('--moos-root', required=True, help='built stock MOOS-IvP checkout')
    parser.add_argument('--mport', '--veh_mport', type=port, default=19121)
    parser.add_argument('--bind_port', '--mav-bind-port', type=port, default=14621)
    parser.add_argument('--peer_port', '--peer-port', type=port, default=14620)
    parser.add_argument('--sample_timeout', default='0.5', help='example setting in seconds; default0.5, host validates batch')
    parser.add_argument('--publish_interval', default='0.1', help='example setting in seconds; default0.1, host validates batch')
    parser.add_argument('--setting', action='append', default=[], metavar='plugin.OWNER.KEY=VALUE',
                        help='additional checked module startup setting; repeatable, app validates complete batch')
    parser.add_argument('--work-dir', default=str(HERE), help='generated target directory; default mission directory')
    parser.add_argument('--duration', type=float, default=12, help='bounded demo duration8..30seconds (default12)')
    parser.add_argument('--just_make', '-j', action='store_true', help='strictly generate target; no sockets/processes')
    parser.add_argument('--auto', '--xlaunched', '-x', action='store_true', help='foreground bounded supervision without uMAC')
    parser.add_argument('--nogui', '-ng', action='store_true', help='accepted compatibility flag; this mission has no GUI')
    parser.add_argument('--defaults', action='store_true', help='omit module settings and demonstrate module2s/.25s defaults')
    parser.add_argument('timewarp', nargs='?', default='1', choices=['1'])
    return parser.parse_args(argv)


def generate(args):
    build, module, moos, work = [Path(v).expanduser().resolve() for v in (args.build,args.module,args.moos_root,args.work_dir)]
    if not math.isfinite(args.duration) or not 8 <= args.duration <= 30:
        raise ValueError('duration outside8..30seconds')
    if len({args.mport,args.bind_port,args.peer_port}) != 3:
        raise ValueError('MOOS, app and peer ports must be distinct')
    if module.name not in {'mavmoos_example_heartbeat_observer.so','mavmoos_example_heartbeat_observer.dylib'}:
        raise ValueError('module must have the loader\'s canonical example filename')
    required = [build/'bin/iMAVLink',module,moos/'bin/nsplug',moos/'bin/pAntler',moos/'bin/MOOSDB']
    if not args.auto and not args.just_make:
        required.append(moos/'bin/uMAC')
    for path in required:
        if not path.is_file():
            raise ValueError('required artifact missing: '+str(path))
    # pAntler's Run grammar is whitespace-delimited; fail clearly rather than
    # emit a target which silently chooses a different executable.
    if any(c.isspace() for c in str(build/'bin/iMAVLink')):
        raise ValueError('pAntler app path must not contain whitespace; use a build path without spaces')
    values = {'MOOS_PORT':str(args.mport),'BIND_PORT':str(args.bind_port),'PEER_PORT':str(args.peer_port),
              'APP_PATH':str(build/'bin/iMAVLink'),'PLUGIN_DIRECTORY':str(module.parent),
              'SAMPLE_TIMEOUT':args.sample_timeout,'PUBLISH_INTERVAL':args.publish_interval}
    for value in values.values():
        safe_value(value)
    for setting in args.setting:
        key,separator,value = setting.partition('=')
        if not separator or len(key)>256 or not re.fullmatch(r'plugin\.[A-Za-z0-9_]+\.[A-Za-z0-9_.]+',key):
            raise ValueError('additional setting must be plugin.OWNER.KEY=VALUE')
        safe_value(value)
    work.mkdir(parents=True,exist_ok=True)
    target = work/'targ_external_observer.moos'
    command = [str(moos/'bin/nsplug'),str(HERE/'meta_observer.moos'),str(target),'--strict','--force','-x']
    command += [key+'='+value for key,value in values.items()]
    subprocess.run(command,check=True,timeout=5)
    if args.defaults:
        target.write_text(''.join(line for line in target.read_text().splitlines(True)
                                  if not line.strip().startswith('plugin.example_heartbeat_observer.')))
    if args.setting:
        text=target.read_text()
        head,separator,tail=text.rpartition('}')
        if not separator or tail.strip():
            raise ValueError('generated module config block unavailable')
        target.write_text(head+''.join('  '+setting+'\n' for setting in args.setting)+'}\n'+tail)
    if '$(' in target.read_text():
        raise ValueError('unresolved mission substitution')
    return build,module,moos,work,target,command


def clear_ports(args):
    checks = {}
    for name,kind,value in (('moos',socket.SOCK_STREAM,args.mport),('app',socket.SOCK_DGRAM,args.bind_port),('peer',socket.SOCK_DGRAM,args.peer_port)):
        try:
            with socket.socket(socket.AF_INET,kind) as probe:
                if kind == socket.SOCK_STREAM:
                    probe.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
                probe.bind(('127.0.0.1',value))
        except OSError:
            checks[name] = False
        else:
            checks[name] = True
    return checks


def app_roster(records,target,app):
    """Actual pAntler descendants, scoped to this exact generated mission."""
    found = {}
    for row in records:
        try:
            tokens = shlex.split(row['command'])
        except ValueError:
            continue
        if not tokens or str(target) not in tokens:
            continue
        name=Path(tokens[0]).name
        if name not in {'MOOSDB','iMAVLink'}:
            continue
        if name in found or (name=='iMAVLink' and tokens[0]!=str(app)):
            raise RuntimeError('duplicate/unexpected exact mission app identity')
        found[name]={'pid':row['pid'],'command':row['command']}
    return found


def supervise(args, generated):
    build,module,moos,work,target,generation = generated
    ownership = load(SOURCE/'tests/owned_processes.py','_external_observer_owned')
    peer_module = load(HERE/'fake_peer.py','_external_observer_peer')
    out = work/'logs'/f'run-{time.time_ns()}-{os.getpid()}'
    out.mkdir(parents=True,exist_ok=False)
    before = {str(p):sha(p) for p in (build/'bin/iMAVLink',module,moos/'bin/pAntler',moos/'bin/MOOSDB',target,HERE/'fake_peer.py',HERE/'supervise.py',SOURCE/'tests/owned_processes.py')}
    result = {'schema':1,'result':'FAIL','scope':'local fake-heartbeat receive-only MOOS demonstration; no vehicle',
              'hardware':False,'simulator_used':False,'timewarp':1,'runner_pid':os.getpid(),
              'mission':str(target),'generation_command':generation,'ports':{'moos':args.mport,'app':args.bind_port,'peer':args.peer_port},
              'app_sha256':sha(build/'bin/iMAVLink'),'module_sha256':sha(module),'inputs_before':before,
              'duration_s':args.duration,'defaults':args.defaults,'automatic':args.auto,
              'cleanup_proven':False,'owned_processes_remaining':None}
    processes,streams,peer = [],[],None
    interrupted = False
    old_handlers = {}
    def interrupt(signum, frame):
        nonlocal interrupted
        interrupted = True
    for value in (signal.SIGINT,signal.SIGTERM):
        old_handlers[value] = signal.signal(value,interrupt)
    try:
        if not all(clear_ports(args).values()):
            raise RuntimeError('selected local port unavailable before launch')
        peer = peer_module.HeartbeatPeer(args.peer_port,args.bind_port,out,args.duration)
        env = dict(os.environ)
        env['PATH'] = os.pathsep.join([str(build/'bin'),str(moos/'bin'),env.get('PATH','')])
        log = (out/'pantler.log').open('x');streams.append(log)
        antler = subprocess.Popen([str(moos/'bin/pAntler'),str(target)],cwd=work,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append(antler)
        # Observe the actual roster before the finite source script, rather
        # than assuming pAntler's survival proves that its apps stayed alive.
        startup_deadline=time.monotonic()+3
        owned_roster={}
        while time.monotonic()<startup_deadline:
            if antler.poll() is not None:
                raise RuntimeError('owned pAntler exited during startup')
            records=ownership.owned_records(antler)
            owned_roster=app_roster(records,target,build/'bin/iMAVLink')
            if set(owned_roster)=={'MOOSDB','iMAVLink'}:
                break
            time.sleep(.05)
        else:
            raise RuntimeError('actual MOOSDB/iMAVLink startup roster deadline')
        result['owned_app_roster']=owned_roster
        time.sleep(.5)  # subscriber registration only; never an observation stamp
        peer.start()
        if not args.auto:
            console = subprocess.Popen([str(moos/'bin/uMAC'),str(target)],cwd=work,env=env,start_new_session=True)
            processes.append(console)
        result['processes'] = [{'pid':p.pid,'command':p.args} for p in processes]
        result['peer_started_mono'] = peer.started
        started,next_inventory = time.monotonic(),0.
        with (out/'owned-processes.jsonl').open('x') as inventory:
            while time.monotonic()-started < args.duration and not interrupted:
                peer.check()
                if antler.poll() is not None:
                    raise RuntimeError('owned pAntler exited before demo completed')
                if not args.auto and console.poll() is not None:
                    result['console_exit'] = console.returncode
                    if console.returncode:
                        raise RuntimeError('owned uMAC exited unsuccessfully')
                    break
                at = time.monotonic()
                if at >= next_inventory:
                    records = ownership.owned_records(antler)
                    if app_roster(records,target,build/'bin/iMAVLink')!=owned_roster:
                        raise RuntimeError('owned MOOSDB/iMAVLink roster changed/exited')
                    inventory.write(json.dumps({'mono':time.monotonic(),'records':records})+'\n');inventory.flush()
                    result['last_owned_records'] = records
                    next_inventory = time.monotonic()+.5
                time.sleep(.01)
        peer.check()
        result['elapsed_s'] = time.monotonic()-started
        result['interrupted'] = interrupted
        result['result'] = 'PASS'
    except BaseException as error:
        result['error'] = f'{type(error).__name__}: {error}'
    finally:
        remaining,cleanup_errors = [],[]
        for process in reversed(processes):
            try:
                answer = ownership.stop_process_tree(process)
                if not isinstance(answer,list):
                    raise RuntimeError('owned cleanup returned unknown inventory')
                remaining.extend(answer)
            except BaseException as error:
                cleanup_errors.append(f'{type(error).__name__}: {error}')
        if peer:
            try:
                peer.close()
            except BaseException as error:
                cleanup_errors.append(f'{type(error).__name__}: {error}')
            result['peer_capture'] = peer.status()
        for stream in streams:
            stream.close()
        result['cleanup_errors'] = cleanup_errors
        result['owned_processes_remaining'] = None if cleanup_errors else remaining
        result['ports_clear_after'] = clear_ports(args)
        result['cleanup_proven'] = not cleanup_errors and not remaining and all(result['ports_clear_after'].values())
        try:
            result['inputs_after'] = {p:sha(p) for p in before}
            result['input_drift'] = [p for p,v in before.items() if result['inputs_after'][p] != v]
        except BaseException as error:
            result['input_drift'] = ['input read failed: '+str(error)]
        if not result['cleanup_proven'] or result['input_drift'] or result.get('peer_capture',{}).get('received_datagrams',1):
            result['result'] = 'FAIL'
        result['files'] = {p.name:sha(p) for p in out.iterdir() if p.is_file() and p.name!='result.json'}
        (out/'result.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
        for value,handler in old_handlers.items():
            signal.signal(value,handler)
    print('External observer evidence: '+str(out/'result.json'),flush=True)
    return 0 if result['result']=='PASS' else 1


def main(argv=None):
    args = arguments(argv)
    generated = generate(args)
    print('Generated '+str(generated[4]),flush=True)
    if args.just_make:
        return 0
    return supervise(args,generated)


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (ValueError,OSError,subprocess.SubprocessError) as error:
        print('External observer: '+str(error),file=sys.stderr)
        raise SystemExit(1)
