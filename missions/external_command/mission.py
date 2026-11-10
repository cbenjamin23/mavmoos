#!/usr/bin/env python3
"""Generate an ordinary local mission and supervise only its pAntler children."""
import argparse
import importlib.util
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys

HERE = Path(__file__).resolve().parent
SOURCE = HERE.parents[1]


def port(value):
    if not re.fullmatch(r'[0-9]{1,5}', value) or not 1 <= int(value) <= 65535:
        raise argparse.ArgumentTypeError('port must be an integer in 1..65535')
    return int(value)


def canonical_module(path):
    return path.name in {'mavmoos_example_disarmed_command.so', 'mavmoos_example_disarmed_command.dylib'}


def generate(*, build, module, moos, work, db, bind, peer, mode, pid, request_source):
    build, module, moos, work = [Path(path).expanduser().resolve() for path in (build, module, moos, work)]
    app, backend = build / 'bin/iMAVLink', build / 'plugins' / ('mavmoos_arducopter_control' + module.suffix)
    if not canonical_module(module):
        raise ValueError('example must use its canonical loadable module filename')
    if any(type(p) is not int or not 1 <= p <= 65535 for p in (db, bind, peer)) or len({db, bind, peer}) != 3:
        raise ValueError('distinct valid MOOS, bind and peer ports required')
    if (mode not in {'rx', 'shadow', 'sitl'} or type(pid) is not int or
        (mode == 'sitl' and not 1 <= pid <= 2147483647) or (mode != 'sitl' and pid != 0)):
        raise ValueError('only explicit SITL mode accepts a positive native Copter PID')
    if not re.fullmatch(r'[A-Za-z0-9_-]{1,48}', request_source):
        raise ValueError('request source must be a case-sensitive MOOS token')
    required = [app, backend, module, *(moos / 'bin' / name for name in ('MOOSDB', 'pAntler', 'nsplug'))]
    if any(not path.is_file() for path in required):
        raise ValueError('matching app, backend, example and stock MOOS tools are required')
    plugins = work / 'plugins'
    target = work / 'targ_external_command.moos'
    for path in (app, plugins):
        if any(c.isspace() or c in '{}#$"' for c in str(path)):
            raise ValueError('use app and work paths without whitespace or MOOS syntax characters')
    work.mkdir(parents=True, exist_ok=True)
    if target.exists():
        raise ValueError('use a fresh generated mission directory')
    plugins.mkdir()
    for path in (backend, module):
        shutil.copyfile(path, plugins / path.name)
    macros = {
        'APP_PATH': app, 'PLUGIN_DIRECTORY': plugins, 'MOOS_PORT': db, 'BIND_PORT': bind,
        'PEER_PORT': peer, 'SIMULATION_ONLY': 'true' if mode == 'sitl' else 'false',
        'SHADOW_MODE': 'true' if mode == 'shadow' else 'false', 'SITL_PID': pid,
        'REQUEST_SOURCE': request_source,
    }
    argv = [str(moos / 'bin/nsplug'), str(HERE / 'meta_command.moos'), str(target),
            '--strict', '--force', '-x', *(f'{key}={value}' for key, value in macros.items())]
    generated = subprocess.run(argv, capture_output=True, text=True, timeout=10)
    (work / 'nsplug.log').write_text(generated.stdout + generated.stderr)
    if generated.returncode or not target.is_file() or '$(' in target.read_text():
        raise RuntimeError('strict external command mission generation failed')
    return target, plugins, argv


def run(args):
    target, _, _ = generate(build=args.build, module=args.module, moos=args.moos_root, work=args.work_dir,
                             db=args.mport, bind=args.bind_port, peer=args.peer_port, mode=args.mode,
                             pid=args.sitl_pid, request_source=args.request_source)
    print(target, flush=True)
    if args.just_make:
        return 0
    moos_bin = Path(args.moos_root).expanduser().resolve() / 'bin'
    spec = importlib.util.spec_from_file_location('_external_command_ownership', SOURCE / 'tests/owned_processes.py')
    ownership = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(ownership)
    env = dict(os.environ, PATH=os.pathsep.join((str(moos_bin), os.environ.get('PATH', ''))))
    process = None
    status = 1
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    try:
        process = subprocess.Popen([str(moos_bin / 'pAntler'), str(target)], cwd=target.parent,
                                   env=env, start_new_session=True)
        status = process.wait()
    except KeyboardInterrupt:
        status = 0
    finally:
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        if process is not None:
            remaining = ownership.stop_process_tree(process)
            if type(remaining) is not list or remaining:
                raise RuntimeError('scoped mission cleanup is unresolved')
    return status


def parser():
    result = argparse.ArgumentParser(description=__doc__)
    for name in ('build', 'module', 'moos-root', 'work-dir'):
        result.add_argument('--' + name, required=True)
    result.add_argument('--mport', type=port, default=19131)
    result.add_argument('--bind-port', type=port, default=14631)
    result.add_argument('--peer-port', type=port, default=14630)
    result.add_argument('--mode', choices=('rx', 'shadow', 'sitl'), default='rx')
    result.add_argument('--sitl-pid', type=int, default=0)
    result.add_argument('--request-source', default='uPokeDB')
    result.add_argument('--just_make', '-j', action='store_true', help='strict generation only')
    result.add_argument('--nogui', action='store_true', help='this mission always runs headless')
    result.add_argument('timewarp', nargs='?', choices=('1',), default='1')
    return result


if __name__ == '__main__':
    try:
        raise SystemExit(run(parser().parse_args()))
    except (ValueError, OSError, RuntimeError, subprocess.SubprocessError) as error:
        print(str(error), file=sys.stderr)
        raise SystemExit(1)
