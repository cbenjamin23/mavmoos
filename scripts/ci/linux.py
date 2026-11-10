#!/usr/bin/env python3
"""Source-built local Linux CI using the existing owned Docker recipe.

Caller supplies existing dependency checkouts. This wrapper creates no remote,
downloads no repository and activates no hosted workflow. The underlying image
build may retrieve pinned-version build dependencies and records their inventory.
"""
import argparse
import json
from pathlib import Path
import re
import signal
import subprocess
import sys

from run import CIInterrupted, ROOT, accepted, bounded, dump, parse, require
from isolation import DOMAIN, LinuxDomainLease, owned_container


def cleanup_container(args):
    """Recover only this recipe's labelled container after abnormal termination."""
    evidence = args.output / 'linux/evidence'
    owner_path = evidence / 'owned-container.json'
    if not owner_path.is_file():
        return {'result': 'NOT_CREATED', 'scope': 'no pre-create owned-container declaration'}
    owner = parse(owner_path.read_text())
    # Validate the producer's name/label before even querying Docker.
    require(owner.get('schema') == 1 and re.fullmatch(r'mavmoos-linux-\d+', owner.get('name', '')) and
            owner.get('label_key') == 'mavmoos.ci.owner' and owner.get('label_value') == owner['name'],
            'invalid owned-container cleanup declaration')
    commands = []
    def call(argv):
        log = args.output / ('container-postflight-' + str(len(commands) + 1) + '.log')
        result = bounded(argv, args.source_root, log, 15)
        commands.append(result)
        return result, log.read_text()
    command = [args.docker, 'ps', '--no-trunc', '-aq', '--filter', 'name=^/' + owner['name'] + '$']
    outcome, text = call(command)
    require(accepted(outcome), 'owned Docker cleanup inventory failed')
    if not text.strip():
        return {'result': 'PASS', 'already_absent': True, 'commands': commands}
    outcome, text = call([args.docker, 'inspect', owner['name']])
    require(accepted(outcome), 'owned Docker identity inspect failed')
    identity = owned_container(owner, parse(text))
    call([args.docker, 'stop', '--time', '3', identity])
    call([args.docker, 'rm', identity])
    outcome, text = call(command)
    require(accepted(outcome) and not text.strip(), 'owned Docker container remains after cleanup')
    return {'result': 'PASS', 'recovered_after_abnormal_exit': True, 'container_id': identity,
            'name': owner['name'], 'commands': commands}


def recipe_command(args):
    require(args.tier in ('software', 'smoke', 'full'), 'invalid Linux CI tier')
    if args.tier != 'software':
        require(args.ardupilot_source is not None and args.source_ref and args.firmware_manifest,
                'native source-built CI requires --ardupilot-source, --source-ref and --firmware-manifest')
    command = [sys.executable, '-B', str(args.source_root / 'scripts/linux_reproduce.py'),
               '--moos-source', str(args.moos_source), '--uav-source', str(args.uav_source),
               '--output', str(args.output / 'linux'), '--docker', args.docker, '--ci-tier', args.tier]
    if getattr(args, 'isolated_native_domain', False):
        require(args.tier in ('smoke', 'full') and args.mavlink_archive is not None and
                isinstance(args.source_ref, str) and re.fullmatch(r'[0-9a-f]{40}', args.source_ref) and
                isinstance(args.image, str) and re.fullmatch(r'sha256:[0-9a-f]{64}', args.image),
                'isolated Linux domain requires native tier, full immutable source/image and offline MAVLink archive')
        command += ['--isolated-native-domain']
    for name, option in [('ardupilot_source', '--ardupilot-source'), ('source_ref', '--source-ref'),
                         ('firmware_manifest', '--ci-firmware-manifest'), ('image', '--image'),
                         ('mavlink_archive', '--mavlink-archive')]:
        value = getattr(args, name)
        if value is not None:
            command += [option, str(value)]
    return command


def main():
    def interrupted(number, _frame):
        raise CIInterrupted(number, 'Linux CI')
    signal.signal(signal.SIGTERM, interrupted)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-root', type=Path, default=ROOT)
    for key in ('moos-source', 'uav-source', 'output'):
        parser.add_argument('--' + key, type=Path, required=True)
    parser.add_argument('--tier', choices=['software', 'smoke', 'full'], required=True)
    parser.add_argument('--ardupilot-source', type=Path)
    parser.add_argument('--firmware-manifest', help='Path relative to the selected MAVMOOS source snapshot')
    parser.add_argument('--source-ref')
    parser.add_argument('--image')
    parser.add_argument('--mavlink-archive', type=Path)
    parser.add_argument('--docker', default='docker')
    parser.add_argument('--max-time', type=int, default=10800)
    parser.add_argument('--isolated-native-domain', action='store_true',
                        help='Hold the fixed Linux-domain host lease; enforce resource/isolation proof before offline app/native execution')
    args = parser.parse_args()
    require(1 <= args.max_time <= 14400, 'invalid source-build time budget')
    for name in ('source_root', 'moos_source', 'uav_source', 'output', 'ardupilot_source', 'mavlink_archive'):
        value = getattr(args, name)
        if value is not None:
            setattr(args, name, value.resolve())
    args.output.mkdir(parents=True, exist_ok=False)
    report = {'schema': 1, 'result': 'FAIL', 'tier': args.tier, 'hosted': False,
              'scope': 'owned local source-built Linux CI; not physical or hosted qualification'}
    lease = None
    try:
        command = recipe_command(args)
        if args.isolated_native_domain:
            lease = LinuxDomainLease(args.output / 'domain-lease.json', args.source_ref)
            lease.acquire()
            report['execution_domain'] = DOMAIN
        # Give host-wide inventory more time during the long Docker recipe
        # without extending the recipe's own deadline.
        report['command'] = bounded(command, args.source_root,
                                    args.output / 'linux-reproduce.log', args.max_time,
                                    inventory_timeout=15)
        require(accepted(report['command']), 'Linux recipe failed, timed out or leaked descendants')
        result = parse((args.output / 'linux/evidence/result.json').read_text())
        require(result['result'] == 'PASS' and result['container_removed'] is True and
                result['retained_container'] is False, 'owned Linux container completion/cleanup failed')
        if args.isolated_native_domain:
            require(result.get('execution_domain') == DOMAIN and result.get('isolation_before', {}).get('result') == 'PASS' and
                    result.get('isolation_after', {}).get('result') == 'PASS', 'isolated Linux pre/postflight proof absent')
        ci = parse((args.output / 'linux/evidence/ci/result.json').read_text())
        require(ci['result'] == 'PASS' and ci['tier'] == args.tier and ci['cases'],
                'source-built local CI is absent/failed/wrong tier')
        report['source_build'] = result
        report['case_count'] = len(ci['cases'])
        report['native_case_count'] = len(ci['native_evidence'])
        report['result'] = 'PASS'
    except CIInterrupted as error:
        report['interrupted'] = True
        report['error'] = str(error)
        raise
    except (OSError, ValueError, KeyError, TypeError, subprocess.SubprocessError) as error:
        report['error'] = str(error)
    finally:
        try:
            source_report = args.output / 'linux/evidence/result.json'
            if source_report.is_file() and parse(source_report.read_text()).get('container_removed') is True:
                report['container_cleanup'] = {'result': 'PASS', 'scope': 'recipe recorded explicit stop/remove/absence proof'}
            else:
                report['container_cleanup'] = cleanup_container(args)
        except (OSError, ValueError, KeyError, TypeError, subprocess.SubprocessError) as error:
            report['result'] = 'FAIL'
            report['container_cleanup_error'] = str(error)
            report.setdefault('error', 'owned Docker cleanup not proven')
        if lease is not None:
            try:
                lease.release()
                report['domain_lease'] = lease.record
            except (OSError, ValueError) as error:
                report['result'] = 'FAIL'
                report['domain_lease_error'] = str(error)
                report.setdefault('error', 'Linux-domain lease release not proven')
        dump(args.output / 'result.json', report)
    print(json.dumps({'result': report['result'], 'tier': args.tier, 'error': report.get('error'),
                      'output': str(args.output)}, indent=2))
    return 0 if report['result'] == 'PASS' else 1


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except CIInterrupted as error:
        print(json.dumps({'result': 'FAIL', 'error': str(error), 'interrupted': True}), file=sys.stderr)
        raise SystemExit(128 + error.number)
