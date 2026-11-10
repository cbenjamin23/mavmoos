#!/usr/bin/env python3
"""Internal per-native-case manifest revalidation; no socket or route exception."""
import argparse
import json
from pathlib import Path
import signal
import subprocess
import sys

from run import CIInterrupted, NATIVE_INVENTORY_TIMEOUT, accepted, bounded, dump, parse, require, sha


def main():
    def interrupted(number, _frame):
        raise CIInterrupted(number, 'native CI wrapper')
    signal.signal(signal.SIGTERM, interrupted)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True, type=Path)
    args = parser.parse_args()
    config = parse(args.config.read_text())
    output, source = Path(config['output']), Path(config['source_root'])
    report = {'schema': 1, 'id': config['id'], 'result': 'FAIL',
              'vehicle_profile': config['vehicle_profile'],
              'manifest_before': sha(config['manifest']),
              'attestation_before': sha(config['attestation']),
              'scope': 'per-case launch identity check; behavior evidence remains in child raw reports'}
    try:
        if config['vehicle_profile'] == 'arducopter':
            command = [sys.executable, '-B', str(source / 'scripts/firmware/manifest.py'),
                       'verify', '--manifest', config['manifest'], '--attestation', config['attestation'],
                       '--ardupilot-root', config['ardupilot_root'], '--expected-profile', 'arducopter']
            check = bounded(command, source, output / 'firmware-before-case.log', 60)
            report['verification'] = check
            require(accepted(check), 'native identity failed immediately before case')
            report['firmware'] = parse((output / 'firmware-before-case.log').read_text())
        else:
            require(config['vehicle_profile'] in ('ardurover', 'ardurover_boat'), 'unknown native vehicle profile')
            report['firmware_binding'] = 'separate Rover executable; native runner verifies the profile and fixture'
        command = bounded(config['command'], Path(config['working_directory']),
                          output / 'native-command.log', config['timeout_s'],
                          inventory_timeout=NATIVE_INVENTORY_TIMEOUT)
        report['command'] = command
        require(accepted(command), 'native child failed/timed out or leaked owned descendants')
        report['result'] = 'PASS'
    except CIInterrupted as error:
        report['interrupted'] = True
        report['error'] = str(error)
        raise
    except (OSError, ValueError, KeyError, TypeError, subprocess.SubprocessError) as error:
        report['error'] = str(error)
    finally:
        report['manifest_after'] = sha(config['manifest']) if Path(config['manifest']).is_file() else None
        report['attestation_after'] = sha(config['attestation']) if Path(config['attestation']).is_file() else None
        if report['manifest_before'] != report['manifest_after'] or report['attestation_before'] != report['attestation_after']:
            report['result'] = 'FAIL'
            report['postflight_error'] = 'manifest/attestation changed during native case'
            report.setdefault('error', report['postflight_error'])
        dump(output / 'native-launch-result.json', report)
    return 0 if report['result'] == 'PASS' else 1


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except CIInterrupted as error:
        print(json.dumps({'result': 'FAIL', 'error': str(error), 'interrupted': True}), file=sys.stderr)
        raise SystemExit(128 + error.number)
