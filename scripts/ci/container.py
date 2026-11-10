#!/usr/bin/env python3
"""Owned source-build container stages; invoked by the Linux reproduction recipe."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys


def execute(argv, cwd=None, log=None):
    if log:
        with Path(log).open('w') as stream:
            subprocess.run(argv, cwd=cwd, stdout=stream, stderr=subprocess.STDOUT, check=True)
    else:
        subprocess.run(argv, cwd=cwd, check=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stage', choices=['build-native', 'run'], required=True)
    args = parser.parse_args()
    config = json.loads(Path('/inputs/ci-config.json').read_text())
    if config['schema'] != 1 or config['tier'] not in ('software', 'smoke', 'full'):
        raise ValueError('invalid source-build CI context')
    source, ap, evidence = Path('/work/mavmoos'), Path('/work/ardupilot'), Path('/work/evidence')
    python = Path('/work/ap-venv/bin/python')
    manifest = Path('/inputs/firmware-manifest.json')
    receipt = evidence / 'native-build'
    jobs = int(os.environ.get('JOBS', '4'))
    if not 1 <= jobs <= 8:
        raise ValueError('bounded native build jobs must be1..8')
    if args.stage == 'build-native':
        execute([str(python), '-B', str(source / 'scripts/firmware/build_native.py'),
                 '--manifest', str(manifest), '--source', str(ap), '--output', str(receipt),
                 '--python', str(python), '--jobs', str(jobs)])
        if config['tier'] == 'full':
            # The host recipe permits this only for the baseline source shared
            # by the independently scoped legacy Rover case.
            execute([str(python), 'waf', 'rover', '-j', str(jobs)], cwd=ap,
                    log=evidence / 'rover-build.log')
        return 0
    command = [str(python) if python.is_file() else sys.executable, '-B',
               str(source / 'scripts/ci/run.py'), '--source-root', str(source),
               '--build', '/work/mavmoos-build', '--tier', config['tier'],
               '--output', str(evidence / 'ci'), '--jobs', str(jobs)]
    if config['tier'] != 'software':
        command += ['--firmware-manifest', str(manifest), '--firmware-attestation',
                    str(receipt / 'native-attestation.json'), '--ardupilot-root', str(ap),
                    '--moos-root', '/work/moos-ivp', '--behavior-dir', '/work/altitude/build/lib',
                    '--sitl-python', str(python)]
    return subprocess.run(command, cwd=source).returncode


if __name__ == '__main__':
    raise SystemExit(main())
