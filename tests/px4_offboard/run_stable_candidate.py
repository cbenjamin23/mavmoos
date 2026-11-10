#!/usr/bin/env python3
"""Run the original native Offboard assertions on the pinned stable PX4 candidate."""
import argparse
from hashlib import sha256
import os
from pathlib import Path
import re
import signal
import subprocess
import sys

from run_px4_offboard import HOLD, STABLE_PIN, STABLE_PX4_REVISION

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'scripts/release'))
from profiles import (CIInterrupted, MOOS_REVISION, TOOLS_IMAGE, LocalDocker, audit_native, clean_revision,
                      dump, interrupted, need, parse, run_px4, sha)

APP_REVISION = '4eb4e295acdbfd7f119040da49e4847d6e8dc337'
APP_SHA = '3f34ab99453133d0e0a13de47cf0013a0d939770688c2d0517e4433d443236f0'
HELM_SHA = '81f2bf8931a77977b8e0bd5de17bffec7a64a5d37f6cbceea59071f74e679b0e'


def audit_stable(out, pin):
    report = parse((out / 'stable-result.json').read_text())
    fixture = report['fixture_revision']
    need(re.fullmatch(r'[0-9a-f]{40}', fixture) and report['result'] == 'PASS' and
         report['scope'] == 'owned_sitl_stable_candidate' and report['source_revision'] == APP_REVISION and
         report['firmware_pin'] == pin and report['firmware_pin_sha256'] == sha(STABLE_PIN) and
         report['app_sha256'] == APP_SHA and report['helm_sha256'] == HELM_SHA and
         report['moos_revision'] == MOOS_REVISION and report['tools_image'] == TOOLS_IMAGE,
         'archived stable candidate identity differs')
    build_inputs = parse((out / 'linux-inputs.json').read_text())
    fixture_inputs = dict(build_inputs['fixture_sources'])
    runner = 'tests/px4_offboard/run_stable_candidate.py'
    fixture_inputs[runner] = report['fixture_runner_sha256']
    need(build_inputs['fixture_revision'] == fixture and build_inputs['app'] == APP_SHA and
         build_inputs['helm'] == HELM_SHA and set(fixture_inputs) == {
             runner, 'scripts/release/profiles.py', 'tests/px4_offboard/run_px4_offboard.py',
             'tests/px4_offboard/firmware-v1.17.0.json'},
         'archived candidate fixture inputs are incomplete')
    for name, expected in fixture_inputs.items():
        blob = subprocess.check_output(['git', '-C', ROOT, 'show', f'{fixture}:{name}'], timeout=15)
        need(sha256(blob).hexdigest() == expected, 'archived candidate fixture commit differs')
    before = parse((out / 'container-before.json').read_text())
    after = parse((out / 'container-after.json').read_text())
    cleanup = parse((out / 'container-cleanup.json').read_text())
    lease = parse((out / 'domain-lease.json').read_text())
    need(before['Id'] == after['Id'] == cleanup['container'] and
         before['Image'] == after['Image'] == TOOLS_IMAGE and not after['State']['Running'] and
         after['State']['ExitCode'] == 0 and cleanup['removed'] is True and
         lease['acquired'] is lease['released'] is True and lease['source_ref'] == APP_REVISION,
         'archived container or lease teardown differs')
    native = parse((out / 'native/result.json').read_text())
    expected_files = {'/opt/px4/' + name: digest for name, digest in pin['files'].items()}
    expected_files.update({'/work/' + name: digest for name, digest in pin['retained_work_files'].items()})
    need(all(native['artifacts_before'][name] == expected for name, expected in expected_files.items()) and
         'Quadcopter' in native['px4_sih_status'] and
         'Build variant: default' in native['px4_cli_version'] and
         re.search(r'PX4 version:\s*Release 1\.17\.0\b', native['px4_cli_version']),
         'archived stable candidate package or SIH identity differs')
    home = native['stable_home_settle']
    samples = home['samples']
    confirmed = home['independent_hold_after_settle']
    first_acquire = next(row for row in native['events'] if row.get('key') == 'MAVMOOS_NAV_CONTROL' and
                         row.get('value') == 'acquire')
    need(home['correction_window_us'] == pin['home_correction_window_us'] and
         home['initial_hold']['mode'] == confirmed['mode'] == confirmed['intended_mode'] == HOLD and
         home['initial_hold']['armed'] is True and home['initial_observer_exit'] == 0 and
         home['initial_observer_pid'] != home['final_observer_pid'] and
         home['processes_before'] == home['processes_after'] and set(home['processes_before']) == {'app', 'helm'} and
         len(samples) >= 3 and home['started_at'] < samples[0]['mono'] < samples[-1]['mono'] <
         confirmed['mono'] < first_acquire['mono'] and
         samples[-1]['mono'] - samples[0]['mono'] >= 6 and
         all(sample['flight_time_us'] >= home['correction_window_us'] for sample in samples) and
         len({(row['takeoff_time_us'], row['home_timestamp_us'], row['home_update_count']) for row in samples}) == 1 and
         {'preflight-gcs.mavlink', 'preflight-moos-wire.jsonl', 'fixture-cli.log'} <= native['files'].keys(),
         'archived stable PX4 home settle or unchanged mission process evidence differs')
    audited = audit_native(out / 'native', 'px4_offboard', APP_REVISION, {'app': APP_SHA, 'helm': HELM_SHA},
                           px4_firmware=(pin['revision'], pin['files']['bin/px4']))
    need(all(report[key] == value for key, value in audited.items()), 'archived stable proof differs')
    return audited


def run_candidate(args, pin):
    out = args.output.resolve()
    out.mkdir(mode=0o700, parents=True, exist_ok=False)
    args.revision = APP_REVISION
    args.px4_build_volume = 'mavmoos-release-017-4eb4e29'
    args.px4_app_sha256 = APP_SHA
    args.px4_helm_sha256 = HELM_SHA
    report = {'schema': 1, 'result': 'FAIL', 'scope': 'owned_sitl_stable_candidate', 'hardware': False,
              'case': 'px4_offboard', 'platform': 'linux-arm64', 'moos_revision': MOOS_REVISION,
              'source_revision': APP_REVISION, 'app_sha256': APP_SHA, 'helm_sha256': HELM_SHA,
              'tools_image': TOOLS_IMAGE, 'cleanup_proven': False}
    try:
        fixture = clean_revision(ROOT)
        report.update(fixture_revision=fixture, fixture_runner_sha256=sha(Path(__file__)),
                      firmware_pin=pin, firmware_pin_sha256=sha(STABLE_PIN))
        endpoint = LocalDocker()
        try:
            report['docker_endpoint'] = endpoint.evidence()
            artifacts = run_px4(args, out, endpoint, stable_fixture=fixture)
        finally:
            endpoint.close()
        need(clean_revision(ROOT) == fixture, 'stable candidate fixture changed during qualification')
        report.update(audit_native(out / 'native', 'px4_offboard', APP_REVISION, artifacts,
                                   px4_firmware=(pin['revision'], pin['files']['bin/px4'])))
        report['result'] = 'PASS'
    except (CIInterrupted, KeyboardInterrupt, OSError, ValueError, RuntimeError,
            LookupError, AttributeError, TypeError, subprocess.SubprocessError) as error:
        report['error'] = f'{type(error).__name__}: {error}'
    finally:
        dump(out / 'stable-result.json', report)
    print(f"{report['result']} stable PX4 Offboard candidate: {out / 'stable-result.json'}", flush=True)
    return report['result'] != 'PASS'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--output', type=Path, help='fresh qualification directory')
    mode.add_argument('--audit', type=Path, help='read back a passing qualification directory offline')
    parser.add_argument('--px4-package-volume')
    args = parser.parse_args()
    if bool(args.output) != bool(args.px4_package_volume):
        parser.error('a local PX4 package volume is required only for a new run')
    pin = parse(STABLE_PIN.read_text())
    need(pin['revision'] == STABLE_PX4_REVISION, 'stable candidate source revision differs from the upstream tag')
    if args.audit:
        audit_stable(args.audit.resolve(), pin)
        print(f'PASS archived stable PX4 Offboard candidate: {args.audit.resolve()}')
        return 0
    signal.signal(signal.SIGTERM, interrupted)
    os.umask(0o077)
    return run_candidate(args, pin)


if __name__ == '__main__':
    raise SystemExit(main())
