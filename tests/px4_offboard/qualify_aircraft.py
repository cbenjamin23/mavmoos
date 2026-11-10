#!/usr/bin/env python3
"""Build and qualify native PX4 aircraft control in the owned Linux SITL domain."""
import argparse
from hashlib import sha256
import json
import os
from pathlib import Path
import re
import signal
import subprocess

from qualify_fixed_wing import (CIInterrupted, DEPENDENCIES, FIRMWARE, FIXTURE_INPUTS as OFFBOARD_INPUTS,
                                FRESH_BUILD_FILES, LinuxDomainLease, LocalDocker, MAVLINK_SHA, MOOS_REVISION,
                                ROOT, TOOLS_IMAGE, audit_container, build_worker, clean_revision,
                                container, dump, git, interrupted, mount, native_files, need, parse, sha, volume)
from run_aircraft import VEHICLES, HOLD, OFFBOARD, RETURN, STABLE_PIN, run_aircraft
from run_fixed_wing import stock_work_pins
from run_px4_offboard import fields
from audit_aircraft_wire import replay, stress_route

FIXTURE_INPUTS = (*OFFBOARD_INPUTS, 'tests/px4_offboard/qualify_aircraft.py', 'tests/px4_offboard/run_aircraft.py',
                  'tests/px4_offboard/meta_aircraft.moos', 'tests/px4_offboard/meta_aircraft.bhv')
AUTONOMY_FIXTURE = 'tests/px4_offboard/meta_vtol_autonomy.bhv'
STRESS_REPLAY = 'tests/px4_offboard/audit_aircraft_wire.py'


def native_proof(out, revision, vehicle, *, build_root='/candidate', stock_witnesses=None,
                 transition_observation_loss=False, autonomy=False, stress=False):
    build = parse((out / 'build-manifest.json').read_text())
    result = parse((out / 'native/result.json').read_text())
    pin = parse(STABLE_PIN.read_text())
    model, profile, mav_type, airframe, _, takeoff_height, helm_height, _, pins = VEHICLES[vehicle]
    artifacts = build['artifacts']
    need(build['source_revision'] == result['source_revision'] == revision and
         build['tools_image'] == TOOLS_IMAGE and build['moos_revision'] == MOOS_REVISION and
         build['mavlink_sha256'] == MAVLINK_SHA and set(artifacts) == set(FRESH_BUILD_FILES) and
         result['result'] == 'PASS' and result['firmware_revision'] == pin['revision'] and
         (result['vehicle'], result['profile'], result['model'], result['airframe']) == (vehicle, profile, model, airframe)
         and result['artifact_drift'] == result['owned_processes_remaining'] == [] and result['resume_error'] is None and
         set(result['ports_clear']) == {'14540', '14550', '14580', '18570', 'db'} and
         all(result['ports_clear'].values()), 'aircraft build, vehicle, native result or cleanup differs')
    inputs = {f'{build_root}/{name}': digest for name, digest in artifacts.items()}
    inputs.update({f'/opt/px4/{name}': digest for name, digest in {**pin['files'], **pins}.items()})
    stock = stock_work_pins(pin, build if stock_witnesses is not None else None)
    need(stock_witnesses is None or stock == stock_witnesses,
         'native aircraft stock witnesses differ from the verified Linux kit')
    inputs.update(stock)
    need(all(result['artifacts_before'].get(name) == digest for name, digest in inputs.items()),
         'native aircraft artifact readback differs')
    for isolation in (build['namespace'], result['namespace']):
        need(isolation['ipv4_routes'] == [] and isolation['cap_eff'] == '0' * 16 and
             isolation['no_new_privileges'] is isolation['root_read_only'] is True and
             [name for name, flags in isolation['interface_flags'].items() if flags & 1] == ['lo'],
             'aircraft effective namespace proof is incomplete')
    native_files(out / 'native', result['files'], ('gcs.mavlink', 'moos-wire.jsonl', 'preflight-gcs.mavlink',
                                                  'fixture-cli.log', 'targ_px4.moos', 'targ_px4.bhv'))
    operations = result['native_actions']
    expected = ['takeoff', 'arm', 'disarm', 'arm', 'takeoff', 'hold']
    if vehicle == 'standard-vtol':
        expected.extend(('transition_fw', 'hold', 'transition_mc'))
    expected.extend(('rtl', 'land', 'disarm'))
    need([row['action'] for row in operations] == expected and
         fields(operations[0]['result'])['status'] == 'REJECTED' and operations[0]['send'] == [] and
         all(len(row['send']) == 1 and fields(row['result'])['status'] == 'COMPLETED' and
             {'ADMITTED', 'CONFIRMED'} <= {fields(stage)['status'] for stage in row['stages']} and
             {'ACK_OBSERVED', 'ACKNOWLEDGED'} & {fields(stage)['status'] for stage in row['stages']}
             for row in operations[1:]), 'native aircraft command outcomes or wire receipts differ')
    need(result['takeoff']['mode']['mode'] == result['takeoff']['mode']['intended_mode'] == HOLD and
         result['takeoff']['heartbeat']['mav_type'] == mav_type and
         abs(result['takeoff']['position']['relative_m'] - takeoff_height) <= (6 if vehicle == 'fixed-wing' else 1.2),
         'aircraft takeoff did not attain the pinned height and Hold')
    leg_order = (['hover', 'forward', 'hover'] if vehicle == 'standard-vtol' else
                 ['forward', 'forward'] if vehicle == 'fixed-wing' else ['hover', 'hover'])
    legs = result['legs']
    need([leg['phase'] for leg in legs] == leg_order and
         all(leg['offboard']['mode'] == leg['offboard']['intended_mode'] == OFFBOARD and
             leg['prestream'] >= 8 and leg['hold_frames'] >= 8 and leg['east_m'] >= (8 if leg['phase']=='hover' else 25)
             and abs(leg['stock_inputs']['DESIRED_ALTITUDE']['value'] - helm_height) <= .05 and
             all(row['source'] == 'pHelmIvP' for row in leg['stock_inputs'].values()) for leg in legs),
         'stock helm or independent flight motion proof differs')
    transitions = result['transitions']
    if vehicle == 'standard-vtol':
        need([row['action'] for row in transitions] == ['transition_fw', 'transition_mc'] and
             [(row['transient']['vtol_state'], row['stable']['vtol_state']) for row in transitions] == [(1, 4), (2, 3)] and
             all(row['transient']['mono'] < row['stable']['mono'] for row in transitions),
             'both independent stable and transient VTOL phases are required')
    else:
        need(not transitions, 'unexpected VTOL transition evidence')
    distance = result['return_motion']
    need(distance['mode']['mode'] == distance['mode']['intended_mode'] == RETURN and distance['homeward_m'] >= 1.75 and
         operations[-2]['ground']['heartbeat']['armed'] is True and
         result['landed_disarmed']['heartbeat']['armed'] is False and
         operations[-2]['ground']['state']['landed_state'] == result['landed_disarmed']['state']['landed_state'] == 1,
         'independent return, touchdown or final disarm differs')
    fault_proof = {}
    if transition_observation_loss:
        fault = transitions[0]['observation_loss']
        phase = fault['bridge_unknown_phase']
        operation = next(row for row in operations if row['action'] == 'transition_fw')
        need(fault['disabled_at'] <= fault['independent_stable']['mono'] <= phase['mono'] <=
             fault['restore_requested_at'] <= operation['result']['mono'] < legs[1]['acquired_at'] and
             fault['independent_stable']['vtol_state'] == 4 and phase['value'] == 0 and
             fields(fault['fenced_runtime'])['permission'] == 'false' and len(operation['send']) == 1,
             'transition observation loss, restoration, or fresh acquisition proof differs')
        fault_proof['transition_observation_loss'] = True
    if autonomy:
        survey = result['autonomy_survey']
        need(vehicle == 'standard-vtol' and survey['index']['source'] == 'pHelmIvP'
             and survey['index']['value'] == 2 and survey['north_span_m'] >= 7
             and survey['east_span_m'] >= 8 and survey['sample_count'] >= 20
             and survey['index']['mono'] < operations[6]['requested_at'],
             'independent stock VTOL survey proof differs')
        fault_proof['autonomy_survey'] = survey
    if stress:
        trace, _ = replay(out / 'native', '')
        fault_proof['stress_route'] = stress_route(result, trace)
    return {'vehicle': vehicle, 'model': model, 'firmware_revision': pin['revision'],
            'app_sha256': artifacts['bin/iMAVLink'], 'plugin_sha256': artifacts['plugins/mavmoos_px4_control.so'],
            'helm_sha256': stock['/work/moos-ivp/bin/pHelmIvP'],
            'actions_completed': len(operations) - 1, 'homeward_m': distance['homeward_m'],
            'eastward_legs_m': [leg['east_m'] for leg in legs], 'transitions_completed': len(transitions), **fault_proof}


def audit(out):
    report = parse((out / 'qualification.json').read_text())
    revision = report['source_revision']
    need(report['result'] == 'PASS' and report['scope'] == 'owned_sitl' and report['hardware'] is False and
         re.fullmatch(r'[0-9a-f]{40}', revision) and report['vehicle'] in VEHICLES and
         set(report['fixture_sources']) == (set(FIXTURE_INPUTS) |
             ({AUTONOMY_FIXTURE} if report.get('autonomy') else set()) |
             ({STRESS_REPLAY} if report.get('stress') else set())), 'invalid archived PX4 aircraft identity')
    for name, expected in report['fixture_sources'].items():
        blob = subprocess.check_output(['git', '-C', ROOT, 'show', f'{revision}:{name}'], timeout=15)
        need(sha256(blob).hexdigest() == expected, f'aircraft fixture source differs: {name}')
    need(git(ROOT, 'rev-parse', f'{revision}^{{tree}}') == report['source_tree'], 'archived source tree differs')
    audit_container(out, 'native')
    if report['built_in_this_run']:
        audit_container(out, 'build')
    lease = parse((out / 'domain-lease.json').read_text())
    need(lease['source_ref'] == revision and lease['acquired'] is lease['released'] is True and
         lease['release_errors'] == [], 'aircraft Linux-domain teardown differs')
    observed = native_proof(out, revision, report['vehicle'],
                            transition_observation_loss=report.get('transition_observation_loss', False),
                            autonomy=report.get('autonomy', False), stress=report.get('stress', False))
    need(observed == report['proof'], 'archived aircraft summary differs from independent readback')
    return observed


def run_container(endpoint, out, stage, mounts, options, timeout):
    container(endpoint, out, stage, mounts, options, timeout,
              entry='/source/tests/px4_offboard/qualify_aircraft.py',
              scope_prefix='px4-aircraft', name_prefix='mavmoos-px4-air')


def qualify(args):
    out = args.output.resolve()
    out.mkdir(mode=0o700, parents=True, exist_ok=False)
    report = {'schema': 1, 'result': 'FAIL', 'scope': 'owned_sitl', 'hardware': False,
              'vehicle': args.vehicle, 'tools_image': TOOLS_IMAGE, 'built_in_this_run': args.built is None}
    endpoint = None
    try:
        revision = clean_revision(ROOT)
        sources = set(FIXTURE_INPUTS)
        if args.autonomy:
            sources.add(AUTONOMY_FIXTURE)
        if args.stress:
            sources.add(STRESS_REPLAY)
        report.update(source_revision=revision, source_tree=git(ROOT, 'rev-parse', 'HEAD^{tree}'),
                      fixture_sources={name: sha(ROOT / name) for name in sorted(sources)})
        endpoint = LocalDocker()
        report['docker_endpoint'] = endpoint.evidence()
        image, = parse(endpoint.capture('image', 'inspect', TOOLS_IMAGE))
        need((image['Id'], image['Os'], image['Architecture']) == (TOOLS_IMAGE, 'linux', 'arm64'),
             'pinned offline Linux ARM64 tools image differs')
        dependencies = volume(endpoint.capture, args.stock_volume)
        mounts = [mount('bind', ROOT, '/source'), mount('volume', dependencies, '/work'),
                  mount('bind', out, '/evidence', True)]
        build = out / 'build'
        if args.built:
            built = args.built.resolve(strict=True)
            build = built / 'build'
            manifest = parse((built / 'build-manifest.json').read_text())
            need(manifest['source_revision'] == revision and manifest['tools_image'] == TOOLS_IMAGE and
                 set(manifest['artifacts']) == set(FRESH_BUILD_FILES) and
                 all(sha(build / name) == digest for name, digest in manifest['artifacts'].items()),
                 'sealed aircraft build does not match the clean revision and artifacts')
            audit_container(built, 'build')
            dump(out / 'build-manifest.json', manifest)
        else:
            archive = args.mavlink_archive.resolve(strict=True)
            need(sha(archive) == MAVLINK_SHA, 'offline MAVLink archive differs from its immutable pin')
            build.mkdir(mode=0o700)
            run_container(endpoint, out, 'build',
                          [*mounts, mount('bind', build, '/candidate', True),
                           mount('bind', archive, '/input/mavlink.tar.gz')],
                          ['--worker', 'build', '--source-revision', revision], 450)
        need(clean_revision(ROOT) == revision, 'source changed during the offline build')
        if args.build_only:
            report.update(result='PASS', scope='offline_linux_build')
        else:
            firmware = volume(endpoint.capture, args.px4_volume)
            need(firmware != dependencies, 'aircraft firmware and dependencies volumes alias')
            lease = LinuxDomainLease(out / 'domain-lease.json', revision)
            lease.acquire()
            try:
                run_container(endpoint, out, 'native',
                              [*mounts, mount('bind', build, '/candidate'), mount('volume', firmware, '/opt/px4')],
                              ['--worker', args.vehicle, '--source-revision', revision] +
                              (['--transition-observation-loss'] if args.transition_observation_loss else []) +
                              (['--autonomy'] if args.autonomy else []) + (['--stress'] if args.stress else []), 540)
            finally:
                lease.release()
            need(clean_revision(ROOT) == revision, 'source changed during aircraft SITL')
            if args.transition_observation_loss:
                report['transition_observation_loss'] = True
            if args.autonomy:
                report['autonomy'] = True
            if args.stress:
                report['stress'] = True
            report.update(result='PASS', proof=native_proof(
                out, revision, args.vehicle, transition_observation_loss=args.transition_observation_loss,
                autonomy=args.autonomy, stress=args.stress))
    except (CIInterrupted, KeyboardInterrupt, OSError, ValueError, RuntimeError, LookupError, AttributeError,
            TypeError, subprocess.SubprocessError) as error:
        report['error'] = f'{type(error).__name__}: {error}'
    finally:
        if endpoint is not None:
            endpoint.close()
        dump(out / 'qualification.json', report)
    print(f"{report['result']} PX4 {args.vehicle}: {out / 'qualification.json'}", flush=True)
    return report['result'] != 'PASS'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_mutually_exclusive_group(required=True)
    modes.add_argument('--output', type=Path, help='fresh evidence directory')
    modes.add_argument('--audit', type=Path, help='verify archived results without starting a simulator')
    modes.add_argument('--worker', choices=('build', *VEHICLES), help=argparse.SUPPRESS)
    parser.add_argument('--vehicle', choices=VEHICLES, default='quadrotor')
    parser.add_argument('--build-only', action='store_true')
    parser.add_argument('--built', type=Path)
    parser.add_argument('--mavlink-archive', type=Path)
    parser.add_argument('--stock-volume', default=DEPENDENCIES)
    parser.add_argument('--px4-volume', default=FIRMWARE)
    parser.add_argument('--source-revision', help=argparse.SUPPRESS)
    parser.add_argument('--transition-observation-loss', action='store_true',
                        help='withhold VTOL phase from the bridge during the forward transition')
    parser.add_argument('--autonomy', action='store_true', help='require stock waypoint survey before VTOL transition')
    parser.add_argument('--stress', action='store_true',
                        help='withhold forward phase evidence and revise the stock VTOL route before reacquiring')
    args = parser.parse_args()
    if args.stress:
        args.autonomy = args.transition_observation_loss = True
    if args.autonomy and 'standard-vtol' not in (args.worker, args.vehicle):
        parser.error('--autonomy requires the standard VTOL')
    if args.transition_observation_loss and 'standard-vtol' not in (args.worker, args.vehicle):
        parser.error('transition observation loss requires the standard VTOL')
    if args.worker:
        if not re.fullmatch(r'[0-9a-f]{40}', args.source_revision or ''):
            parser.error('worker requires an exact source revision')
        if args.worker == 'build':
            return build_worker(args.source_revision)
        return run_aircraft(args.worker, args.source_revision, Path('/evidence/native'),
                            Path('/evidence/build-manifest.json'),
                            transition_observation_loss=args.transition_observation_loss, autonomy=args.autonomy,
                            stress=args.stress)
    if args.audit:
        print(json.dumps(audit(args.audit.resolve()), indent=2))
        return 0
    if bool(args.mavlink_archive) == bool(args.built) or (args.build_only and args.built):
        parser.error('select an offline MAVLink archive for a fresh build or an existing sealed build')
    signal.signal(signal.SIGTERM, interrupted)
    os.umask(0o077)
    return qualify(args)


if __name__ == '__main__':
    raise SystemExit(main())
