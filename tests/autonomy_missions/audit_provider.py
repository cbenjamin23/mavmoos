#!/usr/bin/env python3
"""Recompute mission-level evidence from the closed simulator and helm captures."""
import argparse
import json
import math
from pathlib import Path
import re
import sys

TESTS = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(TESTS / 'plane_native'), str(TESTS / 'px4_offboard'), str(TESTS)]
from autonomy_missions.stock_grade import checkpoints, local_xy
from replay import verify_capture
from audit_aircraft_wire import FIELDS, replay, wire_rows


def need(value, message):
    if not value:
        raise RuntimeError(message)


def lines(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line.startswith('{')]


def origin(config):
    return tuple(float(re.search(rf'(?im)^{key}\s*=\s*([-+.0-9]+)', config)[1])
                 for key in ('LatOrigin', 'LongOrigin'))


def assert_stages(mission, rows, keys):
    stamps = []
    for key in keys:
        row = mission[key]
        need(row in rows and row['source'] == 'pHelmIvP', 'stock phase absent from capture')
        stamps.append(row['mono'])
    need(all(a < b for a, b in zip(stamps, stamps[1:])), 'stock phase sequence differs')


def ardupilot(directory, profile, report):
    config, trace, mission, stages = {
        'arduplane': ('targ_plane.moos', 'moos-probe.jsonl', report.get('workflow'),
                      ('return_stage', 'done_stage')),
        'quadplane': ('quadplane.moos', 'moos.jsonl', report.get('autonomy'),
                      ('climb_stage', 'done_stage')),
        'ardusub': ('targ_sub.moos', 'moos.jsonl', report.get('autonomy'),
                    ('deeper_stage', 'done_stage')),
    }[profile]
    need(mission and mission['grade'] == (directory / 'results.txt').read_text().strip() == 'grade=pass',
         'stock mission evaluator did not pass')
    checked = verify_capture(directory, 'monitor')
    wire = lines(directory / 'monitor-rx.jsonl')
    moos = lines(directory / trace)
    need(not any(row.get('kind') == 'request' and
                 (row['key'] == 'AUTONOMY_STAGE' or row['key'].startswith('DESIRED_')) for row in moos),
         'the launcher injected a helm phase or decision')
    assert_stages(mission, moos, stages)
    lat, lon = origin((directory / config).read_text())
    fixes = []
    for point in mission['checkpoints']:
        fix = point['fix']
        need(fix in wire and fix['mavpackettype'] == 'GLOBAL_POSITION_INT' and
             math.isclose(math.dist(local_xy(fix, lat, lon), point['goal']), point['error_m']),
             'independent physical checkpoint differs')
        fixes.append(fix)
    need(all(a['mono'] < b['mono'] for a, b in zip(fixes, fixes[1:])), 'physical checkpoints were reordered')
    if profile == 'arduplane':
        home = mission['home_fix']
        need(home in wire and math.isclose(math.hypot(*local_xy(home, lat, lon)), mission['home_range_m'])
             and any(row['mavpackettype'] == 'HEARTBEAT' and row['custom_mode'] == 12 and
                     row['mono'] == mission['native_loiter_at'] for row in wire),
             'independent homeward path or native Loiter handoff did not replay')
    elif profile == 'quadplane':
        high = mission['high_hover']['position']
        need(high in wire and math.isclose((high['relative_alt'] - fixes[0]['relative_alt']) / 1000,
                                            mission['climb_m'])
             and mission['touchdown']['heartbeat'] in wire
             and not mission['touchdown']['heartbeat']['base_mode'] & 128,
             'independent climb or QRTL disarm did not replay')
    else:
        home = next(row for row in reversed(wire) if row['mavpackettype'] == 'HOME_POSITION'
                    and row['mono'] <= fixes[0]['mono'])
        depths = [(home['altitude'] - fix['alt']) / 1000 for fix in fixes]
        need(depths == mission['checkpoint_depths_m'] and mission['surface_entry'] in wire
             and mission['depth_hold'] in wire and report['disarm'] in wire,
             'independent depth sequence, Surface or disarm did not replay')
    return {'profile': profile, 'mavlink_frames': checked, 'checkpoints': len(fixes), 'stock_grade': 'grade=pass'}


def px4(directory, profile, report):
    if profile == 'px4-vtol':
        wire, checked = replay(directory, '')
    else:
        # The ground probe ignores VFR_HUD. Compare every other shared
        # message in order against the CRC-checked capture.
        wire = lines(directory / 'moos-wire.jsonl')
        tracked = FIELDS.keys() - {'VFR_HUD'}
        decoded = [(row['message'], tuple(row[k] for k in FIELDS[row['message']]))
                   for row in wire if row.get('message') in tracked]
        captured = [row for row in wire_rows((directory / 'gcs.mavlink').read_bytes()) if row[0] in tracked]
        need(decoded and decoded == captured, 'Ackermann capture and decoded trace differ')
        checked = len(captured)
    lat, lon = origin((directory / 'targ_px4.moos').read_text())

    def project(row):
        return local_xy({'lat': row['latitude_e7'], 'lon': row['longitude_e7']}, lat, lon)

    def fixes(start, end):
        return [row for row in wire if row.get('message') == 'GLOBAL_POSITION_INT' and start <= row['mono'] <= end]

    if profile == 'px4-vtol':
        survey = report['autonomy_survey']
        gate = survey['index']
        assert_stages(survey, wire, ('index',))
        points = checkpoints(fixes(report['legs'][0]['acquired_at'], gate['mono'] + .4),
                             [(12, 0), (14, 12)], 3.5, project)
        need(gate['mono'] < report['transitions'][0]['transient']['mono'] and
             report['landed_disarmed']['heartbeat'] in wire, 'VTOL survey to native lifecycle did not replay')
    else:
        route = report['autonomy_route']
        assert_stages(route, wire, ('corner', 'exit_gate'))
        points = checkpoints(fixes(report['helm']['first_acquire']['acquired_at'], route['exit_gate']['mono'] + .4),
                             [tuple(map(float, report['stock_route'][key].split(',')))
                              for key in ('CORNER', 'BEND', 'EXIT')], 3, project)
        need(report['disarmed'] in wire and report['helm']['release_stopped']['firmware_dwell_s'] >= 1.5,
             'Ackermann stop or final disarm did not replay')
    return {'profile': profile, 'mavlink_frames': checked, 'checkpoints': len(points),
            'checkpoint_errors_m': [point['error_m'] for point in points]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('profile', choices=('arduplane', 'quadplane', 'ardusub', 'px4-vtol', 'px4-ackermann'))
    parser.add_argument('run', type=Path)
    args = parser.parse_args()
    report = json.loads((args.run / 'result.json').read_text())
    envelope = (json.loads((args.run.parent / 'qualification.json').read_text())
                if args.profile.startswith('px4-') else report)
    need(report['result'] == envelope['result'] == 'PASS' and envelope['hardware'] is False
         and all(report['ports_clear'].values()) and not report.get('owned_processes_remaining'),
         'readback requires a passing owned SITL run with scoped cleanup')
    audit = px4 if args.profile.startswith('px4-') else ardupilot
    print(json.dumps(audit(args.run, args.profile, report), indent=2))


if __name__ == '__main__':
    main()
