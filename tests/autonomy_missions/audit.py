#!/usr/bin/env python3
"""Read back an owned autonomy mission using its archived MOOS and MAVLink evidence."""
import argparse
from itertools import groupby
import json
import math
from pathlib import Path

from pymavlink.dialects.v20 import ardupilotmega

from run_native import BOAT_FINISH, STRESS_MISSIONS, SURVEY_ROUTE, contact_xy, require, sha, speed, xy


def jsonl(path):
    with path.open() as stream:
        return [json.loads(line) for line in stream]


def wire_readback(directory):
    rows = jsonl(directory / 'fc.jsonl')
    messages = iter(rows)
    parser = ardupilotmega.MAVLink(None)
    parser.robust_parsing = True
    observed = set()
    with (directory / 'fc.mavlink').open('rb') as stream:
        for chunk in iter(lambda: stream.read(65536), b''):
            for packet in parser.parse_buffer(chunk) or []:
                kind = packet.get_type()
                if kind not in {'HEARTBEAT', 'GLOBAL_POSITION_INT', 'MISSION_CURRENT',
                                'MISSION_ITEM_REACHED', 'HOME_POSITION', 'STATUSTEXT', 'EXTENDED_SYS_STATE'}:
                    continue
                require((packet.get_srcSystem(), packet.get_srcComponent()) == (1, 1),
                        'independent capture sender changed')
                decoded = packet.to_dict()
                row = next(messages, None)
                require(row is not None and
                        json.dumps({k: v for k, v in row.items() if k != 'at'}, sort_keys=True) ==
                        json.dumps(decoded, sort_keys=True),
                        'independent trace does not reproduce from raw MAVLink')
                observed.add(kind)
    require(next(messages, None) is None and {'HEARTBEAT', 'GLOBAL_POSITION_INT'} <= observed,
            'raw capture missing expected independent telemetry')
    return rows


def audit_stress(directory, report, moos, wire, requests, grants):
    from mode_stress import BOAT_EXIT, COPTER_EXIT, audit_stress_wire

    require([row['value'] for row in grants] == ['acquire'] * 3 and
            not any(row['key'].startswith('DESIRED_') for row in requests),
            'expected initial grant, denied competing acquire and one fresh reacquire')
    replay = audit_stress_wire(directory, report)
    require(replay == report['wire_audit'], 'closed control-route replay differs')
    outcome, points = report['outcome'], [row for row in wire if row['mavpackettype'] == 'GLOBAL_POSITION_INT']
    copter = report['mission'] == 'copter-mode-stress'
    route = COPTER_EXIT if copter else BOAT_EXIT
    finish = outcome['finish']
    changed, = [row for row in requests if row['key'] == 'SURVEY_UPDATE']
    require(changed['value'] == 'xpoints=' + ':'.join(f'{x},{y}' for x, y in route) and
            grants[0]['at'] < grants[1]['at'] < grants[2]['at'] < finish['done']['at'],
            'stock route update or authority sequence differs')
    require(finish['done'] in moos and finish['done']['source'] == 'pHelmIvP' and
            finish['disarmed'] in wire and finish['disarmed']['custom_mode'] == (9 if copter else 4) and
            not finish['disarmed']['base_mode'] & 128 and finish['final_position'] in points and
            math.dist(xy(finish['final_position']), route[-1]) <= 2.5,
            'stock completion and terminal native disarm did not replay')
    require([tuple(hit['goal']) for hit in finish['checkpoints']] == list(route), 'revised route gates differ')
    at = grants[2]['at']
    for hit in finish['checkpoints']:
        require(hit['fix'] in points and at < hit['fix']['at'] < finish['done']['at'] + .25 and
                math.dist(xy(hit['fix']), hit['goal']) <= 2.5,
                'the revised gate was not independently traversed after reacquisition')
        at = hit['fix']['at']
    denial = outcome['denied_acquire']
    require(denial in moos and denial['at'] > grants[1]['at'] and
            'permission=false' in denial['value'] and
            any(row.get('key') == 'MAVMOOS_HELM_STATUS' and
                grants[1]['at'] < row['at'] < grants[2]['at'] and
                'reason=acquisition_rejected' in row['value'] for row in moos) and
            outcome['duplicate']['status'] == 'DUPLICATE', 'competing acquire or duplicate denial did not replay')
    summary = {'mission': report['mission'], 'verified_wire_rows': len(wire),
               'revised_waypoints': len(route), **replay}
    if copter:
        phases = [outcome[name] for name in ('auto_entry', 'auto_travel', 'resume_origin', 'resumed_travel')]
        paused = outcome['paused_cursor']
        require(all(row in points for row in phases) and paused in wire and
                paused['mission_state'] == ardupilotmega.MISSION_STATE_PAUSED and
                paused['seq'] == 1 and phases[1]['at'] < paused['at'] < phases[2]['at'] and
                xy(phases[1])[1] - xy(phases[0])[1] >= 4 and xy(phases[3])[1] - xy(phases[2])[1] >= 3 and
                outcome['stock_return'] in moos and outcome['stock_return']['source'] == 'pHelmIvP' and
                all(row in moos for row in outcome['staged_stock'].values()),
                'AUTO pause, same-cursor resume or changed stock route did not replay')
        return {**summary, 'paused_cursor': paused['seq'],
                'native_travel_before_pause_m': xy(phases[1])[1] - xy(phases[0])[1],
                'native_travel_after_resume_m': xy(phases[3])[1] - xy(phases[2])[1]}
    contact = report['contact_model']
    encounter = [row for row in points if contact['at'] <= row['at'] <= outcome['reacquired']['request_at']]
    minimum = min(math.dist(xy(row), contact_xy(contact, row['at'])) for row in encounter)
    require(outcome['avoidance'] in moos and outcome['avoidance']['source'] == 'pHelmIvP' and
            outcome['yielded'] in encounter and outcome['loiter'] in wire and outcome['loiter']['custom_mode'] == 5 and
            outcome['native_stop']['seconds'] >= 1 and outcome['duplicate']['status'] == 'DUPLICATE' and
            math.isclose(minimum, outcome['minimum_separation_m']) and minimum >= 3.5,
            'avoidance, observed Loiter, duplicate denial or crossing separation did not replay')
    return {**summary, 'minimum_separation_m': minimum, 'encounter_samples': len(encounter)}


def audit(directory):
    report = json.loads((directory / 'result.json').read_text())
    require(report['result'] == 'PASS' and report['hardware'] is False and
            report['independent_monitor_writes'] == 0 and not report['input_drift'] and
            not report['owned_processes_remaining'] and all(report['ports_clear'].values()),
            'expected a passing, cleaned-up SITL receipt')
    for name, digest in report['files'].items():
        candidate = directory / name
        require(candidate.resolve().is_relative_to(directory) and sha(candidate) == digest,
                f'archived file differs: {name}')
    moos = jsonl(directory / 'moos.jsonl')
    wire = wire_readback(directory)
    require(any(row['kind'] == 'event' and row['key'] == 'MISSION_EVALUATED' and
                row['source'] == 'pMissionEval' and row['value'] == 'true' for row in moos) and
            (directory / 'results.txt').read_text().strip() == report['mission_grade'] == 'grade=pass',
            'stock evaluator receipt differs')
    requests = [row for row in moos if row['kind'] == 'request']
    grants = [row for row in requests if row['key'] == 'MAVMOOS_NAV_CONTROL']
    if report['mission'] in STRESS_MISSIONS:
        return audit_stress(directory, report, moos, wire, requests, grants)
    require([row['value'] for row in grants] == ['acquire'] and
            not any(row['key'].startswith('DESIRED_') for row in requests),
            'probe injected helm inputs or acquired more than once')
    result = report['outcome']
    points = [row for row in wire if row['mavpackettype'] == 'GLOBAL_POSITION_INT']
    require(not [row for row in moos if row['kind'] == 'event' and row['key'] == 'BHV_ERROR'],
            'stock helm logged a behavior error')
    if report['mission'] == 'copter-survey':
        change, = [row for row in requests if row['key'] == 'SURVEY_UPDATE']
        expected_update = 'xpoints=' + ':'.join(f'{x},{y}' for x, y in SURVEY_ROUTE)
        require(change['value'] == expected_update and result['first_leg'] in points and
                result['first_leg']['at'] < change['at'] < result['stock_survey_done']['at'],
                'in-flight route replacement did not replay')
        indices = [row['value'] for row in moos if row['kind'] == 'event' and row['key'] == 'SURVEY_INDEX'
                   and row['source'] == 'pHelmIvP' and change['at'] < row['at'] < result['stock_survey_done']['at']]
        require([index for index, _ in groupby(indices)] == [2, 3, 4],
                'stock waypoint index reset or skipped a revised leg')
        require([tuple(hit['goal']) for hit in result['updated_checkpoints']] == list(SURVEY_ROUTE[1:]),
                'revised checkpoints differ')
        elapsed = change['at']
        for checkpoint in result['updated_checkpoints']:
            fix = checkpoint['fix']
            require(fix in points and elapsed < fix['at'] < result['stock_survey_done']['at'] + .25 and
                    math.dist(xy(fix), checkpoint['goal']) <= 2.5,
                    'revised waypoint did not physically replay')
            elapsed = fix['at']
        require(all(result[key] in moos and result[key]['source'] == 'pHelmIvP' for key in
                    ('stock_survey_done', 'stock_return_started', 'stock_return_done')) and
                result['touchdown'] in points and math.dist(xy(result['touchdown']), (0, 0)) <= 2.5,
                'stock return or touchdown did not replay')
        return {'mission': report['mission'], 'verified_wire_rows': len(wire),
                'revised_waypoints': len(result['updated_checkpoints']),
                'touchdown_error_m': math.dist(xy(result['touchdown']), (0, 0))}
    contact = report['contact_model']
    reports = [row for row in requests if row['key'] == 'NODE_REPORT']
    published = jsonl(directory / 'contact.jsonl')
    require(len(reports) == len(published) and len(reports) >= 50,
            'scripted contact report trace is incomplete')
    for request, row in zip(reports, published):
        data = dict(field.split('=', 1) for field in request['value'].split(','))
        require(data['NAME'] == 'crossing' and math.dist((float(data['X']), float(data['Y'])),
                contact_xy(contact, row['at'])) <= .0015, 'contact report departed from independent model')
    encounter = [row for row in points if contact['at'] <= row['at'] <= result['stock_finish']['at']]
    minimum = min(math.dist(xy(row), contact_xy(contact, row['at'])) for row in encounter)
    require(math.isclose(minimum, result['minimum_separation_m']) and minimum >= 3.5 and
            all(result[key] in moos and result[key]['source'] == 'pHelmIvP'
                for key in ('avoidance', 'stock_finish')) and
            result['rejoined'] in encounter and speed(result['rejoined']) >= .55 and
            math.dist(xy(result['final_station']['fix']), BOAT_FINISH) <= 2,
            'crossing separation, route rejoin or station did not replay')
    return {'mission': report['mission'], 'verified_wire_rows': len(wire),
            'minimum_separation_m': minimum, 'contact_reports': len(reports)}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    print(json.dumps(audit(parser.parse_args().run.resolve()), indent=2))
