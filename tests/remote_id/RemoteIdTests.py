#!/usr/bin/env python3
"""Check bounded and redacted publications from the loaded Remote ID provider."""

import json
import subprocess
import sys


def verify(driver, module):
    result = subprocess.check_output([driver, module], text=True, timeout=8)
    observations = {row['phase']: row['posts'] for row in map(json.loads, result.splitlines())}
    for text in ('PRIVATE-TEST-ID', 'OTHER-VEHICLE', 'CHANGED-PRIVATE-ID', 'hidden transmitter text', 'id_or_mac'):
        assert text not in result
    reports = {family: {phase: posts[f'MAVMOOS_REMOTE_ID_{family.upper()}']
                        for phase, posts in observations.items()}
               for family in ('identity', 'location', 'health')}
    identity, location, health = (reports[key] for key in ('identity', 'location', 'health'))
    assert not identity['not_selected']['metadata']['present']
    assert not identity['wrong_component']['metadata']['present']
    before = identity['healthy_input']
    assert before['metadata']['age_is_upper_bound']
    assert before['metadata']['component_id'] == 236
    assert before['metadata']['effective_at'] == 100.20 and before['metadata']['read_at'] == 100.22
    assert before['data'] == {'id_type': 1, 'ua_type': 2, 'id_present': True, 'identity_generation': 1}
    position = location['healthy_input']['data']
    assert (position['latitude_deg'], position['longitude_deg']) == (47.3977436, 8.5455942)
    assert position['wgs84_altitude_m'] == 489 and position['height_m'] == 17
    assert position['course_deg'] == 278.25 and position['vertical_speed_up_m_s'] == -.5
    assert position['barometric_altitude_m'] is None and position['ground_speed_m_s'] is None
    assert position['utc_second_of_hour'] is None
    assert health['healthy_input']['data'] == {'status': 1, 'reported_ready': False, 'error_present': True}
    assert location['rejected_inputs']['metadata']['wire_rejected'] == 4
    assert identity['rejected_inputs']['metadata']['order_rejected'] == 1
    assert location['rejected_inputs']['metadata']['observation_id'] == location['healthy_input']['metadata']['observation_id']
    assert identity['same_identity']['data']['identity_generation'] == 1
    assert identity['changed_identity']['data']['identity_generation'] == 2
    assert identity['independent_ages']['data']['identity_generation'] == 2
    assert location['independent_ages']['metadata']['present'] and not location['independent_ages']['metadata']['fresh']
    assert location['independent_ages']['data'] is None
    assert health['independent_ages']['data'] == {'status': 0, 'reported_ready': True, 'error_present': False}
    assert all(rows['heartbeat_loss']['data'] is None and rows['no_heartbeat']['data'] is None
               for rows in reports.values())
    assert identity['rebound_unknowns']['data']['identity_generation'] == 1
    assert identity['rebound_unknowns']['metadata']['session_epoch'] > before['metadata']['session_epoch']
    assert all(location['rebound_unknowns']['data'][key] is None for key in
               ('latitude_deg', 'longitude_deg', 'course_deg', 'vertical_speed_up_m_s', 'height_m', 'height_reference'))
    assert health['rebound_unknowns']['data'] is None
    assert all(rows['clock_fault']['metadata']['clock_fault'] and rows['clock_fault']['data'] is None
               for rows in reports.values())
    assert all(row['metadata']['read_only'] and not row['metadata']['authoritative_for_control']
               for rows in reports.values() for row in rows.values())
    print(result, end='')


if __name__ == '__main__':
    verify(*sys.argv[1:])
