#!/usr/bin/env python3
"""Check publication semantics from the loaded mode catalog provider."""

import json
import subprocess
import sys


def verify(driver, module):
    result = subprocess.check_output([driver, module], text=True, timeout=8)
    observations = {row['phase']: row['posts'] for row in map(json.loads, result.splitlines())}
    tables = {phase: posts['MAVMOOS_MODE_CATALOG'] for phase, posts in observations.items()}
    current = {phase: posts['MAVMOOS_CURRENT_MODE'] for phase, posts in observations.items()}
    assert not tables['prebind']['metadata']['present']
    assert tables['partial']['expected_count'] == 2
    assert tables['partial']['received_count'] == 1 and not tables['partial']['complete']
    assert current['partial']['data']['active_label'] == '17'
    assert current['partial']['data']['intended_label'] == '93'
    complete = tables['complete']
    assert complete['complete'] and complete['sequence_bound'] and complete['revision'] == 254
    assert [item['name'] for item in complete['entries']] == ['Hold', 'Offboard']
    assert complete['entries'][1]['properties'] > 0
    assert complete['metadata']['effective_at'] == 100.12
    assert complete['metadata']['read_at'] == 100.15
    assert complete['metadata']['age_is_upper_bound']
    assert current['complete']['data'] == {
        'source': 'CURRENT_MODE', 'active_custom_mode': 17, 'standard_mode': None,
        'active_label': 'Hold', 'intended_custom_mode': 93, 'intended_label': 'Offboard',
    }
    assert tables['revision_change']['revision'] == 255
    assert not tables['revision_change']['entries'] and not tables['revision_change']['complete']
    assert tables['conflict']['conflict'] and not tables['conflict']['entries']
    assert tables['unversioned']['complete'] and not tables['unversioned']['sequence_bound']
    assert tables['unversioned']['revision'] == 0 and tables['unversioned']['metadata']['wire_rejected'] == 3
    assert tables['newer_heartbeat']['monitor_sequence'] == 0
    assert tables['newer_heartbeat']['metadata']['effective_at'] == 100.70
    assert current['newer_heartbeat']['data']['source'] == 'HEARTBEAT'
    assert current['newer_heartbeat']['data']['intended_custom_mode'] is None
    assert not tables['expired_snapshot']['entries']
    assert current['heartbeat_loss']['data'] is None and tables['heartbeat_loss']['revision'] is None
    assert current['no_custom_flag']['data'] is None
    assert current['numeric_zero']['data']['active_custom_mode'] == 0
    assert current['clock_fault']['metadata']['clock_fault'] and current['clock_fault']['data'] is None
    assert all(row['metadata']['read_only'] and not row['metadata']['authoritative_for_control']
               for row in current.values())
    print(result, end='')


if __name__ == '__main__':
    verify(*sys.argv[1:])
