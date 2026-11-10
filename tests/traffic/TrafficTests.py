#!/usr/bin/env python3
"""Verify traffic field, time, identity, and bounded storage behavior."""
import argparse
import json
import subprocess

from audit import COMPLETE, COMPLETE_ICAO, PARTIAL, PARTIAL_ICAO, need, payload, validate


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--driver', required=True)
    parser.add_argument('--module', required=True)
    args = parser.parse_args()
    rows = [json.loads(line) for line in subprocess.check_output(
        [args.driver, args.module], text=True, timeout=10).splitlines()]
    validate([row['report'] for row in rows], .8)
    phases = {row['phase']: row['report'] for row in rows}
    for name in ('startup', 'unbound', 'heartbeat_lost', 'clock_regression'):
        need(not phases[name]['present'] and phases[name]['latest_report']['system_id'] is None,
             name + ' must clear all traffic contacts')

    def contacts(name):
        return {x['icao_address']: x for x in phases[name]['contacts']}

    initial = contacts('three_contacts')
    need(list(initial) == [42, PARTIAL_ICAO, COMPLETE_ICAO], 'traffic contact identity differs')
    need(payload(initial[COMPLETE_ICAO]) == COMPLETE and payload(initial[PARTIAL_ICAO]) == PARTIAL,
         'decoded MAVLink 1 values or null sentinel mapping differ')
    bare = initial[42]
    need(bare['flags_raw'] == 0 and bare['reported_tslc_s'] == 0 and bare['callsign'] is None and
         all(bare[key] is None for key in ('latitude_deg', 'longitude_deg', 'altitude_m_asl', 'course_deg',
                                         'horizontal_velocity_m_s', 'vertical_velocity_up_m_s', 'squawk_decimal')),
         'trimmed MAVLink 2 identity fabricated a known field')
    summary = phases['three_contacts']['latest_report']
    need(summary['wire_rejected'] == 3 and summary['source_rejected'] == 5,
         'traffic source or malformed report was not rejected')
    original = initial[COMPLETE_ICAO]['metadata']
    need(original['age_basis'] == 'QUEUE_EMPTY_FLOOR' and original['effective_at'] == 100.17 and
         original['read_at'] == 100.18, 'original local floor was replaced by the socket read time')
    repeat = contacts('duplicate_and_out_of_order')[COMPLETE_ICAO]
    need(payload(repeat) == COMPLETE and repeat['metadata']['effective_at'] == 100.26 and
         repeat['metadata']['order_rejected'] == 1, 'old traffic report replaced an identical newer payload')
    need(list(contacts('two_expired')) == [COMPLETE_ICAO] and phases['two_expired']['expired'] == 2 and
         phases['two_expired']['stale_ingress_rejected'] == 1, 'traffic expiry/stale ingress differs')

    full = contacts('at_capacity')
    need(list(full) == [*range(1000, 1031), COMPLETE_ICAO] and phases['at_capacity']['capacity_rejected'] == 1 and
         full[COMPLETE_ICAO]['course_deg'] == 90, 'traffic capacity was bypassed or prevented an existing update')
    need(list(contacts('one_left')) == [COMPLETE_ICAO] and phases['one_left']['expired'] == 33 and
         list(contacts('capacity_available')) == [1031, COMPLETE_ICAO], 'expired traffic did not free capacity')

    lost = phases['heartbeat_lost']['latest_report']
    rebound = phases['reconnected']['latest_report']
    need(lost['metadata_rejected'] == 1 and lost['session_epoch'] > summary['session_epoch'] and
         payload(contacts('reconnected')[COMPLETE_ICAO]) == COMPLETE and
         rebound['session_epoch'] == lost['session_epoch'], 'traffic heartbeat barrier did not fence old frames')
    need(phases['clock_regression']['latest_report']['clock_fault'] is True, 'clock regression remained fresh')


if __name__ == '__main__':
    main()
