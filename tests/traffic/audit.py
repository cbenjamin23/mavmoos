"""Offline acceptance of traffic snapshots and their forwarded wire values."""

import math

KEY = 'MAVMOOS_TRAFFIC_CONTACTS'
COMPLETE_ICAO = 0x5f0021
PARTIAL_ICAO = 0x05a4c0
COMPLETE = {'icao_address': COMPLETE_ICAO, 'flags_raw': 255, 'simulated': True,
            'reported_tslc_s': 7, 'emitter_type_raw': 1, 'latitude_deg': 42.358456,
            'longitude_deg': -71.087589, 'altitude_type_raw': 1, 'altitude_m_asl': 123.45,
            'course_deg': 271.25, 'horizontal_velocity_m_s': 25.75, 'vertical_velocity_up_m_s': -1.25,
            'callsign': 'SIM"\\3', 'squawk_decimal': 7700}
PARTIAL = {**COMPLETE, 'icao_address': PARTIAL_ICAO, 'reported_tslc_s': 255,
           **dict.fromkeys(('latitude_deg', 'longitude_deg', 'altitude_m_asl', 'course_deg',
                            'horizontal_velocity_m_s', 'vertical_velocity_up_m_s',
                            'callsign', 'squawk_decimal'))}


def need(condition, error):
    if not condition:
        raise ValueError(error)


def payload(contact):
    return {key: value for key, value in contact.items() if key != 'metadata'}


def validate(rows, timeout):
    need(bool(rows), 'traffic MOOS publication missing')
    seen = {}
    for row in rows:
        contacts = row['contacts']
        need(row['schema'] == 1 and row['present'] is bool(contacts) and row['read_only'] is True and
             row['atomic'] is False and row['authoritative_for_control'] is False and
             row['coverage'] == 'unknown' and row['track_age'] == 'unverified' and
             row['contact_capacity'] == 32 and len(contacts) <= 32, 'traffic snapshot scope differs')
        addresses = [contact['icao_address'] for contact in contacts]
        need(addresses == sorted(set(addresses)) and all(0 < x <= 0xffffff for x in addresses),
             'traffic contact identities are invalid, duplicate or unsorted')
        latest = row['latest_report']
        need(latest['present'] is bool(contacts), 'traffic summary present state differs')
        if contacts:
            expected = max((contact['metadata'] for contact in contacts), key=lambda meta: meta['observation_id'])
            need(latest == expected, 'latest traffic metadata does not match retained contacts')
        for contact in contacts:
            meta = contact['metadata']
            effective, read, age = (meta[key] for key in ('effective_at', 'read_at', 'age_s'))
            need(meta['schema'] == 1 and meta['present'] is True and meta['fresh'] is True and
                 meta['read_only'] is True and meta['authoritative_for_control'] is False and
                 meta['source_age'] == 'unknown' and meta['sample_timeout_s'] == timeout and
                 meta['session_epoch'] > 0 and meta['observation_id'] > 0 and
                 (meta['system_id'], meta['component_id'], meta['autopilot']) == (1, 1, 3),
                 'traffic provenance, identity or lifetime differs')
            need(all(map(math.isfinite, (effective, read, age))) and 0 < effective <= read and
                 read - effective <= age <= timeout, 'traffic local acquisition age differs')
            need((meta['age_basis'] == 'QUEUE_EMPTY_FLOOR' and meta['age_is_upper_bound'] is True) or
                 (meta['age_basis'] == 'READ_ACQUISITION' and meta['age_is_upper_bound'] is False and
                  effective == read), 'traffic local age basis differs')
            identity = (meta['session_epoch'], meta['observation_id'])
            original = (effective, read, payload(contact))
            need(seen.setdefault(identity, original) == original, 'traffic republication changed original report')
    return seen


def audit_loopback(prebind, prior, all_rows):
    need(bool(prebind) and not any(row['present'] for row in prebind), 'traffic contact present before heartbeat')
    observed = validate(all_rows, .8)
    need(all(value[2] in (COMPLETE, PARTIAL) for value in observed.values()),
         'foreign or malformed traffic entered the contact table')
    both = next((index for index, row in enumerate(prior) if len(row['contacts']) == 2), None)
    need(both is not None, 'two concurrent traffic identities missing')
    epoch = prior[both]['latest_report']['session_epoch']
    partial_expiry = next((index for index, row in enumerate(prior[both + 1:], both + 1)
                           if row['expired'] >= 1 and [x['icao_address'] for x in row['contacts']] == [COMPLETE_ICAO]), None)
    need(partial_expiry is not None, 'individual traffic contact did not expire while another was refreshed')
    tail = prior[partial_expiry:]
    need(any(not row['present'] and row['latest_report']['system_id'] == 1 and row['expired'] >= 2
             for row in tail), 'traffic contacts did not expire independently of heartbeat')
    need(any(not row['present'] and row['latest_report']['system_id'] is None and
             row['latest_report']['session_epoch'] > epoch for row in tail), 'lost traffic session did not clear')
    need(not any(row['present'] and row['latest_report']['session_epoch'] > epoch for row in tail),
         'data-only packets restored an expired traffic session')
    need(any(row['latest_report']['wire_rejected'] > 0 for row in prior),
         'invalid ICAO did not reach the traffic rejection counter')
    recovered = next((row for row in all_rows[len(prior):] if len(row['contacts']) == 2 and
                      row['latest_report']['session_epoch'] > epoch), None)
    need(recovered is not None, 'two traffic contacts did not return after rebinding')
    repeated_reports = sum(value[2] == COMPLETE for value in observed.values())
    need(repeated_reports > 1, 'identical traffic payload did not receive advancing local observations')
    return {'initial_epoch': epoch, 'recovered_epoch': recovered['latest_report']['session_epoch'],
            'distinct_observations': len(observed), 'repeated_complete_payload_observations': repeated_reports,
            'per_contact_expiry': True, 'data_only_rebind_rejected': True}


def audit_native(rows, wire):
    observed = validate(rows, 3)
    # The pinned SITL producer generates fully populated reports under the
    # explicit Shortcut fixture. Check that signature before comparing values.
    contacts = [frame for frame in wire if frame['mavpackettype'] == 'ADSB_VEHICLE']
    need(len(contacts) >= 4, 'native ADS-B producer did not forward enough reports')
    signatures = set()
    for frame in contacts:
        address = frame['ICAO_address']
        callsign = frame['callsign']
        need((frame['system'], frame['component']) == (1, 1) and 0 <= address < 10000 and
             callsign == f'SIM{address}' and frame['flags'] == 511 and frame['tslc'] == 1 and
             frame['altitude_type'] == 0 and frame['squawk'] == 1200,
             'forwarded ADS-B message lacks exact SIM_ADSB producer signature')
        signatures.add((address, frame['flags'], frame['tslc'], frame['emitter_type'],
                        frame['lat'] / 1e7, frame['lon'] / 1e7, frame['altitude_type'],
                        frame['altitude'] / 1e3, frame['heading'] / 1e2,
                        frame['hor_velocity'] / 1e2, frame['ver_velocity'] / 1e2,
                        callsign, frame['squawk']))
    names = ('icao_address', 'flags_raw', 'reported_tslc_s', 'emitter_type_raw', 'latitude_deg',
             'longitude_deg', 'altitude_type_raw', 'altitude_m_asl', 'course_deg', 'horizontal_velocity_m_s',
             'vertical_velocity_up_m_s', 'callsign', 'squawk_decimal')
    for _effective, _read, value in observed.values():
        need(value['simulated'] is True and tuple(value[name] for name in names) in signatures,
             'MOOS ADS-B contact differs from independently replayed native wire')
    addresses = sorted({value[2]['icao_address'] for value in observed.values()})
    need(len(addresses) >= 2 and len(observed) >= 4 and any(len(row['contacts']) >= 2 for row in rows),
         'native concurrent traffic contacts or advancing observations missing')
    return {'result': 'PASS', 'forwarded_frames': len(contacts), 'fresh_observations': len(observed),
            'observed_icao_addresses': addresses, 'simulated_payload_signature_seen': True}
