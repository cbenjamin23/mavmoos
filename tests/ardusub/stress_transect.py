#!/usr/bin/env python3
"""Sub depth transect across native handback, Manual and a bridge watchdog."""
import argparse
import json
import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from autonomy_missions.stock_grade import checkpoints, local_xy, stage_row
from plane_faults import fault_tap
from plane_native.flight_fixture import HOME, json_data, need, now
from pymavlink.dialects.v20 import ardupilotmega as mav

DEPTH_HOLD, GUIDED, SURFACE, POSITION_HOLD, MANUAL = 2, 4, 9, 16, 19
SCALARS = ('DESIRED_HEADING', 'DESIRED_SPEED', 'DESIRED_DEPTH')
NAV_CONTROL, RUNTIME, HELM_STATUS = 'MAVMOOS_NAV_CONTROL', 'MAVMOOS_RUNTIME_STATUS', 'MAVMOOS_HELM_STATUS'
HANDBACK, HANDBACK_RESULT = 'MAVMOOS_SUB_HANDBACK', 'MAVMOOS_SUB_HANDBACK_RESULT'
HOLD = 'schema=1,id=operator-hold,action=hold'
GATES = ((5, 0), (5, 5), (0, 5))
DEPTH_BANDS = ((1.4, 2.5), (2.4, 3.6), (2.4, 3.6))
OBSERVATION_INTERVALS = {
    (mav.MAVLINK_MSG_ID_SYS_STATUS, 250000),
    (mav.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 50000),
    (mav.MAVLINK_MSG_ID_EKF_STATUS_REPORT, 500000),
    (mav.MAVLINK_MSG_ID_HOME_POSITION, 500000),
    (mav.MAVLINK_MSG_ID_EXTENDED_SYS_STATE, 250000),
}


def position(row):
    return local_xy(row, HOME[0], HOME[1])


def depth(home, row):
    return (home['altitude'] - row['alt']) / 1000


def gate(rows, home, index, start, finish):
    low, high = DEPTH_BANDS[index]
    candidates = (r for r in rows if r['mavpackettype'] == 'GLOBAL_POSITION_INT' and
                  start <= r['mono'] <= finish and low <= depth(home, r) <= high)
    return checkpoints(candidates, [GATES[index]], 1.6, position)[0]


def stock_tuple(probe, after):
    rows = {key: probe.last.get(key, {}) for key in SCALARS}
    if all(row.get('source') == 'pHelmIvP' and after < row['mono'] and now()-row['mono'] < .75
           for row in rows.values()) and rows['DESIRED_DEPTH']['value'] == 3. and rows['DESIRED_SPEED']['value'] > 0:
        return rows
    return None


def denied_acquire(bench, mode, after):
    probe = bench.probe
    retired = bench.wait(3, lambda: probe.runtime(after, permission='false', fc_mode=str(mode)),
                         f'bridge did not observe native Sub mode {mode}')
    scalars = bench.wait(4, lambda: stock_tuple(probe, retired['mono']),
                         'fresh deeper stock output missing behind the native fence')
    count = probe.navigation_count()
    request = probe.post(NAV_CONTROL, 'acquire')
    denied = bench.wait(3, lambda: probe.match(HELM_STATUS, request, reason='acquisition_rejected'),
                        f'helm acquire did not reject in native Sub mode {mode}')
    heartbeat = bench.wait(3, lambda: bench.heartbeat(mode, True, denied['mono']),
                           'wrong-time request changed the independently observed mode')
    need(probe.navigation_count() == count, 'wrong-time request emitted a helm target')
    return {'retired': retired, 'scalars': scalars, 'requested': request, 'denied': denied, 'native': heartbeat}


def stress(bench):
    probe, monitor = bench.probe, bench.monitor.messages
    report = bench.report['stress'] = {'initial': bench.acquire(route=True)}
    home = bench.sample('HOME_POSITION')
    need(home, 'independent Sub surface datum missing')
    report['home'] = home

    def stage(value, after, timeout):
        return bench.wait(timeout, lambda: stage_row(probe.last.get('AUTONOMY_STAGE'), value, after),
                          f'stock Sub stage {value} not observed')

    deep = stage('deep', report['initial']['requested'], 22)
    report['deep_stage'] = deep
    hits = report['checkpoints'] = [gate(monitor, home, 0, report['initial']['requested'], deep['mono'] + .2)]

    def ready_for_hold():
        if probe.runtime(deep['mono'], permission='true', fc_mode=str(GUIDED)):
            return stock_tuple(probe, deep['mono'])
        return None

    report['pre_hold_stock'] = bench.wait(4, ready_for_hold, 'deeper stock tuple lost Guided ownership before handback')
    hold_at = probe.post(HANDBACK, HOLD)
    native = bench.wait(5, lambda: bench.heartbeat(POSITION_HOLD, True, hold_at),
                        'native Position Hold missing after the bridge handback')
    confirmed = bench.wait(3, lambda: probe.match(HANDBACK_RESULT, hold_at, id='operator-hold', status='CONFIRMED'),
                           'bridge did not confirm the native Position Hold handback')
    retired = bench.wait(3, lambda: probe.runtime(native['mono'], permission='false', fc_mode=str(POSITION_HOLD)),
                         'Position Hold did not retire helm')

    def settled(after):
        fix = bench.sample('GLOBAL_POSITION_INT', after, .5)
        return fix if fix and math.hypot(fix['vx'], fix['vy']) <= 18 else None

    stationary = bench.wait(6, lambda: settled(retired['mono']), 'native Position Hold did not stop the Sub')
    still = bench.wait(3, lambda: settled(stationary['mono'] + 1), 'no stationary native Position Hold dwell')
    need(math.dist(position(stationary), position(still)) <= .35 and abs(depth(home, still) - depth(home, stationary)) <= .3,
         'native Position Hold drifted during the fenced dwell')
    report['hold'] = {'requested': hold_at, 'native': native, 'confirmed': confirmed, 'retired': retired,
                      'dwell_start': stationary, 'dwell_end': still}

    pilot = bench.mode(MANUAL)
    report['manual'] = {'entry': pilot, **denied_acquire(bench, MANUAL, pilot['mono'])}
    repeat = probe.post(HANDBACK, HOLD)
    report['manual']['duplicate'] = bench.wait(3, lambda: probe.match(
        HANDBACK_RESULT, repeat, id='operator-hold', status='DUPLICATE'),
        'old handback ID was not rejected during pilot Manual')
    bench.mode(POSITION_HOLD)
    report['deep_acquire'] = bench.acquire(route=True, depth=3.)
    deep_start = bench.wait(3, lambda: bench.sample('GLOBAL_POSITION_INT', report['deep_acquire']['heartbeat']['mono'], .5),
                            'deep leg lacks an independently observed start')
    cross = stage('cross', report['deep_acquire']['requested'], 24)
    report['cross_stage'], report['deep_start'] = cross, deep_start
    hits.append(gate(monitor, home, 1, deep_start['mono'], cross['mono'] + .2))

    lost = bench.pause('iMAVLink')
    fallback = bench.wait(8, lambda: bench.heartbeat(DEPTH_HOLD, True, lost),
                          'firmware watchdog did not select native Depth Hold')
    resumed = now()
    bench.resume()
    report['watchdog'] = {'paused': lost, 'entry': fallback, 'resumed': resumed,
                          **denied_acquire(bench, DEPTH_HOLD, resumed)}
    bench.mode(POSITION_HOLD)
    report['cross_acquire'] = bench.acquire(route=True, depth=3.)
    cross_start = bench.wait(3, lambda: bench.sample('GLOBAL_POSITION_INT', report['cross_acquire']['heartbeat']['mono'], .5),
                             'cross leg lacks an independently observed start')
    done = stage('done', report['cross_acquire']['requested'], 24)
    report['done_stage'], report['cross_start'] = done, cross_start
    hits.append(gate(monitor, home, 2, cross_start['mono'], done['mono'] + .2))

    start = bench.wait(3, lambda: bench.sample('GLOBAL_POSITION_INT', done['mono'], .5),
                       'independent submerged recovery fix absent')
    need(depth(home, start) >= 2.5 and bench.heartbeat(GUIDED, True), 'Surface recovery needs a submerged Guided start')
    request = probe.post(HANDBACK, 'surface')
    retired = bench.wait(3, lambda: probe.runtime(request, permission='false'), 'Surface did not retire helm')
    entered = bench.wait(5, lambda: bench.heartbeat(SURFACE, True, request), 'native Surface entry absent')
    completed = bench.wait(45, lambda: probe.match(HANDBACK_RESULT, request, status='COMPLETED'),
                           'native Surface did not confirm its independent ascent')
    held = bench.wait(6, lambda: bench.heartbeat(DEPTH_HOLD, True, entered['mono']), 'native shallow Depth Hold absent')
    end = bench.wait(3, lambda: bench.sample('GLOBAL_POSITION_INT', held['mono'], .5), 'independent shallow fix absent')
    report['surface'] = {'requested': request, 'retired': retired, 'entry': entered, 'completed': completed,
                         'depth_hold': held, 'start': start, 'end': end}


def audit(directory, report):
    """Replay the generated phase ledger, simulator datagrams and exact bridge route."""
    proof = report['stress']
    moos = [json.loads(line) for line in (directory/'moos.jsonl').read_text().splitlines()]
    monitor = [json.loads(line) for line in (directory/'monitor-rx.jsonl').read_text().splitlines()]
    raw = (directory/'monitor-rx.mavlink').read_bytes()
    ports = report['ports']
    records = fault_tap.read_trace(directory/'app-wire.plf', ports['bridge'], ports['bridge_peer'])
    modes, targets, streams = [], [], []
    for row in records:
        need(row.disposition in ('T', 'R'), 'unexpected disposition on the audited Sub pass-through route')
        if row.disposition != 'T':
            continue
        frames = mav.MAVLink(None).parse_buffer(row.data) or []
        need(len(frames) == 1 and bytes(frames[0].get_msgbuf()) == row.data and
             (frames[0].get_srcSystem(), frames[0].get_srcComponent()) == (245, 191), 'bridge wire identity differs')
        packet = frames[0]
        kind = packet.get_type()
        if kind in ('COMMAND_LONG', 'SET_POSITION_TARGET_LOCAL_NED'):
            need((packet.target_system, packet.target_component) == (1, 1), 'Sub wire target differs')
        if kind == 'COMMAND_LONG':
            need(packet.confirmation == 0 and all(getattr(packet, f'param{i}') == 0 for i in range(3, 8)),
                 'unexpected Sub command arguments')
            if packet.command == mav.MAV_CMD_SET_MESSAGE_INTERVAL:
                interval = (packet.param1, packet.param2)
                need(interval in OBSERVATION_INTERVALS, 'unexpected Sub observation stream request')
                streams.append((row.at, interval))
            else:
                need(packet.command == mav.MAV_CMD_DO_SET_MODE and packet.param1 == 1, 'unexpected Sub mode command')
                modes.append((row.at, int(packet.param2)))
        elif kind == 'SET_POSITION_TARGET_LOCAL_NED':
            targets.append((row.at, packet))
        else:
            need(kind == 'HEARTBEAT', 'unexpected frame on the Sub control route')

    initial, deep, cross = (proof[key] for key in ('initial', 'deep_acquire', 'cross_acquire'))
    hold, manual, loss, surface = (proof[key] for key in ('hold', 'manual', 'watchdog', 'surface'))
    for start, finish in ((0, initial['requested']), (loss['resumed'], cross['requested'])):
        intervals = [interval for at, interval in streams if start <= at < finish]
        need(len(intervals) == len(set(intervals)), 'duplicate stream request in one observation epoch')
    need(all(at < initial['requested'] or loss['resumed'] <= at < cross['requested'] for at, _ in streams),
         'observation stream request outside startup or watchdog recovery')
    expected = [(initial['requested'], initial['heartbeat'], GUIDED), (hold['requested'], hold['native'], POSITION_HOLD),
                (deep['requested'], deep['heartbeat'], GUIDED), (cross['requested'], cross['heartbeat'], GUIDED),
                (surface['requested'], surface['entry'], SURFACE)]
    need(len(modes) == len(expected) and all(begin <= sent <= readback['mono'] and
         readback['base_mode'] & 128 and value == mode == readback['custom_mode']
         for (sent, value), (begin, readback, mode) in zip(modes, expected)), 'wrong mode request count, order or timing')
    need(all(not any(begin <= at <= end for at, _ in targets) for begin, end in (
         (modes[1][0], deep['requested']), (loss['entry']['mono'], cross['requested']),
         (modes[4][0], report['disarm']['mono']))), 'navigation escaped a native authority fence')
    phases = [proof[key] for key in ('deep_stage', 'cross_stage', 'done_stage')]
    need(all(row in moos and row['source'] == 'pHelmIvP' and row['value'] == value
             for row, value in zip(phases, ('deep', 'cross', 'done'))) and
         not any(r['kind'] == 'request' and (r['key'].startswith('DESIRED_') or r['key'] == 'AUTONOMY_STAGE')
                 for r in moos), 'mission progress was not produced by the stock helm')
    starts = [a['heartbeat']['mono'] for a in (initial, deep, cross)]
    need(all(start < end['mono'] and any(start <= at <= end['mono'] and
             packet.coordinate_frame == mav.MAV_FRAME_LOCAL_NED and math.hypot(packet.vx, packet.vy) > .15
             for at, packet in targets) for start, end in zip(starts, phases)), 'missing stock-guided motion leg')
    need(all(row in moos and row['source'] == 'pHelmIvP' and a['requested'] < row['mono']
             for a in (initial, deep, cross) for row in a['stock_scalars'].values()),
         'stock acquisition scalar readback differs')

    def witnessed(row):
        row = json_data(row)
        need(row in monitor, 'independent Sub witness absent from monitor log')
        offset, length = row['datagram_offset'], row['datagram_length']
        decoded = mav.MAVLink(None).parse_buffer(raw[offset:offset + length]) or []
        fields = {k: v for k, v in row.items() if k not in ('mono', 'datagram_offset', 'datagram_length')}
        need(any((p.get_srcSystem(), p.get_srcComponent()) == (1, 1) and json_data(p.to_dict()) == fields
                 for p in decoded), 'independent Sub witness did not replay from CRC-checked bytes')

    home = proof['home']
    hits = [gate(monitor, home, i, start, finish['mono'] + .2)
            for i, (start, finish) in enumerate(zip((initial['requested'], proof['deep_start']['mono'],
                                                    proof['cross_start']['mono']), phases))]
    need(json_data(hits) == json_data(proof['checkpoints']), 'independent Sub checkpoints differ')
    progress, finishes = [], []
    for i, key in enumerate(('deep_start', 'cross_start'), 1):
        start = proof[key]
        low, high = DEPTH_BANDS[i]
        # The gate records the first entry into its tolerance circle. Measure
        # travel through the completed leg, including the approach after entry.
        finish = min((r for r in monitor if r['mavpackettype'] == 'GLOBAL_POSITION_INT' and
                      hits[i]['fix']['mono'] <= r['mono'] <= phases[i]['mono'] + .2 and
                      low <= depth(home, r) <= high),
                     key=lambda r: math.dist(position(r), GATES[i]))
        finishes.append(finish)
        progress.append(math.dist(position(start), position(finish)))
    need(min(progress) >= 2.5, 'physical progress after native recovery did not meet the distance gate')
    for fence in (manual, loss):
        need(fence['denied'] in moos and 'reason=acquisition_rejected,' in fence['denied']['value'] and
             fence['retired']['mono'] < fence['requested'] <= fence['denied']['mono'] < fence['native']['mono'] and
             all(r in moos and r['source'] == 'pHelmIvP' and fence['retired']['mono'] < r['mono'] < fence['requested']
                 for r in fence['scalars'].values()), 'wrong-time acquire or fresh stock fence did not replay')
    need(manual['duplicate'] in moos and 'status=DUPLICATE,' in manual['duplicate']['value'] and
         hold['confirmed'] in moos and surface['completed'] in moos,
         'duplicate native handback did not replay')
    need((manual['native']['custom_mode'], loss['native']['custom_mode']) == (MANUAL, DEPTH_HOLD) and
         loss['paused'] < loss['entry']['mono'] < loss['resumed'], 'independent pilot or watchdog mode sequence differs')
    for row in [home, *(h['fix'] for h in hits), *finishes, proof['deep_start'], proof['cross_start'],
                hold['dwell_start'],
                hold['dwell_end'], manual['entry'], manual['native'], loss['entry'], loss['native'], surface['start'],
                surface['end'], surface['depth_hold'], report['disarm'], *(p for _, p, _ in expected)]:
        witnessed(row)
    need(hold['dwell_end']['mono'] - hold['dwell_start']['mono'] >= 1 and
         math.dist(position(hold['dwell_start']), position(hold['dwell_end'])) <= .35 and
         abs(depth(home, hold['dwell_end']) - depth(home, hold['dwell_start'])) <= .3,
         'native Position Hold stationary dwell did not replay')
    ascent = depth(home, surface['start']) - depth(home, surface['end'])
    need(ascent >= 2.1 and -.6 <= depth(home, surface['end']) <= .35 and
         surface['depth_hold']['custom_mode'] == DEPTH_HOLD and not report['disarm']['base_mode'] & 128 and
         (directory/'results.txt').read_text().strip() == proof['grade'] == 'grade=pass',
         'native Surface ascent, terminal disarm, or stock evaluation differs')
    return {'bridge_datagrams': len(records), 'bridge_modes': [value for _, value in modes], 'helm_targets': len(targets),
            'observation_stream_requests': len(streams), 'wrong_time_acquires_denied': 2,
            'targets_inside_native_fences': 0,
            'checkpoint_depths_m': [depth(home, hit['fix']) for hit in hits], 'post_reacquire_travel_m': progress,
            'native_ascent_m': ascent}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    directory = parser.parse_args().directory
    report = json.loads((directory/'result.json').read_text())
    need(report['result'] == 'PASS' and report['stress_requested'] and report['hardware'] is False and
         report['cleanup_proven'] and not report['input_drift'], 'replay requires a passing scoped stress mission')
    readback = audit(directory, report)
    need(readback == report['stress_wire_audit'], 'replayed audit differs from the recorded receipt')
    print(json.dumps(readback, indent=2))
