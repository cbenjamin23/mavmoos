"""Fly a stock QuadPlane hover dogleg with a waypoint-triggered climb."""
from autonomy_missions.stock_grade import checkpoints, grade, local_xy, stage_row
from flight_fixture import HOME, need
from hover_helm import acquire, adapter_loss, hold_ready
from live_helm import GROUND, GUIDED, MC, QRTL, archived_rows, observe, wire
from replay import verify_capture


def project(row):
    return local_xy(row, HOME[0], HOME[1])


def survey(flight):
    probe = flight.probe
    initial = acquire(flight, fixed_inputs=False)

    def stage(value, after, timeout):
        return flight.wait(timeout, lambda: stage_row(probe.last.get('AUTONOMY_STAGE'), value, after),
                           f'stock QuadPlane phase {value} not observed')

    climb = stage('climb', initial['request_at'], 19)
    flight.wait(4, lambda: observe(flight, GUIDED, MC, climb['mono'], minimum=61),
                'independent climb did not start before adapter loss')
    loss = adapter_loss(flight, fixed_inputs=False)
    hold_ready(flight, loss['quiet']['through'])
    reacquired = acquire(flight, fixed_inputs=False)
    done = stage('done', reacquired['request_at'], 28)
    fixes = [row for row in flight.monitor.messages if row['mavpackettype'] == 'GLOBAL_POSITION_INT'
             and initial['request_at'] <= row['mono'] <= done['mono'] + .25]
    hit = checkpoints(fixes, [(8, 0), (8, 7), (15, 7)], 2.9, project)
    height = flight.wait(10, lambda: observe(flight, GUIDED, MC, done['mono'], minimum=63.5),
                         'independent post-climb hover altitude absent')
    climb_m = height['position']['relative_alt'] / 1000 - hit[0]['fix']['relative_alt'] / 1000
    need(climb_m >= 3, 'stock waypoint phase did not produce a physical climb')
    at, _ = flight.operator_mode(QRTL, True)
    retired = flight.wait(5, lambda: probe.runtime(at, permission='false', fc_mode=str(QRTL)),
                          'native QRTL did not retire the helm grant')
    touchdown = flight.wait(125, lambda: flight.observation(after=at, mode=QRTL, armed=False,
                              phase=MC, landed=GROUND), 'native QRTL did not land and disarm')
    verdict = grade(probe.post, lambda check: flight.wait(5, check, 'stock QuadPlane mission grade absent'),
                    flight.out)
    return {'acquisition': initial, 'climb_stage': climb, 'done_stage': done,
            'checkpoints': hit, 'climb_m': climb_m, 'high_hover': height,
            'adapter_loss': loss, 'reacquired': reacquired,
            'qrtl_requested_at': at, 'retired': retired, 'touchdown': touchdown, 'grade': verdict}


def audit(out, report):
    count = verify_capture(out, 'monitor')
    monitor = archived_rows(out, 'monitor-rx.jsonl')
    moos = archived_rows(out, 'moos.jsonl')
    mission = report['autonomy']
    need(all(hit['fix'] in monitor for hit in mission['checkpoints']) and
         all(mission[key] in moos for key in ('climb_stage', 'done_stage', 'retired')),
         'independent QuadPlane survey and source-qualified phases did not replay')
    decoded = list(wire(out, report))
    targets = [at for at, side, packet in decoded if side == 'T' and packet.get_type() == 'TUNNEL']
    loss = mission['adapter_loss']
    restored = mission['reacquired']['request_at']
    need(any(side == 'h' and at == loss['pending_at'] for at, side, _ in decoded)
         and any(side == 'i' and loss['lost_at'] <= at < loss['restored_at'] for at, side, _ in decoded)
         and not any(loss['hold']['runtime']['mono'] <= at < restored for at in targets),
         'pending target or loss fence did not replay through explicit reacquire')
    need(sum(mission['climb_stage']['mono'] <= at < mission['done_stage']['mono'] for at in targets) >= 3 and
         not any(at >= mission['retired']['mono'] for at in targets),
         'hover adapter traffic did not respect the mission handoff')
    return {'monitor_crc_frames': count, 'hover_targets': len(targets), 'targets_after_retirement': 0}
