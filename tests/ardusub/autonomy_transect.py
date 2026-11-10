"""Follow a stock two-depth transect and recover via native Surface."""
from autonomy_missions.stock_grade import checkpoints, local_xy, stage_row
from plane_native.flight_fixture import HOME, need

GUIDED, SURFACE, DEPTH_HOLD = 4, 9, 2


def project(row):
    return local_xy(row, HOME[0], HOME[1])


def transect(bench):
    probe = bench.probe
    acquisition = bench.acquire(route=True)
    home = bench.sample('HOME_POSITION')
    need(home, 'independent Sub surface datum missing')

    def depth(row):
        return (home['altitude'] - row['alt']) / 1000

    def positions(a, b):
        return [row for row in bench.monitor.messages if row['mavpackettype'] == 'GLOBAL_POSITION_INT'
                and a <= row['mono'] <= b]

    def stage(value, after, timeout):
        return bench.wait(timeout, lambda: stage_row(probe.last.get('AUTONOMY_STAGE'), value, after),
                          f'stock Sub phase {value} not observed')

    change = stage('deep', acquisition['requested'], 22)
    shallow = positions(acquisition['requested'], change['mono'] + .2)
    hits = checkpoints([row for row in shallow if 1.4 <= depth(row) <= 2.5], [(5, 0)], 1.6, project)
    done = stage('done', change['mono'], 35)
    deep = positions(change['mono'], done['mono'] + .2)
    hits += checkpoints([row for row in deep if 2.4 <= depth(row) <= 3.6], [(5, 5), (1, 5)], 1.6, project)
    start = bench.wait(4, lambda: bench.sample('GLOBAL_POSITION_INT', done['mono'], .5),
                       'independent recovery fix absent')
    need(depth(start) >= 2.5 and bench.heartbeat(GUIDED, True), 'native Surface needs an owned submerged handoff')
    request = probe.post('MAVMOOS_SUB_HANDBACK', 'surface')
    entered = bench.wait(5, lambda: bench.heartbeat(SURFACE, True, request), 'independent Surface entry absent')
    completed = bench.wait(45, lambda: probe.match('MAVMOOS_SUB_HANDBACK_RESULT', request, status='COMPLETED'),
                           'native Surface did not complete')
    held = bench.wait(6, lambda: bench.heartbeat(DEPTH_HOLD, True, entered['mono']),
                      'native shallow Depth Hold absent')
    end = bench.wait(3, lambda: bench.sample('GLOBAL_POSITION_INT', held['mono'], .5),
                     'independent shallow fix absent')
    need(-.6 <= depth(end) <= .35 and depth(start) - depth(end) >= 2.1
         and probe.runtime(request, permission='false'), 'independent Surface or authority readback differs')
    return {'acquisition': acquisition, 'deeper_stage': change, 'done_stage': done, 'checkpoints': hits,
            'checkpoint_depths_m': [depth(hit['fix']) for hit in hits],
            'surface_entry': entered, 'surface_result': completed, 'depth_hold': held,
            'surface_depth_m': depth(end), 'ascent_m': depth(start) - depth(end)}
