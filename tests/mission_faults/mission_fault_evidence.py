"""Pure validation of the disarmed partial-upload fault fixture; no I/O effects."""
import json
import math
from missions.mission_evidence import mission_capabilities_match


def fields(text):
    if not isinstance(text, str):
        raise RuntimeError('non-string provider record')
    result = {}
    for part in text.split(','):
        if '=' not in part:
            raise RuntimeError('malformed provider record')
        key, value = part.split('=', 1)
        if key in result:
            raise RuntimeError('duplicate record key')
        result[key] = value
    return result


def finite(value, minimum=0):
    if isinstance(value, bool):
        raise RuntimeError('boolean is not numeric evidence')
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as error:
        raise RuntimeError('invalid numeric evidence') from error
    if not math.isfinite(result) or result < minimum:
        raise RuntimeError('nonfinite or out-of-bound evidence')
    return result


def validate(events, scenario):
    try:
        return _validate(events, scenario)
    except (KeyError, ValueError, TypeError, IndexError, OverflowError) as error:
        raise RuntimeError('missing or malformed mission-fault evidence') from error


def _validate(events, scenario):
    if scenario not in ('cancel', 'reset') or not isinstance(events, list) or not events:
        raise RuntimeError('invalid scenario/events')
    previous = -1
    for event in events:
        stamp = finite(event['elapsed'])
        if stamp < previous:
            raise RuntimeError('probe clock moved backwards')
        previous = stamp
    if not any(e['type'] == 'workflow' and e['key'] == 'result' and e['value'] == 'PASS' for e in events):
        raise RuntimeError('probe did not finish')
    catalogs = [json.loads(e['value']) for e in events if e['type'] == 'capabilities']
    if not catalogs or len({json.dumps(c, sort_keys=True) for c in catalogs}) != 1:
        raise RuntimeError('missing/changing observed catalog')
    catalog = catalogs[0]
    modules = catalog['composition']['modules']
    if (catalog['schema'] != 1 or catalog['execution_mode'] != 'SITL_CONTROL' or
            len(modules) != 1 or modules[0]['name'] != 'missions' or modules[0]['control_provider'] or
            not mission_capabilities_match(modules[0], catalog['composition']['plugin_abi']) or
            modules[0]['request_owners'] != ['MAVMOOS_MISSION_REQUEST']):
        raise RuntimeError('unexpected provider/authority composition')
    requests = [fields(e['value']) for e in events if e['type'] == 'request']
    order = [('mf-baseline', 'clear'), ('mf-upload', 'upload')]
    if scenario == 'cancel':
        order.append(('mf-upload', 'cancel'))
    order += [('mf-upload', 'upload'), ('mf-busy-read', 'download'), ('mf-readback', 'download'), ('mf-final-clear', 'clear')]
    if [(r.get('id'), r.get('action')) for r in requests] != order:
        raise RuntimeError('fixture request order changed')
    expected = {'mf-baseline': ('clear', 1), 'mf-upload': ('upload', 2), 'mf-busy-read': ('download', 1),
                'mf-readback': ('download', 1), 'mf-final-clear': ('clear', 1)}
    for ident, (action, count) in expected.items():
        if len([r for r in requests if r.get('id') == ident and r.get('action') == action]) != count:
            raise RuntimeError('missing/repeated explicit fixture request')
    cancellations = [r for r in requests if r.get('action') == 'cancel']
    if len(cancellations) != int(scenario == 'cancel') or any(r != {'id': 'mf-upload', 'action': 'cancel'} for r in cancellations):
        raise RuntimeError('unexpected cancellation request')
    if len(requests) != 6 + int(scenario == 'cancel'):
        raise RuntimeError('unexpected data/execution request')
    uploads = [r for r in requests if r.get('action') == 'upload']
    if uploads[0] != uploads[1] or len(uploads[0]['items'].split('|')) != 64:
        raise RuntimeError('duplicate request did not preserve the bounded table')
    for seq, row in enumerate(uploads[0]['items'].split('|')):
        values = row.split(':')
        expected_row = [str(seq), '16', '0' if seq == 0 else '3', '0', '1', '0', '0', '0', '0',
                        str(423584560 + seq * 10), str(-710875890 + seq * 5), '10' if seq == 0 else '5']
        if values != expected_row:
            raise RuntimeError('uploaded fixture row changed')
    results = [(index, fields(e['value'])) for index, e in enumerate(events) if e['type'] == 'result']
    for ident, (action, _) in expected.items():
        operation = [r for _, r in results if r['id'] == ident and r['status'] != 'DUPLICATE']
        if not operation or any(r['action'] != action for r in operation):
            raise RuntimeError('missing or changed operation action')
        bounds = [finite(r['expires'], 1e-9) for r in operation]
        if len(set(bounds)) != 1 or any(finite(r['at']) >= bounds[0] for r in operation):
            raise RuntimeError('original operation deadline changed or was exceeded')
    old = [(index, r) for index, r in results if r['id'] == 'mf-upload']
    retired = [(index, r) for index, r in old if r['status'] == 'CANCELLED']
    if len(retired) != 1:
        raise RuntimeError('missing/repeated retirement')
    retired_index, terminal = retired[0]
    terminal_at = finite(terminal['at'])
    expected_reason = 'operator_cancelled' if scenario == 'cancel' else 'connection_reset'
    if (terminal['reason'] != expected_reason or terminal['action'] != 'upload' or
            terminal['mutation_may_have_applied'] != 'true' or terminal['remote_accepted'] != 'false' or
            terminal['readback_confirmed'] != 'false'):
        raise RuntimeError('partial-write uncertainty was lost')
    expiries = [finite(r['expires'], 1e-9) for _, r in old if r['status'] != 'DUPLICATE']
    if not expiries or len(set(expiries)) != 1:
        raise RuntimeError('original operation bound changed')
    for _, r in old:
        finite(r['at'])
    if not any(index > retired_index and r['status'] == 'DUPLICATE' for index, r in old):
        raise RuntimeError('old ID was not rejected after retirement')
    forbidden = {'ADMITTED', 'SUBMITTED', 'SENT', 'RETRY_DUE', 'ACKNOWLEDGED', 'COMPLETED'}
    if any(index > retired_index and r['status'] in forbidden for index, r in old):
        raise RuntimeError('retired operation resumed')
    sent = [r for index, r in old if index < retired_index and r['status'] == 'SENT']
    if not sent or not 4 <= max(int(r['seq']) for r in sent) < 63:
        raise RuntimeError('no qualifying partial-upload trigger')
    tx = [(index, fields(e['value'])) for index, e in enumerate(events) if e['type'] == 'tx']
    for _, r in tx:
        finite(r['at'])
    old_accepted = [r for _, r in tx if r['operation_id'] == 'mf-upload' and r['status'] == 'LOCAL_ACCEPTED']
    old_ids = {int(r['msgid']) for r in old_accepted if finite(r['at']) < terminal_at}
    if not {44, 73} <= old_ids:
        raise RuntimeError('missing actual local mutation acceptance')
    if any(finite(r['at']) >= terminal_at for r in old_accepted):
        raise RuntimeError('retired upload accepted at/after terminal host time')
    for _, r in tx:
        ident = r['operation_id'];message = int(r['msgid'])
        permitted = {44, 73} if ident == 'mf-upload' else {43} if ident == 'mf-busy-read' else {43, 51, 47} if ident == 'mf-readback' else {45, 43, 47} if ident in ('mf-baseline', 'mf-final-clear') else set()
        if message not in permitted or r.get('late') != 'false' or r.get('clock_valid') != 'true':
            raise RuntimeError('unexpected message family, late send or bad clock')
    busy = [r for _, r in results if r['id'] == 'mf-busy-read' and r['status'] == 'FAILED']
    if len(busy) != 1 or busy[0]['reason'] != 'remote_mission_error_14':
        raise RuntimeError('remote receive state after local retirement not demonstrated')
    for ident in ('mf-baseline', 'mf-readback', 'mf-final-clear'):
        if len([r for _, r in results if r['id'] == ident and r['status'] == 'COMPLETED']) != 1:
            raise RuntimeError('new explicit recovery operation did not complete')
    assertions = {e['key']: e['value'] for e in events if e['type'] == 'assertion'}
    if not {'stable_home_ground_fixture', 'partial_trigger', 'retirement', 'old_id_not_replayed',
            'remote_busy_after_local_retirement', 'settlement_wait', 'partial_remote_storage', 'final_idle'} <= set(assertions):
        raise RuntimeError('required scenario assertion missing')
    wait = fields(assertions['settlement_wait'])
    if finite(wait['elapsed_since_retirement_s']) < 10.5 or wait['native_source_timeout_s'] != '8' or wait['remote_timeout_observed'] != 'false':
        raise RuntimeError('remote settlement wait/scope invalid')
    wait_index = next(index for index, e in enumerate(events) if e['type'] == 'assertion' and e['key'] == 'settlement_wait')
    read_index = next(index for index, e in enumerate(events) if e['type'] == 'request' and fields(e['value'])['id'] == 'mf-readback')
    if (wait_index <= retired_index or read_index <= wait_index or
            finite(events[wait_index]['elapsed']) - finite(events[retired_index]['elapsed']) < 10.5):
        raise RuntimeError('raw probe chronology does not demonstrate the settlement wait')
    snapshots = [json.loads(e['value']) for e in events if e['type'] == 'snapshot']
    complete = [s for s in snapshots if s.get('status') == 'complete']
    if {s['id'] for s in complete} != {'mf-baseline', 'mf-readback', 'mf-final-clear'} or len(complete) != 3:
        raise RuntimeError('missing/repeated complete snapshot or old upload completion')
    partial = next(s for s in complete if s['id'] == 'mf-readback')
    size = partial['count'];rows = partial['items']
    if (type(size) is not int or not 1 < size < 64 or size > max(int(r['seq']) for r in sent) + 1 or
            len(rows) != size or partial.get('verification') != 'observation' or partial.get('mission_type') != 0 or partial.get('atomic') is not False):
        raise RuntimeError('not an honest partial-prefix observation')
    for seq, row in enumerate(rows):
        if any(type(row[k]) is not int for k in ('seq', 'command', 'frame', 'autocontinue', 'current', 'x', 'y')):
            raise RuntimeError('noninteger wire identity or coordinate')
        if row['seq'] != seq or row['command'] != 16 or row['frame'] != (0 if seq == 0 else 3) or row['autocontinue'] != 1 or row['current'] not in (0, 1) or row['params'] != [0, 0, 0, 0]:
            raise RuntimeError('partial prefix identity/flags changed')
        for value in row['params'] + [row['z']]:
            finite(value, -20000)
        if seq:
            if row['x'] != 423584560 + seq * 10 or row['y'] != -710875890 + seq * 5 or abs(row['z'] - 5) > .010001:
                raise RuntimeError('partial prefix row differs from submitted data')
        elif abs(row['x']) > 900000000 or abs(row['y']) > 1800000000 or row['z'] < -1000 or row['z'] > 20000:
            raise RuntimeError('invalid observed dynamic HOME')
    for s in complete:
        if s['schema'] != 1 or s['status'] != 'complete':
            raise RuntimeError('invalid snapshot schema')
        if s['id'] != 'mf-readback' and (s['count'] != 0 or s['items'] or s['verification'] != 'semantic_profile_match'):
            raise RuntimeError('verified clear was not empty')
    runtimes = [fields(e['value']) for e in events if e['type'] == 'runtime']
    if not runtimes or any(r.get('armed') not in ('0', 'false') or r.get('permission') != 'false' or r.get('nav_tx_accepted') != '0' or r.get('control_profile') != 'none' for r in runtimes):
        raise RuntimeError('control was acquired or unknown')
    if fields(assertions['final_idle']) != {'mission_queue': '0', 'mission_operations': '0', 'mission_mutations': '0', 'armed': 'false', 'permission': 'false'}:
        raise RuntimeError('final local state not idle')
    return {'scenario': scenario, 'catalog': catalog, 'requested_count': 64, 'partial_count': size,
            'retirement_reason': expected_reason, 'mutation_may_have_applied': True,
            'busy_read': 'FAILED_REMOTE_DENIED', 'new_id_readback': 'COMPLETED_PARTIAL_PREFIX',
            'final_clear': 'COMPLETED_VERIFIED_EMPTY', 'remote_timeout_observed': False,
            'settlement_wait_s': finite(wait['elapsed_since_retirement_s']), 'original_upload_expires': expiries[0],
            'dynamic_home_compared': False, 'retired_local_accepted_at_or_after_terminal': 0,
            'retirement_host_at': terminal_at, 'latest_old_local_accepted_host_at': max(finite(r['at']) for r in old_accepted),
            'send_order_evidence': 'producer TX.at and terminal result.at in the same host clock; MOOS delivery order is not producer order'}
