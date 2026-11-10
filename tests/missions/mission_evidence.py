"""Offline acceptance checks for the owned disarmed mission-data fixture."""
import json
import math

LEGACY_CAPABILITIES = {'missions.download', 'missions.upload.simulator', 'missions.clear.simulator',
                       'missions.cancel', 'missions.progress'}
INSPECTION_CAPABILITIES = {'missions.inspect.flight', 'missions.inspect.fence', 'missions.inspect.rally'}
CAPABILITIES_BY_ABI = {9: LEGACY_CAPABILITIES, 10: LEGACY_CAPABILITIES | INSPECTION_CAPABILITIES}


def mission_capabilities_match(module, abi):
    expected = CAPABILITIES_BY_ABI.get(abi)
    names = [capability['name'] for capability in module['capabilities']]
    return expected is not None and len(names) == len(expected) and set(names) == expected


def validate_snapshots(events):
    snapshots = [json.loads(e['value']) for e in events if e.get('type') == 'snapshot']
    expected_ids = {'missions-initial': 0, 'missions-upload': 3, 'missions-download': 3,
                    'missions-clear': 0, 'missions-empty': 0}
    if len(snapshots) != len(expected_ids) or {s['id'] for s in snapshots} != set(expected_ids):
        raise RuntimeError('missing or repeated complete mission snapshot')
    for snapshot in snapshots:
        expected = expected_ids[snapshot['id']]
        if snapshot['schema'] != 1 or snapshot['status'] != 'complete' or snapshot['count'] != expected:
            raise RuntimeError('snapshot state/count mismatch')
        rows = snapshot['items']
        if len(rows) != expected or [r['seq'] for r in rows] != list(range(expected)):
            raise RuntimeError('snapshot did not deliver each distinct ordered sequence')
        if snapshot['id'] in ('missions-upload', 'missions-clear') and snapshot['verification'] != 'semantic_profile_match':
            raise RuntimeError('mutation completion lacks explicit readback verification')
        for item in rows:
            if len(item['params']) != 4 or not all(math.isfinite(v) for v in item['params'] + [item['z']]):
                raise RuntimeError('nonfinite or malformed observed item')
            if item['current'] not in (0, 1) or item['autocontinue'] not in (0, 1):
                raise RuntimeError('unexpected wire flags in observed item')
        if expected:
            if rows[0]['command'] != 16 or rows[0]['frame'] != 0:
                raise RuntimeError('ArduPilot HOME reference convention not explicit')
            for seq, x, y in [(1, 423584660, -710875890), (2, 423584760, -710875790)]:
                item = rows[seq]
                if (item['command'] != 16 or item['frame'] != 3 or item['autocontinue'] != 1 or item['x'] != x or item['y'] != y
                        or abs(item['z'] - 5) > .010001 or any(item['params'])):
                    raise RuntimeError('observed mission waypoint differs from submitted profile')
    return snapshots


def observed_catalog(events, mode='SITL_CONTROL'):
    values = [json.loads(e['value']) for e in events if e.get('type') == 'capabilities']
    if not values or len({json.dumps(v, sort_keys=True) for v in values}) != 1:
        raise RuntimeError('missing/changing capability composition')
    value = values[0]
    modules = value['composition']['modules']
    if value['schema'] != 1 or value['execution_mode'] != mode:
        raise RuntimeError('wrong native capability mode/ABI')
    if len(modules) != 1 or modules[0]['name'] != 'missions' or modules[0]['control_provider']:
        raise RuntimeError('mission bench unexpectedly loaded a control or unrelated provider')
    if not mission_capabilities_match(modules[0], value['composition']['plugin_abi']):
        raise RuntimeError('wrong mission capability surface')
    return value
