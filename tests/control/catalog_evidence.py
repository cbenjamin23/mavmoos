"""Validate the app's observed composition; inspecting files is not loaded-state proof."""
import hashlib
import json
from pathlib import Path

COPTER_LIFECYCLE = {'control.arm', 'control.disarm', 'control.takeoff', 'control.hold.native_loiter',
                    'control.land', 'control.return.native_rtl', 'control.guided'}
NATIVE = COPTER_LIFECYCLE | {'control.operations.native', 'control.operations.guided', 'control.operations.cancel'}
HELM = {'control.navigation.helm', 'control.hold.guided_recovery'}
ROVER_NATIVE = {'control.arm', 'control.disarm', 'control.guided',
                'control.hold.native_stop', 'control.return.native_rtl',
                'control.operations.native', 'control.operations.guided', 'control.operations.cancel'}
ROVER_HELM = {'control.navigation.helm', 'control.hold.guided_zero_velocity'}
ROVER_OWNERS = {'MAVMOOS_COMMAND', 'MAVMOOS_NAV_CONTROL', 'MAVMOOS_CONTROL_REQUEST'}


def observed_catalog(path, expected_modules, native=False, helm=False, mode=None,
                     operations_helm=False, mission_execution=False, control_profile=None,
                     mission_resume=False, frame_lifecycle=False):
    observations = []
    for line in Path(path).read_text().splitlines():
        if line.startswith('CAPTURE\tMAVMOOS_CAPABILITIES\t'):
            observations.append(json.loads(line.split('\t', 2)[2]))
            continue
        try:
            event = json.loads(line)
        except json.JSONDecodeError:
            continue
        if isinstance(event, dict) and event.get('key') == 'MAVMOOS_CAPABILITIES':
            observations.append(json.loads(event['value']))
    if not observations or len({json.dumps(c, sort_keys=True) for c in observations}) != 1:
        raise RuntimeError('missing or changing observed capability composition')
    catalog = observations[0]
    if catalog.get('schema') != 1 or (mode is not None and catalog.get('execution_mode') != mode):
        raise RuntimeError('observed capability schema/execution mode differs from requested profile')
    composition = catalog.get('composition', {})
    modules = composition.get('modules', [])
    names = [m['name'] for m in modules]
    if composition.get('plugin_abi') != 10 or len(names) != len(set(names)) or set(names) != set(expected_modules):
        raise RuntimeError('observed capability modules/ABI differ from the requested profile')
    controls = [m for m in modules if m['control_provider']]
    expected_controls = set(expected_modules) & {'arducopter_control', 'ardurover_control'}
    if len(expected_controls) > 1 or {m['name'] for m in controls} != expected_controls:
        raise RuntimeError('unexpected number of control providers')
    capabilities = [c for m in modules for c in m['capabilities']]
    ids = [c['name'] for c in capabilities]
    owners = [r for m in modules for r in m['request_owners']]
    if len(ids) != len(set(ids)) or len(owners) != len(set(owners)):
        raise RuntimeError('ambiguous capability or request ownership in observed catalog')
    rover = 'ardurover_control' in expected_controls
    native_capabilities = ROVER_NATIVE if rover else COPTER_LIFECYCLE if frame_lifecycle else NATIVE
    expected_control = (native_capabilities if native else set()) | ((ROVER_HELM if rover else HELM) if helm else set())
    if rover:
        profile = 'ardurover' if control_profile is None else control_profile
        if profile not in ('ardurover', 'ardurover_boat') or controls[0].get('profile') != profile:
            raise RuntimeError('observed Rover control profile differs from selected subtype')
        if set(controls[0]['request_owners']) != ROVER_OWNERS:
            raise RuntimeError('Rover control provider is not the sole owner of all control requests')
        if native and profile == 'ardurover_boat':
            expected_control |= {'control.hold.native_loiter'}
        for cap in controls[0]['capabilities']:
            if cap['name'].startswith('control.operations.') and (
                    cap['access'] != 'request' or cap['request_variable'] != 'MAVMOOS_CONTROL_REQUEST'):
                raise RuntimeError('Rover operation is not bound to the versioned request')
    if operations_helm:
        if rover or not helm:
            raise RuntimeError('invalid operation-helm evidence expectation')
        expected_control |= {'control.operations.helm', 'control.operations.cancel'}
    if mission_execution:
        if rover or not native:
            raise RuntimeError('mission execution requires native Copter capability')
        expected_control |= {'control.mission.start', 'control.mission.pause', 'control.mission.abort', 'control.mission.release', 'control.mission.progress'}
        if mission_resume:
            expected_control.add('control.mission.resume')
    if {name for name in ids if name.startswith('control.')} != expected_control:
        raise RuntimeError('observed control variants differ from the enabled profile')
    for capability in capabilities:
        if capability['name'] in {'control.hold.guided_recovery', 'control.hold.guided_zero_velocity'} and (capability['access'] != 'internal' or capability['request_variable']):
            raise RuntimeError('internal guided recovery exposed as public hold')
    return catalog


def loaded_module_hashes(plugin_dir, catalog):
    result = {}
    for module in catalog['composition']['modules']:
        candidates = [p for p in Path(plugin_dir).glob('mavmoos_' + module['name'] + '.*')
                      if p.is_file() and p.suffix in ('.so', '.dylib')]
        if len(candidates) != 1:
            raise RuntimeError('cannot identify the exact loaded module artifact: ' + module['name'])
        path = candidates[0]
        result[path.name] = hashlib.sha256(path.read_bytes()).hexdigest()
    return result
