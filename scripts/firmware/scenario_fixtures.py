"""Pinned scenario expectations; selection never observes traffic or grants I/O.

Call select only after the existing strict native-profile verifier. Offline
readers use from_declaration with archived bytes and the same bound identity.
The exact declaration hashes are part of this source, not caller overrides.
"""
import copy
import hashlib
import json
import math
from pathlib import Path
import re

SCENARIOS = {'armed_wiped_mission_read', 'downward_distance'}
PROFILES = {
    'arducopter-dev-04cf6486-quad': {
        'declaration_sha256': 'c834d32f19cd05691bacd8c36fa5bc590c71059ede98f3131934b3a3674230d8',
        'firmware_manifest_sha256': 'ce4643b52a8b09686f3998cda5dc00ecd9e9d1b5e82b1410067b40686eba8e3b',
        'source_revision': '04cf6486e8d42ce138de816e8466244346ed77cb'},
    'arducopter-4.6.3-quad': {
        'declaration_sha256': 'c4b480c1e53147907a971403d2f5a669b4cd227694c5f6426ec4a25a7d45ac25',
        'firmware_manifest_sha256': 'decd934de8057bcc363b273fddd5cc806475fabe9e64475512df71e8f14a3639',
        'source_revision': '92b0cd788ec29406f26c6f9c31d5ceedbd1cc538'}}
SHA = re.compile(r'[0-9a-f]{64}\Z')


def require(value, why):
    if not value:
        raise ValueError(why)


def digest(data):
    return hashlib.sha256(data).hexdigest()


def parse(data):
    def pairs(rows):
        result = {}
        for key, value in rows:
            require(key not in result, 'duplicate scenario declaration key')
            result[key] = value
        return result
    def reject(_):
        raise ValueError('nonfinite scenario declaration')
    def real(value):
        result = float(value)
        require(math.isfinite(result), 'nonfinite scenario declaration')
        return result
    return json.loads(data, object_pairs_hook=pairs, parse_constant=reject, parse_float=real)


def identity_profile(identity):
    require(isinstance(identity, dict) and type(identity.get('schema')) is int and identity['schema'] == 1 and
            identity.get('qualification') == 'not_inferred' and identity.get('frame') == 'quad',
            'scenario selection requires the existing verified native identity')
    profile = identity.get('profile_id')
    require(profile in PROFILES, 'unknown exact scenario profile')
    pin = PROFILES[profile]
    require(identity.get('manifest_sha256') == pin['firmware_manifest_sha256'] and
            identity.get('source_revision') == pin['source_revision'], 'scenario profile/manifest/source mismatch')
    require(all(isinstance(identity.get(key), str) and SHA.fullmatch(identity[key])
                for key in ('native_sha256', 'attestation_sha256')), 'missing native/attestation identity')
    return profile, pin


def from_declaration(data, verified_identity, scenario):
    """Pure exact-byte selection; does not reverify native files or authority."""
    profile, pin = identity_profile(verified_identity)
    require(scenario in SCENARIOS, 'unknown scenario fixture')
    require(isinstance(data, bytes) and 0 < len(data) <= 65536 and digest(data) == pin['declaration_sha256'],
            'scenario declaration bytes are not the pinned source declaration')
    value = parse(data)
    require(set(value) == {'schema', 'kind', 'qualification', 'binding', 'fixtures'} and
            type(value['schema']) is int and value['schema'] == 1 and
            value['kind'] == 'mavmoos.scenario_fixtures' and value['qualification'] == 'not_inferred',
            'scenario declaration kind/scope')
    binding = {'profile_id': profile, 'firmware_manifest_sha256': pin['firmware_manifest_sha256'],
               'source_revision': pin['source_revision'], 'vehicle_profile': 'arducopter'}
    require(value['binding'] == binding and set(value['fixtures']) == SCENARIOS, 'scenario declaration binding differs')
    mission = value['fixtures']['armed_wiped_mission_read']
    require(mission['shape'] in ('empty', 'bound_home_only') and type(mission['expected_count']) is int and
            mission['expected_count'] == (1 if mission['shape'] == 'bound_home_only' else 0) and
            mission['mission_type'] == 0 and type(mission['mission_type']) is int and
            mission['compare_item_to_bound_home'] is (mission['expected_count'] == 1) and
            mission['item_records_exercised'] == mission['expected_count'] and
            mission['required_host_message_ids'] == ([43, 51, 47] if mission['expected_count'] else [43, 47]) and
            mission['forbidden_host_message_ids'] == ([] if mission['expected_count'] else [51]),
            'scenario mission shape differs')
    distance = value['fixtures']['downward_distance']
    expected_parameters = ({'RNGFND1_MIN': 0.1, 'RNGFND1_MAX': 30} if distance['native_units'] == 'm'
                           else {'RNGFND1_MIN_CM': 10, 'RNGFND1_MAX_CM': 3000})
    require(distance['native_units'] in ('m', 'cm') and distance['preboot_parameters'] == expected_parameters and
            distance['physical_min_m'] == 0.1 and distance['physical_max_m'] == 30 and
            distance['reported_min_cm'] == 10 and distance['reported_max_cm'] == 3000 and
            distance['orientation'] == 25 and distance['instance'] == 0, 'scenario distance physical mapping differs')
    return {'schema': 1, 'kind': 'mavmoos.selected_scenario_fixture', 'qualification': 'not_inferred',
            'binding': {**binding, 'native_sha256': verified_identity['native_sha256'],
                        'attestation_sha256': verified_identity['attestation_sha256']},
            'scenario': scenario, 'declaration_sha256': pin['declaration_sha256'],
            'expectation': copy.deepcopy(value['fixtures'][scenario])}


def select(source_root, verified_identity, scenario):
    root = Path(source_root).resolve()
    profile, _ = identity_profile(verified_identity)
    selector = root / 'scripts/firmware/scenario_fixtures.py'
    declaration = root / 'config/scenario-fixtures' / (profile + '.json')
    manifest = root / 'config/firmware' / (profile + '.json')
    require(selector.resolve() == Path(__file__).resolve(), 'scenario selector is not from selected source')
    for path in (selector, declaration, manifest):
        require(path.is_file() and not path.is_symlink() and root in path.resolve().parents,
                'scenario source input missing/nonregular/escaped')
    manifest_bytes = manifest.read_bytes()
    require(digest(manifest_bytes) == verified_identity['manifest_sha256'], 'scenario source manifest bytes differ')
    selected = from_declaration(declaration.read_bytes(), verified_identity, scenario)
    selected.update(selector_path=str(selector), selector_sha256=digest(selector.read_bytes()),
                    declaration_path=str(declaration), firmware_manifest_path=str(manifest))
    return selected


def protected_paths(selected):
    return [Path(selected[key]) for key in ('selector_path', 'declaration_path', 'firmware_manifest_path')]


def reselect(data, verified_identity, scenario, recorded):
    """Offline replay: derive from pinned bytes, then check recorded metadata."""
    selected = from_declaration(data, verified_identity, scenario)
    require(isinstance(recorded, dict) and all(recorded.get(key) == value for key, value in selected.items()),
            'recorded scenario fixture differs from pinned declaration')
    return selected


def range_readback(selected, parameters):
    """Check actual raw names/units before reporting the common physical range."""
    require(selected.get('scenario') == 'downward_distance', 'distance fixture selection required')
    expected = selected['expectation']
    for name, value in expected['preboot_parameters'].items():
        actual = parameters.get(name)
        require(type(actual) in (int, float) and math.isfinite(actual) and
                math.isclose(actual, value, rel_tol=1e-5, abs_tol=1e-4),
                'source-declared range parameter missing/different: ' + name)
    scale = 1 if expected['native_units'] == 'm' else 0.01
    minimum, maximum = sorted(expected['preboot_parameters'], key=lambda key: 0 if 'MIN' in key else 1)
    return {'result': 'PASS', 'native_units': expected['native_units'],
            'raw_parameters': {name: parameters[name] for name in expected['preboot_parameters']},
            'physical_min_m': parameters[minimum] * scale, 'physical_max_m': parameters[maximum] * scale,
            'declared_physical_min_m': expected['physical_min_m'], 'declared_physical_max_m': expected['physical_max_m'],
            'scope': 'closed parameter readback of the declared preboot mapping; not sensor accuracy or control authority'}


def copy_to_evidence(selected, output):
    """Copy unchanged declaration after selection; never modify a source file."""
    declaration = Path(selected['declaration_path']).read_bytes()
    require(digest(declaration) == selected['declaration_sha256'], 'scenario declaration changed before copy')
    destination = Path(output) / 'scenario-fixture.json'
    with destination.open('xb') as stream:
        stream.write(declaration)
    return {'path': destination.name, 'sha256': digest(declaration)}
