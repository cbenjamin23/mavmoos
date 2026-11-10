"""Select a verified native firmware profile for owned test fixtures only.

No executable override, route exception or qualification inference is provided.
Absent selection leaves a legacy fixture's original identity checks intact.
"""
import importlib.util
import os
from pathlib import Path
import sys


def selected_firmware(ardupilot_root, source_root=None, manifest=None,
                      attestation=None, expected_profile='arducopter'):
    source = Path(source_root).resolve() if source_root else Path(__file__).resolve().parents[1]
    selected = []
    for explicit, key in ((manifest, 'MAVMOOS_FIRMWARE_MANIFEST'),
                          (attestation, 'MAVMOOS_FIRMWARE_ATTESTATION')):
        inherited = os.environ.get(key)
        if explicit and inherited and Path(explicit).resolve() != Path(inherited).resolve():
            raise ValueError('explicit and inherited firmware selection differ: ' + key)
        selected.append(explicit or inherited)
    if not any(selected):
        return None
    if not all(selected):
        raise ValueError('firmware manifest and attestation must be selected together')
    manifest_path, attestation_path = [Path(p).resolve() for p in selected]
    helper = source / 'scripts/firmware/manifest.py'
    spec = importlib.util.spec_from_file_location('_mavmoos_selected_firmware_manifest', helper)
    if spec is None or spec.loader is None:
        raise ValueError('firmware verifier unavailable in selected source')
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    identity = module.verify_native(manifest_path, attestation_path,
                                    Path(ardupilot_root).resolve(), expected_profile=expected_profile)
    return {**identity, 'manifest_path': str(manifest_path),
            'attestation_path': str(attestation_path), 'verifier_path': str(helper)}


def load_selected_firmware(source_root, ardupilot_root, manifest=None,
                           attestation=None, expected_profile='arducopter'):
    """Named adapter for runners already importing helpers by explicit source."""
    return selected_firmware(ardupilot_root, source_root, manifest, attestation,
                              expected_profile)
