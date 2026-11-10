"""Read a bounded JSON receipt from a release evidence directory."""
from pathlib import Path

from ci.run import parse, require

RECEIPT_LIMIT = 16 * 1024 * 1024


def local(root, name):
    require(isinstance(name, str) and name and '\\' not in name and '\0' not in name,
            'invalid evidence path')
    relative = Path(name)
    require(not relative.is_absolute() and relative.parts and '..' not in relative.parts
            and relative.as_posix() == name, 'evidence path must be canonical and relative')
    path = root / relative
    require(path.resolve() == path and path.is_file(), f'evidence file missing or aliased: {name}')
    return path


def receipt(path):
    require(path.stat().st_size <= RECEIPT_LIMIT, f'receipt exceeds size limit: {path.name}')
    value = parse(path.read_text())
    require(isinstance(value, dict), f'receipt must be an object: {path.name}')
    return value
