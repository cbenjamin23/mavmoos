#!/usr/bin/env python3
"""Print a mission MOOS request, optionally converting a QGC WPL 110 file.

This utility does not connect, publish, upload, start a mission or open a vehicle.
The provider validates the complete request again before any host admission.
"""
import argparse
from decimal import Decimal, ROUND_HALF_UP
import math
from pathlib import Path
import re


def request(operation_id, action, waypoint_file=None, timeout=None):
    if not re.fullmatch(r'[A-Za-z0-9_.-]{1,48}', operation_id):
        raise ValueError('id must contain 1..48 ASCII letters/digits/dot/underscore/hyphen')
    if action not in ('download', 'upload', 'clear', 'cancel'):
        raise ValueError('unsupported action')
    if action == 'cancel' and timeout is not None:
        raise ValueError('cancel has no new timeout')
    fields = [f'id={operation_id}', f'action={action}']
    if timeout is not None:
        if not math.isfinite(timeout) or not 1 <= timeout <= 120:
            raise ValueError('timeout must be 1..120 seconds')
        fields.append(f'timeout={timeout:.9g}')
    if action != 'upload':
        if waypoint_file is not None:
            raise ValueError('only upload accepts a waypoint file')
        return ','.join(fields)
    if waypoint_file is None:
        raise ValueError('upload requires --waypoints with explicit HOME at sequence zero')
    lines = Path(waypoint_file).read_text().splitlines()
    if not lines or lines[0].strip() != 'QGC WPL 110':
        raise ValueError('expected QGC WPL 110 header')
    rows = []
    for line in lines[1:]:
        if not line.strip() or line.lstrip().startswith('#'):
            continue
        parts = line.split()
        if len(parts) != 12:
            raise ValueError('every waypoint needs the twelve QGC WPL fields')
        seq, current, frame, command = map(int, parts[:4])
        params = list(map(float, parts[4:8]))
        latitude, longitude, height = map(float, parts[8:11])
        auto = int(parts[11])
        if seq != len(rows) or seq >= 4096 or current not in (0, 1) or auto != 1:
            raise ValueError('sequences must be contiguous from zero; current0/1 and autocontinue1 only')
        frame = {5: 0, 6: 3}.get(frame, frame)
        if not all(map(math.isfinite, params + [latitude, longitude, height])):
            raise ValueError('initial upload profile requires finite fields')
        if not -90 <= latitude <= 90 or not -180 <= longitude <= 180 or not -1000 <= height <= 20000:
            raise ValueError('coordinate/height outside initial data profile')
        x = int((Decimal(parts[8]) * 10_000_000).to_integral_value(rounding=ROUND_HALF_UP))
        y = int((Decimal(parts[9]) * 10_000_000).to_integral_value(rounding=ROUND_HALF_UP))
        if seq == 0:
            if command != 16 or frame != 0 or any(params):
                raise ValueError('sequence zero must be explicit GLOBAL HOME waypoint with zero params')
        elif command == 20:
            if frame != 2 or any(params) or x != 0 or y != 0 or height != 0:
                raise ValueError('RTL requires MISSION frame and zero data fields')
        elif command in (16, 21, 22):
            if frame not in (0, 3) or any(params[1:]) or (command != 16 and params[0] != 0):
                raise ValueError('unsupported frame/params in initial navigation item profile')
            if command == 16 and (params[0] != int(params[0]) or not 0 <= params[0] <= 600):
                raise ValueError('waypoint hold time must be integer0..600')
        else:
            raise ValueError('only HOME/WAYPOINT/TAKEOFF/LAND/RTL are supported for upload')
        values = [str(seq), str(command), str(frame), str(current), str(auto)]
        values += [format(value, '.9g') for value in params]
        values += [str(x), str(y), format(height, '.9g')]
        rows.append(':'.join(values))
    if not rows:
        raise ValueError('upload must include HOME; use clear to store an empty mission')
    fields.append('items=' + '|'.join(rows))
    result = ','.join(fields)
    if len(result) > 1024 * 1024:
        raise ValueError('request exceeds1MiB')
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--id', required=True)
    parser.add_argument('--action', required=True, choices=('download', 'upload', 'clear', 'cancel'))
    parser.add_argument('--waypoints')
    parser.add_argument('--timeout', type=float)
    args = parser.parse_args()
    try:
        print(request(args.id, args.action, args.waypoints, args.timeout))
    except (ValueError, OSError) as error:
        parser.error(str(error))
