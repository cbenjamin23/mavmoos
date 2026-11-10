"""Exact stock Copter layouts used by owned frame qualification."""
from pathlib import Path


STANDARD_COPTER_FRAMES = {'quad': (2, 1), 'hexa': (13, 2), 'octa': (14, 3)}


def copter_default_paths(ardupilot, frame):
    directory = ardupilot/'Tools/autotest/default_params'
    paths = [directory/'copter.parm']
    if frame != 'quad':
        paths.append(directory/f'copter-{frame}.parm')
    if any(not path.is_file() or path.is_symlink() for path in paths):
        raise RuntimeError('required stock Copter model defaults missing or indirect')
    return paths


def bind_copter_frame(text, frame):
    if frame == 'quad':
        return text
    anchor = '  expected_type = 2\n'
    if text.count(anchor) != 1:
        raise RuntimeError('stock Copter mission identity anchor differs')
    return text.replace(anchor, f'  expected_type = {STANDARD_COPTER_FRAMES[frame][0]}\n  copter_frame = {frame}\n', 1)


def copter_frame_parameters(records, frame):
    parameters = {row.Name: row.Value for row in records if row.Name in ('FRAME_CLASS', 'FRAME_TYPE')}
    expected = {'FRAME_CLASS': STANDARD_COPTER_FRAMES[frame][1], 'FRAME_TYPE': 0}
    if parameters != expected:
        raise RuntimeError(f'owned {frame} DataFlash frame parameters differ: {parameters}')
    return parameters


def one_copter_frame_log(directory, frame):
    from pymavlink import DFReader

    paths = sorted(Path(directory).glob('*.BIN'))
    if len(paths) != 1:
        raise RuntimeError('expected one owned DataFlash log for frame corroboration')
    reader = DFReader.DFReader_binary(str(paths[0]))
    return copter_frame_parameters(iter(lambda: reader.recv_match(type='PARM'), None), frame)
