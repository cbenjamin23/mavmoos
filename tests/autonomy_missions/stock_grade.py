"""Shared grade boundary and geodetic readback for owned autonomy missions."""
import math

EVALUATOR = """
ProcessConfig = pMissionEval
{
  AppTick = 8
  CommsTick = 20
  mailflag = @BHV_ERROR#BHV_ERROR_SEEN=true
  lead_condition = AUTONOMY_GRADE_READY = true
  pass_condition = AUTONOMY_STAGE = done
  pass_condition = BHV_ERROR_SEEN = false
  result_flag = MISSION_EVALUATED = true
  report_file = results.txt
  report_column = grade=$[GRADE]
}
"""


def with_evaluator(text, executable):
    anchor = 'ProcessConfig = ANTLER\n{\n'
    if text.count(anchor) != 1:
        raise ValueError('mission must have one exact ANTLER roster')
    return text.replace(anchor, anchor + f'  Run = {executable} @ NewConsole = false\n', 1) + EVALUATOR


def grade(post, wait, out):
    result = out / 'results.txt'
    post('AUTONOMY_GRADE_READY', 'true')
    verdict = wait(lambda: result.read_text().strip() if result.is_file() else None)
    if verdict != 'grade=pass':
        raise RuntimeError(f'stock mission evaluator reported {verdict}')
    return verdict


def local_xy(row, lat_origin, lon_origin):
    scale = 111195e-7
    return ((row['lon'] - lon_origin * 1e7) * scale * math.cos(math.radians(lat_origin)),
            (row['lat'] - lat_origin * 1e7) * scale)


def checkpoints(rows, route, radius, projection):
    hits = []
    iterator = iter(rows)
    for goal in route:
        for row in iterator:
            error = math.dist(projection(row), goal)
            if error <= radius:
                hits.append({'goal': goal, 'fix': row, 'error_m': error})
                break
        else:
            raise RuntimeError(f'independent track missed waypoint {goal} within {radius} m')
    return hits


def stage_row(row, value, after):
    return row if (row and row['source'] == 'pHelmIvP' and row['mono'] > after and
                   row['value'] == value) else None
