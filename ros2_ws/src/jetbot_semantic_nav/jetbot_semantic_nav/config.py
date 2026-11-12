"""One authoritative configuration embedded in the owned case."""
import json
import fcntl
import math
from pathlib import Path


def load(case):
    cfg=json.loads((Path(case)/'CASE.json').read_text())
    semantic=cfg['navigation']['semantic']
    if semantic['mode']not in ('ground_truth_oracle','vision') or not cfg['simulation_only']:
        raise ValueError('Explicit simulation perception mode required')
    for scope in ('global_costmap/global_costmap','local_costmap/local_costmap'):
        if cfg['navigation']['parameter_overrides'][scope+'/ros__parameters/robot_radius']!=semantic['planning_radius_m']:
            raise ValueError('Semantic geometry must match active navigation radius')
    policy=semantic.get('speed_policy')
    if policy:
        maximum=cfg['motion_limits']['max_linear_m_s']
        if not isinstance(policy.get('topic'),str)or not policy['topic'].startswith('/'):
            raise ValueError('Absolute native speed limit topic required')
        values=[policy[k]for k in('near_speed_m_s','near_distance_m','release_distance_m')]
        if not all(type(v)in(int,float)and math.isfinite(v)for v in values):raise ValueError('Invalid speed policy')
        if not(0<values[0]<=maximum and values[1]>=maximum*1.5+cfg['motion_limits']['scan_clearance_m']+.1 and values[2]>values[1]):
            raise ValueError('Speed policy must preserve base cap and allow predictive clearance buffer')
    return cfg,semantic


def write_json(path, row):
    path=Path(path); temporary=path.with_suffix(path.suffix+'.tmp')
    temporary.write_text(json.dumps(row,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    temporary.replace(path)


def claim(case,role):
    handle=(Path(case)/(role+'.lock')).open('a')
    fcntl.flock(handle,fcntl.LOCK_EX|fcntl.LOCK_NB)
    return handle
