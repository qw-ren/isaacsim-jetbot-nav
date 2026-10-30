"""Offline global pose accuracy, distinct from endpoint/arrival error.
No pose fitting. AMCL samples use their own scan timestamps, not receipt time.
The 5Hz probe can miss AMCL publications; sample metrics are not time-weighted.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import numpy as np


def yaw(q):
    q = np.asarray(q, dtype=float)
    q = q / np.linalg.norm(q)
    x, y, z, w = q
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def stats(xy, angle):
    error = np.linalg.norm(xy, axis=1)
    angular = np.abs(np.arctan2(np.sin(angle), np.cos(angle))) * 180 / np.pi
    return dict(samples=len(error), xy_rmse_m=float(np.sqrt(np.mean(error**2))),
                xy_median_m=float(np.median(error)), xy_p95_m=float(np.percentile(error,95)),
                xy_max_m=float(error.max()), yaw_rmse_deg=float(np.sqrt(np.mean(angular**2))),
                yaw_p95_deg=float(np.percentile(angular,95)),
                mean_signed_xy_error_m=np.mean(xy,axis=0).tolist())


def evaluate(case):
    assert (case/'RUNNER_RESULT.json').exists()
    read = lambda name: [json.loads(line) for line in (case/name).read_text().splitlines() if line.strip()]
    truth = {row['stamp']:row for row in read('truth_trajectory.jsonl')}
    tt = np.array(sorted(truth))
    txy = np.array([truth[t]['xyz'][:2] for t in tt])
    tyaw = np.unwrap([yaw(truth[t]['wxyz'][1:]+truth[t]['wxyz'][:1]) for t in tt])
    nav = {row['stamp']:row for row in read('navigation_feedback.jsonl')}
    nt = np.array(sorted(nav))
    nxy = np.array([nav[t]['xy'] for t in nt])
    nyaw = np.unwrap([nav[t]['yaw'] for t in nt])
    assert len(nt)>1
    begin, end = max(tt[0],nt[0]), min(tt[-1],nt[-1])
    grid = np.arange(begin,end,.05)
    assert len(grid)>10
    interp_xy = lambda t,st,sxy: np.column_stack([np.interp(t,st,sxy[:,i]) for i in range(2)])
    result = dict(scope=__doc__, case=case.name, mission_window_s=[float(begin),float(end)],
        continuous_navigation_global_pose=dict(grid_dt_s=.05,
            **stats(interp_xy(grid,nt,nxy)-interp_xy(grid,tt,txy),
                    np.interp(grid,nt,nyaw)-np.interp(grid,tt,tyaw))))
    reference_grid=interp_xy(grid,tt,txy)
    residual_grid=interp_xy(grid,nt,nxy)-reference_grid
    ds=np.linalg.norm(np.diff(reference_grid,axis=0),axis=1)
    squared=np.sum(residual_grid**2,axis=1)
    paired=(squared[:-1]+squared[1:])/2
    moving=ds/.05>.1
    if ds.sum()>0:
        result['continuous_navigation_global_pose']['distance_weighted_xy_rmse_m']=float(np.sqrt(np.sum(paired*ds)/ds.sum()))
    if moving.any():
        result['continuous_navigation_global_pose']['moving_xy_rmse_m']=float(np.sqrt(np.mean(paired[moving])))
    result['continuous_navigation_global_pose']['moving_threshold_m_s']=.1
    names=['RUNNER_RESULT.json','truth_trajectory.jsonl','navigation_feedback.jsonl']
    probe=case/'localization_probe.jsonl'
    if probe.exists():
        raw={row['amcl']['stamp']:row['amcl'] for row in read(probe.name) if row.get('amcl')}
        at=np.array(sorted(t for t in raw if begin<=t<=end))
        assert len(at)>1
        axy=np.array([raw[t]['xy'] for t in at])
        ayaw=np.array([yaw(raw[t]['xyzw']) for t in at])
        result['amcl_published_pose_samples']=dict(
            coverage_s=[float(at[0]),float(at[-1])],
            **stats(axy-interp_xy(at,tt,txy),ayaw-np.interp(at,tt,tyaw)))
        names.append(probe.name)
    result['inputs_sha256']={name:hashlib.sha256((case/name).read_bytes()).hexdigest() for name in names}
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    result=evaluate(args.case)
    with args.output.open('x') as stream:
        json.dump(result,stream,indent=2);stream.write('\n')
