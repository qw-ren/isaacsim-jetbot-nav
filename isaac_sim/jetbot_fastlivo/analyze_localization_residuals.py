"""Describe existing AMCL residual directions and end-phase pose steps.
Offline only; no fitted pose, timestamp shift, or navigation correction.
Truth speed/heading only classify evaluation samples, never algorithm input.
"""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np


def analyze(case):
    read=lambda name:[json.loads(line) for line in (case/name).read_text().splitlines() if line.strip()]
    truth={row['stamp']:row for row in read('truth_trajectory.jsonl')}
    t=np.array(sorted(truth));xy=np.array([truth[s]['xyz'][:2] for s in t])
    w,x,y,z=np.array([truth[s]['wxyz'] for s in t]).T
    h=np.unwrap(np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z)))
    speed=np.linalg.norm(np.gradient(xy,t,axis=0),axis=1);rate=np.gradient(h,t)
    raw={row['amcl']['stamp']:row['amcl'] for row in read('localization_probe.jsonl') if row.get('amcl')}
    at=np.array(sorted(s for s in raw if t[0]<=s<=t[-1] and s>5))
    measured=np.array([raw[s]['xy'] for s in at])
    reference=np.column_stack([np.interp(at,t,xy[:,i]) for i in range(2)])
    e=measured-reference;heading=np.interp(at,t,h)
    along=e[:,0]*np.cos(heading)+e[:,1]*np.sin(heading)
    cross=-e[:,0]*np.sin(heading)+e[:,1]*np.cos(heading)
    straight=(np.interp(at,t,speed)>.4)&(abs(np.interp(at,t,rate))<.1)
    report=dict(scope=__doc__,straight_rule=dict(min_reference_speed_m_s=.4,max_abs_reference_yaw_rate_rad_s=.1),
        straight_samples=int(straight.sum()),
        straight_mean_along_cross_error_m=np.mean(np.column_stack([along,cross])[straight],axis=0).tolist(),
        directions={})
    for angle,name in [(0,'+X'),(np.pi/2,'+Y'),(np.pi,'-X'),(-np.pi/2,'-Y')]:
        mask=straight&(abs(np.arctan2(np.sin(heading-angle),np.cos(heading-angle)))<.2)
        if mask.any():report['directions'][name]=dict(samples=int(mask.sum()),mean_world_xy_error_m=e[mask].mean(axis=0).tolist())
    nav={row['stamp']:row for row in read('navigation_feedback.jsonl')};nt=np.array(sorted(nav))
    tail=nt[nt>=nt[-1]-5.];nxy=np.array([nav[s]['xy'] for s in tail])
    truth_tail=np.column_stack([np.interp(tail,t,xy[:,i]) for i in range(2)])
    residual=nxy-truth_tail
    report['last_5s']=dict(samples=len(tail),
        note='May include deliberate final approach/rotation; residual spread is not a pure stationary jitter measurement.',
        global_residual_xy_std_m=residual.std(axis=0).tolist(),
        p95_estimated_pose_step_m=float(np.percentile(np.linalg.norm(np.diff(nxy,axis=0),axis=1),95)))
    report['inputs_sha256']={name:hashlib.sha256((case/name).read_bytes()).hexdigest() for name in ['truth_trajectory.jsonl','localization_probe.jsonl','navigation_feedback.jsonl']}
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--case',type=Path,required=True);parser.add_argument('--output',type=Path,required=True);a=parser.parse_args()
    report=analyze(a.case)
    with a.output.open('x') as stream:json.dump(report,stream,indent=2);stream.write('\n')
