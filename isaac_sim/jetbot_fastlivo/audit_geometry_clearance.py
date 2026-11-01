"""Offline center-to-AABB clearance audit for completed simulation cases.
Uses recorded reference poses only after completion; no ROS or control output.
CAD projection and a circular body are conservative approximations, not contact
force measurements or swept-body collision certification.
"""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np


def nearest(points, boxes):
    points=np.asarray(points,dtype=float);boxes=np.asarray(boxes,dtype=float)
    assert len(points) and len(boxes) and np.isfinite(points).all() and np.isfinite(boxes).all()
    distances=[];indices=[]
    for start in range(0,len(points),256):
        chunk=points[start:start+256,None,:]
        delta=np.maximum(np.maximum(boxes[None,:,0,:]-chunk,chunk-boxes[None,:,1,:]),0.)
        squared=np.sum(delta**2,axis=2);index=np.argmin(squared,axis=1)
        distances.extend(np.sqrt(squared[np.arange(len(index)),index]));indices.extend(index)
    return np.asarray(distances),np.asarray(indices)


def summarize(times,xy,parts,radius):
    boxes=[(p['min'][:2],p['max'][:2])for p in parts]
    distances,indices=nearest(xy,boxes);i=int(np.argmin(distances));margin=distances-radius
    return dict(reference_samples=len(times),minimum_center_distance_m=float(distances[i]),
        minimum_approximate_body_margin_m=float(margin[i]),
        samples_with_approximate_margin_below_zero=int(np.sum(margin<0)),
        samples_with_approximate_margin_below_5cm=int(np.sum(margin<.05)),
        closest=dict(stamp=float(times[i]),xy=xy[i].tolist(),asset=parts[int(indices[i])]['path']))


def evaluate(case):
    assert (case/'RUNNER_RESULT.json').is_file(),'Completed cases only'
    cfg=json.loads((case/'CASE.json').read_text());radius=cfg['motion_limits']['footprint_radius_m']
    read=lambda p:[json.loads(x)for x in p.read_text().splitlines()if x.strip()]
    raw={v['stamp']:v for v in read(case/'truth_trajectory.jsonl')}
    times=np.array(sorted(raw));xy=np.array([raw[t]['xyz'][:2]for t in times])
    parts=json.loads((case/'map/collision_boxes.json').read_text())
    result=dict(case=case.name,scope=__doc__,truth_usage='offline completed-case audit only',
        physical_circle_radius_m=radius,planning_radius_m={k:v for k,v in cfg['navigation']['parameter_overrides'].items()if k.endswith('/robot_radius')},
        static=dict(boxes=len(parts),**summarize(times,xy,parts,radius)),dynamic=[])
    paths=[case/'CASE.json',case/'RUNNER_RESULT.json',case/'truth_trajectory.jsonl',case/'map/collision_boxes.json']
    event_file=case/'obstacle_events.jsonl'
    if event_file.exists():
        paths.append(event_file)
        for e in read(event_file):
            mask=times>=e['sim']
            assert mask.any(),'No reference data after obstacle spawn'
            result['dynamic'].append(dict(kind=e['kind'],active_from_sim=float(e['sim']),
                **summarize(times[mask],xy[mask],e['actual_parts'],radius)))
    result['inputs_sha256']={str(p.relative_to(case)):hashlib.sha256(p.read_bytes()).hexdigest()for p in paths}
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--case',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    result=evaluate(a.case)
    with a.output.open('x')as f:json.dump(result,f,indent=2);f.write('\n')
    print(json.dumps({k:result[k]for k in ['case','physical_circle_radius_m','static','dynamic']},indent=2))
