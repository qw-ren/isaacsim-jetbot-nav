"""Offline box-envelope projection audit; not annotated detection accuracy."""
import argparse
import csv
import itertools
import json
import math
from pathlib import Path
import cv2
import numpy as np


def rows(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def rotation(q):
    x,y,z,w=np.asarray(q)/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def iou(a,b):
    dx=max(0,min(a[2],b[2])-max(a[0],b[0]))
    dy=max(0,min(a[3],b[3])-max(a[1],b[1]))
    inter=dx*dy
    union=(a[2]-a[0])*(a[3]-a[1])+(b[2]-b[0])*(b[3]-b[1])-inter
    return inter/union if union>0 else 0


def evaluate(case, output=None):
    evidence=case/'route_evidence'
    out=output or case/'range_evaluation'
    out.mkdir(exist_ok=False)
    fixture=json.loads((case/'STATIC_FIXTURES.json').read_text())['parts'][0]
    lo,hi=np.array(fixture['min']),np.array(fixture['max'])
    corners=np.array(list(itertools.product(*zip(lo,hi))))
    truth=sorted(rows(case/'truth_trajectory.jsonl'),key=lambda r:r['stamp'])
    tt=np.array([r['stamp'] for r in truth])
    xyz=np.array([r['xyz'] for r in truth])
    quaternions=np.array([r['wxyz'][1:]+r['wxyz'][:1] for r in truth])
    for i in range(1,len(quaternions)):
        if np.dot(quaternions[i-1],quaternions[i])<0:
            quaternions[i]*=-1
    records=[]
    for frame in rows(evidence/'frames.jsonl'):
        raw=frame.get('raw')
        t=frame['stamp']
        if not raw or not tt[0]<=t<=tt[-1]:
            continue
        pos=np.array([np.interp(t,tt,xyz[:,i]) for i in range(3)])
        rb=rotation([np.interp(t,tt,quaternions[:,i]) for i in range(4)])
        ext=raw['T_base_optical']
        rc=rb@rotation(ext['quaternion_xyzw'])
        camera=pos+rb@np.array(ext['translation'])
        pc=(corners-camera)@rc
        distance=float(np.linalg.norm(pos[:2]-(lo[:2]+hi[:2])/2))
        row=dict(stamp=t,range_center_m=distance,file=frame['file'],in_view=False,depth_consistent_proxy=False,
                 matched_box=False,best_class=None,best_iou=0.,localized_xy=None,aabb_exterior_distance_m=None)
        if np.all(pc[:,2]>.05):
            k=np.array(raw['calibration']['K']).reshape(3,3)
            uv=(pc@k.T); uv=uv[:,:2]/uv[:,2:]
            box=[float(uv[:,0].min()),float(uv[:,1].min()),float(uv[:,0].max()),float(uv[:,1].max())]
            w,h=raw['calibration']['width'],raw['calibration']['height']
            clipped=[max(0,box[0]),max(0,box[1]),min(w-1,box[2]),min(h-1,box[3])]
            row['projected_bbox']=clipped
            row['in_view']=clipped[2]-clipped[0]>=2 and clipped[3]-clipped[1]>=2
            if row['in_view']:
                depth=np.load(evidence/raw['depth'])
                if raw['depth_encoding']=='16UC1':
                    depth=depth*.001
                x0,y0,x1,y1=map(int,clipped)
                patch=depth[y0:y1+1,x0:x1+1]
                valid=patch[np.isfinite(patch)&(patch>0)]
                # Envelope/depth agreement is a visibility proxy, not a segmentation label.
                row['depth_consistent_proxy']=bool(len(valid) and np.mean((valid>=pc[:,2].min()-.35)&(valid<=pc[:,2].max()+.35))>=.25)
                detections=frame['detections']['detections']
                if detections:
                    best=max(detections,key=lambda d:iou(clipped,d['bbox']))
                    row['best_iou']=iou(clipped,best['bbox'])
                    row['best_class']=best['class_name'] if row['best_iou']>=.25 else None
                    row['matched_box']=row['best_class']=='box'
                    if row['matched_box']:
                        matches=[o for o in frame['planar']['objects'] if o['class_name']=='box' and abs(o['confidence']-best['confidence'])<.0001]
                        if len(matches)==1:
                            xy=np.array(matches[0]['position_xy'])
                            row['localized_xy']=xy.tolist()
                            row['localization_method']=matches[0]['localization_method']
                            row['aabb_exterior_distance_m']=float(np.linalg.norm(np.maximum(np.maximum(lo[:2]-xy,xy-hi[:2]),0)))
                image=cv2.imread(str(evidence/frame['file']))
                cv2.rectangle(image,(x0,y0),(x1,y1),(255,0,255),1)
                cv2.putText(image,'BOX ENVELOPE %.1fm; proxy only'%distance,(3,h-24),cv2.FONT_HERSHEY_SIMPLEX,.35,(255,0,255),1)
                cv2.imwrite(str(out/frame['file']),image)
        records.append(row)
    (out/'frames.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in records))
    bins=[]
    for low,high in [(0,3),(3,5),(5,8),(8,12),(12,100)]:
        sample=[r for r in records if low<=r['range_center_m']<high and r['in_view'] and r['depth_consistent_proxy']]
        localized=[r['aabb_exterior_distance_m'] for r in sample if r['aabb_exterior_distance_m'] is not None]
        bins.append(dict(range_m=[low,high],visibility_proxy_frames=len(sample),box_matches=sum(r['matched_box'] for r in sample),
                         localized_matches=len(localized),aabb_exterior_mean_m=float(np.mean(localized)) if localized else None))
    trajectory=rows(evidence/'trajectory.jsonl')
    with (out/'trajectory_map.csv').open('w',newline='') as stream:
        writer=csv.writer(stream);writer.writerow(['capture_stamp_ros_sim_s','map_x_m','map_y_m'])
        writer.writerows((r['stamp'],*r['xy']) for r in trajectory)
    residual=[]
    for r in trajectory:
        t=r['stamp']
        if tt[0]<=t<=tt[-1]:
            reference=np.array([np.interp(t,tt,xyz[:,i]) for i in range(2)])
            residual.append(float(np.linalg.norm(np.array(r['xy'])-reference)))
    # Metric axes are shared by truth and estimates; no fitted alignment.
    points=np.array([r['xy'] for r in trajectory]+xyz[:,:2].tolist()+[fixture['center'][:2]])
    bounds_lo=points.min(axis=0)-1
    bounds_hi=points.max(axis=0)+1
    scale=min(850/(bounds_hi-bounds_lo))
    canvas=np.full((1000,1000,3),255,np.uint8)
    def pixel(p):
        return (int(60+(p[0]-bounds_lo[0])*scale),int(930-(p[1]-bounds_lo[1])*scale))
    for poly,color in ((xyz[:,:2],(30,160,30)),(np.array([r['xy'] for r in trajectory]),(220,80,20))):
        for a,b in zip(poly,poly[1:]):
            cv2.line(canvas,pixel(a),pixel(b),color,2)
    for x in np.arange(math.ceil(bounds_lo[0]),bounds_hi[0],2):
        cv2.putText(canvas,str(int(x)),(pixel([x,bounds_lo[1]])[0],955),cv2.FONT_HERSHEY_SIMPLEX,.4,(0,0,0),1)
    for y in np.arange(math.ceil(bounds_lo[1]),bounds_hi[1],2):
        cv2.putText(canvas,str(int(y)),(12,pixel([bounds_lo[0],y])[1]),cv2.FONT_HERSHEY_SIMPLEX,.4,(0,0,0),1)
    cv2.circle(canvas,pixel(fixture['center'][:2]),7,(0,0,240),-1)
    cv2.putText(canvas,'MAP meters: green truth; blue estimated trajectory; red target',(20,25),cv2.FONT_HERSHEY_SIMPLEX,.55,(0,0,0),1)
    cv2.putText(canvas,'No fitted alignment; truth used for offline evaluation only',(20,50),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,0,0),1)
    cv2.imwrite(str(out/'trajectory_comparison.png'),canvas)
    report=dict(scope='Single known box, projected CAD envelope and depth visibility proxy; not manual detection recall, not eight-class accuracy. Exterior AABB distance is a lower bound, not exact target point error.',
                target=fixture,frames=len(records),bins=bins,
                estimated_map_trajectory_vs_truth=dict(samples=len(residual),rmse_m=float(np.sqrt(np.mean(np.square(residual)))) if residual else None,
                                                      p95_m=float(np.percentile(residual,95)) if residual else None),
                reference_pose='truth translation and normalized interpolated quaternion; static optical extrinsics from TF',
                all_class_counts={c:sum(d['class_name']==c for f in rows(evidence/'frames.jsonl') for d in f['detections']['detections']) for c in ['shelf','wall','box','pallet','barrier','cart','sign','person']})
    (out/'SUMMARY.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2))


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--case',type=Path,required=True);ap.add_argument('--out',type=Path)
    args=ap.parse_args()
    evaluate(args.case,args.out)
