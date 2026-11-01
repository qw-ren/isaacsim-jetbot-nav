"""Completed-trial position errors on a shared uniform time grid. No trajectory fit or online input."""
import argparse,json,hashlib
from pathlib import Path
import numpy as np
p=argparse.ArgumentParser();p.add_argument('--case',type=Path,required=True);p.add_argument('--output',type=Path);p.add_argument('--mission');a=p.parse_args();assert(a.case/'RUNNER_RESULT.json').exists()
directory=a.case
if a.mission:
 assert a.mission.replace('_','').isalnum();directory=a.case/'missions'/a.mission
def rows(name):return[json.loads(l)for l in(a.case/name).read_text().splitlines()if l.strip()]
def unique(v,key,offset):
 by_time={r['stamp']:r[key][:2]for r in v};t=np.array(sorted(by_time));return t,np.array([by_time[s]for s in t])+offset
truth=unique(rows('truth_trajectory.jsonl'),'xyz',[0.,0.]);navrows=[json.loads(l)for l in(directory/'feedback.jsonl').read_text().splitlines()if l.strip()]if a.mission else rows('navigation_feedback.jsonl');nav=unique(navrows,'xy',[0.,0.]);odom=unique(rows('estimated_odom.jsonl'),'xyz',[0.,6.]);begin=max(x[0][0]for x in[truth,nav,odom]);end=min(x[0][-1]for x in[truth,nav,odom]);grid=np.arange(begin,end,.05);assert len(grid)>10
interpolate=lambda v:np.c_[np.interp(grid,v[0],v[1][:,0]),np.interp(grid,v[0],v[1][:,1])]
reference=interpolate(truth);report=dict(scope='offline simulator reference; declared start only, no fit; duplicates deduplicated before uniform resampling',begin_stamp=float(begin),end_stamp=float(end),uniform_grid_dt_s=.05,grid_samples=len(grid),odom_to_declared_world=dict(translation=[0.,6.],yaw=0.,fitted=False),metrics={})
for name,v in[('navigation_feedback',nav),('livo_odom',odom)]:
 error=np.linalg.norm(interpolate(v)-reference,axis=1);i=int(error.argmax());report['metrics'][name]=dict(unique_source_samples=len(v[0]),rmse_m=float(np.sqrt(np.mean(error**2))),median_m=float(np.median(error)),p95_m=float(np.percentile(error,95)),max_m=float(error[i]),max_stamp=float(grid[i]))
report['source_sha256']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest();out=a.output or(directory/'TRAJECTORY_DIAGNOSTIC.json');assert not out.exists();out.write_text(json.dumps(report,indent=2)+'\n');out.with_name(out.stem+'_source.py').write_bytes(Path(__file__).read_bytes());print(json.dumps(report,indent=2))
