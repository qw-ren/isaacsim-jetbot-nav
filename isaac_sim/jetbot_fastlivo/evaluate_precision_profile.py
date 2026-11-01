"""Closed-trial pose errors, with distance weighting to avoid stationary dwell bias."""
import argparse,json,hashlib
from pathlib import Path
import numpy as np
p=argparse.ArgumentParser();p.add_argument('--case',type=Path,required=True);a=p.parse_args();r=a.case;assert(r/'RUNNER_RESULT.json').exists()
def rows(n):return[json.loads(l)for l in(r/n).read_text().splitlines()if l.strip()]
def unique(v,key,offset):
 by={x['stamp']:x[key][:2]for x in v};t=np.array(sorted(by));return t,np.array([by[s]for s in t])+offset
truth=unique(rows('truth_trajectory.jsonl'),'xyz',[0,0]);nav=unique(rows('navigation_feedback.jsonl'),'xy',[0,0]);odom=unique(rows('estimated_odom.jsonl'),'xyz',[0,6]);start=max(v[0][0]for v in[truth,nav,odom]);end=min(v[0][-1]for v in[truth,nav,odom]);t=np.arange(start,end,.05)
interp=lambda v:np.c_[np.interp(t,v[0],v[1][:,0]),np.interp(t,v[0],v[1][:,1])]
xy=interp(truth);distance=np.linalg.norm(np.diff(xy,axis=0),axis=1);moving=distance/.05>.1;assert distance.sum()>1
report=dict(scope='offline reference only, no fitting; per-time and per-distance metrics kept separate; not a repeated-trial statistical claim',interval_sim_s=[float(start),float(end)],grid_dt_s=.05,path_length_in_interval_m=float(distance.sum()),metrics={},source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),inputs={n:hashlib.sha256((r/n).read_bytes()).hexdigest()for n in ['truth_trajectory.jsonl','navigation_feedback.jsonl','estimated_odom.jsonl']})
for name,v in [('navigation',nav),('livo',odom)]:
 e=np.linalg.norm(interp(v)-xy,axis=1);sq=(e[:-1]**2+e[1:]**2)/2;report['metrics'][name]=dict(time_rmse_m=float(np.sqrt(np.mean(e**2))),distance_weighted_rmse_m=float(np.sqrt(np.sum(sq*distance)/distance.sum())),moving_time_rmse_m=float(np.sqrt(np.mean(sq[moving]))),moving_threshold_reference_m_s=.1,p95_m=float(np.percentile(e,95)))
out=r/'PRECISION_PROFILE.json';assert not out.exists();out.write_text(json.dumps(report,indent=2)+'\n');out.with_name('PRECISION_PROFILE_source.py').write_bytes(Path(__file__).read_bytes());print(json.dumps(report,indent=2))
