"""Four-condition historical comparison; truth is read offline only."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import shutil
import sqlite3
import sys
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from nav_msgs.msg import Odometry
sys.path.insert(0,str(Path(__file__).resolve().parent.parent/'warehouse_lio'))
from analyze_lio_dataset import load
from trajectory_metrics import evaluate, unique
p=argparse.ArgumentParser()
p.add_argument('--case',type=Path,required=True)
p.add_argument('--noisy-tag',required=True)
p.add_argument('--native-tag',default='native_01')
p.add_argument('--output',type=Path,required=True)
a=p.parse_args()
methods={};proof={}
for label,tag in [('noisy',a.noisy_tag),('native',a.native_tag)]:
 for estimator in ['lio','livo']:
  name=f'{estimator}_{label}'
  data=json.loads((a.case/f'replay_{estimator}_{tag}/odometry.json').read_text())
  if data['statistics'].get('error'): raise ValueError(name+' failed')
  expected='/evaluation/imu_ideal' if label=='native' else '/lio/imu'
  if data['statistics'].get('imu_topic')!=expected:raise ValueError('Wrong IMU source: '+name)
  methods[name]=data['poses'];proof[name]=data['statistics']
for key in ['point_input_sha256','imu_timestamps_sha256']:
 if len({v[key] for v in proof.values()})!=1:raise ValueError('Input mismatch: '+key)
if proof['livo_native'].get('image_input_sha256')!=proof['livo_noisy'].get('image_input_sha256'):raise ValueError('Unmatched image input')
begin=max(5.,max(min(r['stamp'] for r in rows) for rows in methods.values()))
end=min(max(r['stamp'] for r in rows) for rows in methods.values())
if end<=begin:raise ValueError('No shared evaluation interval')
db=sqlite3.connect(f'file:{a.case/"recording/sensors.sqlite3"}?mode=ro',uri=True)
truth=load(db,'/truth/odom',Odometry)
# Compare both output rates at the identical timestamps and initial anchor.
grid=np.arange(np.ceil(begin*10)/10,np.floor(end*10)/10+.0001,.1)
if len(grid)<2:raise ValueError('No common 10Hz evaluation grid')
metrics={};series={}
for name,rows in methods.items():
 rows=unique(rows);stamps=np.array([r['stamp'] for r in rows]);xyz=np.array([r['xyz'] for r in rows])
 positions=np.column_stack([np.interp(grid,stamps,xyz[:,i]) for i in range(3)])
 rotations=Slerp(stamps,Rotation.from_quat([r['xyzw'] for r in rows]))(grid).as_quat()
 sampled=[dict(stamp=float(t),xyz=p.tolist(),xyzw=q.tolist()) for t,p,q in zip(grid,positions,rotations)]
 metrics[name],series[name]=evaluate(truth,sampled,begin=float(grid[0]))

a.output.mkdir(exist_ok=False)
report=dict(methods=metrics,interval_s=[float(grid[0]),float(grid[-1])],evaluation_sampling="common 10Hz output interpolation, XYZ linear and orientation SLERP; same initial anchor time",replay_input_proof=proof,
 native_input='/evaluation/imu_ideal: native accelerometer/gyro captured before artificial noise',
 noisy_input='/lio/imu: same original recording with historical artificial noise enabled',
 alignment='single initial SE3 for each estimate; no trajectory/scale fitting',
 truth_usage='offline evaluation only',case=str(a.case),
 limitation='One simulated warehouse recording and noise seed; not real sensor accuracy or statistical significance')
(a.output/'metrics.json').write_text(json.dumps(report,indent=2)+'\n')
with gzip.open(a.output/'trajectories.json.gz','wt') as f:json.dump(dict(raw_truth=truth,aligned_series=series),f)
(a.output/'analysis_source').mkdir()
for src in [Path(__file__).resolve(),*[Path(__file__).resolve().parent.parent/'warehouse_lio'/n for n in ['analyze_lio_dataset.py','trajectory_metrics.py','lio_nav_math.py']]]:shutil.copy2(src,a.output/'analysis_source'/src.name)
(a.output/'analysis_source.sha256').write_text(''.join(hashlib.sha256(f.read_bytes()).hexdigest()+'  '+str(f.relative_to(a.output))+'\n' for f in sorted((a.output/'analysis_source').iterdir())))
print(json.dumps({k:v['xy_error_m'] for k,v in metrics.items()},indent=2))
