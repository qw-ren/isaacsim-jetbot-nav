"""Read-only comparison of actual LIO and isolated LIVO on a common capture."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import re
import shutil
import sqlite3
import sys

sys.path.insert(0,str(Path(__file__).resolve().parent.parent/'warehouse_lio'))
from analyze_lio_dataset import load
from trajectory_metrics import evaluate
from nav_msgs.msg import Odometry

parser=argparse.ArgumentParser()
parser.add_argument('--case',type=Path,required=True)
parser.add_argument('--livo',type=Path,required=True)
parser.add_argument('--lio',type=Path)
parser.add_argument('--ablation',type=Path)
parser.add_argument('--output',type=Path,required=True)
args=parser.parse_args()
args.output.mkdir(exist_ok=False)
db=sqlite3.connect(f'file:{args.case/"recording/sensors.sqlite3"}?mode=ro',uri=True)
truth=load(db,'/truth/odom',Odometry)
methods={'online_lio':load(db,'/lio/odometry',Odometry)}
proof={}
for name,path in [('livo',args.livo),('replay_lio',args.lio),('livo_no_image',args.ablation)]:
    if path is None:continue
    d=json.loads((path/'odometry.json').read_text())
    if d['statistics'].get('error'):raise ValueError('Failed replay '+str(path))
    methods[name]=d['poses'];proof[name]=d['statistics']
if args.lio:
    for k in ['point_input_sha256','imu_timestamps_sha256']:
        if proof['livo'][k]!=proof['replay_lio'][k]:raise ValueError('Unmatched replay input: '+k)
begin=max(5.,max(min(r['stamp'] for r in rows) for rows in methods.values()))
end=min(max(r['stamp'] for r in rows) for rows in methods.values())
metrics,series={},{}
for name,rows in methods.items():
    rows=[r for r in rows if begin<=r['stamp']<=end]
    metrics[name],series[name]=evaluate(truth,rows,begin=begin)
log=args.livo.parent/(args.livo.name+'_fastlivo.log')
text=log.read_text(errors='replace') if log.exists() else ''
sizes=[int(v) for v in re.findall(r'Sparse Map Size\s*\|\s*(\d+)',text)]
times=[];section=None
for line in text.splitlines():
    if 'VIO Time' in line:section='visual'
    elif 'LIO Mapping Time' in line:section='lidar'
    match=re.search(r'Current Total Time\s*\|\s*([\d.]+)',line)
    if match and section=='visual':times.append(float(match.group(1)))
report=dict(methods=metrics,interval_s=[begin,end],replay_input_proof=proof,
    visual_updates=len(sizes),last_visual_sparse_map_size=sizes[-1] if sizes else None,
    visual_processing_mean_s=sum(times)/len(times) if times else None,
    truth_used='offline evaluation only; no estimator or navigation aiding',
    alignment='single initial SE3 for each estimate; no full-trajectory or scale fitting',
    comparison='same captured noisy IMU and LiDAR; camera enabled for LIVO only')
if not times or not sizes or max(sizes)==0:raise ValueError('No evidence of active visual fusion')
(args.output/'metrics.json').write_text(json.dumps(report,indent=2)+'\n')
with gzip.open(args.output/'trajectories.json.gz','wt') as f:
    json.dump(dict(raw_truth=truth,raw_estimates=methods,aligned_series=series),f)
frozen=args.output/'analysis_source';frozen.mkdir()
for p in [Path(__file__),*[Path(__file__).resolve().parent.parent/'warehouse_lio'/n for n in ['analyze_lio_dataset.py','trajectory_metrics.py','lio_nav_math.py']]]:
    shutil.copy2(p,frozen/p.name)
(args.output/'analysis_source.sha256').write_text(''.join(hashlib.sha256(p.read_bytes()).hexdigest()+'  '+str(p.relative_to(args.output))+'\n' for p in sorted(frozen.iterdir())))
print(json.dumps({k:{'p95_m':v['xy_error_m']['p95'],'maximum_m':v['xy_error_m']['maximum'],'distance_m':v['physical_xy_path_m']} for k,v in metrics.items()},indent=2))
