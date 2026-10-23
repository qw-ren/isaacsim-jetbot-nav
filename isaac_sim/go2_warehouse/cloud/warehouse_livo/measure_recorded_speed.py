"""Offline .2-second XY displacement speed; never creates a ROS publisher."""
import argparse,hashlib,json,shutil,sqlite3,zlib
from pathlib import Path
import numpy as np
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
p=argparse.ArgumentParser();p.add_argument('--case',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
if not (a.case/'RUNNER_RESULT.json').exists():raise SystemExit('Require a finished case')
a.output.mkdir(exist_ok=False)
db=sqlite3.connect(f'file:{a.case/"recording/sensors.sqlite3"}?mode=ro',uri=True);t=[];xy=[]
for stamp,payload in db.execute('SELECT stamp_ns,payload FROM records WHERE topic=? ORDER BY stamp_ns,id',('/truth/odom',)):
 m=deserialize_message(zlib.decompress(payload),Odometry);t.append(stamp/1e9);xy.append([m.pose.pose.position.x,m.pose.pose.position.y])
t=np.array(t);xy=np.array(xy);unique=np.r_[True,np.diff(t)>0];t,xy=t[unique],xy[unique]
q=np.arange(max(5.,t[0]+.1),t[-1]-.1,.1)
def at(q):return np.column_stack([np.interp(q,t,xy[:,j]) for j in range(2)])
v=np.linalg.norm(at(q+.1)-at(q-.1),axis=1)/.2
segments=[];start=None
for j,flag in enumerate(np.r_[v>1.5,False]):
 if flag and start is None:start=j
 if not flag and start is not None:
  end=j-1;segments.append(dict(start_sim_s=float(q[start]),end_sim_s=float(q[end]+.1),duration_s=(j-start)*.1,mean_mps=float(v[start:j].mean())));start=None
report=dict(case=a.case.name,reference_usage='offline simulation reference only; never an algorithm/control input',
    method='planar displacement over .2 seconds at 10Hz; no trajectory fit or speed-up',
    time_range_s=[float(q[0]),float(q[-1])],maximum_mps=float(v.max()),p95_mps=float(np.quantile(v,.95)),
    seconds_above_1_5_mps=float((v>1.5).sum()*.1),fast_segments=sorted(segments,key=lambda x:x['duration_s'],reverse=True)[:10])
(a.output/'speed_audit.json').write_text(json.dumps(report,indent=2)+'\n')
source=Path(__file__).resolve();shutil.copy2(source,a.output/'analysis_source.py')
(a.output/'analysis_source.sha256').write_text(hashlib.sha256(source.read_bytes()).hexdigest()+'  analysis_source.py\n')
print(json.dumps(report,indent=2))
