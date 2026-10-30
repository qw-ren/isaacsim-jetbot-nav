import hashlib,json,numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation
p=Path('/home/admin/jetbot-warehouse/artifacts/mesh_beamskip_sigma005_BA_20261030_87')
rows=[json.loads(l)for l in(p/'estimated_odom.jsonl').read_text().splitlines()if l.strip()]
r=[x for x in rows if x['stamp']>rows[-1]['stamp']-5]
world=np.array([x['native_velocity_world']for x in r]);body=Rotation.from_quat([x['xyzw']for x in r]).inv().apply(world);t=np.array([x['stamp']for x in r])
truth=[json.loads(l)for l in(p/'truth_trajectory.jsonl').read_text().splitlines()if l.strip()]
tt=np.array([x['stamp']for x in truth]);tx=np.array([x['xyz'][:2]for x in truth]);gtv=np.gradient(tx,tt,axis=0);gt=np.c_[np.interp(t,tt,gtv[:,0]),np.interp(t,tt,gtv[:,1])]
result=dict(case=p.name,terminal_window_s=[float(t[0]),float(t[-1])],samples=len(r),native_world_mean_m_s=world.mean(axis=0).tolist(),approximate_body_mean_m_s=body.mean(axis=0).tolist(),reference_xy_speed_median_m_s=float(np.median(np.linalg.norm(gt,axis=1))),native_xy_speed_median_m_s=float(np.median(np.linalg.norm(world[:,:2],axis=1))),body_transform_limit='Approximate: recorded aligned odom orientation applied to native world velocity; does not undo frozen gravity alignment or subtract IMU lever arm. Speed norm comparison does not require this approximate transform.',truth_usage='Offline descriptive audit only; no fitting or correction.',inputs_sha256={n:hashlib.sha256((p/n).read_bytes()).hexdigest()for n in ['estimated_odom.jsonl','truth_trajectory.jsonl']},diagnostic_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
with Path('/home/admin/jetbot-warehouse/turning_20261030/TERMINAL87.json').open('x')as f:json.dump(result,f,indent=2)
print(json.dumps(result,indent=2))
