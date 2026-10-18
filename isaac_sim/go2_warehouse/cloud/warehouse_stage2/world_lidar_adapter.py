"""Experimental RTX world-buffer to canonical current LiDAR frame conversion."""
import json
import os
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

_saved=set()
def sensor_points(annotator,env,stamp):
    OUT=Path(os.environ['GO2_LIDAR_WORLD_ARTIFACTS'])
    raw=annotator.get_data();world=np.asarray(raw['data']).reshape(-1,3).copy()
    state=env.unwrapped.scene['unitree_go2'].data.root_state_w[0].detach().cpu().numpy().copy()
    q=state[3:7];rot=Rotation.from_quat(q[[1,2,3,0]]);origin=state[:3]+rot.apply([.2,0,.2]);points=rot.inv().apply(world-origin).astype(np.float32)
    # Narrow windows retain static, moving, and braking frames without recording a long bag.
    key=round(stamp,5)
    if key not in _saved and any(t<=stamp<=t+.055 for t in [1.,3.,4.,5.,6.,7.,18.,19.]):
        _saved.add(key);OUT.mkdir(parents=True,exist_ok=True)
        np.savez_compressed(OUT/f'world_{key:.5f}.npz',world=world,sensor=points,root_state=state,transform=np.asarray(raw.get('info',{}).get('transform',[])))
        (OUT/f'world_{key:.5f}.json').write_text(json.dumps({'stamp':stamp,'transformPoints':True,'published_frame':'unitree_go2/lidar_frame','points':len(world),'origin_world':origin.tolist()},indent=2)+'\n')
    return points
