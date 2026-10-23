"""Third-person viewport only; simulator pose is NEVER fed to navigation.

Capture requests and their completion times are recorded in simulation time.
Encoding uses these timestamps, not wall time or an arbitrary speed-up factor.
The algorithm's forward RGB camera and its calibration remain independent.
"""
import csv
import json
import math
from pathlib import Path
import numpy as np
from isaacsim.core.utils.viewports import set_camera_view
from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file


class FollowVideoCapture:
    def __init__(self, env, output):
        self.env=env.unwrapped
        self.root=Path(output)/'follow_video';self.root.mkdir(exist_ok=False)
        self.viewport=get_active_viewport()
        if self.viewport is None:raise RuntimeError('Follow video requires an active viewport')
        self.viewport.resolution=(1280,720)
        self.stream=(self.root/'frames.csv').open('x',newline='')
        self.csv=csv.writer(self.stream)
        self.csv.writerow(['frame','requested_sim_s','completed_sim_s','file'])
        self.pending=[];self.next_time=0.;self.count=0;self.yaw=None
        (self.root/'contract.json').write_text(json.dumps(dict(
            role='visualization only, isolated from algorithm inputs',resolution=[1280,720],
            target_fps_sim=12.5,camera_offset_body_m=[-.7,-1.7,1.6],look_at_height_m=.25,
            navigation_truth_input=False,playback='1x simulation time using recorded requests',
            timestamp_limitation='asynchronous viewport readback; request/completion bounds retained'),indent=2)+'\n')

    def tick(self):
        now=float(self.env.sim.current_time)
        robot=self.env.scene['unitree_go2'].data.root_state_w[0].cpu().numpy()
        w,x,y,z=robot[3:7];yaw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
        if self.yaw is None:self.yaw=yaw
        self.yaw+=.12*math.atan2(math.sin(yaw-self.yaw),math.cos(yaw-self.yaw))
        c,s=math.cos(self.yaw),math.sin(self.yaw)
        eye=robot[:3]+np.array([-.7*c+1.7*s,-.7*s-1.7*c,1.6])
        set_camera_view(eye,robot[:3]+np.array([.3*c,.3*s,.25]))
        waiting=[]
        for number,stamp,path,task in self.pending:
            if path.exists() and path.stat().st_size:
                self.csv.writerow([number,stamp,now,path.name]);self.stream.flush()
            else:
                if now-stamp>1.:raise RuntimeError('Viewport capture did not complete within 1 simulation second')
                waiting.append((number,stamp,path,task))
        self.pending=waiting
        if now+1e-6>=self.next_time and len(self.pending)<3:
            path=self.root/f'frame_{self.count:06d}.jpg'
            task=capture_viewport_to_file(self.viewport,str(path))
            self.pending.append((self.count,now,path,task));self.count+=1;self.next_time=now+.08

    def close(self):
        self.stream.close()
        (self.root/'SUMMARY.json').write_text(json.dumps(dict(requested=self.count,pending_at_shutdown=len(self.pending)))+'\n')
