"""Fixed rear camera and raw viewport capture. NO in-scene trajectory drawing."""
import csv,json,math
from pathlib import Path
import numpy as np
from isaacsim.core.utils.viewports import set_camera_view
from omni.kit.viewport.utility import get_active_viewport,capture_viewport_to_file

class FollowVideoCapture:
    def __init__(self,env,output):
        self.env=env.unwrapped;self.output=Path(output);self.root=self.output/'follow_video';self.root.mkdir(exist_ok=False)
        self.viewport=get_active_viewport()
        if self.viewport is None:raise RuntimeError('Active viewport required')
        self.viewport.resolution=(1280,720)
        self.stream=(self.root/'frames.csv').open('x',newline='');self.csv=csv.writer(self.stream);self.csv.writerow(['frame','requested_sim_s','completed_sim_s','file'])
        self.pending=[];self.next_time=0.;self.count=0
        (self.root/'contract.json').write_text(json.dumps(dict(role='raw visualization camera only; trajectory composed outside simulator',resolution=[1280,720],target_fps_sim=12.5,camera_offset_body_m=[-2.4,0.,1.7],fixed_relative_pose=True,adaptive_occlusion_adjustment=False,look_at_body_m=[.4,0.,.25],navigation_truth_input=False,scene_draw_calls=False,playback='1x simulation time; crop initial shelf occlusion'),indent=2)+'\n')
    def tick(self):
        now=float(self.env.sim.current_time);robot=self.env.scene['unitree_go2'].data.root_state_w[0].cpu().numpy();w,x,y,z=robot[3:7]
        yaw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z));co,si=math.cos(yaw),math.sin(yaw)
        eye=robot[:3]+np.array([-2.4*co,-2.4*si,1.7]);target=robot[:3]+np.array([.4*co,.4*si,.25]);set_camera_view(eye,target)
        waiting=[]
        for number,stamp,path,task,snapshot in self.pending:
            if path.exists() and path.stat().st_size:
                snapshot.update(completed_sim_s=now,view=np.asarray(task.view).reshape(4,4).tolist(),projection=np.asarray(task.projection).reshape(4,4).tolist())
                path.with_suffix('.json').write_text(json.dumps(snapshot)+'\n');self.csv.writerow([number,stamp,now,path.name]);self.stream.flush()
            else:
                if now-stamp>1.:raise RuntimeError('Viewport readback timeout')
                waiting.append((number,stamp,path,task,snapshot))
        self.pending=waiting
        if now+1e-6>=self.next_time and len(self.pending)<3:
            local=None;p=self.output/'navigation_paths.json'
            if p.exists():
                try:
                    d=json.loads(p.read_text());candidate=d['paths'].get('local')
                    if candidate and 0<=now-candidate['received_sim']<.6:local=candidate
                except (ValueError,OSError):pass
            snapshot=dict(frame=self.count,requested_sim_s=now,robot_world=robot[:7].tolist(),camera_eye_world=eye.tolist(),camera_target_world=target.tolist(),local=local,overlay_role='composite only; never rendered into sensor RGB')
            path=self.root/f'frame_{self.count:06d}.jpg';task=capture_viewport_to_file(self.viewport,str(path));self.pending.append((self.count,now,path,task,snapshot));self.count+=1;self.next_time=now+.08
    def close(self):
        self.stream.close();(self.root/'SUMMARY.json').write_text(json.dumps(dict(requested=self.count,pending_at_shutdown=len(self.pending)))+'\n')
