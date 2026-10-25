"""Raw observer viewport recording; never modifies the algorithm camera or scene."""
import csv,json
from pathlib import Path
from omni.kit.viewport.utility import capture_viewport_to_file

class ViewportVideo:
 def __init__(self,viewport,case):
  self.viewport=viewport;self.root=Path(case)/'observer_video';self.root.mkdir(exist_ok=False);self.next_time=0.;self.number=0;self.pending=[];self.completed=0
  self.stream=(self.root/'frames.csv').open('x',newline='');self.csv=csv.writer(self.stream);self.csv.writerow(['frame','requested_sim_s','completed_sim_s','file'])
  (self.root/'CONTRACT.json').write_text(json.dumps(dict(camera='separate observer viewport only',target_fps_sim=6.25,overlays=False,algorithm_camera_changed=False,navigation_truth_input=False,playback_speed=4.,timebase='simulation clock; independent of slow wall-time rendering'),indent=2)+'\n')
 def collect(self,now):
  waiting=[]
  for number,stamp,path,task in self.pending:
   if path.exists()and path.stat().st_size:
    self.csv.writerow([number,stamp,now,path.name]);self.stream.flush();self.completed+=1
   else:waiting.append((number,stamp,path,task))
  self.pending=waiting
 def tick(self,now):
  self.collect(now)
  if now+1e-6>=self.next_time and len(self.pending)<3:
   path=self.root/('frame_%06d.jpg'%self.number);task=capture_viewport_to_file(self.viewport,str(path));self.pending.append((self.number,now,path,task));self.number+=1;self.next_time=now+.16
 def close(self,now):
  self.collect(now);self.stream.close();(self.root/'SUMMARY.json').write_text(json.dumps(dict(requested=self.number,completed=self.completed,pending_at_shutdown=len(self.pending)),indent=2)+'\n')
