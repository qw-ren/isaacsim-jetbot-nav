"""Owned warehouse props inserted on the sim thread using estimated navigation feedback."""
import json, math
from pathlib import Path
import omni.usd
from pxr import Gf, UsdGeom, UsdPhysics

class DynamicFixture:
 def __init__(self,root,world):
  self.root=Path(root);self.world=world;self.events=json.loads((self.root/'CASE.json').read_text()).get('navigation',{}).get('obstacles',[]);self.done=set();self.display_boxes=[];self.stage=omni.usd.get_context().get_stage();self.prefix='/World/JetbotDynamicTrial'
  assert not self.stage.GetPrimAtPath(self.prefix).IsValid()
  UsdGeom.Xform.Define(self.stage,self.prefix)
 def box(self,path,center,size,color):
  assert len(center)==len(size)==3 and all(math.isfinite(v) for v in center+size) and all(v>0 for v in size)
  yaw=self.event_yaw;c,s=math.cos(yaw),math.sin(yaw);x,y=self.event_center;dx,dy=center[0]-x,center[1]-y
  placed=[x+c*dx-s*dy,y+s*dx+c*dy,center[2]];bounds=[abs(c)*size[0]+abs(s)*size[1],abs(s)*size[0]+abs(c)*size[1],size[2]]
  cube=UsdGeom.Cube.Define(self.stage,path);cube.CreateSizeAttr(1.);cube.AddTranslateOp().Set(Gf.Vec3d(*placed))
  if yaw:cube.AddRotateZOp().Set(math.degrees(yaw))
  cube.AddScaleOp().Set(Gf.Vec3f(*size));cube.CreateDisplayColorAttr([Gf.Vec3f(*color)]);UsdPhysics.CollisionAPI.Apply(cube.GetPrim()).CreateCollisionEnabledAttr(True)
  return dict(path=path,center=placed,size=bounds,authored_size=size,yaw_rad=yaw,collision=True)
 def tick(self):
  if len(self.done)==len(self.events):return
  try:
   pose=json.loads((self.root/'NAVIGATION_PROGRESS.json').read_text());gate=json.loads((self.root/'GATE_PROGRESS.json').read_text())
  except (FileNotFoundError,json.JSONDecodeError):return
  now=float(self.world.current_time)
  if gate['reason']!='ENABLED' or gate['output'][0]<.03 or not 0<=now-pose['stamp']<.5:return
  for i,event in enumerate(self.events):
   axis=event.get('trigger_axis','x');assert axis in ('x','y')
   threshold=float(event.get('trigger_above',event.get('trigger_below',event.get('trigger_x_below',float('nan')))));assert math.isfinite(threshold)
   coordinate=pose['xy'][0 if axis=='x'else 1]
   if i in self.done or (coordinate<threshold if 'trigger_above'in event else coordinate>threshold):continue
   x,y=event['center'];self.event_center=[x,y];self.event_yaw=float(event.get('yaw_rad',0.));assert abs(self.event_yaw/(math.pi/2)-round(self.event_yaw/(math.pi/2)))<1e-7;kind=event['kind'];prefix=self.prefix+'/event_'+str(i);parts=[]
   if kind=='A':
    # Wooden pallet with four separate cardboard crates, sized for the small base.
    for j,dy in enumerate([-.30,0.,.30]):parts.append(self.box(prefix+'/plank_'+str(j),[x,y+dy,.035],[.65,.12,.07],[.48,.29,.12]))
    for j,(dx,dy) in enumerate([(-.15,-.17),(.15,-.17),(-.15,.17),(.15,.17)]):parts.append(self.box(prefix+'/crate_'+str(j),[x+dx,y+dy,.22],[.28,.31,.30],[.68,.46,.24]))
   elif kind=='B':
    # Factory rack across the direct route; real low rails make the span observable.
    span=float(event.get('span_m',6.2));assert 1.<=span<=6.2
    for j,dy in enumerate([v*(span-.2)/6 for v in [-3.,-1.,1.,3.]]):parts.append(self.box(prefix+'/upright_'+str(j),[x,y+dy,.45],[.18,.10,.90],[.18,.29,.38]))
    for j,z in enumerate([.15,.50,.85]):parts.append(self.box(prefix+'/shelf_'+str(j),[x,y,z],[.50,span,.10],[.45,.48,.49]))
   else:raise ValueError('Unknown warehouse fixture')
   row=dict(event_id=i,kind=kind,sim=now,trigger_estimated_pose=pose,gate=gate,actual_parts=parts,truth_trigger=False,robot_teleport=False,map_rewrite=False)
   with (self.root/'obstacle_events.jsonl').open('a') as f:f.write(json.dumps(row)+'\n')
   self.display_boxes.extend(dict(min=[p['center'][j]-p['size'][j]/2 for j in range(3)],max=[p['center'][j]+p['size'][j]/2 for j in range(3)])for p in parts)
   self.done.add(i)
