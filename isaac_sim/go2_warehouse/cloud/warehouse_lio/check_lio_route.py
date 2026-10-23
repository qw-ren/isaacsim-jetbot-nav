"""Guarded original-warehouse out/back route. Truth controls the test, never the LIO estimator."""
import argparse,json,math,time,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent.parent/'warehouse_stage2'))
from check_motion import Probe
import numpy as np
import rclpy
from geometry_msgs.msg import Twist

def wrap(x):return math.atan2(math.sin(x),math.cos(x))
class RouteProbe(Probe):
 def __init__(self):self.unwrapped=None;self.previous_yaw=None;super().__init__()
 def odom_cb(self,m):
  super().odom_cb(m);yaw=self.rows[-1]['yaw']
  self.unwrapped=yaw if self.unwrapped is None else self.unwrapped+wrap(yaw-self.previous_yaw);self.previous_yaw=yaw;self.rows[-1]['yaw_unwrapped']=self.unwrapped
 def send(self,velocity,wz=0.):
  m=Twist();m.linear.x=float(velocity);m.angular.z=float(wz);self.source.publish(m);self.last_source_wall=time.monotonic()
 def run_phase(self,stage,kind,seconds,command,finished,check_gate=True):
  self.stage=stage;begin=self.sim;deadline=time.monotonic()+max(60,seconds*15);next_send=0
  while not finished():
   now=time.monotonic()
   if now>deadline:raise RuntimeError('Phase deadline: '+stage)
   if self.rows:
    row=self.rows[-1]
    if row['tilt']>.5 or not .2<=row['z']<=.85:raise RuntimeError('Posture limit')
    if not .5<row['x']<6.8 or not -1.95<row['y']<-.55:raise RuntimeError('Route envelope exceeded')
   if check_gate and self.states and not self.states[-1]['armed']:raise RuntimeError('Guard stopped route: '+self.states[-1]['reason'])
   if now>=next_send:self.send(*command());next_send=now+.1
   rclpy.spin_once(self,timeout_sec=.02)
  return {'stage':stage,'kind':kind,'begin_sim':begin,'end_sim':self.sim,'end_pose':dict(self.rows[-1])}
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--boxes',type=Path,required=True);p.add_argument('--cycles',type=int,default=2);p.add_argument('--speed',type=float,default=.8);a=p.parse_args();assert 1<=a.cycles<=2 and .6<=a.speed<=.8;a.output.mkdir(parents=True,exist_ok=False)
rclpy.init();n=RouteProbe();result={'scope':'truth-controlled test trajectory; independent FAST-LIO2 output not used by motion controller','cycles':a.cycles,'command_speed_mps':a.speed,'segments':[]};error=None
try:
 until=time.monotonic()+90
 while time.monotonic()<until and not (n.sim>0 and n.rows and n.runtime and n.states):n.send(0);rclpy.spin_once(n,timeout_sec=.05)
 if not (n.rows and n.runtime and n.states):raise RuntimeError('Missing runtime telemetry')
 start=n.sim;result['segments'].append(n.run_phase('standing','hold',2,lambda:(0,0),lambda:n.sim-start>=2,False))
 if not n.arm(True):raise RuntimeError('Guard arming refused')
 result['start']=dict(n.rows[-1])
 for leg in range(a.cycles*2):
  target_x=5. if leg%2==0 else 2.;direction=1 if leg%2==0 else -1;heading=leg*math.pi
  def forward_command():return (a.speed,float(np.clip(1.5*wrap(heading-n.rows[-1]['yaw']),-.3,.3)))
  seg=n.run_phase('forward_'+str(leg),'forward',8,forward_command,lambda:(target_x-n.rows[-1]['x'])*direction<=.1);seg['target_x']=target_x;result['segments'].append(seg);result['forward_end']=dict(n.rows[-1])
  start=n.sim;result['segments'].append(n.run_phase('brake_'+str(leg),'hold',1,lambda:(0,0),lambda:n.sim-start>=1))
  goal=(leg+1)*math.pi
  def turn_command():
   e=goal-n.unwrapped;return (0.,math.copysign(min(.4,max(.18,abs(e))),e))
  seg=n.run_phase('turn_'+str(leg),'turn',15,turn_command,lambda:abs(goal-n.unwrapped)<.035);seg['target_unwrapped_yaw']=goal;result['segments'].append(seg)
  start=n.sim;result['segments'].append(n.run_phase('settle_'+str(leg),'hold',1,lambda:(0,0),lambda:n.sim-start>=1))
 start=n.sim;result['segments'].append(n.run_phase('final_hold','hold',2,lambda:(0,0),lambda:n.sim-start>=2));result['end']=dict(n.rows[-1])
except Exception as exc:error=repr(exc);result['error']=error
finally:
 try:n.arm(False)
 except Exception as exc:result['disarm_error']=repr(exc)
 for _ in range(10):n.send(0);rclpy.spin_once(n,timeout_sec=.02)
 (a.output/'telemetry.json').write_text(json.dumps({'odom':n.rows,'runtime_cmd':n.runtime,'guard':n.states})+'\n')
 xy=np.array([[x['x'],x['y']] for x in n.rows]);distance=float(np.linalg.norm(np.diff(xy,axis=0),axis=1).sum()) if len(xy)>1 else 0.;minimum=math.inf
 for b in json.loads(a.boxes.read_text()):
  if len(xy):minimum=min(minimum,float(np.linalg.norm(np.maximum(np.maximum(np.array(b['min'][:2])-xy,xy-np.array(b['max'][:2])),0),axis=1).min()))
 cruise={}
 for seg in result['segments']:
  if seg['kind']=='forward':
   rows=[x for x in n.rows if x['stage']==seg['stage'] and x['sim']>=seg['begin_sim']+1 and abs(x['x']-seg['target_x'])>.4];cruise[seg['stage']]=float(np.mean([x['vx'] for x in rows])) if rows else 0.
 checks={'no_error':error is None,'all_legs':len(cruise)==a.cycles*2,'cruise_each_leg_ge_06':bool(cruise and all(x>=.6 for x in cruise.values())),'turns_completed':sum(x['kind']=='turn' for x in result['segments'])==a.cycles*2,'path_length':distance>=5*a.cycles,'safe_geometry':minimum>=.55,'posture':bool(n.rows and all(x['tilt']<.5 and .2<=x['z']<=.85 for x in n.rows)),'commands_bounded':bool(n.runtime and all(0<=x['vx']<=.80001 and abs(x['wz'])<=.60001 for x in n.runtime))}
 result.update({'checks':checks,'passed':all(checks.values()),'path_length_m':distance,'min_center_clearance_m':minimum,'cruise_by_leg_mps':cruise,'finished_sim':n.sim,'yaw_span_rad':float(n.unwrapped-n.rows[0]['yaw_unwrapped']) if n.rows else 0.})
 (a.output/'route_check.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2));n.destroy_node();rclpy.shutdown()
