"""Guarded continuous moving turns in a geometrically checked clear warehouse area."""
import argparse,json,math,sys,time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent.parent/'warehouse_stage2'))
from check_motion import Probe
from geometry_msgs.msg import Twist
import numpy as np
import rclpy

def wrap(x):return math.atan2(math.sin(x),math.cos(x))
class CircleProbe(Probe):
 def __init__(self,cx,cy):self.cx=cx;self.cy=cy;self.theta=None;self.previous_theta=None;self.heading=None;self.previous_heading=None;super().__init__()
 def odom_cb(self,m):
  super().odom_cb(m);row=self.rows[-1];theta=math.atan2(row['y']-self.cy,row['x']-self.cx);yaw=row['yaw'];self.theta=theta if self.theta is None else self.theta+wrap(theta-self.previous_theta);self.heading=yaw if self.heading is None else self.heading+wrap(yaw-self.previous_heading);self.previous_theta=theta;self.previous_heading=yaw;row['polar_unwrapped']=self.theta;row['yaw_unwrapped']=self.heading;row['radius']=math.hypot(row['x']-self.cx,row['y']-self.cy)
 def send(self,velocity,wz=0.):
  m=Twist();m.linear.x=float(velocity);m.angular.z=float(wz);self.source.publish(m);self.last_source_wall=time.monotonic()
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--boxes',type=Path,required=True);p.add_argument('--cycles',type=int,default=2);p.add_argument('--radius',type=float,default=2.2);p.add_argument('--speed',type=float,default=.75);a=p.parse_args();assert 1<=a.cycles<=2 and .6<=a.speed<=.8 and 2<=a.radius<=2.4;a.output.mkdir(parents=True,exist_ok=False)
rclpy.init();n=CircleProbe(4.,0.);result={'trajectory':'continuous circular route','command_speed_mps':a.speed,'radius_target_m':a.radius,'cycles':a.cycles,'segments':[],'scope':'truth controls test trajectory; FAST-LIO2 output never used for control'};error=None
try:
 deadline=time.monotonic()+90
 while time.monotonic()<deadline and not (n.sim>0 and n.rows and n.runtime and n.states):n.send(0);rclpy.spin_once(n,timeout_sec=.05)
 if not (n.rows and n.runtime and n.states):raise RuntimeError('Missing runtime telemetry')
 begin=n.sim;n.stage='standing';n.phase('standing',2);result['segments'].append({'kind':'hold','stage':'standing','begin_sim':begin,'end_sim':n.sim})
 if not n.arm(True):raise RuntimeError('Guard arming refused')
 result['start']=dict(n.rows[-1]);begin=n.sim;angle_begin=n.theta;n.stage='curve';next_send=0.;deadline=time.monotonic()+600
 while n.theta-angle_begin<a.cycles*2*math.pi-.02:
  now=time.monotonic();row=n.rows[-1]
  if now>deadline or n.sim-begin>65:raise RuntimeError('Circle deadline')
  if row['tilt']>.5 or not .2<=row['z']<=.85:raise RuntimeError('Posture limit')
  if not .8<row['radius']<2.8:raise RuntimeError('Circle radial envelope exceeded')
  if n.states and not n.states[-1]['armed']:raise RuntimeError('Guard stopped circle: '+n.states[-1]['reason'])
  if now>=next_send:
   theta=math.atan2(row['y'],row['x']-4.);heading_target=theta+math.pi/2+math.atan2(.8*(row['radius']-a.radius),a.speed);omega=float(np.clip(a.speed/a.radius+1.2*wrap(heading_target-row['yaw']),.05,.4));omega*=min(1.,max(0.,(n.sim-begin)/1.5));n.send(a.speed,omega);next_send=now+.1
  rclpy.spin_once(n,timeout_sec=.02)
 result['segments'].append({'kind':'curve','stage':'curve','begin_sim':begin,'end_sim':n.sim});result['forward_end']=dict(n.rows[-1]);result['circular_revolutions']=(n.theta-angle_begin)/(2*math.pi);begin=n.sim;n.phase('final_hold',3,check_gate=True);result['segments'].append({'kind':'hold','stage':'final_hold','begin_sim':begin,'end_sim':n.sim});result['end']=dict(n.rows[-1])
except Exception as exc:error=repr(exc);result['error']=error
finally:
 try:n.arm(False)
 except Exception as exc:result['disarm_error']=repr(exc)
 for _ in range(10):n.send(0);rclpy.spin_once(n,timeout_sec=.02)
 (a.output/'telemetry.json').write_text(json.dumps({'odom':n.rows,'runtime_cmd':n.runtime,'guard':n.states})+'\n');xy=np.array([[x['x'],x['y']] for x in n.rows]);distance=float(np.linalg.norm(np.diff(xy,axis=0),axis=1).sum()) if len(xy)>1 else 0.;minimum=math.inf
 for b in json.loads(a.boxes.read_text()):
  if len(xy):minimum=min(minimum,float(np.linalg.norm(np.maximum(np.maximum(np.array(b['min'][:2])-xy,xy-np.array(b['max'][:2])),0),axis=1).min()))
 curve=[x for x in n.rows if x['stage']=='curve' and x['sim']>result.get('start',{}).get('sim',0)+1.5];cruise=float(np.mean([x['vx'] for x in curve])) if curve else 0.
 checks={'no_error':error is None,'two_full_turns':result.get('circular_revolutions',0)>=a.cycles-.01,'cruise_ge_06':cruise>=.6,'path_length':distance>=8*a.cycles,'safe_geometry':minimum>=.55,'posture':bool(n.rows and all(x['tilt']<.5 and .2<=x['z']<=.85 for x in n.rows)),'commands_bounded':bool(n.runtime and all(0<=x['vx']<=.80001 and abs(x['wz'])<=.60001 for x in n.runtime))}
 result.update({'checks':checks,'passed':all(checks.values()),'path_length_m':distance,'min_center_clearance_m':minimum,'cruise_by_leg_mps':{'curve':cruise},'finished_sim':n.sim,'yaw_span_rad':float(n.heading-n.rows[0]['yaw_unwrapped']) if n.rows else 0.});(a.output/'route_check.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2));n.destroy_node();rclpy.shutdown()
