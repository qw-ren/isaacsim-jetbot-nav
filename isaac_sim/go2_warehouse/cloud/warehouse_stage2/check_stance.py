#!/usr/bin/env python3
"""Bounded simulator-only stop/hold comparison, independent of Nav2."""
import argparse,json,math,time
from pathlib import Path
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from check_motion import Probe


class StandProbe(Probe):
    def __init__(self):
        super().__init__()
        self.yaw_source=0.0
        self.stance=[]
        self.stance_sub=self.create_subscription(String,'/nav_m1/stance_state',self.stance_cb,10)
    def stance_cb(self,m):
        d=json.loads(m.data);d['wall']=time.monotonic();self.stance.append(d)
    def send(self,velocity):
        m=Twist();m.linear.x=float(velocity);m.angular.z=float(self.yaw_source)
        self.source.publish(m);self.last_source_wall=time.monotonic()


def angle(a,b):return math.atan2(math.sin(a-b),math.cos(a-b))

def main():
    a=argparse.ArgumentParser();a.add_argument('--output',type=Path,required=True);a.add_argument('--boxes',type=Path,required=True);a.add_argument('--case',choices=['forward','yaw','combined'],default='forward');a.add_argument('--hold-required',action='store_true');a.add_argument('--resume',action='store_true');args=a.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    rclpy.init();p=StandProbe();r={'case':args.case,'hold_required':args.hold_required};stop_rows=[]
    try:
        until=time.monotonic()+80
        while time.monotonic()<until and not(p.sim>0 and p.rows and p.runtime and p.states and p.stance):
            p.send(0);rclpy.spin_once(p,timeout_sec=.05)
        if not p.stance:raise RuntimeError('Missing stance diagnostics')
        p.phase('standing',2)
        if not p.arm(True):raise RuntimeError('Cannot enable guard')
        r['start']=dict(p.rows[-1]);p.yaw_source=.4 if args.case=='yaw' else .2 if args.case=='combined' else 0.
        vx=0. if args.case=='yaw' else .8
        p.phase('active',3 if args.case=='forward' else 2,vx,check_gate=True)
        r['active_end']=dict(p.rows[-1]);r['stop_trigger_wall']=time.monotonic();p.yaw_source=0
        start=len(p.rows);p.phase('stop_observe',12,0,check_gate=False);stop_rows=p.rows[start:]
        r['stopped']=dict(p.rows[-1]);steady=[x for x in stop_rows if x['sim']>=r['active_end']['sim']+2]
        ref=steady[0];r['after_2s_max_excursion_m']=max(math.hypot(x['x']-ref['x'],x['y']-ref['y']) for x in steady);r['after_2s_max_yaw_change_rad']=max(abs(angle(x['yaw'],ref['yaw'])) for x in steady)
        r['stop_total_displacement_m']=math.hypot(r['stopped']['x']-r['active_end']['x'],r['stopped']['y']-r['active_end']['y'])
        active=[x for x in p.rows if x['stage']=='active' and x['sim']>=r['active_end']['sim']-1];r['active_forward_mean_mps']=float(np.mean([x['vx'] for x in active]));r['active_yaw_change_rad']=abs(angle(r['active_end']['yaw'],r['start']['yaw']))
        recent=[x for x in stop_rows if x['sim']>=stop_rows[-1]['sim']-2];r['final_2s_max_speed_mps']=max(x['speed'] for x in recent);r['final_2s_max_abs_wz']=max(abs(x['wz']) for x in recent)
        statuses=[x for x in p.stance if r['stop_trigger_wall']<=x['wall']<=p.rows[-1]['wall']];r['hold_samples']=sum(x['mode']=='HOLD' for x in statuses);r['entry_eligible_samples']=sum(x['eligible_time']>=.1 for x in statuses)
        r['guard_faults_during_zero']=sorted({x['reason'] for x in p.states if r['stop_trigger_wall']<=x['wall']<=p.rows[-1]['wall'] and x['reason'] not in ['ENABLED','DISARMED']})
        r['first_hold_wall']=next((x['wall'] for x in statuses if x['mode']=='HOLD'),None)
        if args.resume:
            if p.states and not p.states[-1]['armed']:raise RuntimeError('Guard latched during zero; resume forbidden')
            if args.case!='forward':raise RuntimeError('Resume permitted only in verified forward corridor')
            before=dict(p.rows[-1]);p.phase('resume',2,.8,check_gate=True);r['resume_displacement_m']=math.hypot(p.rows[-1]['x']-before['x'],p.rows[-1]['y']-before['y']);resume=[x for x in p.rows if x['stage']=='resume' and x['sim']>=p.rows[-1]['sim']-1];r['resume_forward_mean_mps']=float(np.mean([x['vx'] for x in resume]));p.phase('resume_stop',4,0,check_gate=True)
    except Exception as exc:r['error']=str(exc)
    finally:
        p.yaw_source=0
        p.arm(False)
        for _ in range(10):p.send(0);rclpy.spin_once(p,timeout_sec=.02)
        xy=np.array([[x['x'],x['y']] for x in p.rows]).reshape(-1,2);clearance=float('inf')
        for b in json.loads(args.boxes.read_text()):
            if len(xy):clearance=min(clearance,float(np.linalg.norm(np.maximum(np.maximum(np.array(b['min'][:2])-xy,xy-np.array(b['max'][:2])),0),axis=1).min()))
        r['min_center_aabb_clearance_m']=clearance
        rt=[x for x in p.runtime if r.get('stop_trigger_wall',float('inf'))+.7<x['wall']<r.get('stopped',{}).get('wall',0)]
        r['checks']={'no_error':'error' not in r,'no_sensor_fault_during_zero':not r.get('guard_faults_during_zero',[]),'actual_motion':r.get('active_yaw_change_rad',0)>.5 if args.case=='yaw' else r.get('active_forward_mean_mps',0)>=.6,'posture_bounded':bool(p.rows) and all(.2<=x['z']<=.85 and x['tilt']<.5 for x in p.rows),'zero_feedback':bool(rt) and all(abs(x['vx'])<1e-6 and abs(x['wz'])<1e-6 for x in rt),'settled_excursion_le_005':r.get('after_2s_max_excursion_m',99)<=.05,'settled_yaw_change_le_010':r.get('after_2s_max_yaw_change_rad',99)<=.1,'sustained_linear_stop':r.get('final_2s_max_speed_mps',99)<.05,'sustained_angular_stop':r.get('final_2s_max_abs_wz',99)<.1,'geometry_clearance':clearance>.45,'runtime_bounds':bool(p.runtime) and all(0<=x['vx']<=.80001 and abs(x['wz'])<=.60001 for x in p.runtime)}
        if args.hold_required:r['checks']['hold_entered']=r.get('hold_samples',0)>0
        if args.resume:r['checks']['resumed_motion']=r.get('resume_displacement_m',0)>.5 and r.get('resume_forward_mean_mps',0)>=.6
        r['passed']=all(r['checks'].values());(args.output/'stance_check.json').write_text(json.dumps(r,indent=2)+'\n');(args.output/'telemetry.json').write_text(json.dumps({'odom':p.rows,'runtime':p.runtime,'guard':p.states,'stance':p.stance},indent=2)+'\n');print(json.dumps(r,indent=2));p.destroy_node();rclpy.shutdown()
    return 0 if r['passed'] else 1
if __name__=='__main__':raise SystemExit(main())
