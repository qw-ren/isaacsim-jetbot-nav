"""Read-only Nav2 paths -> bounded visualization snapshot; never publish ROS data."""
import argparse,json,math,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.clock import Clock,ClockType
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import Path as NavPath
from tf2_ros import Buffer,TransformListener

class DisplayPaths(Node):
    def __init__(self,output):
        super().__init__('go2_showcase_display_paths',parameter_overrides=[rclpy.parameter.Parameter('use_sim_time',value=True)])
        self.output=output;self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.paths={}  # Display only local trajectory; global planner remains untouched.
        self.create_subscription(NavPath,'/local_plan',lambda m:self.receive('local',m),10)
        self.create_timer(.1,self.save)
        self.create_timer(.5,self.check_complete,clock=Clock(clock_type=ClockType.STEADY_TIME))
    def receive(self,kind,m):
        if not m.poses:self.paths.pop(kind,None);return
        try:
            if m.header.frame_id=='odom':q=(0.,0.,0.,1.);t=(0.,0.,0.)
            else:
                tr=self.buffer.lookup_transform('odom',m.header.frame_id,Time()).transform
                q=(tr.rotation.x,tr.rotation.y,tr.rotation.z,tr.rotation.w);t=(tr.translation.x,tr.translation.y,tr.translation.z)
            x,y,z,w=q
            # Full quaternion rotation, rather than assuming the local path is map-aligned.
            rot=[[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                 [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                 [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]
            pts=[]
            for v in m.poses:
                p=v.pose.position;a=(p.x,p.y,p.z)
                world=[sum(rot[i][j]*a[j] for j in range(3))+t[i] for i in range(3)]
                if not all(math.isfinite(n) for n in world):raise ValueError('Nonfinite path')
                if not pts or math.dist(world[:2],pts[-1][:2])>=.06:pts.append(world)
            end=m.poses[-1].pose.position;a=(end.x,end.y,end.z)
            endpoint=[sum(rot[i][j]*a[j] for j in range(3))+t[i] for i in range(3)]
            if pts and pts[-1]!=endpoint:pts.append(endpoint)
            stride=max(1,math.ceil(len(pts)/2000));pts=pts[::stride]+([pts[-1]] if stride>1 else [])
            base_points=[]
            if kind=='local':
                tr=self.buffer.lookup_transform('unitree_go2/base_link','odom',Time()).transform
                q=tr.rotation;angle=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));co,si=math.cos(angle),math.sin(angle)
                base_points=[[co*v[0]-si*v[1]+tr.translation.x,si*v[0]+co*v[1]+tr.translation.y] for v in pts]
            self.paths[kind]=dict(points=pts,base_points=base_points,stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9,
                                  received_sim=self.get_clock().now().nanoseconds/1e9,source_frame=m.header.frame_id,
                                  display_frame='odom',topic='/plan' if kind=='global' else '/local_plan')
        except Exception as e:
            self.paths.pop(kind,None);self.get_logger().warning('Cannot display '+kind+': '+str(e),throttle_duration_sec=5.)
    def check_complete(self):
        if (self.output.parent.parent/'RUNNER_RESULT.json').exists():raise SystemExit(0)
    def save(self):
        if (self.output.parent.parent/'RUNNER_RESULT.json').exists():raise SystemExit(0)
        now=self.get_clock().now().nanoseconds/1e9
        d=dict(sim_s=now,paths={k:v for k,v in self.paths.items() if 0<=now-v['received_sim']<(.6 if k=='local' else 2.)},
               purpose='display only; odom aligned with simulator world by configured initial anchor; no truth correction')
        tmp=self.output.with_suffix('.tmp');tmp.write_text(json.dumps(d));tmp.replace(self.output)
        with self.output.with_name('display_paths_audit.jsonl').open('a') as f:
            f.write(json.dumps(dict(sim_s=now,counts={k:len(v['points']) for k,v in d['paths'].items()},sources={k:v['topic'] for k,v in d['paths'].items()}))+'\n')

def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();rclpy.init();n=DisplayPaths(a.output)
    try:rclpy.spin(n)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
