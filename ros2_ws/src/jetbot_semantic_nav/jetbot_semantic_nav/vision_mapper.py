"""Teammate planar observations -> bounded visual object map. No USD or velocity."""
import argparse,json,time,signal
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.clock import Clock,ClockType
from tf2_ros import Buffer,TransformListener
from std_msgs.msg import String
from .config import load,claim,write_json
from .planar_objects import VisualMap


class VisionMapper(Node):
    def __init__(self,case):
        super().__init__('semantic_visual_mapper',parameter_overrides=[Parameter('use_sim_time',value=True)])
        self.case=Path(case);self.owner=claim(case,'semantic_mapper');_,self.cfg=load(case)
        if self.cfg['mode']!='vision' or self.cfg.get('object_geometry')!='measured_point':
            raise ValueError('Explicit vision point mode required')
        self.store=VisualMap(**self.cfg.get('visual_map_policy',{}));self.errors=0;self.pending=[];self.last_history_wall=0.
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.pub=self.create_publisher(String,self.cfg['topics']['objects'],10)
        self.log=(self.case/'visual_observations.jsonl').open('x')
        self.create_subscription(String,self.cfg['topics']['observations'],self.receive,10)
        self.create_timer(.1,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))
        write_json(self.case/'VISUAL_MAP_PROVENANCE.json',dict(source='vision',upstream_commit=self.cfg['vision_upstream_commit'],
            input_topic=self.cfg['topics']['observations'],geometry='measured_surface_points',
            uses_usd_objects=False,robot_truth_input=False,occupancy_map_modified=False,
            identity='session spatial association, not permanent physical identity',policy=self.cfg.get('visual_map_policy',{})))

    def receive(self,msg):
        try:
            if len(msg.data)>2_000_000:raise ValueError('Packet too large')
            packet=json.loads(msg.data)
            if packet.get('frame_id')!='map' or packet.get('source')!='vision':raise ValueError('Map-frame vision required')
            self.pending.append((packet,time.monotonic()))
            if len(self.pending)>5:self.pending.pop(0);self.errors+=1
        except Exception as e:self.errors+=1;self.get_logger().warning(str(e))

    def tick(self):
        now=self.get_clock().now().nanoseconds*1e-9;wall=time.monotonic()
        if now<=0:return
        waiting=[]
        for packet,received in self.pending:
            try:
                tr=self.buffer.lookup_transform('map','chassis',rclpy.time.Time(seconds=float(packet['stamp']))).transform.translation
            except Exception:
                if wall-received<.5:waiting.append((packet,received))
                else:self.errors+=1
                continue
            try:
                self.store.update(packet,now,wall,[tr.x,tr.y])
                self.log.write(json.dumps(dict(packet=packet,observer_xy=[tr.x,tr.y],received_wall=wall),allow_nan=False)+'\n');self.log.flush()
            except Exception as e:self.errors+=1;self.get_logger().warning(str(e))
        self.pending=waiting
        try:result=self.store.snapshot(now,wall)
        except ValueError as e:
            self.pending=[];result=dict(schema='jetbot.visual_object_map',version=1,frame_id='map',source='vision',stamp=now,objects=[],fault=str(e))
        result['rejected_batches']=self.errors
        self.pub.publish(String(data=json.dumps(result,allow_nan=False)));write_json(self.case/'SEMANTIC_OBJECTS.json',result)
        if wall-self.last_history_wall>=1.:
            try:history=self.store.historical_map(now,wall)
            except ValueError:history=dict(result,navigation_input=False)
            write_json(self.case/'SEMANTIC_MAP.json',history);self.last_history_wall=wall


def main():
    p=argparse.ArgumentParser();p.add_argument('--case',required=True);a=p.parse_args(rclpy.utilities.remove_ros_args()[1:])
    rclpy.init();n=VisionMapper(a.case)
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:
        n.log.close();n.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
