"""Timestamped source-frame observations -> independent map-frame object layer."""
import argparse
import json
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.duration import Duration
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer,TransformListener
from .core import ObjectStore, transform_object
from .config import load,write_json,claim


class Mapper(Node):
    def __init__(self,case):
        super().__init__('semantic_mapper',parameter_overrides=[Parameter('use_sim_time',value=True)])
        self.case=Path(case);self.ownership=claim(case,'semantic_mapper');_,self.cfg=load(case);self.store=ObjectStore(self.cfg['object_ttl_s'])
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.pub=self.create_publisher(String,self.cfg['topics']['objects'],10)
        self.markers=self.create_publisher(MarkerArray,self.cfg['topics']['markers'],10)
        self.create_subscription(String,self.cfg['topics']['observations'],self.receive,10)
        self.create_timer(.5,self.publish);self.errors=0;self.batches=0

    def receive(self,msg):
        try:
            if len(msg.data)>2_000_000:raise ValueError('Observation too large')
            packet=json.loads(msg.data);frame=packet['frame_id'];stamp=float(packet['stamp'])
            if frame==self.cfg['world_frame']:
                registration=self.cfg['world_to_map']
                # Registration belongs to the map asset, never estimated from robot truth.
                if registration['method']!='same_usd_world_as_map_raster':raise ValueError('Unsupported registration')
                translation=registration['translation'];rotation=registration['rotation_xyzw']
            elif frame=='map':translation=[0.,0.,0.];rotation=[0.,0.,0.,1.]
            else:
                tr=self.buffer.lookup_transform('map',frame,rclpy.time.Time(seconds=stamp),timeout=Duration(seconds=0.))
                t,q=tr.transform.translation,tr.transform.rotation
                translation=[t.x,t.y,t.z];rotation=[q.x,q.y,q.z,q.w]
            packet['objects']=[transform_object(row,translation,rotation)for row in packet['objects']]
            packet['frame_id']='map';self.store.update(packet,self.get_clock().now().nanoseconds*1e-9,time.monotonic());self.batches+=1
        except Exception as exc:
            self.errors+=1;self.get_logger().warning(str(exc))

    def publish(self):
        now=self.get_clock().now().nanoseconds*1e-9
        rows=self.store.active(now,time.monotonic())
        packet=dict(version=1,frame_id='map',stamp=now,objects=rows,
                    observation_batches=self.batches,rejected_batches=self.errors)
        self.pub.publish(String(data=json.dumps(packet,allow_nan=False)))
        write_json(self.case/'SEMANTIC_OBJECTS.json',packet)
        markers=MarkerArray();clear=Marker();clear.action=Marker.DELETEALL;markers.markers.append(clear)
        for i,row in enumerate(rows):
            m=Marker();m.header.frame_id='map';m.header.stamp=self.get_clock().now().to_msg()
            m.ns='semantic_objects';m.id=i;m.type=Marker.CUBE;m.action=Marker.ADD
            m.pose.position.x,m.pose.position.y,m.pose.position.z=row['position']
            m.pose.orientation.x,m.pose.orientation.y,m.pose.orientation.z,m.pose.orientation.w=row['orientation_xyzw']
            m.scale.x,m.scale.y,m.scale.z=row['dimensions'];m.color.r=.9;m.color.g=.5;m.color.b=.1;m.color.a=.45
            markers.markers.append(m)
        self.markers.publish(markers)


def main():
    p=argparse.ArgumentParser();p.add_argument('--case',required=True,type=Path);args=p.parse_args(rclpy.utilities.remove_ros_args()[1:])
    rclpy.init();node=Mapper(args.case)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
