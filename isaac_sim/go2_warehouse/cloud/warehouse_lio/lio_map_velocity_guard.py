#!/usr/bin/env python3
"""Use estimated map pose for CAD safety geometry; original gate unchanged."""
import argparse
import json
import math
from pathlib import Path
import sys

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.time import Time
from tf2_ros import Buffer,TransformListener

sys.path.insert(0,str(Path(__file__).resolve().parent.parent/'warehouse_stage2'))
from velocity_guard import VelocityGuard
from lio_nav_math import clearance_in_map


class LioMapVelocityGuard(VelocityGuard):
    def __init__(self,boxes):
        super().__init__(boxes)
        self.buffer = Buffer(node=self)
        self.listener = TransformListener(self.buffer,self)
        self.geometry_transform = None
        self.get_logger().info('CAD collision geometry uses map<-odom from AMCL; no truth input')

    def fault(self,now):
        reason = super().fault(now)
        if reason:
            return reason
        try:
            t = self.buffer.lookup_transform('map','odom',Time()).transform
            q = t.rotation
            angle = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            self.geometry_transform = [t.translation.x,t.translation.y,angle]
            if not np.isfinite(self.geometry_transform).all():
                return 'MAP_TF_INVALID'
        except Exception:
            return 'MAP_TF_UNAVAILABLE'
        return None

    def geometry_clearance(self,v,w):
        if self.geometry_transform is None:
            return 0.  # Fail closed; never assume odom equals the CAD map.
        return clearance_in_map(self.pose,self.boxes,self.geometry_transform,v,w)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--boxes',type=Path,required=True)
    args = parser.parse_args()
    rclpy.init(args=[])
    node = LioMapVelocityGuard(json.loads(args.boxes.read_text()))
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__ == '__main__':
    main()
