"""Publish a short-lived XY map for downstream navigation; never publishes velocity."""
import json
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from std_msgs.msg import String

from atongmu_perception.planar_map import PlanarObjectStore


class PlanarMapNode(Node):
    def __init__(self):
        super().__init__("atongmu_planar_map")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("max_age", 5.)
        self.store = PlanarObjectStore(self.get_parameter("map_frame").value,
                                       self.get_parameter("max_age").value)
        self.last_clock = None
        self.clock_fault = False
        self.publisher = self.create_publisher(String, "/semantic/objects/map_2d", 10)
        self.subscription = self.create_subscription(String, "/perception/objects_2d", self.on_objects, 10)
        self.timer = self.create_timer(.2, self.publish_snapshot,
                                       clock=Clock(clock_type=ClockType.STEADY_TIME))

    def now_sim(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last_clock is not None and now < self.last_clock:
            self.clock_fault = True
            self.store.objects.clear()
            self.get_logger().error("Simulation reset: restart planar map node")
        self.last_clock = now
        return now

    def on_objects(self, msg):
        now = self.now_sim()
        if self.clock_fault or now <= 0:
            return
        try:
            self.store.update(json.loads(msg.data), now, time.monotonic())
        except (ValueError, TypeError, KeyError, AttributeError) as exc:
            self.get_logger().warn("Rejected planar observation: %s" % exc, throttle_duration_sec=5.)

    def publish_snapshot(self):
        now = self.now_sim()
        if now <= 0:
            return
        packet = self.store.snapshot(now, time.monotonic())
        self.publisher.publish(String(data=json.dumps(packet, ensure_ascii=False, allow_nan=False)))


def main(args=None):
    rclpy.init(args=args)
    node = PlanarMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
