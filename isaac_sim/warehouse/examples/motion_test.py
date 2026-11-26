#!/usr/bin/env python3
"""Stage 3: corrected motion / odometry acceptance test (5 cases).

Uses integration.motion_probe (PhysX + simulation clock). Reports control error
and odometry error separately, and writes a CSV for plotting.

    /home/admin/isaacsim/python.sh examples/motion_test.py
"""
import csv
import json
import os
import sys
import time

from isaacsim import SimulationApp
app = SimulationApp({"headless": True, "width": 640, "height": 480})

import omni.kit.app
import omni.timeline
import omni.usd

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

OUT = "/tmp/jetbot_rot"
os.makedirs(OUT, exist_ok=True)

em = omni.kit.app.get_app().get_extension_manager()
em.set_extension_enabled_immediate("isaacsim.ros2.bridge", True)
for _ in range(25):
    app.update()

from isaac_adapter.scene_manager import WarehouseSceneManager  # noqa
from integration import jetbot_integration as ji  # noqa
from integration.motion_probe import MotionProbe  # noqa
import omni.graph.core as og  # noqa

ctx = omni.usd.get_context()
mgr = WarehouseSceneManager(ctx=ctx)
mgr.load_base_scene()
for _ in range(20):
    app.update()
ji.import_jetbot(mgr.stage)
for _ in range(15):
    app.update()
mgr.reset_scene(seed=1001, density="medium", protected_zones=ji.PROTECTED_ZONE)
for _ in range(10):
    app.update()

tl = omni.timeline.get_timeline_interface()
tl.play()
for _ in range(60):
    app.update()

# ---- ROS: publish cmd_vel, receive /clock (sim time) and /odom ----
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock

if not rclpy.ok():
    rclpy.init(args=None)
node = rclpy.create_node("jetbot_motion_test")
pub = node.create_publisher(Twist, "/cmd_vel", 10)
state = {"sim": None, "odom": None}


def on_clock(m):
    state["sim"] = m.clock.sec + m.clock.nanosec * 1e-9


def on_odom(m):
    state["odom"] = m


node.create_subscription(Clock, "/clock", on_clock, 10)
node.create_subscription(Odometry, "/odom", on_odom, 10)

ODO = "/World/ROS2/ROS_Odom/isaac_compute_odometry_node"
ctrl = og.Controller()


def og_wz():
    try:
        a = ctrl.attribute(ODO + ".outputs:angularVelocity")
        v = a.get()
        return float(v[2])
    except Exception:
        return None


probe = MotionProbe(mgr.stage, sim_time_fn=lambda: state["sim"])


def run(name, lin, ang, secs):
    t = time.time()
    while time.time() - t < secs:
        m = Twist()
        m.linear.x = float(lin)
        m.angular.z = float(ang)
        pub.publish(m)
        rclpy.spin_once(node, timeout_sec=0.0)
        app.update()
        probe.sample(name, lin, ang, og_wz(), state["odom"])
    for _ in range(8):
        pub.publish(Twist())
        rclpy.spin_once(node, timeout_sec=0.0)
        app.update()
        probe.sample(name, 0.0, 0.0, og_wz(), state["odom"])


run("straight", 0.15, 0.0, 4.0)
run("rot_pos", 0.0, 0.60, 4.0)
run("rot_neg", 0.0, -0.60, 4.0)
run("stop", 0.0, 0.0, 2.0)

summary = {}
for name in ("straight", "rot_pos", "rot_neg", "stop"):
    summary[name] = probe.steady(name, 0.2, 0.85)

# ---- seed switch + safe reset (case 5) ----
reset = ji.reset_and_apply(mgr, seed=1002, density="hard",
                           protected_zones=ji.PROTECTED_ZONE, resume=False)
for _ in range(10):
    app.update()
summary["seed_switch"] = {"scene": reset["scene"], "paused": reset["paused"],
                          "motion_zeroed": reset["motion_zeroed"],
                          "collision_ok": reset["collision"]["ok"],
                          "ok": reset["ok"]}

csv_path = os.path.join(OUT, "motion_test.csv")
with open(csv_path, "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(probe.rows[0].keys()))
    w.writeheader()
    w.writerows(probe.rows)

print("RESULT_BEGIN")
print(json.dumps(summary, indent=2, ensure_ascii=False, default=str))
print("RESULT_END")
print("CSV:", csv_path, "rows:", len(probe.rows))
print("MOTION_TEST_DONE")
sys.stdout.flush()
os._exit(0)
