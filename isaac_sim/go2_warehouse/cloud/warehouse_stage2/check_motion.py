#!/usr/bin/env python3
"""Bounded speed/braking and real watchdog fault tests in cloud simulation."""
import argparse
import json
import math
import os
import signal
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from std_srvs.srv import SetBool


class Probe(Node):
    def __init__(self):
        super().__init__("go2_motion_probe", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.source = self.create_publisher(Twist, "/nav_m1/cmd_vel", 10)
        self.enable = self.create_client(SetBool, "/nav_m1/enable")
        self.sim = 0.0
        self.rows, self.runtime, self.states = [], [], []
        self.stage = "warmup"
        self.last_source_wall = 0.0
        self.subs = [self.create_subscription(Clock, "/clock", self.clock_cb, qos_profile_sensor_data),
                     self.create_subscription(Odometry, "/unitree_go2/odom", self.odom_cb, qos_profile_sensor_data),
                     self.create_subscription(TwistStamped, "/nav_m1/runtime_cmd", self.runtime_cb, 10),
                     self.create_subscription(String, "/nav_m1/guard_state", self.state_cb, 10)]

    def clock_cb(self, message):
        self.sim = message.clock.sec + message.clock.nanosec*1e-9

    def odom_cb(self, m):
        p, q, v = m.pose.pose.position, m.pose.pose.orientation, m.twist.twist
        self.rows.append({"wall": time.monotonic(), "sim": m.header.stamp.sec + m.header.stamp.nanosec*1e-9,
                          "stage": self.stage, "x": p.x, "y": p.y, "z": p.z,
                          "yaw": math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),
                          "tilt": math.acos(float(np.clip(1-2*(q.x*q.x+q.y*q.y), -1, 1))),
                          "vx": v.linear.x, "vy": v.linear.y, "speed": math.hypot(v.linear.x,v.linear.y), "wz": v.angular.z})

    def runtime_cb(self, message):
        self.runtime.append({"wall": time.monotonic(), "vx": message.twist.linear.x, "wz": message.twist.angular.z})

    def state_cb(self, message):
        self.states.append(json.loads(message.data))

    def send(self, velocity):
        message = Twist()
        message.linear.x = float(velocity)
        self.source.publish(message)
        self.last_source_wall = time.monotonic()

    def phase(self, stage, seconds, velocity=0.0, publish=True, check_gate=False):
        self.stage = stage
        start = self.sim
        deadline = time.monotonic()+max(45,seconds*10)
        next_send = 0
        while self.sim-start < seconds:
            now = time.monotonic()
            if now > deadline:
                raise RuntimeError("Simulation clock stalled")
            if publish and now >= next_send:
                self.send(velocity)
                next_send = now+.1
            rclpy.spin_once(self, timeout_sec=.02)
            if self.rows and (self.rows[-1]["tilt"] > .5 or self.rows[-1]["z"] < .2):
                raise RuntimeError("Posture limit")
            if check_gate and self.states and not self.states[-1]["armed"]:
                raise RuntimeError("Guard stopped motion: "+self.states[-1]["reason"])

    def arm(self, enabled):
        if not self.enable.wait_for_service(timeout_sec=3):
            return False
        request = SetBool.Request()
        request.data = enabled
        future = self.enable.call_async(request)
        until = time.monotonic()+5
        while not future.done() and time.monotonic() < until:
            self.send(0)
            rclpy.spin_once(self,timeout_sec=.05)
        if not future.done():
            return False
        response = future.result()
        if enabled and not response.success:
            raise RuntimeError("Arming refused: "+response.message)
        if enabled and response.success:
            until = time.monotonic()+2
            while time.monotonic()<until and (not self.states or not self.states[-1]["armed"]):
                self.send(0)
                rclpy.spin_once(self,timeout_sec=.02)
            if not self.states or not self.states[-1]["armed"]:
                raise RuntimeError("Enable response was not followed by an armed status")
        return response.success


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--boxes", type=Path, required=True)
    p.add_argument("--speed", type=float, default=.6)
    p.add_argument("--case", choices=["normal","input_timeout","gateway_death"],default="normal")
    p.add_argument("--gateway-pid",type=int)
    args = p.parse_args()
    assert .6 <= args.speed <= .8
    args.output.mkdir(parents=True,exist_ok=False)
    rclpy.init()
    probe = Probe()
    result = {"case":args.case,"command_speed_mps":args.speed}
    try:
        until = time.monotonic()+60
        while time.monotonic()<until and not (probe.sim>0 and probe.rows and probe.runtime and probe.states):
            probe.send(0)
            rclpy.spin_once(probe,timeout_sec=.1)
        if not (probe.rows and probe.runtime and probe.states):
            raise RuntimeError("Missing runtime telemetry")
        probe.phase("standing",2)
        if not probe.arm(True):
            raise RuntimeError("No gate enable service")
        result["start"] = dict(probe.rows[-1])
        seconds = 3 if args.case=="normal" else 2
        probe.phase("forward",seconds,args.speed,check_gate=True)
        result["forward_end"] = dict(probe.rows[-1])
        trigger = time.monotonic()
        result["trigger_wall"] = trigger
        if args.case=="gateway_death":
            if not args.gateway_pid or b"velocity_guard.py" not in Path(f"/proc/{args.gateway_pid}/cmdline").read_bytes():
                raise RuntimeError("PID is not this simulation velocity guard")
            if not probe.runtime or probe.runtime[-1]["vx"] < .59:
                raise RuntimeError("Nonzero runtime command not confirmed before fault injection")
            os.kill(args.gateway_pid,signal.SIGKILL)
            probe.phase("gateway_killed_braking",5,publish=False)
        elif args.case=="input_timeout":
            probe.phase("input_stopped_braking",5,publish=False)
            before = len(probe.runtime)
            probe.phase("no_auto_rearm",.6,args.speed)
            result["new_input_did_not_rearm"] = bool(probe.runtime[before:]) and all(abs(v["vx"])<1e-6 for v in probe.runtime[before:])
        else:
            probe.phase("zero_braking",5)
        result["stopped"] = dict(probe.rows[-1])
        zeros = [v for v in probe.runtime if v["wall"]>=trigger and abs(v["vx"])<1e-6]
        result["runtime_zero_latency_wall_s"] = zeros[0]["wall"]-trigger if zeros else None
        a,b = result["forward_end"],result["stopped"]
        result["post_trigger_distance_m"] = math.hypot(b["x"]-a["x"],b["y"]-a["y"])
        forward = [r for r in probe.rows if r["stage"]=="forward" and r["sim"]>=a["sim"]-1]
        result["steady_forward_vx_mps"] = float(np.mean([r["vx"] for r in forward]))
        result["forward_displacement_m"] = math.hypot(a["x"]-result["start"]["x"],a["y"]-result["start"]["y"])
    except Exception as exc:
        result["error"] = str(exc)
    finally:
        if args.case!="gateway_death":
            probe.arm(False)
        for _ in range(10):
            probe.send(0)
            rclpy.spin_once(probe,timeout_sec=.02)
        xy=np.array([[r["x"],r["y"]] for r in probe.rows]).reshape(-1,2)
        minimum=float("inf")
        for box in json.loads(args.boxes.read_text()):
            delta=np.maximum(np.maximum(np.array(box["min"][:2])-xy,xy-np.array(box["max"][:2])),0)
            if len(delta):minimum=min(minimum,float(np.linalg.norm(delta,axis=1).min()))
        result["min_center_aabb_clearance_m"] = minimum
        result["checks"] = {
            "no_error":"error" not in result,
            "runtime_command_received":any(v["vx"]>=.59 for v in probe.runtime),
            "forward_motion":result.get("forward_displacement_m",0)>.5,
            "steady_speed_tracks_command":result.get("steady_forward_vx_mps",0) >= args.speed*.8,
            "stopped_speed_low":result.get("stopped",{}).get("speed",99)<.05,
            "posture_bounded":bool(probe.rows) and max(r["tilt"] for r in probe.rows)<.5 and all(.2 <= r["z"] <= .85 for r in probe.rows),
            "static_clearance":minimum>.45,
            "runtime_watchdog_or_stop":result.get("runtime_zero_latency_wall_s") is not None and result["runtime_zero_latency_wall_s"]<(.9 if args.case=="gateway_death" else .65),
            "runtime_limit":all(0<=v["vx"]<=.80001 and abs(v["wz"])<=.60001 for v in probe.runtime),
        }
        if args.case=="input_timeout":
            result["checks"]["latches_until_explicit_enable"] = result.get("new_input_did_not_rearm",False)
        result["passed"] = all(result["checks"].values())
        (args.output/"motion_check.json").write_text(json.dumps(result,indent=2)+"\n")
        (args.output/"telemetry.json").write_text(json.dumps({"odom":probe.rows,"runtime_cmd":probe.runtime,"guard":probe.states},indent=2)+"\n")
        print(json.dumps(result,indent=2))
        probe.destroy_node()
        rclpy.shutdown()
    return 0 if result["passed"] else 1


if __name__=="__main__":
    raise SystemExit(main())
