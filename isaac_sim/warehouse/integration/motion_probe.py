#!/usr/bin/env python3
"""Motion probe: measures the REAL robot motion from PhysX + simulation clock.

Why this module exists
----------------------
The earlier report computed the yaw rate from the USD Xform over **wall-clock**
time. Headless Isaac Sim runs slower than real time (measured sim/wall = 0.387),
so that number was wrong by that factor (0.276 rad/s instead of the true ~0.45).

This probe therefore:
  * reads the chassis rigid-body pose / angular velocity from PhysX
    (dynamic_control), never from the USD Xform;
  * stamps every sample with the **simulation clock** (/clock from the ROS graph,
    falling back to the physics timestamp);
  * also records the OmniGraph odometry node output and the /odom message, so
    control error and odometry error are reported separately.

All rates are computed per **simulation second**.
"""
from __future__ import annotations

import math

from pxr import Usd, UsdGeom

from . import jetbot_integration as ji


def _quat_yaw(q):
    if hasattr(q, "GetReal"):
        w = float(q.GetReal())
        x, y, z = [float(v) for v in q.GetImaginary()]
    elif hasattr(q, "w"):
        w, x, y, z = float(q.w), float(q.x), float(q.y), float(q.z)
    else:
        w, x, y, z = [float(v) for v in q]
    return 2.0 * math.atan2(z, w)


def _vec3(v):
    try:
        return float(v[0]), float(v[1]), float(v[2])
    except Exception:
        return float(v.x), float(v.y), float(v.z)


def _unroll(vals):
    out = [vals[0]]
    for v in vals[1:]:
        d = v - out[-1]
        while d > math.pi:
            d -= 2 * math.pi
        while d < -math.pi:
            d += 2 * math.pi
        out.append(out[-1] + d)
    return out


class MotionProbe:
    """Samples the robot at every frame and reports per-sim-second rates."""

    def __init__(self, stage, sim_time_fn=None):
        from omni.isaac.dynamic_control import _dynamic_control
        self.dc = _dynamic_control.acquire_dynamic_control_interface()
        self.stage = stage
        self.art = self.dc.get_articulation(ji.ROBOT_PATH)
        if self.art:
            self.dc.wake_up_articulation(self.art)
            self.body = self.dc.get_articulation_body(self.art, 0)
            self.dofs, self.dof_method = ji._dof_handles(self.dc, self.art)
        else:
            self.body, self.dofs, self.dof_method = None, [], None
        self.sim_time = sim_time_fn or (lambda: None)
        self.rows = []

    # ------------------------------------------------------------------ sample
    def sample(self, phase="", cmd_lin=0.0, cmd_ang=0.0, og_wz=None, odom=None):
        if not self.body:
            return None
        tr = self.dc.get_rigid_body_pose(self.body)
        av = _vec3(self.dc.get_rigid_body_angular_velocity(self.body))
        lv = _vec3(self.dc.get_rigid_body_linear_velocity(self.body))
        dof = [float(self.dc.get_dof_velocity(h)) for h in self.dofs] if self.dofs else []
        row = {"sim": self.sim_time(), "phase": phase,
               "cmd_lin": cmd_lin, "cmd_ang": cmd_ang,
               "yaw": _quat_yaw(tr.r), "physx_wz": av[2],
               "lin_x": lv[0], "lin_y": lv[1],
               "dof_l": dof[0] if len(dof) > 0 else None,
               "dof_r": dof[1] if len(dof) > 1 else None,
               "og_wz": og_wz,
               "odom_wz": None, "odom_yaw": None, "odom_t": None}
        if odom is not None:
            q = odom.pose.pose.orientation
            row["odom_wz"] = float(odom.twist.twist.angular.z)
            row["odom_yaw"] = 2.0 * math.atan2(q.z, q.w)
            row["odom_t"] = odom.header.stamp.sec + odom.header.stamp.nanosec * 1e-9
        self.rows.append(row)
        return row

    # ---------------------------------------------------------------- steady
    def steady(self, phase, lo=0.2, hi=0.9):
        """Steady-state rates for one phase, all per SIMULATION second."""
        seg = [r for r in self.rows if r["phase"] == phase and r["sim"] is not None]
        if len(seg) < 8:
            return {"error": "too few samples", "n": len(seg)}
        seg = seg[int(len(seg) * lo): int(len(seg) * hi)]
        t = [r["sim"] for r in seg]
        dt = t[-1] - t[0]
        if dt <= 0:
            return {"error": "bad sim clock", "dt": dt}
        yaw = _unroll([r["yaw"] for r in seg])
        res = {"n": len(seg), "dt_sim": round(dt, 4),
               "cmd_ang": round(seg[0]["cmd_ang"], 4),
               "physx_wz_mean": round(sum(r["physx_wz"] for r in seg) / len(seg), 4),
               "yaw_rate_physx_pose": round((yaw[-1] - yaw[0]) / dt, 4)}
        dof_l = [r["dof_l"] for r in seg if r["dof_l"] is not None]
        dof_r = [r["dof_r"] for r in seg if r["dof_r"] is not None]
        if dof_l and dof_r:
            ml, mr = sum(dof_l) / len(dof_l), sum(dof_r) / len(dof_r)
            res["dof_l"] = round(ml, 4)
            res["dof_r"] = round(mr, 4)
            res["wheel_yaw_rate"] = round((mr - ml) * ji.WHEEL_RADIUS / ji.WHEEL_DISTANCE, 4)
            res["linear_speed_wheel"] = round((mr + ml) / 2 * ji.WHEEL_RADIUS, 4)
        og = [r["og_wz"] for r in seg if r["og_wz"] is not None]
        if og:
            res["og_wz"] = round(sum(og) / len(og), 4)
        od = [r["odom_wz"] for r in seg if r["odom_wz"] is not None]
        if od:
            res["odom_wz"] = round(sum(od) / len(od), 4)
            odt = [r for r in seg if r["odom_yaw"] is not None]
            if len(odt) > 4:
                t2 = [r["odom_t"] for r in odt]
                y2 = _unroll([r["odom_yaw"] for r in odt])
                if t2[-1] > t2[0]:
                    res["yaw_rate_odom"] = round((y2[-1] - y2[0]) / (t2[-1] - t2[0]), 4)
        # ---- the two errors that must never be confused ----
        cmd = res["cmd_ang"]
        if cmd:
            res["control_error_pct"] = round(
                100.0 * (res["physx_wz_mean"] - cmd) / cmd, 2)
            if "odom_wz" in res:
                res["odometry_error_pct"] = round(
                    100.0 * (res["odom_wz"] - cmd) / cmd, 2)
            if "wheel_yaw_rate" in res and res["wheel_yaw_rate"]:
                res["slip_ratio"] = round(
                    1.0 - res["physx_wz_mean"] / res["wheel_yaw_rate"], 3)
        return res
