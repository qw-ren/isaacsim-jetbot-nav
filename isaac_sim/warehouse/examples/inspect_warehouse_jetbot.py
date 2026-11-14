#!/usr/bin/env python3
"""Standalone integration check: v1.6.0 warehouse + existing JetBot + ROS 2 graphs.

    /home/admin/isaacsim/python.sh examples/inspect_warehouse_jetbot.py

Prints a JSON report: robot pose/contact, sensor prims, graph bindings,
A/B object counts and the protected zone check.
"""
import json
import os
import sys

from isaacsim import SimulationApp

app = SimulationApp({"headless": True, "width": 640, "height": 480})

import omni.usd  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from isaac_adapter.scene_manager import WarehouseSceneManager  # noqa: E402
from integration import jetbot_integration as ji  # noqa: E402


def main():
    mgr = WarehouseSceneManager(ctx=omni.usd.get_context())
    stage = mgr.load_base_scene()
    for _ in range(30):
        app.update()

    rep = {"stage_prims_before": sum(1 for _ in stage.Traverse())}
    rep["import"] = ji.import_jetbot(stage)
    for _ in range(20):
        app.update()

    rep["wheels"] = ji.measure_wheels(stage)
    rep["spawn"] = dict(ji.SPAWN)
    rep["protected_zones"] = ji.PROTECTED_ZONE
    rep["bindings"] = ji.verify_bindings(stage)
    rep["graph_nodes"] = len(ji.graph_nodes(stage))
    rep["graphs"] = [c.GetName() for c in
                     stage.GetPrimAtPath(ji.GRAPH_PATH).GetChildren()]

    # camera world position after the mount override
    from pxr import Usd, UsdGeom
    xc = UsdGeom.XformCache(Usd.TimeCode.Default())
    cam = stage.GetPrimAtPath(ji.CAMERA_PRIM)
    t = xc.GetLocalToWorldTransform(cam).ExtractTranslation()
    rep["camera_world"] = [round(float(v), 4) for v in t]

    # scene
    spec = mgr.reset_scene(seed=1001, density="medium",
                           protected_zones=ji.PROTECTED_ZONE)
    for _ in range(10):
        app.update()
    info = mgr.get_scene_info()
    rep["scene"] = {"a": info["a_count"], "b": info["b_count"], "total": info["count"],
                    "blocked": info["blocked_aisles"], "hash": info["layout_hash"]}

    # protected zone must stay empty
    zx0, zy0, zx1, zy1 = ji.PROTECTED_ZONE[0]
    inside = [o["id"] for o in spec["objects"]
              if zx0 <= o["position"][0] <= zx1 and zy0 <= o["position"][1] <= zy1]
    rep["zone_objects"] = inside

    # robot AABB vs every A/B object (should never intersect)
    b = ji.measure_wheels(stage)
    rx0, ry0 = b["robot_bbox_min"][0], b["robot_bbox_min"][1]
    rx1, ry1 = b["robot_bbox_max"][0], b["robot_bbox_max"][1]
    hits = []
    for o in spec["objects"]:
        ox, oy = o["position"][0], o["position"][1]
        hw, hd = o["size"][0] / 2, o["size"][1] / 2
        if not (rx1 < ox - hw or ox + hw < rx0 or ry1 < oy - hd or oy + hd < ry0):
            hits.append(o["id"])
    rep["robot_overlaps"] = hits
    rep["contact"] = {
        "wheel_bottom_z": b["wheel_bottom_z"],
        "on_floor": abs(b["wheel_bottom_z"]) < 0.005,
        "robot_bbox_min_z": b["robot_bbox_min"][2],
    }
    rep["ok"] = (all(x["valid"] for x in rep["bindings"])
                 and not inside and not hits and rep["contact"]["on_floor"]
                 and len(rep["graphs"]) == 7)

    print("REPORT_BEGIN")
    print(json.dumps(rep, indent=2, ensure_ascii=False))
    print("REPORT_END")
    sys.stdout.flush()
    os._exit(0)


if __name__ == "__main__":
    main()
