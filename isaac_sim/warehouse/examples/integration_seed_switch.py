#!/usr/bin/env python3
"""Checkpoint 2: seed switching + renders for the JetBot integration.

    /home/admin/isaacsim/python.sh examples/integration_seed_switch.py

1. import the existing JetBot + ROS 2 graphs into the v1.6.0 warehouse
2. Seed 1001M -> 1002H -> 1001M with the robot protected zone
3. verify the robot/graph are never touched, no duplicates, layout reproduces
4. render panorama / close-up / robot camera view
"""
import json
import os
import sys

from isaacsim import SimulationApp

app = SimulationApp({"headless": True, "width": 1600, "height": 900})

import omni.replicator.core as rep  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, Usd, UsdGeom  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from isaac_adapter.scene_manager import WarehouseSceneManager  # noqa: E402
from integration import jetbot_integration as ji  # noqa: E402

OUT = "/tmp/jetbot_final"
os.makedirs(OUT, exist_ok=True)


def log(*a):
    print(*a, flush=True)


def look_at(prim, eye, target):
    eye = Gf.Vec3d(*eye); target = Gf.Vec3d(*target)
    f = (target - eye).GetNormalized()
    up = Gf.Vec3d(0, 0, 1)
    if abs(Gf.Dot(f, up)) > 0.999:
        up = Gf.Vec3d(0, 1, 0)
    rr = Gf.Cross(f, up).GetNormalized()
    uu = Gf.Cross(rr, f).GetNormalized()
    m = Gf.Matrix4d(rr[0], rr[1], rr[2], 0, uu[0], uu[1], uu[2], 0,
                    -f[0], -f[1], -f[2], 0, eye[0], eye[1], eye[2], 1)
    x = UsdGeom.Xformable(prim)
    x.ClearXformOpOrder()
    x.AddTransformOp().Set(m)


def render(stage, tag, cam_path, eye, target, focal, res, use_existing=False):
    outdir = os.path.join(OUT, tag)
    os.makedirs(outdir, exist_ok=True)
    if not use_existing:
        old = stage.GetPrimAtPath(cam_path)
        if old and old.IsValid():
            stage.RemovePrim(cam_path)
        cam = UsdGeom.Camera.Define(stage, cam_path)
        cam.GetClippingRangeAttr().Set(Gf.Vec2f(0.05, 4000.0))
        cam.GetHorizontalApertureAttr().Set(36.0)
        cam.GetVerticalApertureAttr().Set(36.0 * res[1] / res[0])
        look_at(stage.GetPrimAtPath(cam_path), eye, target)
        cam.GetFocalLengthAttr().Set(focal)
    rp = rep.create.render_product(cam_path, res)
    w = rep.WriterRegistry.get("BasicWriter")
    w.initialize(output_dir=outdir, rgb=True)
    w.attach([rp])
    for _ in range(60):
        app.update()
    rep.orchestrator.step()
    for _ in range(15):
        app.update()
    src = os.path.join(outdir, "rgb_0000.png")
    dst = os.path.join(OUT, tag + ".png")
    if os.path.exists(src):
        if os.path.exists(dst):
            os.remove(dst)
        os.rename(src, dst)
        log("saved", dst)
    try:
        w.detach()
    except Exception:
        pass
    return dst


ctx = omni.usd.get_context()
mgr = WarehouseSceneManager(ctx=ctx)
stage = mgr.load_base_scene()
for _ in range(30):
    app.update()
for p in stage.Traverse():
    if p.GetName().startswith(("SM_CeilingA", "SM_LampCeilingA", "SM_BeamA",
                               "SM_BracketBeam", "SM_BracketSlot")):
        UsdGeom.Imageable(p).GetVisibilityAttr().Set(UsdGeom.Tokens.invisible)

ji.import_jetbot(stage)
for _ in range(15):
    app.update()

robot_prims_initial = sum(1 for _ in Usd.PrimRange(stage.GetPrimAtPath(ji.ROBOT_PATH)))
graphs_initial = len(stage.GetPrimAtPath(ji.GRAPH_PATH).GetChildren())


def snapshot(tag, seed, dens):
    info = mgr.get_scene_info()
    xc = UsdGeom.XformCache(Usd.TimeCode.Default())
    t = xc.GetLocalToWorldTransform(
        stage.GetPrimAtPath(ji.ROBOT_PATH)).ExtractTranslation()
    w = ji.measure_wheels(stage)
    return {
        "tag": tag, "seed": seed, "density": dens,
        "a": info["a_count"], "b": info["b_count"], "total": info["count"],
        "blocked": info["blocked_aisles"], "hash": info["layout_hash"],
        "robot_world": [round(float(v), 4) for v in t],
        "robot_prims": sum(1 for _ in Usd.PrimRange(stage.GetPrimAtPath(ji.ROBOT_PATH))),
        "graph_children": len(stage.GetPrimAtPath(ji.GRAPH_PATH).GetChildren()),
        "graph_nodes": len(ji.graph_nodes(stage)),
        "wheel_bottom_z": w["wheel_bottom_z"],
        "bindings_ok": all(b["valid"] for b in ji.verify_bindings(stage)),
    }


snaps = []
for seed, dens, tag in ((1001, "medium", "1001M"), (1002, "hard", "1002H"),
                        (1001, "medium", "1001M_again")):
    mgr.reset_scene(seed=seed, density=dens, protected_zones=ji.PROTECTED_ZONE)
    for _ in range(10):
        app.update()
    s = snapshot(tag, seed, dens)
    snaps.append(s)
    log("%-10s A=%2d B=%d blocked=%s hash=%s robot=%s prims=%d nodes=%d"
        % (tag, s["a"], s["b"], s["blocked"], (s["hash"] or "")[:12],
           s["robot_world"], s["robot_prims"], s["graph_nodes"]))

a, b, c = snaps
res = {"snapshots": snaps,
       "robot_prims_initial": robot_prims_initial,
       "graphs_initial": graphs_initial,
       "robot_prims_constant": len({s["robot_prims"] for s in snaps}) == 1,
       "graph_children_constant": len({s["graph_children"] for s in snaps}) == 1,
       "graph_nodes": c["graph_nodes"],
       "robot_pose_constant": a["robot_world"] == c["robot_world"],
       "layout_reproduced": a["hash"] == c["hash"],
       "layout_differs_1002": a["hash"] != b["hash"],
       "wheel_contact_ok": abs(c["wheel_bottom_z"]) < 0.005,
       "bindings_ok": all(x["bindings_ok"] for x in snaps)}

# leaves a 1001M scene on screen for the renders
sx, sy = ji.SPAWN["x"], ji.SPAWN["y"]
render(stage, "panorama", "/World/_pano", (-10.0, -30.0, 26.0), (-10.0, 2.0, 0.0),
       24.0, (1600, 900))
render(stage, "closeup", "/World/_close", (sx - 1.9, sy - 1.7, 0.95),
       (sx, sy, 0.30), 45.0, (1400, 900))
render(stage, "camera_view", ji.CAMERA_PRIM, None, None, None, (960, 540),
       use_existing=True)

res["ok"] = (res["robot_prims_constant"] and res["graph_children_constant"]
             and res["robot_pose_constant"] and res["layout_reproduced"]
             and res["layout_differs_1002"] and res["wheel_contact_ok"]
             and res["bindings_ok"] and res["graph_nodes"] == 39)
with open(os.path.join(OUT, "report.json"), "w") as f:
    json.dump(res, f, indent=2, ensure_ascii=False, default=str)
print("FINAL_REPORT_BEGIN")
print(json.dumps({k: v for k, v in res.items() if k != "snapshots"}, indent=2))
print("FINAL_REPORT_END")
sys.stdout.flush()
os._exit(0)
