#!/usr/bin/env python3
"""JetBot ↔ v1.6.0 warehouse integration layer (non-destructive).

Design
  * the existing robot and ROS 2 graphs are imported by REFERENCE, with their
    original prim paths (/World/jetbot, /World/ROS2), so every absolute-path
    binding inside the graphs keeps working -> nothing is re-created;
  * all adjustments (spawn pose, camera mount, differential parameters) are
    authored as OVERRIDES in the session layer only; neither jetbot_sim.usd nor
    any NVIDIA asset is modified or saved;
  * the robot lives outside /World/RandomObjects, so reset_scene() never touches
    it.

Measured facts (see README.md / docs/warehouse_random_scene.md):
  wheel radius (world)   = 0.1005 m
  wheel distance (world) = 0.2802 m
  wheel bottom at z=0.03 = -0.07087  ->  spawn z = 0.1009 puts it on the floor
"""
from __future__ import annotations

import math
import os

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

def _resolve_jetbot_src() -> str:
    """Locate the JetBot stage no matter where the repo was cloned.

    Order: $JETBOT_USD -> canonical path -> <repo>/isaac_sim/ -> <module>/project/ ->
    <repo>/project/... -> legacy absolute path (kept for the original machine).
    """
    here = os.path.dirname(os.path.abspath(__file__))          # .../integration
    mod_root = os.path.dirname(here)                            # .../warehouse
    repo_root = os.path.dirname(os.path.dirname(mod_root))      # repo root
    candidates = [
        os.environ.get("JETBOT_USD", ""),
        # canonical location: its own refs are ../../../isaacsim_assets, so it
        # must sit exactly here for the official assets to resolve
        "/home/admin/Desktop/jetbot_project/project/jetbot_sim.usd",
        os.path.join(repo_root, "isaac_sim", "jetbot_sim.usd"),
        os.path.join(mod_root, "project", "jetbot_sim.usd"),
        os.path.join(mod_root, "jetbot_sim.usd"),
        os.path.join(repo_root, "project", "jetbot_sim.usd"),
    ]
    for c in candidates:
        if c and os.path.exists(c):
            return os.path.abspath(c)
    return os.path.abspath(candidates[1])


JETBOT_SRC = _resolve_jetbot_src()
ROBOT_PATH = "/World/jetbot"
GRAPH_PATH = "/World/ROS2"

SPAWN = {"x": -9.97, "y": -7.37, "z": 0.1009, "yaw_deg": 0.0}
PROTECTED_ZONE = [(-10.57, -7.97, -9.37, -6.77)]   # 1.2 m x 1.2 m around the spawn

WHEEL_RADIUS = 0.1005        # measured world radius (NOT 0.09)
WHEEL_DISTANCE = 0.2802      # measured world centre distance (NOT 0.3375)

CAMERA_PRIM = "/World/jetbot/chassis/rgb_camera/jetbot_camera"
CAM_ROT_XYZ = (0.0, -90.0, -90.0)
CAM_TRANSLATE = (0.0, 0.0, 0.0)

BINDINGS = [
    ("differential_controller/ArticulationController", "inputs:targetPrim", ROBOT_PATH),
    ("ROS_Camera/isaac_create_render_product", "inputs:cameraPrim", CAMERA_PRIM),
    ("ROS_IMU/isaac_read_imu_node", "inputs:imuPrim",
     "/World/jetbot/chassis/imu/Imu_Sensor"),
    ("ROS_Lidar/isaac_create_render_product", "inputs:cameraPrim",
     "/World/jetbot/chassis/lidar/XT_32_10hz/PandarXT_32_10hz"),
    ("ROS_Odom/isaac_compute_odometry_node", "inputs:chassisPrim",
     "/World/jetbot/chassis"),
    ("ROS_TF/ros2_publish_transform_tree", "inputs:parentPrim",
     "/World/jetbot/chassis"),
]


# --------------------------------------------------------------------- import
def is_imported(stage):
    p = stage.GetPrimAtPath(ROBOT_PATH)
    return bool(p and p.IsValid() and p.GetTypeName())


def import_jetbot(stage, apply_overrides=True):
    """Reference the existing robot + graph subtrees. Idempotent."""
    if is_imported(stage):
        return {"imported": False, "reason": "already present"}

    for path, src_path in ((ROBOT_PATH, "/World/jetbot"), (GRAPH_PATH, "/World/ROS2")):
        prim = stage.DefinePrim(path)
        prim.GetReferences().AddReference(assetPath=JETBOT_SRC, primPath=src_path)

    if apply_overrides:
        apply_spawn(stage)
        apply_camera_override(stage)
        apply_controller_params(stage, WHEEL_RADIUS, WHEEL_DISTANCE)
        rebind_cross_scope(stage)
    return {"imported": True, "robot": ROBOT_PATH, "graph": GRAPH_PATH,
            "rebound": [r["prim"] + "." + r["rel"] for r in source_cross_scope_rels()]}


# ------------------------------------------------------------------- overrides
def _set_vec(op, v):
    """Write a 3-vector honouring the attribute's own precision (float/double)."""
    tn = op.GetAttr().GetTypeName()
    op.Set(Gf.Vec3d(*v) if tn == Sdf.ValueTypeNames.Double3 else Gf.Vec3f(*v))


def _set_quat_z(op, yaw_deg):
    h = math.radians(yaw_deg) / 2.0
    c, s = math.cos(h), math.sin(h)
    if op.GetAttr().GetTypeName() == Sdf.ValueTypeNames.Quatd:
        op.Set(Gf.Quatd(c, Gf.Vec3d(0, 0, s)))
    else:
        op.Set(Gf.Quatf(float(c), Gf.Vec3f(0, 0, float(s))))


def apply_spawn(stage, x=None, y=None, z=None, yaw_deg=None):
    x = SPAWN["x"] if x is None else x
    y = SPAWN["y"] if y is None else y
    z = SPAWN["z"] if z is None else z
    yaw_deg = SPAWN["yaw_deg"] if yaw_deg is None else yaw_deg
    prim = stage.GetPrimAtPath(ROBOT_PATH)
    xf = UsdGeom.Xformable(prim)
    for op in xf.GetOrderedXformOps():
        if op.GetOpName() == "xformOp:translate":
            _set_vec(op, (x, y, z))
        elif op.GetOpName() == "xformOp:orient":
            _set_quat_z(op, yaw_deg)
    return {"x": x, "y": y, "z": z, "yaw_deg": yaw_deg}


def apply_camera_override(stage):
    """translate = 0 and orient = Rz(-90)*Ry(-90)*Rx(0)  (user-specified mount)."""
    prim = stage.GetPrimAtPath(CAMERA_PRIM)
    xf = UsdGeom.Xformable(prim)
    existing = {op.GetOpName() for op in xf.GetOrderedXformOps()}
    if "xformOp:translate" not in existing:
        xf.AddTranslateOp()
    if "xformOp:orient" not in existing:
        xf.AddOrientOp()
    rx, ry, rz = CAM_ROT_XYZ
    rot = (Gf.Rotation(Gf.Vec3d(0, 0, 1), rz)
           * Gf.Rotation(Gf.Vec3d(0, 1, 0), ry)
           * Gf.Rotation(Gf.Vec3d(1, 0, 0), rx))
    q = rot.GetQuat()
    for op in xf.GetOrderedXformOps():
        if op.GetOpName() == "xformOp:translate":
            _set_vec(op, CAM_TRANSLATE)
        elif op.GetOpName() == "xformOp:orient":
            tn = op.GetAttr().GetTypeName()
            if tn == Sdf.ValueTypeNames.Quatd:
                op.Set(Gf.Quatd(float(q.GetReal()), q.GetImaginary()))
            else:
                op.Set(Gf.Quatf(float(q.GetReal()),
                                Gf.Vec3f(*[float(c) for c in q.GetImaginary()])))
    keep = [op for op in xf.GetOrderedXformOps()
            if op.GetOpName() in ("xformOp:translate", "xformOp:orient")]
    xf.SetXformOpOrder(keep)
    return {"translate": CAM_TRANSLATE, "rotateXYZ": CAM_ROT_XYZ,
            "orient_quat": [round(float(q.GetReal()), 6)]
                           + [round(float(c), 6) for c in q.GetImaginary()]}


def source_cross_scope_rels():
    """Relationships inside /World/ROS2 that point OUTSIDE that subtree.

    A subtree reference only maps paths inside the referenced prim, so these
    targets are dropped by USD unless they are re-authored in our layer.
    """
    src = Usd.Stage.Open(JETBOT_SRC)
    out = []
    root = src.GetPrimAtPath(GRAPH_PATH)
    for prim in Usd.PrimRange(root):
        for rel in prim.GetRelationships():
            targets = [str(t) for t in rel.GetTargets()]
            outside = [t for t in targets
                       if not (t == GRAPH_PATH or t.startswith(GRAPH_PATH + "/"))]
            if outside:
                out.append({"prim": str(prim.GetPath()), "rel": rel.GetName(),
                            "targets": targets})
    return out


def rebind_cross_scope(stage):
    """Re-author those relationship targets in our layer (same values)."""
    done = []
    for item in source_cross_scope_rels():
        prim = stage.GetPrimAtPath(item["prim"])
        if not prim or not prim.IsValid():
            continue
        rel = prim.GetRelationship(item["rel"])
        if not rel:
            rel = prim.CreateRelationship(item["rel"])
        rel.SetTargets([Sdf.Path(t) for t in item["targets"]])
        done.append(item)
    return done


def apply_controller_params(stage, radius=None, distance=None):
    radius = WHEEL_RADIUS if radius is None else radius
    distance = WHEEL_DISTANCE if distance is None else distance
    node = stage.GetPrimAtPath(GRAPH_PATH + "/differential_controller/DifferentialController")
    if not node or not node.IsValid():
        raise RuntimeError("differential controller node not found")
    a = node.GetAttribute("inputs:wheelRadius")
    b = node.GetAttribute("inputs:wheelDistance")
    if a:
        a.Set(float(radius))
    if b:
        b.Set(float(distance))
    return {"wheelRadius": float(radius), "wheelDistance": float(distance)}


# ------------------------------------------------------------------ inspection
def measure_wheels(stage):
    xc = UsdGeom.XformCache(Usd.TimeCode.Default())
    out = {}
    for side in ("left_wheel", "right_wheel"):
        prim = stage.GetPrimAtPath("%s/%s" % (ROBOT_PATH, side))
        c = xc.GetLocalToWorldTransform(prim).ExtractTranslation()
        sph = stage.GetPrimAtPath("%s/%s/Collision" % (ROBOT_PATH, side))
        r_local = UsdGeom.Sphere(sph).GetRadiusAttr().Get() if sph else None
        ws = xc.GetLocalToWorldTransform(sph).TransformDir(Gf.Vec3d(1, 0, 0)).GetLength() \
            if sph else None
        bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
        rng = bbox.ComputeWorldBound(prim).ComputeAlignedRange()
        out[side] = {"center": [round(float(v), 5) for v in c],
                     "world_radius": round(float(r_local * ws), 5) if r_local else None,
                     "bbox_min_z": round(float(rng.GetMin()[2]), 5),
                     "bbox_max_z": round(float(rng.GetMax()[2]), 5)}
    d = Gf.Vec3d(*out["left_wheel"]["center"]) - Gf.Vec3d(*out["right_wheel"]["center"])
    out["wheel_distance"] = round(float(d.GetLength()), 5)
    out["wheel_radius"] = out["left_wheel"]["world_radius"]
    out["wheel_bottom_z"] = min(out["left_wheel"]["bbox_min_z"],
                                out["right_wheel"]["bbox_min_z"])
    bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_,
                                                     UsdGeom.Tokens.render])
    r = bbox.ComputeWorldBound(stage.GetPrimAtPath(ROBOT_PATH)).ComputeAlignedRange()
    out["robot_bbox_min"] = [round(float(v), 4) for v in r.GetMin()]
    out["robot_bbox_max"] = [round(float(v), 4) for v in r.GetMax()]
    out["robot_size"] = [round(float(r.GetMax()[i] - r.GetMin()[i]), 4) for i in range(3)]
    return out


def verify_bindings(stage):
    res = []
    for node_rel, rel_name, expected in BINDINGS:
        prim = stage.GetPrimAtPath("%s/%s" % (GRAPH_PATH, node_rel))
        rel = prim.GetRelationship(rel_name) if prim else None
        targets = [str(t) for t in rel.GetTargets()] if rel else []
        ok = (expected in targets) and all(
            stage.GetPrimAtPath(t).IsValid() for t in targets)
        res.append({"node": node_rel, "rel": rel_name, "expected": expected,
                    "targets": targets, "valid": bool(ok)})
    # TF targetPrims (multi)
    tf = stage.GetPrimAtPath(GRAPH_PATH + "/ROS_TF/ros2_publish_transform_tree")
    rel = tf.GetRelationship("inputs:targetPrims") if tf else None
    targets = [str(t) for t in rel.GetTargets()] if rel else []
    res.append({"node": "ROS_TF/ros2_publish_transform_tree", "rel": "inputs:targetPrims",
                "expected": "3 sensor prims", "targets": targets,
                "valid": len(targets) == 3 and all(stage.GetPrimAtPath(t).IsValid()
                                                   for t in targets)})
    return res


def graph_nodes(stage):
    out = []
    root = stage.GetPrimAtPath(GRAPH_PATH)
    if not root:
        return out
    for prim in Usd.PrimRange(root):
        if prim.GetTypeName() == "OmniGraphNode":
            a = prim.GetAttribute("node:type")
            out.append({"path": str(prim.GetPath()),
                        "type": a.Get() if a else None})
    return out


def reset_robot(stage):
    """Restore the fixed spawn pose (no re-creation)."""
    apply_spawn(stage)
    return dict(SPAWN)


# --------------------------------------------------------------- safe reset
def _dc():
    from omni.isaac.dynamic_control import _dynamic_control
    return _dynamic_control, _dynamic_control.acquire_dynamic_control_interface()


def _dof_handles(dc, art):
    """Return (handles, method) for the articulation DOFs, trying known APIs."""
    n = dc.get_articulation_dof_count(art)
    # 1) get_articulation_dof(art, i)
    try:
        hs = [dc.get_articulation_dof(art, i) for i in range(n)]
        if hs and all(h for h in hs):
            return hs, "get_articulation_dof"
    except Exception:
        pass
    # 2) find_articulation_dof(art, name)
    names = ("left_wheel_joint", "right_wheel_joint")
    try:
        hs = [dc.find_articulation_dof(art, nm) for nm in names]
        if hs and all(h for h in hs):
            return hs, "find_articulation_dof"
    except Exception:
        pass
    return [], "unavailable"


def articulation_state():
    """Wheel joint velocities (requires physics initialised)."""
    out = {"valid": False, "dof_count": 0, "dof_velocities": [], "method": None,
           "error": None}
    try:
        DC, dc = _dc()
        art = dc.get_articulation(ROBOT_PATH)
        if not art:
            out["error"] = "articulation handle invalid (play the timeline once first)"
            return out
        dc.wake_up_articulation(art)
        out["dof_count"] = dc.get_articulation_dof_count(art)
        hs, method = _dof_handles(dc, art)
        out["method"] = method
        out["dof_velocities"] = [round(float(dc.get_dof_velocity(h)), 5) for h in hs]
        out["valid"] = bool(hs)
        if not hs:
            out["error"] = "no DOF handle API available"
    except Exception as e:
        out["error"] = str(e)
    return out


def zero_motion():
    """Zero wheel joint velocities and the chassis linear/angular velocity."""
    out = {"dof_zeroed": None, "root_linear": None, "root_angular": None,
           "method": None, "errors": []}
    try:
        DC, dc = _dc()
        art = dc.get_articulation(ROBOT_PATH)
        if not art:
            out["errors"].append("articulation handle invalid (physics not initialised)")
            return out
        dc.wake_up_articulation(art)
        hs, method = _dof_handles(dc, art)
        out["method"] = method
        for h in hs:
            dc.set_dof_velocity(h, 0.0)
        out["dof_zeroed"] = len(hs)
        body = dc.get_articulation_body(art, 0)
        for label, fn in (("root_linear", dc.set_rigid_body_linear_velocity),
                          ("root_angular", dc.set_rigid_body_angular_velocity)):
            try:
                fn(body, [0.0, 0.0, 0.0])
                out[label] = True
            except Exception as e:
                out["errors"].append("%s: %s" % (label, e))
    except Exception as e:
        out["errors"].append(str(e))
    return out


def clear_cmd_vel(topic="/cmd_vel", spins=8):
    """Publish a zero Twist so the controller holds no residual command."""
    out = {"published": 0, "ok": False, "error": None}
    try:
        import rclpy
        from geometry_msgs.msg import Twist
        if not rclpy.ok():
            rclpy.init(args=None)
        node = rclpy.create_node("jetbot_reset_cmdvel")
        pub = node.create_publisher(Twist, topic, 10)
        msg = Twist()
        for _ in range(spins):
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.02)
        node.destroy_node()
        out.update(published=spins, ok=True)
    except Exception as e:
        out["error"] = str(e)
    return out


def robot_aabb(stage):
    b = measure_wheels(stage)
    return b["robot_bbox_min"], b["robot_bbox_max"]


def check_spawn_collisions(stage, spec, md=None):
    """Robot AABB vs A/B objects, plus the protected zone must stay empty."""
    mn, mx = robot_aabb(stage)
    hits = []
    for o in spec.get("objects", []):
        ox, oy = o["position"][0], o["position"][1]
        hw, hd = o["size"][0] / 2.0, o["size"][1] / 2.0
        if not (mx[0] < ox - hw or ox + hw < mn[0] or mx[1] < oy - hd or oy + hd < mn[1]):
            hits.append(o["id"])
    zx0, zy0, zx1, zy1 = PROTECTED_ZONE[0]
    inside = [o["id"] for o in spec.get("objects", [])
              if zx0 <= o["position"][0] <= zx1 and zy0 <= o["position"][1] <= zy1]
    return {"robot_overlaps": hits, "objects_in_zone": inside,
            "ok": not hits and not inside}


def reset_and_apply(mgr, seed, density="medium", protected_zones=None,
                    resume=False):
    """Safe sequence: stop control -> pause -> reset pose/velocities -> new layout.

    The robot / sensors / graphs are never deleted or recreated.
    """
    import omni.timeline
    import omni.kit.app
    tl = omni.timeline.get_timeline_interface()
    was_playing = bool(tl.is_playing())
    if was_playing:
        tl.pause()
        for _ in range(3):
            omni.kit.app.get_app().update()

    cmd = clear_cmd_vel()
    apply_spawn(mgr.stage)
    motion = zero_motion()

    spec = mgr.reset_scene(seed=seed, density=density,
                           protected_zones=protected_zones or PROTECTED_ZONE)
    chk = check_spawn_collisions(mgr.stage, spec, mgr.gen.md)

    result = {"seed": seed, "density": density, "was_playing": was_playing,
              "paused": not bool(tl.is_playing()), "cmd_vel_zeroed": cmd,
              "motion_zeroed": motion, "spawn": dict(SPAWN),
              "scene": {"a": sum(1 for o in spec["objects"] if o.get("kind") == "A"),
                        "b": sum(1 for o in spec["objects"] if o.get("kind") == "B"),
                        "blocked": spec.get("blocked_aisles", []),
                        "hash": spec.get("layout_hash")},
              "collision": chk, "ok": bool(chk["ok"])}
    if resume and was_playing:
        tl.play()
    return result


def status(stage):
    return {
        "robot_present": bool(stage.GetPrimAtPath(ROBOT_PATH).IsValid()),
        "graph_present": bool(stage.GetPrimAtPath(GRAPH_PATH).IsValid()),
        "spawn": dict(SPAWN),
        "protected_zones": [list(z) for z in PROTECTED_ZONE],
        "wheels": measure_wheels(stage),
        "bindings": verify_bindings(stage),
        "graph_nodes": len(graph_nodes(stage)),
    }
