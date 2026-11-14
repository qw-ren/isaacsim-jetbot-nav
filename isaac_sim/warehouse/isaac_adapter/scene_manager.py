"""Isaac Sim scene manager for the warehouse seed system (Stage 2).

Teammates only need this class; no USD prim management required:

    mgr = WarehouseSceneManager(ctx)      # omni.usd.get_context()
    mgr.load_base_scene()                 # fixed warehouse (derived, read-only)
    spec = mgr.reset_scene(1001, "medium")# generate + apply
    info = mgr.get_scene_info()           # what is actually on the stage
    mgr.save_scene("out.json")            # current spec -> JSON
    mgr.load_scene("out.json")            # JSON -> stage
    mgr.clear_random_obstacles()          # empty random layer

The derived USD on disk is never modified; everything happens in the session.
"""
from __future__ import annotations

import hashlib
import json
import math
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from pxr import Gf, Usd, UsdGeom, UsdPhysics  # noqa: E402

from generator import config  # noqa: E402
from generator import scene_mode  # noqa: E402
from generator.scene_generator import SceneGenerator, layout_hash  # noqa: E402

RANDOM_ROOT = "/World/RandomObjects"


def _sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()


class WarehouseSceneManager:
    # ---------------------------------------------------------------- setup
    def __init__(self, ctx=None, config_dir=None):
        self.cfgs = config.load_all() if config_dir is None else None
        self.reg = self.cfgs["asset_registry"]
        self.wh = self.cfgs["warehouse"]
        self.ctx = ctx
        self.stage = None
        self.gen = SceneGenerator.from_config(config_dir)
        self._ground_offset = {}
        self.current_spec = None

    # ---------------------------------------------------------------- base
    def base_versions(self):
        derived = os.path.join(ROOT, self.wh["base"]["entry_point"])
        meta = json.load(open(os.path.join(ROOT, "scenes", "base_meta.json")))
        mapm = json.load(open(os.path.join(ROOT, self.wh["map"]["meta"])))
        h = _sha256(derived)
        return {
            "derived_usd": self.wh["base"]["entry_point"],
            "derived_sha256": h,
            "derived_matches_build": h == meta["derived_usd_sha256"],
            "official_usd_sha256": meta["official_usd_sha256"],
            "map_sha256": mapm["map_sha256"],
            "map_z_band_m": list(mapm["z_band_m"]),
        }

    def load_base_scene(self):
        derived = os.path.join(ROOT, self.wh["base"]["entry_point"])
        if not os.path.exists(derived):
            raise FileNotFoundError(derived)
        if self.ctx is not None:
            self.ctx.open_stage(derived)
            self.stage = self.ctx.get_stage()
        else:
            self.stage = Usd.Stage.Open(derived)
        if not self.stage.GetPrimAtPath(RANDOM_ROOT):
            UsdGeom.Xform.Define(self.stage, RANDOM_ROOT)
        self.ensure_physics_scene()
        return self.stage

    def ensure_physics_scene(self):
        p = "/World/PhysicsScene"
        if not self.stage.GetPrimAtPath(p):
            sc = UsdPhysics.Scene.Define(self.stage, p)
            sc.CreateGravityDirectionAttr().Set(Gf.Vec3f(0, 0, -1))
            sc.CreateGravityMagnitudeAttr().Set(9.81)
        return p

    def asset_path(self, aid):
        return os.path.join(self.reg["source_root"], self.reg["assets"][aid]["path"])

    # ---------------------------------------------------------------- random layer
    def clear_random_obstacles(self):
        root = self.stage.GetPrimAtPath(RANDOM_ROOT)
        if not root or not root.IsValid():
            return 0
        n = 0
        for ch in list(root.GetChildren()):
            self.stage.RemovePrim(ch.GetPath())
            n += 1
        return n

    def ground_offset(self, aid):
        if aid in self._ground_offset:
            return self._ground_offset[aid]
        cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                                  [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
        tmp = Usd.Stage.CreateInMemory()
        p = UsdGeom.Xform.Define(tmp, "/T").GetPrim()
        p.GetReferences().AddReference(self.asset_path(aid))
        r = cache.ComputeWorldBound(p).ComputeAlignedRange()
        off = 0.0 if r.IsEmpty() else -float(r.GetMin()[2])
        self._ground_offset[aid] = off
        return off

    def apply_scene(self, spec, ground=True):
        if not self.stage.GetPrimAtPath(RANDOM_ROOT):
            UsdGeom.Xform.Define(self.stage, RANDOM_ROOT)
        info = []
        for o in spec["objects"]:
            path = "%s/%s" % (RANDOM_ROOT, o["id"])
            xf = UsdGeom.Xform.Define(self.stage, path)
            prim = xf.GetPrim()
            prim.GetReferences().AddReference(self.asset_path(o["asset_key"]))
            # collision is authored in the official asset; make sure it is enabled
            for q in Usd.PrimRange(prim):
                if q.HasAPI(UsdPhysics.CollisionAPI):
                    a = UsdPhysics.CollisionAPI(q).GetCollisionEnabledAttr()
                    if not a or a.Get() is None:
                        a = UsdPhysics.CollisionAPI(q).CreateCollisionEnabledAttr()
                    a.Set(True)
            z = float(o["position"][2]) + (self.ground_offset(o["asset_key"]) if ground else 0.0)
            xf.AddTranslateOp().Set(Gf.Vec3d(o["position"][0], o["position"][1], z))
            xf.AddRotateZOp().Set(float(np.degrees(o["yaw_rad"])))
            info.append({"id": o["id"], "path": path})
        self.current_spec = spec
        return info

    # ---------------------------------------------------------------- high level
    def generate(self, seed, density="medium", protected_zones=None):
        """Official v1.6.0 scene: A-class random obstacles + one sealed B aisle.

        This is the only scene type exposed to teammates; `mode` is not a
        parameter. Raises generator.scene_mode.SceneGenerationError if a valid
        A+B scene cannot be produced for this seed.
        """
        return scene_mode.build_mixed(self.gen.md, self.cfgs, seed,
                                      density=density,
                                      protected_zones=protected_zones)

    def generate_and_apply(self, seed, density="medium", protected_zones=None):
        spec = self.generate(seed, density, protected_zones)
        self.clear_random_obstacles()
        self.apply_scene(spec)
        return spec

    def reset_scene(self, seed, density="medium", protected_zones=None):
        """Switch to another seed: clear the random layer, then regenerate+apply."""
        spec = self.generate(seed, density, protected_zones)
        self.clear_random_obstacles()
        self.apply_scene(spec)
        return spec

    # ---------------------------------------------------------------- io
    def save_scene(self, path, spec=None):
        sp = spec if spec is not None else self.current_spec
        if sp is None:
            raise RuntimeError("no current scene to save")
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        with open(path, "w") as f:
            json.dump(sp, f, indent=2, sort_keys=True,
                      default=lambda o: bool(o) if isinstance(o, np.bool_) else
                      int(o) if isinstance(o, np.integer) else
                      float(o) if isinstance(o, np.floating) else str(o))
        return path

    def load_scene(self, path, apply=True):
        with open(path) as f:
            spec = json.load(f)
        if apply:
            self.clear_random_obstacles()
            self.apply_scene(spec)
        return spec

    # ---------------------------------------------------------------- readback
    def get_scene_info(self):
        cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                                  [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
        root = self.stage.GetPrimAtPath(RANDOM_ROOT)
        out = []
        if not root or not root.IsValid():
            return {"seed": None, "density": None, "layout_hash": None,
                    "layout_version": None, "scene_type": None,
                    "a_count": 0, "b_count": 0, "count": 0,
                    "blocked_aisles": [], "counts_by_class": {}, "objects": []}
        for ch in root.GetChildren():
            r = cache.ComputeWorldBound(ch).ComputeAlignedRange()
            m = UsdGeom.XformCache(Usd.TimeCode.Default()).GetLocalToWorldTransform(ch)
            t = m.ExtractTranslation()
            xa = m.TransformDir(Gf.Vec3d(1, 0, 0))
            yaw = math.degrees(math.atan2(float(xa[1]), float(xa[0]))) % 360.0
            refs = []
            ro = ch.GetMetadata("references")
            if ro is not None and hasattr(ro, "GetAddedOrExplicitItems"):
                refs = [x.assetPath for x in ro.GetAddedOrExplicitItems()]
            ncol, approx = 0, set()
            for p in Usd.PrimRange(ch):
                if p.HasAPI(UsdPhysics.CollisionAPI):
                    ncol += 1
                    ap = UsdPhysics.MeshCollisionAPI(p).GetApproximationAttr() \
                        if p.HasAPI(UsdPhysics.MeshCollisionAPI) else None
                    approx.add(ap.Get() if ap else "default")
            mn, mx = r.GetMin(), r.GetMax()
            out.append({
                "id": ch.GetName(), "path": str(ch.GetPath()),
                "reference": refs[0] if refs else None,
                "world_translate": [round(float(t[i]), 4) for i in range(3)],
                "world_yaw_deg": round(float(yaw), 3),
                "bbox_min": [round(float(v), 4) for v in mn],
                "bbox_max": [round(float(v), 4) for v in mx],
                "size": [round(float(mx[i] - mn[i]), 4) for i in range(3)],
                "collision_prims": ncol,
                "collision_approximations": sorted(str(a) for a in approx),
            })
        sp = self.current_spec or {}
        a_count = sum(1 for o in out if not o["id"].startswith("blockage_"))
        b_count = len(out) - a_count
        return {"seed": sp.get("seed"), "density": sp.get("density"),
                "layout_hash": sp.get("layout_hash"),
                "layout_version": sp.get("layout_version"),
                "scene_type": sp.get("scene_type", "A_B_mixed"),
                "a_count": a_count, "b_count": b_count, "count": len(out),
                "blocked_aisles": sp.get("blocked_aisles", []),
                "counts_by_class": sp.get("counts_by_class", {}),
                "objects": out}
