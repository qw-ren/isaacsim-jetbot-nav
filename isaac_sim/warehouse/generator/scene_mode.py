#!/usr/bin/env python3
"""Scene assembly for the three modes (experimental, v1.6-exp).

  A_only     random A-class obstacles only, with aisle connectivity guard
  B_only     deliberate full-cross-section aisle blockage only
  A_B_mixed  B first (chooses + seals the aisle), then A fills the LEGAL space
             that is left, and may not disconnect any other open aisle

Order matters and is deterministic: B is decided first, A is sampled afterwards
from the SAME layout RNG stream, so the whole scene is reproducible.
"""
from __future__ import annotations

import numpy as np

from . import blockage
from . import placement_validator as pv
from . import sampler_v2
from .seed_manager import InstanceCounter, layout_rng
from .topology import Topology

MODES = ("A_only", "B_only", "A_B_mixed")

# ---- v1.6.0 official parameters (single public scene type) ----
AISLE_PREF = 1.4
NUM_BLOCKED_AISLES = 1
LAYOUT_VERSION = "1.6.0"


class SceneGenerationError(RuntimeError):
    """Raised when a valid A+B mixed scene cannot be produced."""


def _open_aisles(topo, objs, base_grid):
    """Aisles that must stay passable with the given object set."""
    g = topo.grid(objs)
    out = {}
    for seg in topo.aisle_segments():
        ok, _ = topo.aisle_traversable(seg, g)
        out[seg["index"]] = ok
    return out


def enforce_connectivity(topo, objs, base_grid, blocked_ids=(), max_rounds=40):
    """Remove A objects until every aisle that should stay open is still open
    AND every blocked aisle still has a global alternative route.

    Deterministic: removes the LATEST placed objects first. B-class objects are
    never removed.
    """
    removed = []
    must_open = {s["index"] for s in topo.aisle_segments()} - set(blocked_ids)

    def global_ok(g):
        for bi in blocked_ids:
            seg = topo.aisle_segments()[bi]
            pts = topo.mouth_points(seg)
            if len(pts) != 2 or not topo.global_connected(pts[0], pts[1], g):
                return False
        return True

    for _ in range(max_rounds):
        g = topo.grid(objs)
        broken_local = []
        for si in sorted(must_open):
            seg = topo.aisle_segments()[si]
            ok, _ = topo.aisle_traversable(seg, g)
            base_ok, _ = topo.aisle_traversable(seg, base_grid)
            if base_ok and not ok:
                broken_local.append(si)
        if not broken_local and global_ok(g):
            break
        victim = None
        if broken_local:
            si = broken_local[0]
            inside = [o for o in objs if o.get("kind") == "A" and o.get("segment") == si]
            if inside:
                victim = inside[-1]
        if victim is None:
            # targeted: A objects sitting inside a blocked aisle, or close to one
            # of its two mouths, are the ones that can cut the detour
            a_objs = [o for o in objs if o.get("kind") == "A"]
            mouth_pts = []
            for bi in blocked_ids:
                mouth_pts.extend(topo.mouth_points(topo.aisle_segments()[bi]))

            def score(o):
                x, y = o["position"][0], o["position"][1]
                best = min(((x - mx) ** 2 + (y - my) ** 2) ** 0.5 for (mx, my) in mouth_pts) \
                    if mouth_pts else 1e9
                for bi in blocked_ids:
                    seg = topo.aisle_segments()[bi]
                    ix = int((x - topo.ox) / topo.res)
                    iy = int((y - topo.oy) / topo.res)
                    ix = max(0, min(topo.nx - 1, ix))
                    iy = max(0, min(topo.ny - 1, iy))
                    if seg["mask"][iy, ix]:
                        best = min(best, 0.0)
                return best

            if a_objs:
                ranked = sorted(a_objs, key=lambda o: (score(o), o["id"]))
                victim = ranked[0]
        if victim is None:
            break
        objs.remove(victim)
        removed.append(victim["id"])
    return removed


_TOPO_CACHE = {}


def _get_topo(md, wh):
    """Cache the topology + base pass grid: the fixed warehouse never changes."""
    key = id(md)
    if key not in _TOPO_CACHE:
        t = Topology(md, wh)
        _TOPO_CACHE[key] = (t, t.grid())
    return _TOPO_CACHE[key]


def build_scene(md, cfgs, seed, density="medium", mode="A_only", aisle_pref=2.0,
                num_blocked_aisles=1, protected_zones=None, layout_version="1.6.0-exp"):
    if mode not in MODES:
        raise ValueError("mode must be one of %s" % (MODES,))
    rng = layout_rng(seed)
    topo, base_grid = _get_topo(md, cfgs["warehouse"])
    registry = cfgs["asset_registry"]
    obstacles_cfg = cfgs["obstacles"]
    diff = cfgs["difficulty_v15"]
    proto = protected_zones or []

    spec = {"schema_version": layout_version, "seed": seed, "density": density,
            "mode": mode, "layout_version": layout_version,
            "metadata": {"mode": mode, "aisle_pref": aisle_pref,
                         "num_blocked_aisles": num_blocked_aisles,
                         "robot_radius_m": topo.robot_r},
            "objects": [], "blockages": []}

    # ---------------- B first ----------------
    if mode in ("B_only", "A_B_mixed") and num_blocked_aisles > 0:
        cands = blockage.candidate_aisles(topo, base_grid)
        order = [cands[i] for i in rng.permutation(len(cands))] if cands else []
        cnt = InstanceCounter()
        accepted_objs, accepted_info = [], []
        for seg in order:
            if len(accepted_info) >= num_blocked_aisles:
                break
            objs_b, info = blockage.build_blockage(rng, topo, seg, registry,
                                                   obstacles_cfg, proto, counter=cnt)
            if not objs_b:
                continue
            # cumulative check: this seal must block its aisle, keep a global
            # alternative, and not break the previously accepted ones
            v = blockage.verify_blockage(topo, seg, objs_b,
                                         accepted_objs + objs_b, proto)
            if not v["ok"]:
                continue
            accepted_objs.extend(objs_b)
            accepted_info.append({**info, "verify": v})
        spec["objects"].extend(accepted_objs)
        spec["blockages"] = accepted_info
        spec["blocked_aisles"] = [i["aisle"] for i in accepted_info]
        spec["B_requested"] = num_blocked_aisles

    counter_a = InstanceCounter()
    # A ids must not clash with B ids: B uses "blockage_", A uses class prefixes
    if mode in ("A_only", "A_B_mixed"):
        objs_a, stats = sampler_v2.sample_layout(
            rng, md, obstacles_cfg, diff, registry, density=density,
            protected_zones=proto, preplaced=[o for o in spec["objects"]],
            aisle_pref=aisle_pref, counter=counter_a)
        for o in objs_a:
            o["kind"] = "A"
        removed = enforce_connectivity(topo, objs_a, base_grid,
                                       blocked_ids=spec.get("blocked_aisles", []))
        spec["objects"].extend(objs_a)
        spec["a_stats"] = {"requested": stats["requested"], "placed": stats["placed"],
                           "by_region": stats["by_region"],
                           "connectivity_removed": removed,
                           "target_total": stats["target_total"]}
    # ---------------- final verification ----------------
    all_objs = spec["objects"]
    a_objs = [o for o in all_objs if o.get("kind") == "A"]
    b_objs = [o for o in all_objs if o.get("kind") == "B"]
    g = topo.grid(all_objs)
    aisles = {}
    for seg in topo.aisle_segments():
        ok, _ = topo.aisle_traversable(seg, g)
        base_ok, _ = topo.aisle_traversable(seg, base_grid)
        aisles[seg["index"]] = {"passable": bool(ok), "was_passable": bool(base_ok),
                                "blocked_by_design": seg["index"] in spec.get("blocked_aisles", [])}
    spec["aisle_state"] = aisles

    # global alternative check AFTER every A and B object is in place (not only
    # after B): the two mouths of each blocked aisle must still be connected
    # through the rest of the warehouse
    global_after = {}
    for b in spec.get("blockages", []):
        seg = topo.aisle_segments()[b["aisle"]]
        pts = topo.mouth_points(seg)
        global_after[b["aisle"]] = (bool(topo.global_connected(pts[0], pts[1], g))
                                    if len(pts) == 2 else None)
    spec["global_alternative_after_all"] = global_after

    # A objects are validated with the normal clearance rules; B objects follow
    # the blockage rules (touching is legal, only a real overlap is not), because
    # a seal has to be edge-to-edge by construction.
    va = pv.validate({"objects": a_objs}, md, md) if a_objs else {"valid": True, "issues": []}
    ab_ok = True
    for a in a_objs:
        for b in b_objs:
            if pv._intersect(pv._aabb(a), pv._aabb(b)):
                ab_ok = False
                break
    vb_legal = all(b["verify"]["legal"] for b in spec.get("blockages", []))
    vb_sealed = all((not b["verify"]["after_local_passable"]) and b["verify"]["global_alternative_exists"]
                    for b in spec.get("blockages", []))
    spec["validation"] = {"valid": bool(va.get("valid", True) and vb_legal and ab_ok),
                          "A": va, "B_legal": vb_legal, "B_sealed_with_alternative": vb_sealed,
                          "A_B_no_overlap": ab_ok,
                          "B_global_after_A": all(v for v in global_after.values()
                                                  if v is not None)}
    spec["object_count"] = len(all_objs)
    spec["counts_by_class"] = {c: sum(1 for o in all_objs if o["class"] == c)
                               for c in ("box", "pallet", "barrier", "cart", "blockage")}
    return spec


# =============================================================================
# v1.6.0 PUBLIC ENTRY 鈥?the ONLY scene type exposed to teammates
# =============================================================================
def build_mixed(md, cfgs, seed, density="medium", protected_zones=None):
    """Generate the official A+B mixed scene.

    Order is fixed: choose + seal the B aisle first, then fill the remaining
    legal space with A-class obstacles, then verify legality and connectivity.

    Raises SceneGenerationError if a valid sealed aisle with an alternative
    route cannot be produced 鈥?it never silently degrades to an A-only scene.
    """
    from .scene_generator import layout_hash

    if "difficulty_v15" not in cfgs:
        import os
        import yaml
        root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        with open(os.path.join(root, "config", "difficulty_v15_experiment.yaml")) as f:
            cfgs["difficulty_v15"] = yaml.safe_load(f)

    spec = build_scene(md, cfgs, seed=seed, density=density, mode="A_B_mixed",
                       aisle_pref=AISLE_PREF, num_blocked_aisles=NUM_BLOCKED_AISLES,
                       protected_zones=protected_zones, layout_version=LAYOUT_VERSION)

    blockages = spec.get("blockages", [])
    if len(blockages) != NUM_BLOCKED_AISLES:
        raise SceneGenerationError(
            "seed=%s density=%s: could not seal %d aisle(s) (got %d). "
            "diagnostics: %s" % (seed, density, NUM_BLOCKED_AISLES, len(blockages),
                                 [b.get("reason", "rejected") for b in blockages]))
    for b in blockages:
        v = b.get("verify", {})
        if not (v.get("before_passable") and not v.get("after_local_passable")
                and v.get("global_alternative_exists") and v.get("legal")):
            raise SceneGenerationError(
                "seed=%s density=%s aisle=%s failed blockage verification: %s"
                % (seed, density, b.get("aisle"), v))
    if not spec.get("validation", {}).get("valid"):
        raise SceneGenerationError(
            "seed=%s density=%s: final validation failed: %s"
            % (seed, density, spec.get("validation")))
    if not spec["validation"].get("B_global_after_A", True):
        raise SceneGenerationError(
            "seed=%s density=%s: after placing all objects the blocked aisle has no "
            "global alternative route: %s"
            % (seed, density, spec.get("global_alternative_after_all")))

    spec["layout_version"] = LAYOUT_VERSION
    spec["scene_type"] = "A_B_mixed"
    spec["layout_hash"] = layout_hash([{k: o[k] for k in ("id", "class", "asset_key",
                                                          "position", "yaw_rad")}
                                       for o in spec["objects"]])
    # make the spec strictly JSON-safe (numpy scalars come from the numpy-based
    # validation), so save_scene()/json.dump never fails on a teammate's machine
    import json as _json
    spec = _json.loads(_json.dumps(spec, default=_json_default))
    # int dict keys become strings through JSON; restore them for the aisle map
    if isinstance(spec.get("aisle_state"), dict):
        spec["aisle_state"] = {int(k): v for k, v in spec["aisle_state"].items()}
    return spec


def _json_default(o):
    import numpy as _np
    if isinstance(o, _np.bool_):
        return bool(o)
    if isinstance(o, _np.integer):
        return int(o)
    if isinstance(o, _np.floating):
        return float(o)
    if isinstance(o, _np.ndarray):
        return o.tolist()
    return str(o)
