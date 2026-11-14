"""Deterministic obstacle sampling on the placeable mask (Stage 1, v1.1).

Region-weighted placement (configurable)
    aisle      60%  free cells inside the rack band  (between shelf rows)
    shelf_edge 20%  ring of edge_margin_m around shelf structure
    open       20%  everything else placeable

Determinism contract
  * all randomness comes from the caller-supplied layout RNG
  * class iteration order is FIXED (box, pallet, barrier, cart)
  * region choice and every other choice are INTEGER draws (stable)
  * positions are cell centres -> exact, platform-stable values
  * instance ids are assigned per class in placement order
"""
from __future__ import annotations

import numpy as np

from .seed_manager import InstanceCounter
from . import placement_validator as pv

CLASS_ORDER = ["box", "pallet", "barrier", "cart"]
REGION_KEYS = {"aisle": "region_aisle", "shelf_edge": "region_edge", "open": "region_open"}


def _integral(a):
    c = a.astype(np.int64).cumsum(0).cumsum(1)
    t = np.zeros((a.shape[0] + 1, a.shape[1] + 1), dtype=np.int64)
    t[1:, 1:] = c
    return t


def build_mapdata(npz_path, wh, ob):
    d = np.load(npz_path)
    placeable = d["placeable"]
    res = float(d["res"][0])
    origin = (float(d["origin"][0]), float(d["origin"][1]))
    nx = int(d["dims"][0]); ny = int(d["dims"][1])

    cells = {}
    for name, key in REGION_KEYS.items():
        ys, xs = np.nonzero(d[key])
        cells[name] = (xs, ys)

    return {
        "res": res, "origin": origin, "nx": nx, "ny": ny,
        "placeable": placeable,
        "structure_occ": d["structure_occ"],
        "stock_occ": d["stock_occ"],
        "keep_clear": d["keep_clear"],
        "free": d["free"],
        "floor_mask": d["floor_mask"],
        "region_aisle": d["region_aisle"],
        "region_edge": d["region_edge"],
        "region_open": d["region_open"],
        "interior": d["interior"],
        "wall_mask": d["wall_mask"],
        "region_cells": cells,
        "int_placeable": _integral(placeable),
        "int_structure": _integral(d["structure_occ"]),
        "int_keepclear": _integral(d["keep_clear"]),
        "int_free": _integral(d["free"]),
        "int_interior": _integral(d["interior"]),
        "floor_extent": wh["base"]["floor_extent"],
        "obstacle_clearance": float(ob["clearance"]["obstacle"]),
    }


def _resolve_variants(entry):
    """Asset units for a class (supports variants or strategy.units)."""
    v = entry.get("variants")
    if v:
        return list(v)
    strat = entry.get("strategies") or {}
    key = entry.get("strategy")
    if key and key in strat:
        return list(strat[key].get("units") or [])
    return []


def _fits(obj, md):
    x0, y0, x1, y1 = pv._aabb(obj)
    ix0, iy0, ix1, iy1 = pv.rect_cells(x0, y0, x1, y1, md["origin"], md["res"], md["nx"], md["ny"])
    ncell = (ix1 - ix0 + 1) * (iy1 - iy0 + 1)
    return pv.rect_sum(md["int_placeable"], ix0, iy0, ix1, iy1) == ncell


def _too_close(obj, placed, clear):
    a = pv._aabb(obj, clear / 2)
    for o in placed:
        if pv._intersect(a, pv._aabb(o, clear / 2)):
            return True
    return False


def _hits_protected(obj, zones):
    x0, y0, x1, y1 = pv._aabb(obj)
    for (px0, py0, px1, py1) in zones:
        if not (x1 <= px0 or px1 <= x0 or y1 <= py0 or py1 <= y0):
            return True
    return False


def _normalised_weights(weights):
    """(names, cumulative thresholds/10000, per-mille table) for integer draws."""
    names = [n for n in ("aisle", "shelf_edge", "open") if weights and n in weights]
    if not names:
        weights = {"aisle": 60, "shelf_edge": 20, "open": 20}
        names = ["aisle", "shelf_edge", "open"]
    vals = np.array([float(weights[n]) for n in names], dtype=float)
    vals = vals / vals.sum()
    thr = np.cumsum(np.round(vals * 10000).astype(np.int64))
    permyriad = {n: int(round(v * 10000)) for n, v in zip(names, vals)}
    return names, thr, permyriad


def _quota_labels(total, names, permyriad, rng):
    """Region label per object with the exact integer quota (deterministic)."""
    raw = [permyriad[n] / 10000.0 * total for n in names]
    q = [int(np.floor(x)) for x in raw]
    rem = total - sum(q)
    frac_order = sorted(range(len(raw)), key=lambda i: (-(raw[i] - q[i]), i))
    for i in range(max(0, rem)):
        q[frac_order[i % len(raw)]] += 1
    labels = []
    for n, k in zip(names, q):
        labels += [n] * max(0, int(k))
    if len(labels) != total:  # safety, cannot happen with the above
        labels = (labels + [names[-1]] * total)[:total]
    perm = rng.permutation(len(labels))
    return [labels[int(i)] for i in perm]


def sample_layout(rng, md, obstacles_cfg, difficulty_cfg, registry,
                  density="medium", protected_zones=None, max_attempts=None,
                  region_weights=None, region_mode="quota"):
    C = obstacles_cfg["classes"]
    tier = difficulty_cfg["tiers"][density]
    clear = float(obstacles_cfg["clearance"]["obstacle"])
    if max_attempts is None:
        max_attempts = int(difficulty_cfg["limits"]["max_placement_attempts_per_object"])

    res = md["res"]; ox, oy = md["origin"]
    proto = protected_zones or []
    names, thr, permyriad = _normalised_weights(region_weights)

    counter = InstanceCounter()
    objs = []

    # ---- 1) draw counts for every class FIRST (fixed class order) ----
    wants = {}
    for cls in CLASS_ORDER:
        if cls in tier:
            lo, hi = tier[cls]
            wants[cls] = int(rng.integers(lo, hi + 1))
    total = sum(wants.values())

    # ---- 2) region assignment ----
    if str(region_mode).lower() == "quota" and total > 0:
        labels = _quota_labels(total, names, permyriad, rng)
    else:
        labels = []
        for _ in range(total):
            r = int(rng.integers(0, 10000))
            labels.append(names[min(int(np.searchsorted(thr, r, side="right")), len(names) - 1)])

    stats = {"requested": {}, "placed": {}, "attempts": {}, "failed": [],
             "by_region": {n: 0 for n in names},
             "region_weights": permyriad, "region_mode": str(region_mode)}

    idx = 0
    for cls in CLASS_ORDER:
        if cls not in wants:
            continue
        want = wants[cls]
        stats["requested"][cls] = want
        stats["placed"][cls] = 0
        stats["attempts"][cls] = 0
        entry = C.get(cls, {})
        variants = _resolve_variants(entry)
        yaws = [float(np.deg2rad(y)) for y in (entry.get("yaw_choices") or [0])]
        if not variants:
            idx += want
            continue

        for _k in range(want):
            rname = labels[idx] if idx < len(labels) else names[-1]
            idx += 1
            xs, ys = md["region_cells"][rname]
            if len(xs) == 0:
                stats["failed"].append({"class": cls, "region": rname,
                                        "reason": "region_empty"})
                continue

            done = False
            for _a in range(max_attempts):
                stats["attempts"][cls] += 1
                vidx = int(rng.integers(0, len(variants)))
                yidx = int(rng.integers(0, len(yaws)))
                ci = int(rng.integers(0, len(xs)))
                cx = ox + (int(xs[ci]) + 0.5) * res
                cy = oy + (int(ys[ci]) + 0.5) * res
                aid = variants[vidx]
                dims = registry["assets"][aid]["dims_m"]
                obj = {
                    "id": counter.next(cls),
                    "class": cls,
                    "asset_key": aid,
                    "region": rname,
                    "position": [round(cx, 3), round(cy, 3), 0.0],
                    "yaw_rad": round(yaws[yidx], 6),
                    "size": [float(dims[0]), float(dims[1]), float(dims[2])],
                }
                if not _fits(obj, md):
                    continue
                if _too_close(obj, objs, clear):
                    continue
                if _hits_protected(obj, proto):
                    continue
                objs.append(obj)
                stats["placed"][cls] += 1
                stats["by_region"][rname] += 1
                done = True
                break
            if not done:
                stats["failed"].append({"class": cls, "region": rname,
                                        "reason": "attempts_exhausted",
                                        "max_attempts": max_attempts})
    return objs, stats
