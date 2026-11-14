#!/usr/bin/env python3
"""Build the 2D occupancy map FROM THE DERIVED USD (never from a planning rect).

v1.3 changes (map-accuracy audit)
  * rasterise LEAF geometry only (Mesh/Cube/...), never a parent Xform bbox
    -> no bbox inflation, no double counting
  * height filter applied PER LEAF (world bbox intersects z[z_floor, passage])
  * optional requirement: leaf must carry UsdPhysics.CollisionAPI
  * walls now include SM_PillarPartA* (real 9 m floor-standing partitions)

Interior envelope: largest connected component of (floor & ~wall).
"""
from __future__ import annotations

import hashlib
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np  # noqa: E402
from scipy import ndimage  # noqa: E402
from pxr import Usd, UsdGeom, UsdPhysics  # noqa: E402

from generator import config  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

SKIP = ("floor", "ceiling", "light")
GEOM = ("Mesh", "Cube", "Cylinder", "Cone", "Sphere", "Capsule", "Plane")


def classify(n):
    if n.startswith(("SM_FloorDecal", "SM_floor")): return "floor"
    if n.startswith(("SM_CeilingA", "SM_LampCeiling")): return "ceiling"
    if n.startswith(("RectLight", "DistantLight", "SphereLight", "DiskLight")): return "light"
    if n.startswith(("SM_WallA", "SM_WallB", "SM_WallWire", "SM_PillarA", "SM_PillarPartA",
                     "SM_BeamA", "SM_BracketBeam", "SM_BracketSlot")): return "wall"
    if n.startswith(("SM_RackShelf", "SM_RackFrame", "SM_RackPile", "SM_Rackshield", "Section")):
        return "shelf"
    if n.startswith(("Box_", "SM_CardBox")): return "box"
    if n.startswith("SM_PaletteA"): return "pallet"
    if n.startswith(("Crate", "Barrel", "Bottle")): return "loose_goods"
    if n.startswith(("SM_SignCVer", "S_AisleSign", "S_Barcode", "SM_SignA", "SM_SignB")):
        return "sign"
    return "other"


def sha256_file(path, chunk=1 << 20):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        while True:
            b = f.read(chunk)
            if not b:
                break
            h.update(b)
    return h.hexdigest()


def main():
    wh = config.load_warehouse()
    ob = config.load_obstacles()
    derived = os.path.join(ROOT, wh["base"]["entry_point"])
    if not os.path.exists(derived):
        print("ERROR derived base missing")
        return 2

    res = float(wh["map"]["resolution"])
    zf = float(wh["map"]["z_floor_m"])
    ph = float(wh["map"]["passage_height_m"])
    require_coll = bool(wh["map"].get("require_collision", True))
    fx0, fx1 = wh["base"]["floor_extent"]["x"]
    fy0, fy1 = wh["base"]["floor_extent"]["y"]
    nx = int(round((fx1 - fx0) / res))
    ny = int(round((fy1 - fy0) / res))

    st = Usd.Stage.Open(derived)
    holder = st.GetPrimAtPath("/World/Warehouse")
    if not holder or not holder.IsValid():
        print("ERROR /World/Warehouse missing")
        return 2
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                              [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])

    floor_mask = np.ones((ny, nx), dtype=bool)
    occ_structure = np.zeros((ny, nx), dtype=bool)
    occ_stock = np.zeros((ny, nx), dtype=bool)
    shelf_mask = np.zeros((ny, nx), dtype=bool)
    wall_mask = np.zeros((ny, nx), dtype=bool)

    def cell(x, y):
        ix = int((x - fx0) / res)
        iy = int((y - fy0) / res)
        return max(0, min(nx - 1, ix)), max(0, min(ny - 1, iy))

    def stamp(mask, b):
        ix0, iy0 = cell(b[0], b[1])
        ix1, iy1 = cell(b[3], b[4])
        mask[iy0:iy1 + 1, ix0:ix1 + 1] = True

    def wbbox(p):
        r = cache.ComputeWorldBound(p).ComputeAlignedRange()
        if r.IsEmpty():
            return None
        mn, mx = r.GetMin(), r.GetMax()
        return (float(mn[0]), float(mn[1]), float(mn[2]),
                float(mx[0]), float(mx[1]), float(mx[2]))

    def mesh_world_tris(p, xfc):
        """World-space triangle list of a Mesh (None if not a tri-able mesh)."""
        mesh = UsdGeom.Mesh(p)
        pts = mesh.GetPointsAttr().Get()
        counts = mesh.GetFaceVertexCountsAttr().Get()
        idx = mesh.GetFaceVertexIndicesAttr().Get()
        if not pts or not counts or not idx:
            return None
        M = np.array(xfc.GetLocalToWorldTransform(p), dtype=float).reshape(4, 4)
        P = np.array(pts, dtype=float)
        W = P @ M[:3, :3] + M[3, :3]
        tris = []
        k = 0
        for c in counts:
            f = idx[k:k + c]
            k += c
            for i in range(1, c - 1):
                tris.append((f[0], f[i], f[i + 1]))
        if not tris:
            return None
        return W, np.array(tris, dtype=np.int64)

    def stamp_tri(mask, W, tri):
        """Rasterise the XY projection of one triangle (band-clipped already)."""
        a, b, c = W[tri[0]], W[tri[1]], W[tri[2]]
        xs = (a[0], b[0], c[0]); ys = (a[1], b[1], c[1])
        x0, x1 = min(xs), max(xs); y0, y1 = min(ys), max(ys)
        ix0, iy0 = cell(x0, y0)
        ix1, iy1 = cell(x1, y1)
        if ix1 < ix0 or iy1 < iy0:
            return
        if ix0 == ix1 and iy0 == iy1:
            mask[iy0, ix0] = True
            return
        gx = fx0 + (np.arange(ix0, ix1 + 1) + 0.5) * res
        gy = fy0 + (np.arange(iy0, iy1 + 1) + 0.5) * res
        GX, GY = np.meshgrid(gx, gy)
        d = (b[1] - c[1]) * (a[0] - c[0]) + (c[0] - b[0]) * (a[1] - c[1])
        if abs(d) < 1e-12:
            mask[iy0:iy1 + 1, ix0:ix1 + 1] = True
            return
        l1 = ((b[1] - c[1]) * (GX - c[0]) + (c[0] - b[0]) * (GY - c[1])) / d
        l2 = ((c[1] - a[1]) * (GX - c[0]) + (a[0] - c[0]) * (GY - c[1])) / d
        l3 = 1.0 - l1 - l2
        inside = (l1 >= -1e-6) & (l2 >= -1e-6) & (l3 >= -1e-6)
        mask[iy0:iy1 + 1, ix0:ix1 + 1] |= inside

    xfc = UsdGeom.XformCache(Usd.TimeCode.Default())

    def emit(cls, p, b):
        """Stamp one leaf's true in-band geometry."""
        tn = p.GetTypeName()
        if tn == "Mesh":
            mt = mesh_world_tris(p, xfc)
            if mt is not None:
                W, tris = mt
                zt = W[tris][:, :, 2]
                zmin = zt.min(axis=1); zmax = zt.max(axis=1)
                keep = (zmax >= zf) & (zmin <= ph)
                hit = False
                for t in tris[keep]:
                    hit = True
                    if cls == "shelf":
                        stamp_tri(shelf_mask, W, t); stamp_tri(occ_structure, W, t)
                    elif cls == "wall":
                        stamp_tri(wall_mask, W, t); stamp_tri(occ_structure, W, t)
                    elif cls in ("box", "pallet", "loose_goods"):
                        stamp_tri(occ_stock, W, t)
                    else:
                        stamp_tri(occ_structure, W, t)
                if hit:
                    return True
                # no triangle inside the band -> this leaf must NOT block the floor
                return False
        # non-mesh primitives: bbox is the geometry itself
        if cls == "shelf":
            stamp(occ_structure, b); stamp(shelf_mask, b)
        elif cls == "wall":
            stamp(occ_structure, b); stamp(wall_mask, b)
        elif cls in ("box", "pallet", "loose_goods"):
            stamp(occ_stock, b)
        else:
            stamp(occ_structure, b)
        return True

    counts = {}
    n_active = 0
    leaves_bbox_in_band = 0
    leaves_with_ground = 0
    leaves_skipped_no_collision = 0
    prims_in_band = set()
    log_err = []
    fp = {"x": [1e9, -1e9], "y": [1e9, -1e9]}

    for ch in holder.GetChildren():
        if not ch.IsActive():
            continue
        n_active += 1
        cls = classify(ch.GetName())
        counts[cls] = counts.get(cls, 0) + 1

        pb = wbbox(ch)
        if pb is None:
            continue
        if cls == "floor":
            fp["x"][0] = min(fp["x"][0], pb[0]); fp["x"][1] = max(fp["x"][1], pb[3])
            fp["y"][0] = min(fp["y"][0], pb[1]); fp["y"][1] = max(fp["y"][1], pb[4])
        if cls in SKIP:
            continue

        # ---- per-leaf rasterisation: band-clipped triangles, no bbox shortcut ----
        for p in Usd.PrimRange(ch):
            if p.GetTypeName() not in GEOM:
                continue
            b = wbbox(p)
            if b is None:
                continue
            if not (b[5] >= zf and b[2] <= ph):
                continue          # wholly above/below the passage band
            leaves_bbox_in_band += 1
            if require_coll and not p.HasAPI(UsdPhysics.CollisionAPI):
                leaves_skipped_no_collision += 1
                continue
            try:
                hit = emit(cls, p, b)
            except Exception as exc:      # never let one mesh kill the build
                log_err.append((p.GetName(), str(exc)[:60]))
                continue
            if hit:
                leaves_with_ground += 1
                prims_in_band.add(ch.GetName())

    fixed_occ = occ_structure | occ_stock

    # ---- interior envelope ----
    # The floor bbox is LARGER than the walled interior. Take every connected
    # component of (floor & ~wall) that does NOT touch the grid border -> that is
    # the building interior, keeping ALL bays even if tall partitions split it.
    # Fallback to the largest component if a gap merges inside with outside.
    free_envelope = floor_mask & (~wall_mask)
    lab, n_lab = ndimage.label(free_envelope)
    if n_lab == 0:
        print("ERROR no free envelope")
        return 3
    border_labels = set(np.unique(np.concatenate(
        [lab[0, :], lab[-1, :], lab[:, 0], lab[:, -1]])))
    border_labels.discard(0)
    inner_labels = [l for l in range(1, n_lab + 1) if l not in border_labels]
    if inner_labels:
        interior = np.isin(lab, inner_labels)
    else:
        interior = np.zeros_like(free_envelope)
    if interior.sum() < 0.4 * floor_mask.sum():
        sizes = ndimage.sum(free_envelope, lab, range(1, n_lab + 1))
        interior = (lab == int(np.argmax(sizes)) + 1)
        print("  note: interior merged with outside -> fallback to largest component")
    free = interior & (~fixed_occ)

    # keep-clear corridors
    keep = np.zeros((ny, nx), dtype=bool)
    for kc in wh["placement"]["keep_clear"]:
        ix0, iy0 = cell(kc["x"][0], kc["y"][0])
        ix1, iy1 = cell(kc["x"][1], kc["y"][1])
        keep[iy0:iy1 + 1, ix0:ix1 + 1] = True
    free_wo_keep = free & (~keep)

    # erode placeable by the placement margin to fixed structure
    cl = wh["clearance"]
    margin = float(ob["clearance"]["wall"])
    robot_margin = float(cl["robot_radius_m"]) + float(cl["safety_margin_m"])
    k = max(1, int(round(margin / res)))
    eroded = free_wo_keep.copy()
    for dy in range(-k, k + 1):
        for dx in range(-k, k + 1):
            if dx * dx + dy * dy > k * k:
                continue
            shifted = np.zeros_like(eroded)
            ys0, ys1 = max(0, dy), ny + min(0, dy)
            xs0, xs1 = max(0, dx), nx + min(0, dx)
            yd0, yd1 = max(0, -dy), ny + min(0, -dy)
            xd0, xd1 = max(0, -dx), nx + min(0, -dx)
            shifted[ys0:ys1, xs0:xs1] = free_wo_keep[yd0:yd1, xd0:xd1]
            eroded &= shifted
    placeable = eroded

    # ---- sampling regions ----
    rg = wh["placement"]["regions"]
    band0, band1 = rg["aisle_band_y"]
    edge_m = float(rg["edge_margin_m"])
    k_edge = max(1, int(round(edge_m / res)))

    def _dilate_box(m, kk):
        b = m.copy()
        for d in range(1, kk + 1):
            b[:, d:] |= m[:, :-d]
            b[:, :-d] |= m[:, d:]
        out = b.copy()
        for d in range(1, kk + 1):
            out[d:, :] |= b[:-d, :]
            out[:-d, :] |= b[d:, :]
        return out

    band_mask = np.zeros((ny, nx), dtype=bool)
    yb0 = int(round((band0 - fy0) / res))
    yb1 = int(round((band1 - fy0) / res))
    band_mask[max(0, yb0):min(ny, yb1 + 1), :] = True
    near_shelf = _dilate_box(shelf_mask, k_edge) & (~shelf_mask)

    region_aisle = placeable & band_mask
    region_edge = placeable & (~region_aisle) & near_shelf
    region_open = placeable & (~region_aisle) & (~region_edge)

    area = res * res
    print("=== map build (per-leaf, from DERIVED usd) ===")
    print("  derived          :", os.path.relpath(derived, ROOT))
    print("  grid             : %d x %d  res=%.3f  origin=(%.1f, %.1f)" % (nx, ny, res, fx0, fy0))
    print("  z band           : [%.2f, %.2f] m   require_collision=%s" % (zf, ph, require_coll))
    print("  active children  :", n_active)
    print("  leaves bbox-in-band:%d  with-ground-geometry:%d  skipped(no collision):%d"
          % (leaves_bbox_in_band, leaves_with_ground, leaves_skipped_no_collision))
    if log_err:
        print("  mesh errors:", len(log_err), log_err[:3])
    print("  prims in band    :", len(prims_in_band))
    print("  class counts     :", dict(sorted(counts.items(), key=lambda x: -x[1])))
    print("  floor prims bbox : x[%.1f,%.1f] y[%.1f,%.1f]" % (fp["x"][0], fp["x"][1], fp["y"][0], fp["y"][1]))
    print("  wall cells       : %d | envelope components: %d" % (wall_mask.sum(), n_lab))
    print("  interior         : %d cells (%.0f m^2) = %.1f%% of floor"
          % (interior.sum(), interior.sum() * area, 100.0 * interior.sum() / max(1, floor_mask.sum())))
    print("  structure occ    : %d (%.1f%%)" % (occ_structure.sum(), 100.0 * occ_structure.sum() / floor_mask.sum()))
    print("  stock occ        : %d (%.1f%%)" % (occ_stock.sum(), 100.0 * occ_stock.sum() / floor_mask.sum()))
    print("  free             : %d (%.1f%%)" % (free.sum(), 100.0 * free.sum() / floor_mask.sum()))
    print("  keep-clear       : %d" % keep.sum())
    print("  placeable        : %d (%.0f m^2)" % (placeable.sum(), placeable.sum() * area))
    tot = max(1, int(placeable.sum()))
    for nm, m in (("aisle", region_aisle), ("shelf_edge", region_edge), ("open", region_open)):
        print("      region %-10s %7d cells %5.1f%% (%.0f m^2)" % (nm, int(m.sum()), 100.0 * m.sum() / tot, m.sum() * area))
    print("  stock protruding : %.0f m^2 of stock outside shelf footprint"
          % (float(np.count_nonzero(occ_stock & ~shelf_mask)) * area))

    out_npz = os.path.join(ROOT, wh["map"]["cache"])
    os.makedirs(os.path.dirname(out_npz), exist_ok=True)
    np.savez_compressed(out_npz,
                        fixed_occ=fixed_occ, structure_occ=occ_structure,
                        stock_occ=occ_stock, shelf_mask=shelf_mask,
                        floor_mask=floor_mask, free=free, keep_clear=keep,
                        placeable=placeable, interior=interior, wall_mask=wall_mask,
                        region_aisle=region_aisle, region_edge=region_edge,
                        region_open=region_open,
                        res=np.array([res]), origin=np.array([fx0, fy0]),
                        dims=np.array([nx, ny]), z_band=np.array([zf, ph]))

    meta = {
        "schema_version": "1.1",
        "derived_usd": os.path.relpath(derived, ROOT),
        "derived_usd_sha256": sha256_file(derived),
        "resolution_m": res,
        "origin": [fx0, fy0],
        "grid": [nx, ny],
        "z_band_m": [zf, ph],
        "passage_height_m": ph,
        "require_collision": require_coll,
        "rasterisation": "per-leaf geometry (Mesh/Cube/...), world bbox, no parent bbox",
        "robot_radius_m": cl["robot_radius_m"],
        "safety_margin_m": cl["safety_margin_m"],
        "robot_margin_m": robot_margin,
        "placement_margin_m": margin,
        "obstacle_clearance_m": float(ob["clearance"]["obstacle"]),
        "leaves_bbox_in_band": leaves_bbox_in_band,
        "leaves_with_ground_geometry": leaves_with_ground,
        "leaves_skipped_no_collision": leaves_skipped_no_collision,
        "prims_in_band": len(prims_in_band),
        "rasterisation_detail": "per-leaf triangle/band intersection (no bbox shortcut)",
        "cells": {"floor": int(floor_mask.sum()), "structure": int(occ_structure.sum()),
                  "stock": int(occ_stock.sum()), "free": int(free.sum()),
                  "interior": int(interior.sum()), "wall": int(wall_mask.sum()),
                  "envelope_components": int(n_lab),
                  "keep_clear": int(keep.sum()), "placeable": int(placeable.sum()),
                  "region_aisle": int(region_aisle.sum()),
                  "region_edge": int(region_edge.sum()),
                  "region_open": int(region_open.sum())},
        "regions": {"aisle_band_y": [band0, band1], "edge_margin_m": edge_m,
                    "weights": dict(rg["weights"])},
        "placeable_area_m2": float(placeable.sum() * area),
        "stock_protruding_area_m2": float(np.count_nonzero(occ_stock & ~shelf_mask) * area),
        "map_sha256": sha256_file(out_npz),
    }
    mp = os.path.join(ROOT, wh["map"]["meta"])
    with open(mp, "w") as f:
        json.dump(meta, f, indent=2, sort_keys=True)
    print("\n  wrote", out_npz)
    print("  wrote", mp, "(map_sha256=%s...)" % meta["map_sha256"][:16])
    print("MAP_BUILD_DONE")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
