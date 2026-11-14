"""Geometric legality checks on a generated layout (Stage 1, implemented).

All checks are 2D and exact because yaw is restricted to 90-degree steps, so every
obstacle footprint stays axis-aligned. No pxr / Isaac Sim needed.
"""
from __future__ import annotations

import numpy as np


def footprint(obj):
    """(cx, cy, w, h) with w/h swapped for 90/270 deg yaw."""
    dx, dy = obj["size"][0], obj["size"][1]
    yaw = obj["yaw_rad"]
    if abs(np.cos(yaw)) < 1e-9:  # 90 or 270
        return obj["position"][0], obj["position"][1], dy, dx
    return obj["position"][0], obj["position"][1], dx, dy


def _aabb(obj, pad=0.0):
    cx, cy, w, h = footprint(obj)
    return (cx - w / 2 - pad, cy - h / 2 - pad, cx + w / 2 + pad, cy + h / 2 + pad)


def _intersect(a, b):
    return not (a[2] <= b[0] or b[2] <= a[0] or a[3] <= b[1] or b[3] <= a[1])


def rect_cells(x0, y0, x1, y1, origin, res, nx, ny):
    ix0 = int(np.floor((x0 - origin[0]) / res))
    iy0 = int(np.floor((y0 - origin[1]) / res))
    ix1 = int(np.ceil((x1 - origin[0]) / res)) - 1
    iy1 = int(np.ceil((y1 - origin[1]) / res)) - 1
    ix0 = max(0, min(nx - 1, ix0)); ix1 = max(0, min(nx - 1, ix1))
    iy0 = max(0, min(ny - 1, iy0)); iy1 = max(0, min(ny - 1, iy1))
    return ix0, iy0, ix1, iy1


def rect_sum(table, ix0, iy0, ix1, iy1):
    """Sum over inclusive cell rect using an integral image (ny+1, nx+1)."""
    return (table[iy1 + 1, ix1 + 1] - table[iy0, ix1 + 1]
            - table[iy1 + 1, ix0] + table[iy0, ix0])


def validate(spec, mapdata, cfg, protected_zones=None):
    """Independent re-check of a finished layout.

    mapdata: dict with res, origin, nx, ny, int_placeable, int_structure,
             int_keepclear, floor_extent
    Returns {"valid": bool, "checks": {...}, "reasons": [...]}
    """
    res = mapdata["res"]; origin = mapdata["origin"]
    nx = mapdata["nx"]; ny = mapdata["ny"]
    fx0, fx1 = mapdata["floor_extent"]["x"]
    fy0, fy1 = mapdata["floor_extent"]["y"]
    ob_clear = float(cfg["obstacle_clearance"])
    reasons = []
    checks = {"in_bounds": 0, "on_structure": 0, "outside_placeable": 0,
              "outside_interior": 0, "in_keep_clear": 0, "in_protected": 0,
              "too_close": 0}
    has_interior = "int_interior" in mapdata

    objs = spec["objects"]
    for i, o in enumerate(objs):
        cx, cy, w, h = footprint(o)
        x0, y0, x1, y1 = cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2
        if x0 < fx0 or x1 > fx1 or y0 < fy0 or y1 > fy1:
            checks["in_bounds"] += 1
            reasons.append("%s out of bounds" % o["id"])
        ix0, iy0, ix1, iy1 = rect_cells(x0, y0, x1, y1, origin, res, nx, ny)
        ncell = (ix1 - ix0 + 1) * (iy1 - iy0 + 1)
        if rect_sum(mapdata["int_structure"], ix0, iy0, ix1, iy1) > 0:
            checks["on_structure"] += 1
            reasons.append("%s overlaps fixed structure" % o["id"])
        if rect_sum(mapdata["int_keepclear"], ix0, iy0, ix1, iy1) > 0:
            checks["in_keep_clear"] += 1
            reasons.append("%s in keep-clear corridor" % o["id"])
        if rect_sum(mapdata["int_placeable"], ix0, iy0, ix1, iy1) < ncell:
            checks["outside_placeable"] += 1
            reasons.append("%s outside placeable area" % o["id"])
        if has_interior and rect_sum(mapdata["int_interior"], ix0, iy0, ix1, iy1) < ncell:
            checks["outside_interior"] += 1
            reasons.append("%s outside the walled interior" % o["id"])
        for (px0, py0, px1, py1) in (protected_zones or []):
            if not (x1 <= px0 or px1 <= x0 or y1 <= py0 or py1 <= y0):
                checks["in_protected"] += 1
                reasons.append("%s in protected zone" % o["id"])
                break

    # pairwise clearance
    boxes = [_aabb(o) for o in objs]
    for i in range(len(boxes)):
        for j in range(i + 1, len(boxes)):
            if _intersect(_aabb(objs[i], ob_clear / 2), _aabb(objs[j], ob_clear / 2)):
                checks["too_close"] += 1
                reasons.append("%s <-> %s closer than %.2f m"
                               % (objs[i]["id"], objs[j]["id"], ob_clear))

    valid = all(v == 0 for v in checks.values())
    return {"valid": valid, "checks": checks, "reasons": reasons[:20]}
