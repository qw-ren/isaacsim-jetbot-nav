#!/usr/bin/env python3
"""Warehouse aisle topology + robot-space connectivity (no navigation planner).

Everything is derived from the REAL occupancy map (scenes/base_map.npz):
  obstacle  = structure (walls/shelves/etc)  |  existing stock
  free      = interior & ~obstacle
  pass_grid = free eroded by (robot_radius + safety_margin)  -> where a robot fits

Provided:
  aisle_segments()          connected components of the aisle region (region_aisle)
  aisle_ends(seg)           the two open ends of an aisle strip (world y extremes)
  aisle_traversable(...)    is the aisle passable END-TO-END inside the strip
  global_connected(p, q)    are two points connected through the whole warehouse
  grid(extra_objects)       pass grid with additional (A/B) objects rasterised in
"""
from __future__ import annotations

import numpy as np
from scipy import ndimage

from . import placement_validator as pv

DISK_CACHE = {}


def disk(k):
    if k not in DISK_CACHE:
        y, x = np.ogrid[-k:k + 1, -k:k + 1]
        DISK_CACHE[k] = (x * x + y * y) <= k * k
    return DISK_CACHE[k]


class Topology:
    def __init__(self, md, wh, min_aisle_area_m2=8.0):
        self.md = md
        self.wh = wh
        self.res = md["res"]
        self.ox, self.oy = md["origin"]
        self.nx, self.ny = md["nx"], md["ny"]
        cl = wh["clearance"]
        self.robot_r = float(cl["robot_radius_m"]) + float(cl["safety_margin_m"])
        self.k = max(1, int(round(self.robot_r / self.res)))
        self.obstacle = md["structure_occ"] | md["stock_occ"]
        self.interior = md["interior"]
        self.aisle_mask = md["region_aisle"]
        self.min_cells = int(min_aisle_area_m2 / (self.res * self.res))
        self._segs = None

    # ---------------- grids ----------------
    def grid(self, extra_objects=None):
        """Robot-passable boolean grid, optionally with extra objects rasterised."""
        obst = self.obstacle.copy()
        for o in (extra_objects or []):
            x0, y0, x1, y1 = pv._aabb(o)
            ix0, iy0, ix1, iy1 = pv.rect_cells(x0, y0, x1, y1, (self.ox, self.oy),
                                               self.res, self.nx, self.ny)
            obst[iy0:iy1 + 1, ix0:ix1 + 1] = True
        free = self.interior & (~obst)
        return ndimage.binary_erosion(free, disk(self.k), border_value=0)

    # ---------------- aisles ----------------
    def aisle_segments(self):
        if self._segs is not None:
            return self._segs
        lab, n = ndimage.label(self.aisle_mask)
        segs = []
        for i in range(1, n + 1):
            m = (lab == i)
            cells = int(m.sum())
            if cells < self.min_cells:
                continue
            ys, xs = np.nonzero(m)
            y0i, y1i = int(ys.min()), int(ys.max())
            segs.append({
                "index": len(segs), "label": i, "mask": m, "cells": cells,
                "area_m2": cells * self.res * self.res,
                "bbox": [self.ox + xs.min() * self.res, self.oy + y0i * self.res,
                         self.ox + (xs.max() + 1) * self.res, self.oy + (y1i + 1) * self.res],
                "row_range": [y0i, y1i],
                "width_m": self._typical_width(m),
            })
        self._segs = segs
        return segs

    def _typical_width(self, mask):
        ys, xs = np.nonzero(mask)
        w = []
        for iy in np.unique(ys):
            sel = xs[ys == iy]
            w.append((sel.max() - sel.min() + 1) * self.res)
        w = np.array(w)
        return float(np.median(w)) if len(w) else 0.0

    def aisle_mouths(self, seg):
        """Cells at the two extreme y rows of the strip (the open ends)."""
        y0, y1 = seg["row_range"]
        m = seg["mask"]
        out = []
        for iy in (y0, y1):
            sel = np.nonzero(m[iy])[0]
            if len(sel):
                out.append((iy, int(sel.min()), int(sel.max())))
        return out

    def aisle_traversable(self, seg, pass_grid, pad=2):
        """END-TO-END passable inside the strip (not just a cross-section)."""
        sub = seg["mask"].copy()
        # allow a small lateral pad so the robot may use the strip plus its fringe
        sub = ndimage.binary_dilation(sub, disk(pad))
        g = pass_grid & sub
        lab, n = ndimage.label(g)
        if n == 0:
            return False, None
        y0, y1 = seg["row_range"]
        bottom = set(np.unique(lab[y0:y0 + 1])) - {0}
        top = set(np.unique(lab[y1:y1 + 1])) - {0}
        both = bottom & top
        return (len(both) > 0), (lab, list(both))

    def mouth_points(self, seg):
        pts = []
        for (iy, ix0, ix1) in self.aisle_mouths(seg):
            ix = (ix0 + ix1) // 2
            pts.append((self.ox + (ix + 0.5) * self.res, self.oy + (iy + 0.5) * self.res))
        return pts

    def global_connected(self, p, q, pass_grid):
        a = self._cell(p); b = self._cell(q)
        if not pass_grid[a[1], a[0]] or not pass_grid[b[1], b[0]]:
            return None      # endpoint itself is not passable
        lab, _ = ndimage.label(pass_grid)
        return int(lab[a[1], a[0]]) == int(lab[b[1], b[0]]) and lab[a[1], a[0]] != 0

    def _cell(self, p):
        ix = int((p[0] - self.ox) / self.res)
        iy = int((p[1] - self.oy) / self.res)
        return (max(0, min(self.nx - 1, ix)), max(0, min(self.ny - 1, iy)))

    # ---------------- free-space (for B placement) ----------------
    def free_mask(self):
        """Un-eroded free space (so blocking objects may touch the racks)."""
        if not hasattr(self, "_free"):
            self._free = self.interior & (~self.obstacle)
        return self._free

    def _int_free(self):
        """Integral image of free_mask (so a footprint can be tested in O(1))."""
        if not hasattr(self, "_ifree"):
            f = self.free_mask().astype(np.int32)
            ii = np.zeros((f.shape[0] + 1, f.shape[1] + 1), dtype=np.int64)
            ii[1:, 1:] = f.cumsum(0).cumsum(1)
            self._ifree = ii
        return self._ifree
