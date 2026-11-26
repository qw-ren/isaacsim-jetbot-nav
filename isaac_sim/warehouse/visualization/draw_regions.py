#!/usr/bin/env python3
"""Visualise the three sampling regions built from the REAL occupancy map.

    python3 visualization/draw_regions.py --out outputs/regions.png
"""
from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np
from PIL import Image, ImageDraw

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from generator import config  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    wh = config.load_warehouse()
    d = np.load(os.path.join(ROOT, wh["map"]["cache"]))
    floor = d["floor_mask"]; structure = d["structure_occ"]; stock = d["stock_occ"]
    aisle = d["region_aisle"]; edge = d["region_edge"]; openr = d["region_open"]
    ny, nx = floor.shape

    arr = np.zeros((ny, nx, 3), dtype=np.uint8)
    arr[:] = (248, 248, 248)
    arr[floor] = (236, 240, 246)
    arr[openr] = (205, 232, 205)     # open
    arr[edge] = (250, 226, 180)      # shelf edge / periphery
    arr[aisle] = (170, 205, 245)     # aisles
    arr[stock] = (226, 210, 190)
    arr[structure] = (95, 95, 95)

    img = Image.fromarray(np.flipud(arr))
    dr = ImageDraw.Draw(img)

    w = wh["placement"]["regions"]["weights"]
    meta_p = os.path.join(ROOT, wh["map"]["meta"])
    m = json.load(open(meta_p)) if os.path.exists(meta_p) else {}
    cells = m.get("cells", {})

    dr.rectangle([0, 0, nx, 30], fill=(255, 255, 255))
    dr.text((4, 2), "sampling regions (from derived USD occupancy, NOT planning rects)", fill=(0, 0, 0))
    dr.text((4, 16), "aisle=%.0f%% (%d cells)  shelf_edge=%.0f%% (%d)  open=%.0f%% (%d)"
            % (w["aisle"] * 100, cells.get("region_aisle", 0),
               w["shelf_edge"] * 100, cells.get("region_edge", 0),
               w["open"] * 100, cells.get("region_open", 0)), fill=(0, 0, 0))

    lx, ly = 6, ny - 62
    for i, (col, lab) in enumerate([((170, 205, 245), "aisle (between racks)"),
                                    ((250, 226, 180), "shelf edge / periphery"),
                                    ((205, 232, 205), "open floor"),
                                    ((95, 95, 95), "fixed structure")]):
        dr.rectangle([lx, ly + i * 14, lx + 10, ly + i * 14 + 10], fill=col, outline=(0, 0, 0))
        dr.text((lx + 14, ly + i * 14), lab, fill=(0, 0, 0))

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    img.save(args.out)
    print("saved", args.out, img.size)


if __name__ == "__main__":
    main()
