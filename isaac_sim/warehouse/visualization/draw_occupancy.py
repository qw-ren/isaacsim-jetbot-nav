#!/usr/bin/env python3
"""Draw the 2D ground occupancy map (what the generator treats as blocked).

    python3 visualization/draw_occupancy.py --out outputs/2d_fixed_structure.png

Colours
    dark slate  = fixed structure below passage height (walls, rack posts/boards)
    medium gray = pre-existing stock below passage height
    light green = placeable (free, eroded, keep-clear removed)
    blue outline= reserved corridors
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
UP = 2  # pixels per cell (0.05 m -> 0.025 m/px)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--zoom", default=None, help="y0,y1 world window (lower-half check)")
    args = ap.parse_args()

    wh = config.load_warehouse()
    d = np.load(os.path.join(ROOT, wh["map"]["cache"]))
    structure = d["structure_occ"]; stock = d["stock_occ"]
    placeable = d["placeable"]; keep = d["keep_clear"]
    ny, nx = structure.shape
    res = float(d["res"][0])
    fx0, fx1 = wh["base"]["floor_extent"]["x"]
    fy0, fy1 = wh["base"]["floor_extent"]["y"]

    arr = np.full((ny, nx, 3), 250, dtype=np.uint8)
    arr[placeable] = (223, 240, 223)
    # corridor tint must NOT hide real structure
    arr[keep & (~structure) & (~stock)] = (233, 240, 250)
    arr[stock] = (128, 128, 128)
    arr[structure] = (52, 58, 66)

    img = Image.fromarray(np.flipud(arr)).resize((nx * UP, ny * UP), Image.NEAREST)
    dr = ImageDraw.Draw(img)
    W, H = img.size

    # world grid every 5 m
    for x in range(int(fx0) - int(fx0) % 5, int(fx1) + 1, 5):
        px = int((x - fx0) / (fx1 - fx0) * W)
        dr.line([px, 0, px, H], fill=(205, 205, 205), width=1)
        dr.text((px + 2, 3), str(x), fill=(70, 70, 70))
        dr.text((px + 2, H - 14), str(x), fill=(70, 70, 70))
    for y in range(int(fy0) - int(fy0) % 5, int(fy1) + 1, 5):
        py = int((fy1 - y) / (fy1 - fy0) * H)
        dr.line([0, py, W, py], fill=(205, 205, 205), width=1)
        dr.text((2, py + 1), str(y), fill=(70, 70, 70))
        dr.text((W - 26, py + 1), str(y), fill=(70, 70, 70))

    # keep-clear outline
    for kc in wh["placement"]["keep_clear"]:
        dr.rectangle([int((kc["x"][0] - fx0) / (fx1 - fx0) * W), int((fy1 - kc["y"][1]) / (fy1 - fy0) * H),
                      int((kc["x"][1] - fx0) / (fx1 - fx0) * W), int((fy1 - kc["y"][0]) / (fy1 - fy0) * H)],
                     outline=(40, 90, 200), width=2)

    mp = os.path.join(ROOT, wh["map"]["meta"])
    m = json.load(open(mp)) if os.path.exists(mp) else {}
    cells = m.get("cells", {})
    area = res * res
    dr.rectangle([0, 0, W, 34], fill=(255, 255, 255))
    dr.text((4, 3), "2D GROUND OCCUPANCY  (blocked iff geometry intersects z=[%.2f, %.2f] m)"
            % tuple(m.get("z_band_m", [0.02, 1.2])), fill=(0, 0, 0))
    dr.text((4, 18), "structure %d cells (%.1f m2) | stock %d (%.1f m2) | placeable %d (%.0f m2) | res %.2f m"
            % (cells.get("structure", 0), cells.get("structure", 0) * area,
               cells.get("stock", 0), cells.get("stock", 0) * area,
               cells.get("placeable", 0), cells.get("placeable", 0) * area, res), fill=(0, 0, 0))

    lx, ly = 6, H - 92
    for i, (col, lab) in enumerate([((52, 58, 66), "fixed structure (blocked)"),
                                    ((128, 128, 128), "existing stock (blocked)"),
                                    ((223, 240, 223), "placeable (free)"),
                                    ((233, 240, 250), "+ blue outline = keep-clear")]):
        dr.rectangle([lx, ly + i * 20, lx + 14, ly + i * 20 + 14], fill=col, outline=(0, 0, 0))
        dr.text((lx + 20, ly + i * 20 + 1), lab, fill=(0, 0, 0))

    if args.zoom:
        y0, y1 = [float(v) for v in args.zoom.split(",")]
        box = (0, int((fy1 - y1) / (fy1 - fy0) * H), W, int((fy1 - y0) / (fy1 - fy0) * H))
        img = img.crop(box)

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    img.save(args.out)
    print("saved", args.out, img.size)


if __name__ == "__main__":
    main()
