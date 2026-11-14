#!/usr/bin/env python3
"""Compose verification images: official vs derived top views, 2D map overlay,
axis ticks, and a pixel-difference report.

Inputs (produced by /tmp/render_verify.py):
    official_top.png, derived_top.png, official_oblique.png, official_roof_on.png
Outputs (into --outdir):
    01_official_top.png        3D top view (roof hidden) with ticks
    02_derived_top.png         3D top view of the derived scene with ticks
    03_compare_side_by_side.png
    04_overlay_map_on_3d.png   2D structure map (red, 45% alpha) on the 3D top view
    05_map_only_ticks.png      2D occupancy map alone with ticks
    06_pixel_diff.png          |official - derived| magnified
    diff_report.json
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
RED = (235, 40, 40)


def ticks(img, wh, step=5):
    """Draw world-coordinate ticks/labels; image covers floor_extent exactly."""
    fx0, fx1 = wh["base"]["floor_extent"]["x"]
    fy0, fy1 = wh["base"]["floor_extent"]["y"]
    W, H = img.size
    d = ImageDraw.Draw(img)
    for x in range(int(fx0) - int(fx0) % step, int(fx1) + 1, step):
        px = int((x - fx0) / (fx1 - fx0) * W)
        d.line([px, 0, px, 8], fill=(0, 0, 0), width=2)
        d.line([px, H - 8, px, H], fill=(0, 0, 0), width=2)
        d.text((px + 2, 10), str(x), fill=(0, 0, 0))
        d.text((px + 2, H - 22), str(x), fill=(0, 0, 0))
    for y in range(int(fy0) - int(fy0) % step, int(fy1) + 1, step):
        py = int((fy1 - y) / (fy1 - fy0) * H)
        d.line([0, py, 8, py], fill=(0, 0, 0), width=2)
        d.line([W - 8, py, W, py], fill=(0, 0, 0), width=2)
        d.text((10, py - 5), str(y), fill=(0, 0, 0))
        d.text((W - 30, py - 5), str(y), fill=(0, 0, 0))
    return img


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default="/tmp/stage0_render/verify")
    ap.add_argument("--outdir", default=None)
    args = ap.parse_args()
    outdir = args.outdir or os.path.join(ROOT, "outputs", "verify")
    os.makedirs(outdir, exist_ok=True)

    wh = config.load_warehouse()
    d = np.load(os.path.join(ROOT, wh["map"]["cache"]))
    structure = np.flipud(d["structure_occ"])          # row0 = ymax
    stock = np.flipud(d["stock_occ"])
    ny, nx = structure.shape
    fx0, fx1 = wh["base"]["floor_extent"]["x"]
    fy0, fy1 = wh["base"]["floor_extent"]["y"]

    official = Image.open(os.path.join(args.dir, "official_top.png")).convert("RGB")
    derived = Image.open(os.path.join(args.dir, "derived_top.png")).convert("RGB")

    ticks(official.copy(), wh).save(os.path.join(outdir, "01_official_top.png"))
    ticks(derived.copy(), wh).save(os.path.join(outdir, "02_derived_top.png"))

    # side by side
    W, H = official.size
    sbs = Image.new("RGB", (W * 2 + 30, H + 28), (255, 255, 255))
    sbs.paste(ticks(official.copy(), wh), (0, 28))
    sbs.paste(ticks(derived.copy(), wh), (W + 30, 28))
    dr = ImageDraw.Draw(sbs)
    dr.text((6, 8), "OFFICIAL full_warehouse.usd", fill=(0, 0, 0))
    dr.text((W + 36, 8), "DERIVED scenes/warehouse_base.usd", fill=(0, 0, 0))
    sbs.save(os.path.join(outdir, "03_compare_side_by_side.png"))

    # 2D map alone with ticks
    arr = np.full((ny, nx, 3), 250, dtype=np.uint8)
    arr[structure] = (255, 170, 60)
    arr[stock] = (120, 120, 120)
    m = Image.fromarray(arr).resize((W, H), Image.NEAREST)
    ticks(m.copy(), wh).save(os.path.join(outdir, "05_map_only_ticks.png"))

    # overlay map on the 3D top view
    mask_img = Image.fromarray((structure * 255).astype(np.uint8)).resize((W, H), Image.NEAREST)
    overlay = official.copy()
    red_layer = Image.new("RGB", (W, H), RED)
    overlay.paste(Image.blend(overlay, red_layer, 0.45), (0, 0), mask_img.point(lambda v: int(v * 0.45)))
    ticks(overlay.copy(), wh).save(os.path.join(outdir, "04_overlay_map_on_3d.png"))

    # pixel diff
    a = np.asarray(official, dtype=np.int16)
    b = np.asarray(derived, dtype=np.int16)
    diff = np.abs(a - b).sum(axis=2)
    thr = 30
    changed = diff > thr
    rep = {
        "image_size": [W, H],
        "pixels_changed": int(changed.sum()),
        "pixels_total": int(changed.size),
        "pct_changed": round(100.0 * changed.sum() / changed.size, 4),
        "max_abs_diff": int(diff.max()),
        "mean_abs_diff": round(float(diff.mean()), 3),
    }
    if changed.any():
        ys, xs = np.nonzero(changed)
        x0 = fx0 + xs.min() / W * (fx1 - fx0); x1 = fx0 + xs.max() / W * (fx1 - fx0)
        y0 = fy1 - ys.max() / H * (fy1 - fy0); y1 = fy1 - ys.min() / H * (fy1 - fy0)
        rep["changed_world_bbox"] = [round(x0, 2), round(y0, 2), round(x1, 2), round(y1, 2)]
    dv = np.clip(diff * 3, 0, 255).astype(np.uint8)
    Image.fromarray(dv).save(os.path.join(outdir, "06_pixel_diff.png"))
    with open(os.path.join(outdir, "diff_report.json"), "w") as f:
        json.dump(rep, f, indent=2)
    print(json.dumps(rep, indent=2))
    print("saved to", outdir)
    print("OVERLAY_DONE")


if __name__ == "__main__":
    main()
