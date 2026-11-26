#!/usr/bin/env python3
"""2D distribution figure for one scene JSON.

grey = fixed structure, pale green = placeable, blue = A-class, red = B-class,
orange outline = aisle strip, dashed red band = blocked aisle station.
"""
import json
import os
import sys

import numpy as np
from PIL import Image, ImageDraw
from scipy import ndimage

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from generator import config  # noqa

SCALE = 40  # px per metre -> K px per map cell


def main(spec_path, out_path):
    cfgs = config.load_all()
    md = np.load(os.path.join(ROOT, cfgs["warehouse"]["map"]["cache"]))
    res = float(md["res"][0]); ox, oy = float(md["origin"][0]), float(md["origin"][1])
    ny, nx = md["region_open"].shape
    K = max(1, int(round(SCALE * res)))
    W = nx * K; H = ny * K
    img = Image.new("RGB", (W, H), (255, 255, 255))
    px = np.zeros((H, W, 3), dtype=np.uint8)
    interior = md["interior"]; struct = md["structure_occ"]; stock = md["stock_occ"]
    placeable = md["placeable"]
    px[:] = (250, 250, 250)
    for mask, col in ((interior, (226, 240, 226)), (placeable, (205, 230, 205)),
                      (stock, (150, 150, 150)), (struct, (110, 110, 110))):
        big = np.kron(np.flipud(mask), np.ones((K, K), dtype=bool))
        px[big] = col
    img = Image.fromarray(px)
    d = ImageDraw.Draw(img)

    def to_px(x, y):
        return ((x - ox) / res * K, (oy + ny * res - y) / res * K)

    spec = json.load(open(spec_path))
    blocked = set(spec.get("blocked_aisles", []))
    # aisle outlines
    lab, n = ndimage.label(md["region_aisle"])
    for i in range(1, n + 1):
        m = (lab == i)
        if m.sum() < 40:
            continue
        ys, xs = np.nonzero(m)
        x0 = ox + xs.min() * res; x1 = ox + (xs.max() + 1) * res
        y0 = oy + ys.min() * res; y1 = oy + (ys.max() + 1) * res
        a = to_px(x0, y1); b = to_px(x1, y0)
        col = (220, 60, 60) if (i - 1) in blocked else (230, 150, 40)
        d.rectangle([a[0], a[1], b[0], b[1]], outline=col, width=2)
        d.text((a[0] + 3, a[1] + 2), "aisle %d" % (i - 1), fill=col)

    for o in spec["objects"]:
        x, y = o["position"][0], o["position"][1]
        w, h = o["size"][0], o["size"][1]
        c = np.cos(o.get("yaw_rad", 0.0)); s = np.sin(o.get("yaw_rad", 0.0))
        pts = []
        for dx, dy in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
            lx, ly = dx * w / 2, dy * h / 2
            pts.append(to_px(x + lx * c - ly * s, y + lx * s + ly * c))
        colour = (200, 30, 30) if o.get("kind") == "B" else (30, 80, 200)
        d.polygon(pts, fill=colour, outline=(20, 20, 20))

    title = "%s | seed=%d %s | A=%d B=%d | blocked=%s" % (
        spec.get("scene_name", ""), spec["seed"], spec["density"],
        sum(1 for o in spec["objects"] if o.get("kind") == "A"),
        sum(1 for o in spec["objects"] if o.get("kind") == "B"),
        spec.get("blocked_aisles", []))
    d.rectangle([0, 0, W, 22], fill=(255, 255, 255))
    d.text((4, 5), title, fill=(0, 0, 0))
    d.text((4, H - 16), "blue = A-class obstacle, red = B-class blockage, grey = fixed structure/stock",
           fill=(60, 60, 60))
    img.save(out_path)
    print("wrote", out_path)


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
