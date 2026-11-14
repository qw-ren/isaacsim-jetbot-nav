#!/usr/bin/env python3
"""Draw a generated seed: fixed structure + random obstacles, top-down (Stage 1).

    python3 visualization/draw_scene.py --spec outputs/scene_1001.json --out outputs/scene_1001.png
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

COL = {"box": (30, 90, 220), "pallet": (150, 100, 40),
       "barrier": (220, 40, 40), "cart": (140, 40, 160)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--spec", required=True)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    wh = config.load_warehouse()
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    d = np.load(os.path.join(root, wh["map"]["cache"]))
    res = float(d["res"][0])
    ox, oy = d["origin"]
    floor = d["floor_mask"]; structure = d["structure_occ"]
    stock = d["stock_occ"]; placeable = d["placeable"]; keep = d["keep_clear"]
    ny, nx = floor.shape

    arr = np.zeros((ny, nx, 3), dtype=np.uint8)
    arr[:] = (248, 248, 248)
    arr[floor] = (232, 238, 244)
    arr[placeable] = (222, 242, 222)
    arr[stock] = (255, 228, 195)
    arr[structure] = (105, 105, 105)

    img = Image.fromarray(np.flipud(arr))
    dr = ImageDraw.Draw(img)

    spec = json.load(open(args.spec))
    fx0, fy0 = wh["base"]["floor_extent"]["x"][0], wh["base"]["floor_extent"]["y"][0]

    def px(x):
        return (x - fx0) / res

    def py(y):
        return ny - (y - fy0) / res

    for o in spec["objects"]:
        w, h = o["size"][0], o["size"][1]
        if abs(np.cos(o["yaw_rad"])) < 1e-9:
            w, h = h, w
        x0 = px(o["position"][0] - w / 2); x1 = px(o["position"][0] + w / 2)
        y0 = py(o["position"][1] + h / 2); y1 = py(o["position"][1] - h / 2)
        c = COL.get(o["class"], (0, 0, 0))
        dr.rectangle([x0, y0, x1, y1], fill=c, outline=(20, 20, 20))

    # keep-clear corridors outline
    for kc in wh["placement"]["keep_clear"]:
        dr.rectangle([px(kc["x"][0]), py(kc["y"][1]), px(kc["x"][1]), py(kc["y"][0])],
                     outline=(30, 80, 200), width=2)

    title = ("seed=%d %s | objects=%d | hash=%s"
             % (spec["seed"], spec["density"], spec["object_count"],
                spec["layout_hash"][:12]))
    dr.rectangle([0, 0, nx, 14], fill=(255, 255, 255))
    dr.text((4, 2), title, fill=(0, 0, 0))

    # legend
    lx = 6; ly = ny - 68
    for i, (cls, c) in enumerate(COL.items()):
        dr.rectangle([lx, ly + i * 14, lx + 10, ly + i * 14 + 10], fill=c, outline=(0, 0, 0))
        dr.text((lx + 14, ly + i * 14), cls, fill=(0, 0, 0))
    dr.rectangle([lx, ly + 4 * 14, lx + 10, ly + 4 * 14 + 10], fill=(105, 105, 105))
    dr.text((lx + 14, ly + 4 * 14), "fixed structure", fill=(0, 0, 0))

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    img.save(args.out)
    print("saved", args.out, img.size)


if __name__ == "__main__":
    main()
