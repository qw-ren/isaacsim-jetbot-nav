"""Top-down schematic of the FIXED base + planned placement areas (Stage 0).

IMPORTANT: this schematic is for HUMAN ORIENTATION ONLY.
  * the shaded rack band is a visual hint (config: display_only), NOT a collision map
  * real fixed footprints come from per-prim rasterisation of
    scenes/warehouse_base.usd in Stage 1 (scenes/base_map.npz)

Usage:
    python3 visualization/layout_diagram.py --out outputs/base_layout.png
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

from PIL import Image, ImageDraw

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from generator.config import load_warehouse  # noqa: E402

PPM = 9


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--ppm", type=int, default=PPM)
    args = ap.parse_args()
    ppm = args.ppm

    w = load_warehouse()
    fb = w["base"]["floor_extent"]
    x0, x1 = fb["x"]
    y0, y1 = fb["y"]

    W = int((x1 - x0) * ppm) + 130
    H = int((y1 - y0) * ppm) + 120
    ox, oy = 72, 62

    def px(x):
        return int(ox + (x - x0) * ppm)

    def py(y):
        return int(oy + (y1 - y) * ppm)

    img = Image.new("RGB", (W, H), (252, 252, 252))
    d = ImageDraw.Draw(img)

    # floor
    d.rectangle([px(x0), py(y1), px(x1), py(y0)],
                fill=(232, 238, 244), outline=(80, 80, 80), width=2)

    # display-only rack band
    rb = w.get("display_only", {}).get("rack_band_y")
    if rb:
        d.rectangle([px(x0), py(rb[1]), px(x1), py(rb[0])], fill=(252, 240, 226))

    pl = w["placement"]
    # primary / secondary placeable areas
    for key, col in (("primary_area", (206, 236, 206)), ("secondary_area", (224, 244, 224))):
        a = pl[key]
        d.rectangle([px(a["x"][0]), py(a["y"][1]), px(a["x"][1]), py(a["y"][0])],
                    fill=col, outline=(60, 140, 60), width=2)

    # keep-clear corridors
    for kc in pl["keep_clear"]:
        d.rectangle([px(kc["x"][0]), py(kc["y"][1]), px(kc["x"][1]), py(kc["y"][0])],
                    outline=(40, 90, 200), width=3)

    # clean cluster
    cc = w["existing_clutter"]["clean_cluster"]
    d.rectangle([px(cc["x"][0]), py(cc["y"][1]), px(cc["x"][1]), py(cc["y"][0])],
                outline=(210, 40, 40), width=3)

    # demo start / goal
    sd = w.get("start_goal_demo", {})
    for key, col in (("start", (0, 150, 0)), ("goal", (150, 0, 150))):
        if key in sd:
            cx, cy = px(sd[key]["x"]), py(sd[key]["y"])
            d.line([cx - 7, cy, cx + 7, cy], fill=col, width=3)
            d.line([cx, cy - 7, cx, cy + 7], fill=col, width=3)

    # grid + ticks
    gx = int(x0) - (int(x0) % 5)
    while gx <= x1:
        d.line([px(gx), py(y1), px(gx), py(y0)], fill=(214, 214, 214), width=1)
        d.text((px(gx) - 9, py(y0) + 4), str(gx), fill=(90, 90, 90))
        gx += 5
    gy = int(y0) - (int(y0) % 5)
    while gy <= y1:
        d.line([px(x0), py(gy), px(x1), py(gy)], fill=(214, 214, 214), width=1)
        d.text((4, py(gy) - 5), str(gy), fill=(90, 90, 90))
        gy += 5

    d.text((ox, 12), "BASE + PLANNED PLACEMENT (schematic only, NOT a collision map)", fill=(20, 20, 20))
    d.text((ox, 32), "floor %.0f x %.0f m  |  clean=7 cartons  |  grid 5 m  |  official full_warehouse"
           % (x1 - x0, y1 - y0), fill=(60, 60, 60))

    # legend
    lx, ly = px(x1) - 250, py(y0) - 78
    items = [
        ((232, 238, 244), "floor"),
        ((252, 240, 226), "rack band (display only)"),
        ((206, 236, 206), "primary placeable"),
        ((40, 90, 200), "keep-clear corridor"),
        ((210, 40, 40), "clean 7 cartons"),
        ((0, 150, 0), "demo start / goal"),
    ]
    for i, (col, lab) in enumerate(items):
        d.rectangle([lx, ly + i * 13, lx + 10, ly + i * 13 + 9], fill=col, outline=(60, 60, 60))
        d.text((lx + 15, ly + i * 13 - 1), lab, fill=(30, 30, 30))

    # scale bar
    sx, sy = ox, H - 30
    d.line([sx, sy, sx + 10 * ppm, sy], fill=(0, 0, 0), width=3)
    d.text((sx, sy - 14), "10 m", fill=(0, 0, 0))

    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    img.save(args.out)
    print("saved", args.out, "%dx%d" % img.size)


if __name__ == "__main__":
    main()
