#!/usr/bin/env python3
"""Plot captured map/costmap and computed paths; does not connect to ROS."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

parser = argparse.ArgumentParser()
parser.add_argument("--check-dir", type=Path, required=True)
parser.add_argument("--output", type=Path, required=True)
args = parser.parse_args()
report = json.loads((args.check_dir / "plan_check.json").read_text())
grid = report["grid"]
occupancy = np.load(args.check_dir / "map.npy")
cost = np.load(args.check_dir / "costmap.npy")
image = np.full((*occupancy.shape, 3), .96)
image[occupancy < 0] = [.65, .67, .70]
image[occupancy == 100] = [.08, .10, .12]
origin = grid["origin"]
extent = [origin[0], origin[0]+grid["width"]*grid["resolution"],
          origin[1], origin[1]+grid["height"]*grid["resolution"]]
path = np.asarray(report["cases"][0]["path_xy"])
fig, axes = plt.subplots(1, 2, figsize=(11, 7), layout="constrained")
for axis in axes:
    axis.imshow(image, origin="lower", extent=extent)
    axis.imshow(np.ma.masked_where((cost <= 0) | (cost == 100), cost), origin="lower", extent=extent,
                cmap="cool", vmin=0, vmax=100, alpha=.45)
    axis.plot(path[:, 0], path[:, 1], color="#e84919", linewidth=2.5, label="Computed path (not executed)")
    axis.scatter(*path[0], color="#16a34a", s=45, zorder=5, label="Current start")
    axis.scatter(*path[-1], color="#dc2626", marker="*", s=90, zorder=5, label="Goal")
    axis.set_aspect("equal")
    axis.set_xlabel("Map x [m]")
    axis.set_ylabel("Map y [m]")
    axis.grid(alpha=.15)
axes[0].set_title("USD development map + inflated global costmap\nNOT a SLAM result")
axes[1].set_xlim(-2, 4)
axes[1].set_ylim(-4, 2)
axes[1].set_title("Navfn corridor plan: ~2.00 m\nOccupied and outside-map goals rejected")
axes[1].legend(fontsize=8)
args.output.parent.mkdir(parents=True, exist_ok=True)
fig.savefig(args.output.with_suffix(".png"), dpi=160)
fig.savefig(args.output.with_suffix(".pdf"))
