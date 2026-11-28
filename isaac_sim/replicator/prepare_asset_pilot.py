#!/usr/bin/env python3
"""Build a real-asset pilot from the teammate registry; never modifies deployed weights."""
import argparse
import copy
import json
from pathlib import Path, PurePosixPath

import yaml

CLASSES = ["shelf", "wall", "box", "pallet", "barrier", "cart", "sign", "person"]
ASSETS = ["box_A", "box_B", "box_C", "box_D", "pallet_main", "pallet_prop",
          "barrier_cone", "cart_push"]


def build_config(template, registry, output_dir):
    cfg = copy.deepcopy(template)
    cfg.update(seed=20261124, num_frames=60, output_dir=str(output_dir))
    cfg["obstacles"] = [dict(class_name=name, count=0, primitive="cube") for name in CLASSES]
    for key in ASSETS:
        asset = registry["assets"][key]
        if asset.get("verified") is not True or not asset.get("dims_m"):
            raise ValueError("asset not verified: " + key)
        index = CLASSES.index(asset["class"])
        spec = dict(class_name=CLASSES[index], usd=str(PurePosixPath(registry["source_root"]) / asset["path"]),
                    count=1, scale=[[1., 1., 1.], [1., 1., 1.]], z_offset=0.)
        cfg["obstacles"].append(spec)
    cfg["pilot_only"] = True
    cfg["coverage_pending"] = ["shelf", "wall", "sign", "person"]
    cfg["notes"] = ["Keep eight-class IDs by zero-count declarations; pilot is not a complete training set.",
                    "Inspect USD units, ground alignment, visibility and bounding boxes in Isaac before scaling up.",
                    "Wet-floor sign excluded: registry labels it barrier; do not relabel the same appearance as sign.",
                    "Forklift/wheel hard negatives and real warehouse backgrounds still required."]
    return cfg


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--registry", type=Path, required=True)
    ap.add_argument("--output-dir", type=Path, required=True)
    ap.add_argument("--config-out", type=Path, required=True)
    args = ap.parse_args()
    template = yaml.safe_load((Path(__file__).parent / "configs/dataset_config.yaml").read_text(encoding="utf-8"))
    registry = yaml.safe_load(args.registry.read_text(encoding="utf-8"))
    cfg = build_config(template, registry, args.output_dir)
    args.config_out.parent.mkdir(parents=True, exist_ok=True)
    with args.config_out.open("x", encoding="utf-8") as stream:
        yaml.safe_dump(cfg, stream, allow_unicode=True, sort_keys=False)
    print(json.dumps(dict(config=str(args.config_out), pilot_only=True,
                         pending=cfg["coverage_pending"]), ensure_ascii=False))


if __name__ == "__main__":
    main()
