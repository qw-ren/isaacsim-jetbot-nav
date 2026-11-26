"""SceneGenerator 鈥?deterministic seed -> obstacle layout (Stage 1, implemented).

Public API (pure Python, no Isaac Sim required):
    gen  = SceneGenerator.from_config("config/")
    spec = gen.generate(seed=1001, density="medium", protected_zones=[])
    gen.save(spec, "outputs/scene_1001.json")
    spec = SceneGenerator.load("outputs/scene_1001.json")

Determinism
    same seed + same configs + same asset/map version  =>  identical counts,
    classes, asset variants, positions, orientations and instance ids.
    layout_hash depends ONLY on the resulting obstacle layout (seed excluded);
    seed / versions / config hash live in `metadata`.
"""
from __future__ import annotations

import hashlib
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from generator import config as cfgmod  # noqa: E402
from generator import obstacle_sampler as sampler  # noqa: E402
from generator import placement_validator as pv  # noqa: E402
from generator.seed_manager import layout_rng  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
GENERATOR_VERSION = "1.4.0"


def layout_hash(objects) -> str:
    """sha256 over the obstacle layout only (no seed, no versions, no paths)."""
    lines = []
    for o in sorted(objects, key=lambda x: x["id"]):
        lines.append("%s|%s|%s|%.3f|%.3f|%.3f|%.6f" % (
            o["id"], o["class"], o["asset_key"],
            o["position"][0], o["position"][1], o["position"][2], o["yaw_rad"]))
    return hashlib.sha256("\n".join(lines).encode("utf-8")).hexdigest()


class SceneGenerator:
    def __init__(self, cfgs, mapdata, base_meta):
        self.cfgs = cfgs
        self.md = mapdata
        self.base_meta = base_meta

    # -------------------- construction --------------------
    @classmethod
    def from_config(cls, config_dir=None):
        cfgs = cfgmod.load_all() if config_dir is None else {
            k: cfgmod.load(k) for k in cfgmod.CONFIG_FILES}
        wh = cfgs["warehouse"]
        npz = os.path.join(ROOT, wh["map"]["cache"])
        if not os.path.exists(npz):
            raise FileNotFoundError(
                "base map missing: %s\nrun: python3 generator/map_builder.py" % npz)
        md = sampler.build_mapdata(npz, wh, cfgs["obstacles"])

        # merge BOTH metadata files:
        #   scenes/base_meta.json      -> official USD sha256, asset checksums, config hash
        #   scenes/base_map_meta.json  -> map sha256, grid, passage band
        base_meta = {}
        for rel in ("scenes/base_meta.json", wh["map"]["meta"]):
            p = os.path.join(ROOT, rel)
            if os.path.exists(p):
                with open(p) as f:
                    base_meta.update(json.load(f))

        # 2D/3D consistency guard: the map must match the derived USD on disk
        derived = os.path.join(ROOT, wh["base"]["entry_point"])
        if base_meta.get("derived_usd_sha256") and os.path.exists(derived):
            h = hashlib.sha256()
            with open(derived, "rb") as f:
                h.update(f.read())
            if h.hexdigest() != base_meta["derived_usd_sha256"]:
                raise RuntimeError(
                    "derived USD changed since the map was built; rebuild the map "
                    "(2D/3D consistency guard)")
        return cls(cfgs, md, base_meta)

    def assets_manifest_sha256(self):
        """Single hash over every referenced asset's own sha256 (order-stable)."""
        ra = self.base_meta.get("referenced_assets") or {}
        lines = ["%s:%s" % (k, ra[k]["sha256"]) for k in sorted(ra)]
        return hashlib.sha256("\n".join(lines).encode("utf-8")).hexdigest()

    # -------------------- generation --------------------
    def generate(self, seed, density="medium", protected_zones=None):
        seed = int(seed)
        if density not in self.cfgs["difficulty"]["tiers"]:
            raise ValueError("unknown density %r" % (density,))
        rng = layout_rng(seed)
        rg = self.cfgs["warehouse"]["placement"]["regions"]
        objs, stats = sampler.sample_layout(
            rng, self.md, self.cfgs["obstacles"], self.cfgs["difficulty"],
            self.cfgs["asset_registry"], density=density,
            protected_zones=protected_zones,
            region_weights=rg["weights"], region_mode=rg.get("mode", "quota"))

        spec = {
            "schema_version": "1.0",
            "scene_id": "warehouse_%06d_%s" % (seed, density),
            "seed": seed,
            "density": density,
            "objects": objs,
            "object_count": len(objs),
            "counts_by_class": {c: sum(1 for o in objs if o["class"] == c)
                                for c in sampler.CLASS_ORDER},
            "generation_stats": stats,
            "regions": {
                "weights": stats.get("region_weights"),
                "placed_by_region": stats.get("by_region"),
                "candidate_cells": {
                    "aisle": int(self.md["region_aisle"].sum()),
                    "shelf_edge": int(self.md["region_edge"].sum()),
                    "open": int(self.md["region_open"].sum()),
                },
            },
            "metadata": {
                "seed": seed,
                "density": density,
                "generator_version": GENERATOR_VERSION,
                "config_hash": cfgmod.config_hash(),
                "asset_version": self.cfgs["asset_registry"].get("asset_version"),
                "base": {
                    "derived_usd": self.base_meta.get("derived_usd"),
                    "derived_usd_sha256": self.base_meta.get("derived_usd_sha256"),
                    "official_usd": self.base_meta.get("base_source"),
                    "official_usd_sha256": self.base_meta.get("official_usd_sha256"),
                    "map_sha256": self.base_meta.get("map_sha256"),
                    "assets_manifest_sha256": self.assets_manifest_sha256(),
                    "asset_count": len(self.base_meta.get("referenced_assets") or {}),
                },
                "coordinate": {"units": "m", "yaw": "rad", "up_axis": "Z"},
            },
        }
        spec["validation"] = pv.validate(spec, self.md, self.md)
        spec["layout_hash"] = layout_hash(objs)
        return spec

    # -------------------- io --------------------
    @staticmethod
    def save(spec, path):
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        with open(path, "w") as f:
            json.dump(spec, f, indent=2, sort_keys=True)
        return path

    @staticmethod
    def load(path):
        with open(path) as f:
            return json.load(f)
