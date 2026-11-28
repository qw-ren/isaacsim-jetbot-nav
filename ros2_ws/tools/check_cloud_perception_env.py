#!/usr/bin/env python3
"""Check the selected ROS/perception Python environment without installing packages."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--domain", default="51")
    args = parser.parse_args()
    checks = []

    def record(name, ok, detail):
        checks.append(dict(name=name, ok=bool(ok), detail=detail))

    record("ROS_DISTRO", os.environ.get("ROS_DISTRO") == "humble", os.environ.get("ROS_DISTRO"))
    record("ROS_DOMAIN_ID", os.environ.get("ROS_DOMAIN_ID") == args.domain, os.environ.get("ROS_DOMAIN_ID"))
    for name, imports in [
        ("ROS Python bindings", "import rclpy, tf2_ros, message_filters"),
        ("image bridge ABI", "import numpy, cv2; from cv_bridge import CvBridge; CvBridge()"),
        ("perception core", "import yaml; from atongmu_perception import PerceptionPipeline"),
        ("model runtime", "import torch; from ultralytics import YOLO; print('torch', torch.__version__, 'cuda', torch.version.cuda, 'available', torch.cuda.is_available())"),
    ]:
        try:
            result = subprocess.run([sys.executable, "-B", "-c", imports],
                                    capture_output=True, text=True, errors="replace", timeout=45)
            detail = (result.stdout + result.stderr).strip()[-2000:]
            record(name, result.returncode == 0, detail or "import passed")
        except subprocess.TimeoutExpired:
            record(name, False, "import timed out after 45 seconds")
    root = Path(__file__).resolve().parents[2]
    weight = root / "perception/weights/obstacles_best.pt"
    record("obstacle weights present", weight.is_file(), str(weight))
    for name, path in [
        ("Isaac Sim launcher", "/home/admin/isaacsim/python.sh"),
        ("NVIDIA assets", "/home/admin/isaacsim_assets/Assets/Isaac/4.5/Isaac"),
        ("navigation environment", "/home/admin/go2-warehouse/nav_stage2/cloud/warehouse_stage2/nav_env.sh"),
        ("FAST-LIVO backend", "/home/admin/go2-warehouse/nav_stage2/deps/fastlivo_trial_20261022/bin/fastlivo_mapping"),
        ("source robot scene", "/home/admin/jetbot-warehouse/source_scene/jetbot_sim.usd"),
        ("planning prior", "/home/admin/jetbot-warehouse/map_prior_20261023_01/MAP_PROVENANCE.json"),
    ]:
        record(name, Path(path).exists(), path)
    report = dict(python=sys.executable, python_version=sys.version.split()[0], checks=checks,
                  ok=all(item["ok"] for item in checks),
                  scope="import and file-presence checks only; CUDA availability is reported, not required for imports; no simulation or inference executed")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
