"""Create the C runtime without changing the live B runtime or accepted copies."""
import argparse
import ast
import hashlib
import json
from pathlib import Path
import shutil


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", default="motion_live_branch_20261019")
    parser.add_argument("--target", default="motion_live_closed_20261019")
    args = parser.parse_args()
    root = Path("/home/admin/go2-warehouse")
    source, target = (root / args.source).resolve(), (root / args.target).resolve()
    if source.parent != root or target.parent != root or not target.name.startswith("motion_live_closed_"):
        raise SystemExit("Expected isolated sibling motion_live_closed_* target")
    if target.exists():
        raise SystemExit("C runtime exists; inspect it rather than overwrite")
    entry = (source / "isaac_go2_ros2.py").read_text()
    old = "from branch_obstacle_fixture import ObstacleFixture"
    if entry.count(old) != 1 or "live_points.tick()" not in entry:
        raise SystemExit("Source is not the expected live RTX B runtime")
    entry = entry.replace(old, "from closed_route_fixture import ObstacleFixture")
    ast.parse(entry)
    fixture = Path(__file__).with_name("closed_route_fixture.py")
    ast.parse(fixture.read_text())
    shutil.copytree(source, target, ignore=shutil.ignore_patterns("__pycache__", "outputs", ".git"))
    shutil.copy2(fixture, target / fixture.name)
    (target / "isaac_go2_ros2.py").write_text(entry)
    files = ["isaac_go2_ros2.py", fixture.name]
    record = {"source": str(source), "target": str(target),
              "change": "C four-wall start enclosure only; live RTX, GPU physics, policy and bridge preserved",
              "scope": "stationary start enclosed before NavigateToPose; not moving blockage or both remote branches",
              "files_sha256": {f: hashlib.sha256((target / f).read_bytes()).hexdigest() for f in files}}
    (target / "CLOSED_ROUTE_COPY.json").write_text(json.dumps(record, indent=2) + "\n")
    print(target)


if __name__ == "__main__":
    main()
