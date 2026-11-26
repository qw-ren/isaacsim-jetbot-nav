#!/usr/bin/env python3
"""Submit out-and-back Nav2 missions; never sends velocity."""
import argparse
import json
import math
from pathlib import Path
import subprocess
import time

CODE = Path("/home/admin/jetbot-warehouse/code_deploy_20261120")


def read_json(path):
    try:
        return json.loads(path.read_text())
    except (FileNotFoundError, json.JSONDecodeError):
        return None


def wait_for(case, condition, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if read_json(case / "RUNNER_RESULT.json"):
            raise RuntimeError("simulation session ended")
        if (case / "ODOM_FAULT.json").exists():
            raise RuntimeError("odometry fault; inspect case logs")
        gate = read_json(case / "GATE_PROGRESS.json")
        if gate and gate.get("fault"):
            raise RuntimeError("velocity gate fault: " + str(gate["fault"]))
        result = condition()
        if result:
            return result
        time.sleep(.5)
    raise TimeoutError("timed out waiting for operator/mission")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--case", type=Path, required=True)
    args = ap.parse_args()
    case = args.case.resolve()
    if case.parent != Path("/home/admin/jetbot-warehouse/artifacts"):
        raise ValueError("unexpected case directory")
    def ready():
        status, gate = read_json(case / "OPERATOR_STATUS.json"), read_json(case / "GATE_PROGRESS.json")
        progress = read_json(case / "PROGRESS.json")
        return (read_json(case / "OPERATOR_READY.json") and status and status.get("state") == "IDLE"
                and gate and progress and progress.get("sim_s", 0) >= 10)
    results = []
    try:
        wait_for(case, ready, 300)
        # Reuse the two previously traversed corridor segments in both directions.
        # Approximately 8 m total translation; do not relax arrival or safety limits.
        for index, goal in enumerate([(0., 7., math.pi / 2), (-3., 7., math.pi),
                                      (0., 7., 0.), (0., 6., -math.pi / 2)], 1):
            mission = "visible_demo_%02d" % index
            subprocess.run(["python3", str(CODE / "submit_goal.py"), "--case", str(case),
                            "--id", mission, "--x", str(goal[0]), "--y", str(goal[1]),
                            "--yaw", str(goal[2]), "--timeout", "180"], check=True)
            result = wait_for(case, lambda: read_json(case / "missions" / mission / "RESULT.json"), 190)
            results.append(result)
            if not result.get("passed"):
                raise RuntimeError("navigation demonstration failed: " + mission)
            wait_for(case, ready, 15)
        report = dict(passed=True, missions=results, velocity_published=False)
    except Exception as exc:
        report = dict(passed=False, missions=results, error=str(exc), velocity_published=False)
    (case / "VISIBLE_DEMO_RESULT.json").write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
