"""Completed-trial endpoint XY/yaw error at feedback time and after stopping.
Truth is offline reference only. No fitting or online correction.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import numpy as np


def wrapped(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw(wxyz):
    q = np.asarray(wxyz, dtype=float)
    assert np.isfinite(q).all() and np.linalg.norm(q) > 1e-8
    w, x, y, z = q / np.linalg.norm(q)
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def evaluate(case):
    assert (case / 'RUNNER_RESULT.json').exists(), 'Completed trials only'
    load = lambda name: json.loads((case / name).read_text())
    rows = lambda name: [json.loads(line) for line in (case / name).read_text().splitlines() if line.strip()]
    truth = {row['stamp']: row for row in rows('truth_trajectory.jsonl')}
    times = np.array(sorted(truth))
    poses = [truth[t] for t in times]
    xy = np.array([row['xyz'][:2] for row in poses])
    angles = np.unwrap([yaw(row['wxyz']) for row in poses])
    feedback = rows('navigation_feedback.jsonl')[-1]
    t = feedback['stamp']
    assert times[0] <= t <= times[-1], 'No reference extrapolation'
    actual_xy = np.array([np.interp(t, times, xy[:, i]) for i in range(2)])
    actual_yaw = float(np.interp(t, times, angles))
    goal = np.array(load('NAVIGATION_RESULT.json')['goal'])
    estimated_xy = np.array(feedback['xy'])
    return dict(scope=__doc__, native_status=load('NAVIGATION_RESULT.json')['native_status'],
                feedback_stamp=t, goal=goal.tolist(),
                actual_xy_at_feedback_m=float(np.linalg.norm(actual_xy - goal[:2])),
                estimated_xy_at_feedback_m=float(np.linalg.norm(estimated_xy - goal[:2])),
                global_xy_bias_at_feedback_m=float(np.linalg.norm(estimated_xy - actual_xy)),
                actual_yaw_at_feedback_deg=math.degrees(abs(wrapped(actual_yaw - goal[2]))),
                estimated_yaw_at_feedback_deg=math.degrees(abs(wrapped(feedback['yaw'] - goal[2]))),
                global_yaw_bias_at_feedback_deg=math.degrees(abs(wrapped(feedback['yaw'] - actual_yaw))),
                stopped_reference_stamp=float(times[-1]),
                final_actual_xy_m=float(np.linalg.norm(xy[-1] - goal[:2])),
                final_actual_yaw_deg=math.degrees(abs(wrapped(angles[-1] - goal[2]))),
                inputs_sha256={name: hashlib.sha256((case / name).read_bytes()).hexdigest()
                               for name in ['truth_trajectory.jsonl', 'navigation_feedback.jsonl',
                                            'NAVIGATION_RESULT.json', 'RUNNER_RESULT.json']})


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    report = evaluate(args.case)
    with args.output.open('x') as output:
        json.dump(report, output, indent=2)
        output.write('\n')
