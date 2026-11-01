"""Summarize compact DWB records; compare only fully scored valid candidates.
A sampled diagnostic is not proof of a critic's causal responsibility.
"""
import argparse
import hashlib
import json
from pathlib import Path


def analyze(path, start, expected):
    samples = 0
    switches = []
    comparisons = []
    previous = 0
    with path.open() as stream:
        for line in stream:
            row = json.loads(line)
            if row['stamp'] < start:
                continue
            samples += 1
            candidates = row['candidates']
            if not 0 <= row['best_index'] < len(candidates):
                continue
            best = candidates[row['best_index']]
            v, _, w = best['velocity']
            if abs(v) > 0.001:
                previous = 0
                continue
            sign = 1 if w > 0.01 else -1 if w < -0.01 else 0
            switched = bool(sign and previous and sign != previous)
            if switched:
                switches.append(row['stamp'])
            if sign:
                previous = sign
            valid_opposite = [candidate for candidate in candidates
                              if candidate['total'] >= 0 and len(candidate['scores']) == expected
                              and abs(candidate['velocity'][0]) <= 0.001
                              and candidate['velocity'][2] * sign < -0.01]
            if switched and valid_opposite and len(best['scores']) == expected and best['total'] >= 0:
                opposite = min(valid_opposite, key=lambda candidate: candidate['total'])
                opposite_scores = {name: scale * raw for name, scale, raw in opposite['scores']}
                comparisons.append(dict(stamp=row['stamp'], best=best, opposite=opposite,
                    opposite_minus_best_weighted_scores={name: opposite_scores[name] - scale * raw
                        for name, scale, raw in best['scores'] if name in opposite_scores}))
    return dict(scope='sampled DWB evaluations; fully scored opposite-turn comparisons only; no causal attribution',
                source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(), start_sim_s=start,
                samples=samples, zero_linear_best_direction_switches=switches,
                fully_scored_switch_comparisons=comparisons,
                limitations=['Short-circuiting can omit the best opposite-direction comparison.',
                             'Sampling at about 5Hz can miss switches.',
                             'Candidate scores are planner outputs, not executed actuator velocities.'])


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', required=True, type=Path)
    parser.add_argument('--start', required=True, type=float)
    parser.add_argument('--critics', type=int, default=7)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    report = analyze(args.input, args.start, args.critics)
    with args.output.open('x') as output:
        json.dump(report, output, indent=2)
        output.write('\n')
