"""Summarize completed navigation trials, separating localization and arrival.
Basic status checks do not replace A/B/C geometry audits or repeated trials.
"""
import argparse
import hashlib
import json
from pathlib import Path
from evaluate_endpoint_pose import evaluate as endpoint
from evaluate_localization_accuracy import evaluate as localization


def summarize(case):
    load=lambda name:json.loads((case/name).read_text())
    cfg=load('CASE.json');nav=load('NAVIGATION_RESULT.json');runner=load('RUNNER_RESULT.json')
    gate=load('GATE_RESULT.json');audit=load('INPUT_AUDIT.json')
    expected=cfg.get('navigation',{}).get('expected_native_status',4)
    result=dict(case=case.name,expected_native_status=expected,native_status=nav.get('native_status'),
        navigation_succeeded=nav.get('native_status')==4,
        basic_checks=dict(status_matches_expectation=nav.get('native_status')==expected,
                          input_audit_passed=audit.get('passed') is True,
                          gate_no_fault=gate.get('fault') is None and gate.get('runtime_error') is None,
                          shutdown_zero_published=gate.get('shutdown_zero_published') is True),
        process_clean=runner['exit_code']==0 and all(child['exit_code']==0 for child in runner['children']),
        nonzero_children=[child for child in runner['children'] if child['exit_code']!=0],
        arrival_tolerance={key:value for key,value in cfg['navigation'].get('parameter_overrides',{}).items()
                           if 'goal_tolerance' in key},
        note='Basic checks only; no claim of A/B/C correctness or repeated-trial precision.')
    if nav.get('native_status')==4:
        result['endpoint']=endpoint(case)
        result['localization']=localization(case)
    result['inputs_sha256']={name:hashlib.sha256((case/name).read_bytes()).hexdigest()
        for name in ['CASE.json','NAVIGATION_RESULT.json','RUNNER_RESULT.json','GATE_RESULT.json','INPUT_AUDIT.json']}
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--case',type=Path,required=True);parser.add_argument('--output',type=Path,required=True);a=parser.parse_args()
    result=summarize(a.case)
    with a.output.open('x') as stream:json.dump(result,stream,indent=2);stream.write('\n')
    compact={key:result[key] for key in ['case','native_status','basic_checks','process_clean']}
    if 'endpoint' in result:
        compact['endpoint_xy_cm']=result['endpoint']['final_actual_xy_m']*100
        compact['endpoint_yaw_deg']=result['endpoint']['final_actual_yaw_deg']
        compact['localization']=result['localization']['amcl_published_pose_samples']
    print(json.dumps(compact,indent=2))
