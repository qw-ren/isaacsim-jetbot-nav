"""Sequential live-case commands, retaining all CLI logs and terminal results."""
import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path

p=argparse.ArgumentParser();p.add_argument('--case',required=True,type=Path);p.add_argument('--include-cart',action='store_true')
p.add_argument('--relational',action='store_true');p.add_argument('--relation-only',action='store_true');a=p.parse_args()
deadline=time.monotonic()+180
while True:
    try:
        state=json.loads((a.case/'SEMANTIC_STATUS.json').read_text())['state']
        if state=='READY':break
        if state=='STARTUP_FAILED':raise RuntimeError('Semantic startup failed')
    except(FileNotFoundError,json.JSONDecodeError):pass
    if time.monotonic()>deadline:raise TimeoutError('Semantic startup wait expired')
    time.sleep(.2)
checks=[('missing_sign','到最近的标牌那里',[],{'NOT_FOUND'}),
        ('unsupported_person','到最近的人那里',[],{'INVALID_COMMAND'}),
        ('cancel_pallet','到最近的托盘那里',['--cancel-after','1.0'],{'CANCELLED'}),
        ('pallet_nav','到最近的托盘那里',[],{'SUCCEEDED'}),
        ('box_nav','到最近的纸箱那里',[],{'SUCCEEDED'})]
if a.include_cart:checks.insert(0,('cart_nav','到最近的推车那里',[],{'SUCCEEDED'}))
if a.relational:
    checks=[('farthest_preview','找最远的纸箱',['--dry-run'],{'DRY_RUN_COMPLETE'}),
            ('missing_row','到第九排货架旁边的那个纸箱',[],{'REFERENCE_NOT_FOUND'}),
            ('ambiguous_row','走到第一排货架旁边的那个纸箱',[],{'AMBIGUOUS'}),
            ('relational_preview','走到第二排货架旁边的那个纸箱',['--dry-run'],{'DRY_RUN_COMPLETE'}),
            ('row_nearest','到第一排货架旁边最近的纸箱',['--dry-run'],{'DRY_RUN_COMPLETE'}),
            ('row_farthest','到第一排货架附近最远的纸箱',['--dry-run'],{'DRY_RUN_COMPLETE'}),
            ('unsupported_relation','到第二排货架左边的纸箱',[],{'INVALID_COMMAND'}),
            ('farthest_nav','到最远的纸箱那里',[],{'SUCCEEDED'}),
            ('relational_nav','走到第二排货架旁边的那个纸箱',[],{'SUCCEEDED'})]
if a.relation_only:
    checks=[('relational_spawn_nav','走到第二排货架旁边的那个纸箱',[],{'SUCCEEDED'})]
results=[]
for task,text,extra,expected in checks:
    with(a.case/('cli_'+task+'.log')).open('x')as log:
        code=subprocess.run([sys.executable,'-m','jetbot_semantic_nav.cli','--case',str(a.case),
            '--id',task,'--timeout','800',*extra,text],stdout=log,stderr=subprocess.STDOUT).returncode
    lines=(a.case/('cli_'+task+'.log')).read_text().splitlines()
    messages=[]
    for line in lines:
        try:messages.append(json.loads(line))
        except ValueError:pass
    state=messages[-1]['state']if messages else 'NO_ACK'
    result=dict(task=task,state=state,cli_exit_code=code,passed=state in expected)
    task_file=a.case/'semantic_tasks'/(task+'.json')
    if task_file.exists():
        record=json.loads(task_file.read_text());result['selected_object']=record.get('selected_object',{}).get('object_id')
        ranking=record.get('object_ranking',[])
        if ranking:
            lengths=[r['shortest_approach_path_m']for r in ranking]
            reverse=record['parsed']['selection']=='farthest_reachable'
            result['ranking_verified']=lengths==sorted(lengths,reverse=reverse)
            result['passed'] &= result['ranking_verified']
        if (a.relational or a.relation_only) and task in('relational_preview','relational_nav','relational_spawn_nav'):
            result['passed'] &= result['selected_object']=='/World/SemanticOracle/box_relation'
        if a.relational and task in('farthest_preview','farthest_nav','row_farthest'):
            result['passed'] &= result['selected_object']=='/World/SemanticOracle/box_far'
        if a.relational and task=='row_nearest':
            result['passed'] &= result['selected_object']=='/World/SemanticOracle/box_near'
    results.append(result);print(json.dumps(result),flush=True)
    (a.case/'COMMAND_ACCEPTANCE.json').write_text(json.dumps(results,indent=2)+'\n')
    if not result['passed']:raise SystemExit(1)
