"""Sequential live-case acceptance with optional independent, bounded readonly probes."""
import argparse
import json
import signal
import subprocess
import sys
import time
from pathlib import Path

p=argparse.ArgumentParser();p.add_argument('--case',required=True,type=Path);p.add_argument('--runtime',required=True,type=Path)
p.add_argument('--extra-nearest',action='store_true');a=p.parse_args();probes=[]
try:
    # Keep diagnostic graph queries/service calls out of lifecycle bringup.
    deadline=time.monotonic()+180
    while True:
        try:
            state=json.loads((a.case/'SEMANTIC_STATUS.json').read_text())['state']
            if state=='READY':break
            if state=='STARTUP_FAILED':raise RuntimeError('Semantic startup failed before diagnostics')
        except(FileNotFoundError,json.JSONDecodeError):pass
        if time.monotonic()>deadline:raise TimeoutError('Semantic readiness wait expired')
        time.sleep(.2)
    for name,tool in [('scores',a.runtime/'controller_probe.py'),('costmap',a.runtime/'local_costmap_probe.py'),('plans',a.runtime/'semantic_tools/probe_semantic_plans.py')]:
        with(a.case/('external_'+name+'.log')).open('x')as log:
            probes.append(subprocess.Popen([sys.executable,str(tool),'--case',str(a.case)],stdout=log,stderr=subprocess.STDOUT))
    result=subprocess.run([sys.executable,str(a.runtime/'semantic_tools/run_acceptance_commands.py'),'--case',str(a.case),'--relational'])
    if result.returncode:raise SystemExit(result.returncode)
    if a.extra_nearest:
        task='row_nearest_nav';text='到第一排货架旁边最近的纸箱'
        with(a.case/('cli_'+task+'.log')).open('x')as log:
            code=subprocess.run([sys.executable,'-m','jetbot_semantic_nav.cli','--case',str(a.case),'--id',task,text],stdout=log,stderr=subprocess.STDOUT).returncode
        record=json.loads((a.case/'semantic_tasks'/(task+'.json')).read_text())
        row=dict(task=task,state=record['state'],native_status=record.get('native_status'),selected_object=record.get('selected_object',{}).get('object_id'),cli_exit_code=code)
        row['passed']=row['state']=='SUCCEEDED'and row['selected_object']=='/World/SemanticOracle/box_near'
        with(a.case/'MANEUVER_EXTRA_COMMANDS.json').open('x')as f:json.dump([row],f,ensure_ascii=False,indent=2)
        print(json.dumps(row),flush=True)
        if not row['passed']:raise SystemExit(1)
finally:
    for child in probes:
        if child.poll()is None:child.send_signal(signal.SIGINT)
    for child in probes:
        try:child.wait(timeout=10)
        except subprocess.TimeoutExpired:print('Readonly probe still stopping:',child.pid,flush=True)
