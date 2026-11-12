"""Offline audit of native speed-limit events vs settled armed Nav2 command records."""
import argparse
import bisect
import json
from pathlib import Path

p=argparse.ArgumentParser();p.add_argument('--case',required=True,type=Path);p.add_argument('--output',required=True,type=Path)
a=p.parse_args();assert not a.output.exists()
runtime=json.loads((a.case/'CASE.json').read_text());base=runtime['motion_limits']['max_linear_m_s']
events=[json.loads(s)for s in(a.case/'semantic_tasks.jsonl').read_text().splitlines()]
limits=[r for r in events if r['state']in('SPEED_LIMIT','SPEED_LIMIT_RESET')];stamps=[r['stamp']for r in limits]
checked=[];violations=[]
for line in(a.case/'gate_states.jsonl').read_text().splitlines():
    row=json.loads(line)
    if not row.get('armed')or row.get('sim')is None:continue
    i=bisect.bisect_right(stamps,row['sim'])-1
    if i<0 or limits[i]['state']!='SPEED_LIMIT' or row['sim']-limits[i]['stamp']<.6:continue
    cap=limits[i]['limit_m_s'];v=abs(row['raw_candidate'][0]);checked.append((cap,v))
    if v>cap+1e-5:violations.append(dict(sim_s=row['sim'],native_command=row['raw_candidate'],limit_m_s=cap))
observed={str(cap):dict(samples=sum(c==cap for c,v in checked),max_native_command_m_s=max(v for c,v in checked if c==cap))for cap in set(c for c,v in checked)}
protocol=bool(limits)and all(e.get('native_message_speed_limit')==(0. if e['limit_m_s']>=base else e['limit_m_s'])for e in limits if e['state']=='SPEED_LIMIT')
restored=[];previous_near=False
for e in limits:
    if e['state']=='SPEED_LIMIT'and e['limit_m_s']<base:previous_near=True
    elif e['state']=='SPEED_LIMIT'and e['limit_m_s']>=base and previous_near:restored.append(e)
evidence=[];score_file=a.case/'controller_evaluations.jsonl'
if score_file.exists():
    for line in score_file.read_text().splitlines():
        r=json.loads(line);i=bisect.bisect_right(stamps,r['stamp'])-1
        if i<0:continue
        e=limits[i]
        if e not in restored or r['stamp']-e['stamp']<.6:continue
        maximum=max((c['velocity'][0]for c in r['candidates']),default=0.)
        if maximum>base*.75:
            evidence.append(dict(stamp=r['stamp'],maximum_sampled_linear_m_s=maximum,limit_event_sim_s=e['stamp']))
            if len(evidence)>=3:break
tasks=[]
for path in sorted((a.case/'semantic_tasks').glob('*.json')):
    r=json.loads(path.read_text())
    if r.get('native_status')is not None:
        resets=[e for e in limits if e['task_id']==r['request']['task_id']and e['state']=='SPEED_LIMIT_RESET']
        tasks.append(dict(task_id=r['request']['task_id'],state=r['state'],native_status=r['native_status'],disarmed=r.get('disarmed'),limit_reset=bool(resets)))
report=dict(case=a.case.name,native_command_cap_audit_passed=bool(checked)and not violations,
    native_limit_message_protocol_valid=protocol,free_cruise_after_near_limit_events=len(restored),
    restored_cruise_sampling_observed=bool(evidence),restored_cruise_sample_evidence=evidence,
    settled_after_transition_sim_s=.6,observed_native_commands=observed,violations=violations,
    task_results=tasks,all_actual_tasks_succeeded=bool(tasks)and all(r['native_status']==4 for r in tasks),
    all_actual_tasks_disarmed_and_limit_reset=bool(tasks)and all(r['disarmed']and r['limit_reset']for r in tasks),
    limit_events=limits,scope='Native command audit, not instantaneous physical velocity or proof of message delivery during excluded transitions')
with a.output.open('x')as f:json.dump(report,f,ensure_ascii=False,indent=2);f.write('\n')
print(json.dumps({k:v for k,v in report.items()if k not in('limit_events','violations','task_results')}))
