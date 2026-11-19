"""Independent authored commands, concrete recorded object map, no ROS/motion."""
import argparse,json,time,copy,hashlib
from pathlib import Path
import torch
from jetbot_semantic_nav.pretrained_grounding import PretrainedGrounder
p=argparse.ArgumentParser();p.add_argument('--min-confidence',type=float,default=.9);p.add_argument('--model',required=True,type=Path);p.add_argument('--commands',required=True,type=Path);p.add_argument('--output',required=True,type=Path);a=p.parse_args()
if a.output.exists():raise FileExistsError(a.output)
torch.set_num_threads(4);model=PretrainedGrounder(a.model,a.min_confidence);rows=[json.loads(s)for s in a.commands.read_text().splitlines()];results=[];latencies=[]
for r in rows:
    start=time.perf_counter();result=model.predict(r['text'],r['objects'],r['robot_xy']);latencies.append(time.perf_counter()-start)
    expected=r['target'];prediction=result['object_id'] if result['state']=='SELECTED' else result['state'];results.append(dict(case_id=r['case_id'],text=r['text'],expected=expected,prediction=prediction,correct=prediction==expected,result=result))
# Invariance of learned object ID choice to order and identifiers.
r=rows[0];original=model.predict(r['text'],r['objects'],r['robot_xy']);reverse=model.predict(r['text'],list(reversed(r['objects'])),r['robot_xy']);assert original['state']==reverse['state'] and original['object_id']==reverse['object_id']
renamed=copy.deepcopy(r['objects']);ids={}
for i,o in enumerate(renamed):ids[o['object_id']]='renamed_'+str(i);o['object_id']=ids[o['object_id']]
renamed_result=model.predict(r['text'],renamed,r['robot_xy']);assert renamed_result['state']==original['state'];assert renamed_result['object_id']==ids.get(original['object_id'])
# Unknown candidates must not turn into an object selection.
assert model.predict(r['text'],[],r['robot_xy'])['object_id'] is None
accepted=[r for r in results if r['result']['state'] in ('SELECTED','NOT_FOUND','AMBIGUOUS')];wrong=[r for r in accepted if not r['correct']]
report=dict(min_confidence=a.min_confidence,agent_authored_not_human_validated=True,commands_sha256=hashlib.sha256(a.commands.read_bytes()).hexdigest(),samples=len(rows),correct=sum(r['correct'] for r in results),accepted=len(accepted),accepted_wrong=len(wrong),cpu_latency_ms=dict(median=sorted(latencies)[len(latencies)//2]*1000,maximum=max(latencies)*1000),candidate_order_and_id_invariance_passed=True,results=results,live_navigation_qualified=False)
a.output.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n');print(json.dumps({k:v for k,v in report.items() if k!='results'},ensure_ascii=False,indent=2))
