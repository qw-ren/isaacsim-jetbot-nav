"""Choose an acceptance threshold on DEV only, then report frozen TEST metrics.
This is selective classification, not probability calibration or qualification.
"""
import argparse,json,hashlib
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--run',required=True,type=Path);a=p.parse_args();dev=[json.loads(s)for s in(a.run/'dev_predictions.jsonl').read_text().splitlines()];test=[json.loads(s)for s in(a.run/'test_predictions.jsonl').read_text().splitlines()]
def score(rows,t):
    accepted=[r for r in rows if r['confidence']>=t and r['margin']>=.2]
    return dict(samples=len(rows),accepted=len(accepted),coverage=len(accepted)/len(rows),accepted_accuracy=sum(r['correct'] for r in accepted)/len(accepted) if accepted else None,accepted_wrong=sum(not r['correct']for r in accepted))
choices=[(t,score(dev,t))for t in [.5,.55,.6,.65,.7,.75,.8,.85,.9,.95]]
qualified=[(t,m)for t,m in choices if m['accepted']>=100 and m['accepted_accuracy']>=.99]
threshold,metric=max(qualified,key=lambda pair:pair[1]['coverage']) if qualified else (.9,score(dev,.9))
report=dict(threshold_selected_using='dev_only',min_confidence=threshold,min_margin=.2,dev=metric,test=score(test,threshold),dev_choices=[dict(threshold=t,metrics=m)for t,m in choices],live_navigation_qualified=False,not_probability_calibration=True)
with(a.run/'THRESHOLD_SELECTION.json').open('x')as f:json.dump(report,f,indent=2)
print(json.dumps(report,indent=2))
