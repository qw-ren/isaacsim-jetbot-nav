"""Read recorded costmaps around physical obstacle footprints. No online map editing."""
from pathlib import Path
import argparse,json,hashlib
import numpy as np
p=argparse.ArgumentParser();p.add_argument('--case',type=Path,required=True);a=p.parse_args();r=a.case;assert(r/'RUNNER_RESULT.json').exists();events=[json.loads(l)for l in(r/'obstacle_events.jsonl').read_text().splitlines()if l.strip()];before=np.load(r/'global_costmap_initial.npz');results=[]
def region(data,event):
 grid=data['data'];origin=data['origin'];resolution=float(data['resolution']);mask=np.zeros(grid.shape,dtype=bool)
 for part in event['actual_parts']:
  lo=np.array(part['center'][:2])-np.array(part['size'][:2])/2-.2;hi=np.array(part['center'][:2])+np.array(part['size'][:2])/2+.2
  start=np.maximum(0,np.floor((lo-origin)/resolution).astype(int));end=np.minimum(grid.shape[::-1],np.ceil((hi-origin)/resolution).astype(int));mask[start[1]:end[1],start[0]:end[0]]=True
 values=grid[mask];return dict(cells=len(values),lethal_or_inscribed_cells=int(np.sum(values>=99)),occupied_threshold65_cells=int(np.sum(values>=65)),max_cost=int(values.max())if len(values)else None)
for event in events:
 name='global_costmap_'+event['kind']+'.npz';after=np.load(r/name);results.append(dict(kind=event['kind'],event_sim=event['sim'],snapshot_sim=float(after['stamp']),initial=region(before,event),after=region(after,event)))
report=dict(scope='offline recorded global costmaps; physical obstacle footprints with explicit 0.20m query margin, no pose fitting; occupancy evidence is distinct from collision clearance',events=results,source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),inputs={f.name:hashlib.sha256(f.read_bytes()).hexdigest()for f in[r/'obstacle_events.jsonl',*r.glob('global_costmap_*.npz')]});out=r/'OBSTACLE_LAYER_EVIDENCE.json';assert not out.exists();out.write_text(json.dumps(report,indent=2)+'\n');out.with_name('OBSTACLE_LAYER_EVIDENCE_source.py').write_bytes(Path(__file__).read_bytes());print(json.dumps(report,indent=2))
