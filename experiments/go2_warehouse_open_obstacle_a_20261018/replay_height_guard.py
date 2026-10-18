"""Replay actual recorded guard inputs; no ROS or velocity output."""
from pathlib import Path
import json,math,numpy as np
r=Path(__file__).resolve().parent
original=json.loads((r/'replay_source/safety_boxes.json').read_text());boxes=json.loads((r/'height_clipped/static_boxes.json').read_text())+[original[-1]]
b=np.array([[v['min'][:2],v['max'][:2]] for v in boxes]);records=json.loads((r/'replay_source/guard_geometry_diagnostic.json').read_text());out={}
for name,f in records.items():
 x,y,_,yaw,_=f['fault_pose'];v,w=f['input_command'];t=np.linspace(0,1.5,16)
 points=np.c_[x+v/w*(np.sin(yaw+w*t)-np.sin(yaw)),y-v/w*(np.cos(yaw+w*t)-np.cos(yaw))] if abs(w)>1e-6 else np.c_[x+v*t*np.cos(yaw),y+v*t*np.sin(yaw)]
 delta=np.maximum(np.maximum(b[:,0,None,:]-points,points-b[:,1,None,:]),0);d=np.linalg.norm(delta,axis=-1);i,j=np.unravel_index(d.argmin(),d.shape)
 out[name]={'old_min_m':f['predicted_min_m'],'height_clipped_min_m':float(d[i,j]),'limiting_path':boxes[i].get('path','new_test_box'),'would_latch':bool(d[i,j]<.55)}
checks={'new_box_close_commands_still_blocked':all(out[n]['would_latch'] for n in ['navigation_01','navigation_02']),'old_high_mesh_false_blocks_removed':all(not out[n]['would_latch'] for n in ['navigation_03','navigation_05_start1','navigation_06_start1'])}
result={'replay':out,'checks':checks,'passed':all(checks.values()),'threshold_m':.55}
(r/'height_replay_check.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(checks));assert result['passed']
