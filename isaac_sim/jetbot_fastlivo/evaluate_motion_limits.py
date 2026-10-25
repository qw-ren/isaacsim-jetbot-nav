"""Completed-case actuator contract and actual reference speed; offline evaluation only."""
from pathlib import Path
import json,hashlib,argparse,bisect,math,statistics
p=argparse.ArgumentParser();p.add_argument('--case',required=True,type=Path);a=p.parse_args();r=a.case;assert(r/'RUNNER_RESULT.json').exists()
def rows(n):return[json.loads(l)for l in(r/n).read_text().splitlines()if l.strip()]
cfg=json.loads((r/'CASE.json').read_text());limits=cfg['motion_limits'];commands=rows('control_audit.jsonl');reference=rows('truth_trajectory.jsonl');times=[v['sim']for v in commands];violations=[];max_wheel=0.;speeds=[]
radius=limits.get('wheel_radius_m',.03);track=limits.get('wheel_base_m',.1125);speed_threshold=(.9*limits['max_linear_m_s']if cfg.get('robot_variant',{}).get('scale',1.)!=1. else .36)
for i,row in enumerate(commands):
 v,w=row['output'];wheels=[(v-w*track/2)/radius,(v+w*track/2)/radius];peak=max(map(abs,wheels));max_wheel=max(max_wheel,peak)
 if peak>limits['max_wheel_rad_s']+1e-6:violations.append(dict(row=i,sim=row['sim'],wheel_rad_s=wheels))
for first,last in zip(reference,reference[1:]):
 dt=last['stamp']-first['stamp'];i=bisect.bisect_right(times,first['stamp'])-1
 if i<0 or not .015<dt<.03:continue
 v,w=commands[i]['output']
 if abs(v)>=speed_threshold and abs(w)<.05:speeds.append(math.dist(first['xyz'][:2],last['xyz'][:2])/dt)
report=dict(scope='offline only, actual speed from simulator reference; command wheel rates use controller nominal geometry, not a physical joint-velocity measurement',source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),input_sha256={n:hashlib.sha256((r/n).read_bytes()).hexdigest()for n in ['CASE.json','control_audit.jsonl','truth_trajectory.jsonl','RUNTIME_JOINT_LIMITS.json']},command_samples=len(commands),max_abs_linear_command_m_s=max(abs(v['output'][0])for v in commands),max_command_implied_wheel_rad_s=max_wheel,original_wheel_limit_rad_s=limits['max_wheel_rad_s'],command_wheel_limit_violations=violations,joint_properties_modified=limits['joint_limits_modified'],straight_reference_speed=dict(selection=f'abs(command_v)>={speed_threshold} m/s and abs(command_w)<0.05 rad/s; adjacent samples dt 15-30ms',samples=len(speeds),median_m_s=statistics.median(speeds)if speeds else None,p95_m_s=sorted(speeds)[int(.95*(len(speeds)-1))]if speeds else None,max_m_s=max(speeds)if speeds else None))
out=r/'MOTION_LIMIT_EVALUATION.json';assert not out.exists();out.write_text(json.dumps(report,indent=2)+'\n');out.with_name('MOTION_LIMIT_EVALUATION_source.py').write_bytes(Path(__file__).read_bytes());print(json.dumps(report,indent=2))
