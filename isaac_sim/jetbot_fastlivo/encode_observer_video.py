"""Encode recorded raw observer frames at 4x simulation time, without overlays."""
import argparse,csv,json,hashlib,subprocess,math
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--case',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--ffmpeg',type=Path,required=True);a=p.parse_args();root=a.case;assert(root/'RUNNER_RESULT.json').exists();assert json.loads((root/'NAVIGATION_RESULT.json').read_text())['passed'];assert not a.output.exists()
frames=root/'observer_video';rows=list(csv.DictReader((frames/'frames.csv').open()));rows=sorted(rows,key=lambda r:float(r['requested_sim_s']));commands=[json.loads(l)for l in(root/'control_audit.jsonl').read_text().splitlines()if l.strip()];moving=[r['sim']for r in commands if abs(r['output'][0])>.05];assert moving
start=moving[0]+.4
truth=[json.loads(l)for l in(root/'truth_trajectory.jsonl').read_text().splitlines()if l.strip()];first_goal=json.loads((root/'CASE.json').read_text())['navigation']['goals'][0];heading=math.atan2(first_goal[1]-truth[0]['xyz'][1],first_goal[0]-truth[0]['xyz'][0])
for row in truth:
 w,x,y,z=row['wxyz'];yaw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
 if row['stamp']>=start and abs(math.atan2(math.sin(yaw-heading),math.cos(yaw-heading)))<=.3:start=row['stamp'];break
selected=[r for r in rows if float(r['requested_sim_s'])>=start];assert len(selected)>100;times=[float(r['requested_sim_s'])for r in selected];assert all(b>a for a,b in zip(times,times[1:]));events=[json.loads(l)for l in(root/'obstacle_events.jsonl').read_text().splitlines()if l.strip()];assert {e['kind']for e in events}=={'A','B'}and all(times[0]<e['sim']<times[-1]for e in events)
lines=['ffconcat version 1.0']
for i,row in enumerate(selected):
 name=row['file'];assert Path(name).name==name and name.startswith('frame_')and name.endswith('.jpg');assert(frames/name).is_file();duration=(times[i+1]-times[i])/4 if i+1<len(times)else .02;lines.extend(["file '"+name+"'",'duration %.9f'%duration])
lines.append("file '"+selected[-1]['file']+"'");concat=frames/'observer_4x.ffconcat';assert not concat.exists();concat.write_text('\n'.join(lines)+'\n');a.output.parent.mkdir(parents=True,exist_ok=True)
subprocess.run([str(a.ffmpeg),'-hide_banner','-loglevel','warning','-n','-f','concat','-safe','1','-i',str(concat),'-vf','fps=25','-c:v','libx264','-threads','4','-preset','medium','-crf','20','-pix_fmt','yuv420p','-movflags','+faststart',str(a.output)],check=True)
report=dict(playback_speed=4.,timebase='simulation time, not slowed wall clock',overlays=False,start_sim_s=times[0],end_sim_s=times[-1],expected_duration_s=(times[-1]-times[0])/4,input_frames=len(selected),maximum_source_gap_sim_s=max(b-a for a,b in zip(times,times[1:])),events=[dict(kind=e['kind'],sim_s=e['sim'],video_s=(e['sim']-times[0])/4)for e in events],video_sha256=hashlib.sha256(a.output.read_bytes()).hexdigest(),video_bytes=a.output.stat().st_size,frame_index_sha256=hashlib.sha256((frames/'frames.csv').read_bytes()).hexdigest(),encoder_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest());a.output.with_suffix('.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
