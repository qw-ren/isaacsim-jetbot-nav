"""Compose display-only annotations and encode at four times simulation time."""
import argparse,bisect,csv,hashlib,json,math,shutil,subprocess
from pathlib import Path
import numpy as np
from PIL import Image
from compose_presentation import compose
p=argparse.ArgumentParser();p.add_argument('--case',required=True,type=Path);p.add_argument('--output',required=True,type=Path);p.add_argument('--ffmpeg',required=True,type=Path);a=p.parse_args();root=a.case
assert json.loads((root/'NAVIGATION_RESULT.json').read_text())['passed'];assert json.loads((root/'INPUT_AUDIT.json').read_text())['passed'];assert not a.output.exists()
def rows(name):return[json.loads(s)for s in(root/name).read_text().splitlines()if s.strip()]
truth=rows('truth_trajectory.jsonl');odom=rows('estimated_odom.jsonl');camera=rows('observer_pose.jsonl');paths=[r for r in rows('navigation_paths.jsonl')if r['topic']=='/local_plan'];assert paths and all(r['frame']=='odom'for r in paths)
times=lambda data,key:np.array([r[key]for r in data]);tt,ot,ct,pt=times(truth,'stamp'),times(odom,'stamp'),times(camera,'sim'),times(paths,'stamp');txyz=np.array([r['xyz']for r in truth]);oxyz=np.array([r['xyz']for r in odom]);cyaw=np.array([r['observer_yaw']for r in camera]);eyes=np.array([r['eye']for r in camera]);targets=np.array([r['target']for r in camera]);tyaw=np.unwrap([math.atan2(2*(r['wxyz'][0]*r['wxyz'][3]+r['wxyz'][1]*r['wxyz'][2]),1-2*(r['wxyz'][2]**2+r['wxyz'][3]**2))for r in truth]);oyaw=np.unwrap([math.atan2(2*(r['xyzw'][3]*r['xyzw'][2]+r['xyzw'][0]*r['xyzw'][1]),1-2*(r['xyzw'][1]**2+r['xyzw'][2]**2))for r in odom])
interp=lambda t,st,array:np.array([np.interp(t,st,array[:,k])for k in range(array.shape[1])]);events=rows('obstacle_events.jsonl');assert{e['kind']for e in events}=={'A','B'};config=json.loads((root/'CASE.json').read_text());goal=config['navigation']['goals'][-1];boxes=json.loads((root/'map/collision_boxes.json').read_text());control=rows('control_audit.jsonl');moving=[r['sim']for r in control if abs(r['output'][0])>.05];assert moving;start=moving[0]+.4
for r in camera:
 t=r['sim']
 if t<start:continue
 error=math.atan2(math.sin(r['observer_yaw']-np.interp(t,tt,tyaw)),math.cos(r['observer_yaw']-np.interp(t,tt,tyaw)))
 if abs(error)<.25 and t<events[0]['sim']-1:start=t;break
frames=root/'observer_video';index=sorted(list(csv.DictReader((frames/'frames.csv').open())),key=lambda r:float(r['requested_sim_s']));index=[r for r in index if float(r['requested_sim_s'])>=start];assert len(index)>100;stamps=[float(r['requested_sim_s'])for r in index];assert all(stamps[0]<e['sim']<stamps[-1]for e in events)
out=root/('observer_presentation_'+a.output.stem);out.mkdir(exist_ok=False);frozen=out/'source';frozen.mkdir()
for name in ['compose_presentation.py','encode_presentation_video.py']:shutil.copy2(Path(__file__).with_name(name),frozen/name)
statistics=[];concat=['ffconcat version 1.0']
for i,row in enumerate(index):
 t=stamps[i];image=Image.open(frames/row['file']).convert('RGB');xyz=interp(t,tt,txyz);yaw=float(np.interp(t,tt,tyaw));eye=interp(t,ct,eyes);target=interp(t,ct,targets);body=[];j=bisect.bisect_right(pt,t+.01)-1
 if j>=0 and -.01<=t-pt[j]<.5:
  position=interp(t,ot,oxyz);angle=float(np.interp(t,ot,oyaw));co,si=math.cos(angle),math.sin(angle);q=np.array(paths[j]['xy'])-position[:2];body=np.column_stack([co*q[:,0]+si*q[:,1],-si*q[:,0]+co*q[:,1]]).tolist()
 active=boxes+[dict(min=p['min'],max=p['max'])for event in events if event['sim']<=t for p in event['actual_parts']];image,stats=compose(image,eye,target,xyz,yaw,body,goal,active);stats['sim_s']=t;statistics.append(stats);name=row['file'];assert Path(name).name==name;image.save(out/name,quality=95);duration=(stamps[i+1]-t)/4 if i+1<len(index)else.02;concat.extend(["file '"+name+"'",'duration %.9f'%duration])
concat.append("file '"+index[-1]['file']+"'");listing=out/'video_4x.ffconcat';listing.write_text('\n'.join(concat)+'\n');(out/'composition_stats.jsonl').write_text(''.join(json.dumps(r)+'\n'for r in statistics));a.output.parent.mkdir(parents=True,exist_ok=True)
subprocess.run([str(a.ffmpeg),'-hide_banner','-loglevel','warning','-n','-f','concat','-safe','1','-i',str(listing),'-vf','fps=25','-c:v','libx264','-threads','4','-preset','medium','-crf','20','-pix_fmt','yuv420p','-movflags','+faststart',str(a.output)],check=True)
report=dict(encoder_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),composer_sha256=hashlib.sha256(Path(__file__).with_name('compose_presentation.py').read_bytes()).hexdigest(),frame_index_sha256=hashlib.sha256((frames/'frames.csv').read_bytes()).hexdigest(),playback_speed=4.,timebase='simulation clock',start_sim_s=stamps[0],end_sim_s=stamps[-1],expected_duration_s=(stamps[-1]-stamps[0])/4,input_frames=len(index),maximum_source_gap_sim_s=max(np.diff(stamps)),overlays='observer image only; native sensor products unchanged',ribbon=dict(source='/local_plan',display_offset_forward_m=1.10,floor_z_m=.003,shaft_width_m=.30,head_width_m=.64,one_closed_polygon=True,extrapolated=False),goal_beacon=dict(source='declared CAD goal',scene_geometry_added=False),display_reference_pose_usage='observer projection and vehicle masking only, never online estimation/control',arrow_frames=sum(s['arrow_drawn']for s in statistics),goal_beacon_frames=sum(s['goal_beacon_drawn']for s in statistics),events=[dict(kind=e['kind'],sim_s=e['sim'],video_s=(e['sim']-stamps[0])/4)for e in events],video_sha256=hashlib.sha256(a.output.read_bytes()).hexdigest(),video_bytes=a.output.stat().st_size)
assert report['arrow_frames']>100 and report['goal_beacon_frames']>20;a.output.with_suffix('.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
