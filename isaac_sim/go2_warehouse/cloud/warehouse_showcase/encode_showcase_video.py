"""Offline timestamped viewport -> H264. Uses truth only for an honest speed overlay."""
import argparse,csv,gzip,hashlib,json,math,subprocess
from pathlib import Path
import numpy as np
from PIL import Image,ImageDraw,ImageFont
from compose_local_ribbon import compose

p=argparse.ArgumentParser();p.add_argument('--frames',type=Path,required=True);p.add_argument('--evaluation',type=Path,required=True);p.add_argument('--applied',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--ffmpeg',required=True);p.add_argument('--start-sim',type=float,default=0.);p.add_argument('--distance-m',type=float);p.add_argument('--outcome',type=Path,required=True);p.add_argument('--events',type=Path,required=True);a=p.parse_args()
a.output.mkdir(exist_ok=False)
rows=sorted(list(csv.DictReader((a.frames/'frames.csv').open())),key=lambda r:int(r['frame']))
source_frame_count=len(rows)
rows=[r for r in rows if float(r['requested_sim_s'])>=a.start_sim]
if not rows:raise ValueError('No captured frames in selected interval')
with gzip.open(a.evaluation/'trajectories.json.gz','rt') as f:truth=json.load(f)['raw_truth']
t=np.array([v['stamp'] for v in truth]);xy=np.array([v['xyz'][:2] for v in truth])
unique=np.r_[True,np.diff(t)>0];t,xy=t[unique],xy[unique]
path=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(xy,axis=0),axis=1))]
clip_path_start=float(np.interp(float(rows[0]['requested_sim_s']),t,path))
if a.distance_m is not None:
 if a.distance_m<=0 or clip_path_start+a.distance_m>path[-1]:raise ValueError('Requested distance outside recorded path')
 end_sim=float(np.interp(clip_path_start+a.distance_m,path,t))
 rows=[r for r in rows if float(r['requested_sim_s'])<=end_sim]
# .2-second position difference reduces gait-frequency oscillation, no trajectory fitting.
def at(ts):return np.column_stack([np.interp(ts,t,xy[:,j]) for j in range(2)])
def speed(ts):return np.linalg.norm(at(ts+.1)-at(ts-.1),axis=1)/.2
stamp=float(json.loads(a.applied.read_text())['simulation_stamp_s'])
outcome=json.loads(a.outcome.read_text())
events=json.loads(a.events.read_text())
guard_stamp=outcome.get('guard_stop_sim_s');success_stamp=outcome.get('success_sim_s')
font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',24)
small=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',18)
source=[];times=[]
for i,r in enumerate(rows):
 ts=float(r['requested_sim_s']);f=a.frames/r['file']
 if f.parent!=a.frames or not f.is_file():raise ValueError('Missing or unsafe source image')
 source.append(dict(file=f.name,sha256=hashlib.sha256(f.read_bytes()).hexdigest(),requested_sim_s=ts,completed_sim_s=float(r['completed_sim_s']),snapshot_sha256=hashlib.sha256(f.with_suffix('.json').read_bytes()).hexdigest()))
 snapshot=json.loads(f.with_suffix('.json').read_text())
 image=compose(Image.open(f).convert('RGB'),snapshot);d=ImageDraw.Draw(image);v=float(speed(np.array([ts]))[0])
 d.rectangle((12,12,665,108),fill=(12,18,24))
 d.text((24,19),'Go2 | FAST-LIVO2 + AMCL + Nav2',font=font,fill='white')
 distance=float(np.interp(ts,t,path)-clip_path_start)
 d.text((24,53),f'Speed {v:.2f} m/s  |  Distance {distance:.1f} m  |  1x',font=small,fill=(160,235,255))
 appeared=[e for e in events if float(rows[0]['requested_sim_s'])<=e['applied']['simulation_stamp_s']<=ts]
 event=(appeared[-1]['kind']+' obstacle added: '+appeared[-1]['name']) if appeared else 'Warehouse navigation'
 if guard_stamp is not None and ts>=guard_stamp:event='GUARD STOP - route NOT completed'
 elif success_stamp is not None and ts>=success_stamp:event='NATIVE GOAL SUCCEEDED - complete obstacle bypass'
 d.text((24,83),event,font=small,fill=(255,185,80) if ts>=stamp else (185,245,185))
 image.save(a.output/f'overlay_{i:06d}.jpg',quality=92);times.append(ts)
if np.any(np.diff(times)<=0):raise ValueError('Nonmonotonic viewport time')
concat=a.output/'frames.ffconcat';lines=['ffconcat version 1.0']
for i,ts in enumerate(times):
 lines += [f"file 'overlay_{i:06d}.jpg'",f'duration {(times[i+1]-ts) if i+1<len(times) else .08:.9f}']
lines.append(f"file 'overlay_{len(times)-1:06d}.jpg'");concat.write_text('\n'.join(lines)+'\n')
movie=a.output/('go2_rear_follow_AB_short.mp4' if a.distance_m is not None else 'go2_rear_follow_AAB_showcase.mp4')
subprocess.run([a.ffmpeg,'-hide_banner','-loglevel','warning','-f','concat','-safe','1','-i',str(concat),'-vf','fps=25','-c:v','libx264','-preset','medium','-crf','22','-pix_fmt','yuv420p','-movflags','+faststart',str(movie)],check=True)
q=np.arange(max(t[0]+.1,times[0]),min(t[-1]-.1,times[-1]),.1);v=speed(q);fast=v>1.5
segments=[];start=None
for j,flag in enumerate(np.r_[fast,False]):
 if flag and start is None:start=j
 if not flag and start is not None:
  end=j-1;segments.append(dict(start_sim_s=float(q[start]),end_sim_s=float(q[end]+.1),duration_s=float((end-start+1)*.1),mean_speed_mps=float(v[start:j].mean())));start=None
report=dict(video=movie.name,navigation_outcome=outcome,events=[dict(name=e['name'],kind=e['kind'],simulation_stamp_s=e['applied']['simulation_stamp_s']) for e in events],sha256=hashlib.sha256(movie.read_bytes()).hexdigest(),bytes=movie.stat().st_size,
    clip_scope='distance-limited excerpt of existing trial; not a new full navigation acceptance' if a.distance_m is not None else 'full selected time interval',
    requested_distance_m=a.distance_m,
    display_style=dict(shape='single connected ground-plane polygon',ground_z_m=.003,opacity=190/255,shaft_width_m=.30,head_base_width_m=.64,head_max_length_m=.30,body_forward_display_offset_m=.65,robot_occlusion='conservative projected envelope',curve='recorded local-plan shape, shifted forward for display only; no extrapolation or navigation changes'),
    events_within_clip=[dict(name=e['name'],kind=e['kind'],simulation_stamp_s=e['applied']['simulation_stamp_s']) for e in events if times[0]<=e['applied']['simulation_stamp_s']<=times[-1]],
    playback='1x simulation time; source requests retimed with recorded simulation intervals',
    frame_count=len(rows),source_time_range_s=[times[0],times[-1]],requested_duration_s=times[-1]-times[0]+.08,
    actual_full_trial_path_m=float(np.linalg.norm(np.diff(xy,axis=0),axis=1).sum()),
    actual_video_path_m=float(np.linalg.norm(np.diff(at(np.r_[times[0],t[(t>times[0])&(t<times[-1])],times[-1]]),axis=0),axis=1).sum()),
    source_frame_count=source_frame_count,trimmed_initial_sim_seconds=times[0],maximum_measured_speed_mps=float(v.max()),
    seconds_above_1_5_mps=float(fast.sum()*.1),speed_method='offline reference XY displacement over .2s; no use by control',
    fast_segments=sorted(segments,key=lambda r:r['duration_s'],reverse=True)[:10],obstacle_insert_sim_s=stamp,
    maximum_capture_completion_delay_s=max(r['completed_sim_s']-r['requested_sim_s'] for r in source),
    limitations='Turns/avoidance may slow; viewport readback is asynchronous, request/completion bounds retained')
(a.output/'VIDEO_REPORT.json').write_text(json.dumps(report,indent=2)+'\n')
(a.output/'source_frames.json').write_text(json.dumps(source,indent=2)+'\n')
print(json.dumps(report,indent=2))
