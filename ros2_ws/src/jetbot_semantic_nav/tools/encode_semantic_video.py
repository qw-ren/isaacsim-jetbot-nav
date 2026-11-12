"""Offline annotated observer video; simulation time, no sensor/USD modification."""
import argparse
import bisect
import csv
import hashlib
import json
import subprocess
from pathlib import Path
from PIL import Image,ImageDraw,ImageFont

p=argparse.ArgumentParser();p.add_argument('--case',required=True,type=Path);p.add_argument('--task',required=True)
p.add_argument('--output',required=True,type=Path);p.add_argument('--ffmpeg',required=True,type=Path)
p.add_argument('--font',required=True,type=Path);p.add_argument('--speed',type=float,default=4.)
a=p.parse_args();assert a.speed>0 and not a.output.exists()
result=json.loads((a.case/'semantic_tasks'/(a.task+'.json')).read_text())
assert result['state']=='SUCCEEDED' and result['native_status']==4 and result['disarmed']
events=[json.loads(s)for s in(a.case/'semantic_tasks.jsonl').read_text().splitlines()]
events=[r for r in events if r.get('task_id')==a.task]
times=[r['stamp']for r in events];start=times[0]-.5;end=result['finished_stamp']+.5
frames=a.case/'observer_video';index=list(csv.DictReader((frames/'frames.csv').open()))
index=[r for r in index if start<=float(r['requested_sim_s'])<=end];assert len(index)>10
out=a.output.parent/(a.output.stem+'_frames');out.mkdir(exist_ok=False)
font=ImageFont.truetype(str(a.font),25);small=ImageFont.truetype(str(a.font),19)
concat=['ffconcat version 1.0'];stamps=[float(r['requested_sim_s'])for r in index]
for i,row in enumerate(index):
    stamp=stamps[i];j=bisect.bisect_right(times,stamp)-1;state=events[max(j,0)]['state']
    image=Image.open(frames/row['file']).convert('RGB');draw=ImageDraw.Draw(image)
    draw.rectangle((0,0,image.width,128),fill=(14,20,28))
    draw.text((20,8),result['request']['text'],font=font,fill=(255,230,170))
    draw.text((20,44),f'USD Ground Truth objects | FAST-LIVO2 + AMCL | Nav2 | {a.speed:g}x sim time',font=small,fill='white')
    draw.text((20,74),f'{state} | sim {stamp:.2f}s | {result["selected_object"]["object_id"].split("/")[-1]}',font=small,fill=(140,230,180))
    name=f'frame_{i:06d}.jpg';image.save(out/name,quality=93)
    duration=(stamps[i+1]-stamp)/a.speed if i+1<len(index)else .04
    concat.extend([f"file '{name}'",f'duration {duration:.9f}'])
concat.append(f"file 'frame_{len(index)-1:06d}.jpg'")
listing=out/'video.ffconcat';listing.write_text('\n'.join(concat)+'\n')
subprocess.run([str(a.ffmpeg),'-hide_banner','-loglevel','warning','-n','-f','concat','-safe','1','-i',str(listing),
    '-vf','fps=25','-c:v','libx264','-threads','4','-preset','medium','-crf','20','-pix_fmt','yuv420p',
    '-movflags','+faststart',str(a.output)],check=True)
report=dict(task=a.task,source='observer viewport, offline text only',truth_source='semantic objects only',
    localization='native IMU + FAST-LIVO2 + AMCL, not robot truth',start_sim_s=stamps[0],end_sim_s=stamps[-1],
    playback_speed=a.speed,input_frames=len(index),maximum_source_gap_sim_s=max(b-a for a,b in zip(stamps,stamps[1:])),
    video_sha256=hashlib.sha256(a.output.read_bytes()).hexdigest(),video_bytes=a.output.stat().st_size,
    events=events,source_index_sha256=hashlib.sha256((frames/'frames.csv').read_bytes()).hexdigest())
with a.output.with_suffix('.json').open('x')as f:json.dump(report,f,ensure_ascii=False,indent=2);f.write('\n')
print(json.dumps({k:v for k,v in report.items()if k!='events'},indent=2))
