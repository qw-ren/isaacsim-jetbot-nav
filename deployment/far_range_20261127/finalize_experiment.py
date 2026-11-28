import hashlib
import json
from pathlib import Path
import subprocess
import tarfile
import time

ROOT=Path('/home/admin/atongmu_deploy_20261120/far_range_20261127')
CASE=Path('/home/admin/jetbot-warehouse/artifacts/far_range_20261127_01')
deadline=time.monotonic()+2400
while not (ROOT/'EXPERIMENT_RESULT.json').exists():
    if time.monotonic()>deadline:
        raise TimeoutError('experiment completion wait expired')
    time.sleep(5)
subprocess.run(['/home/admin/venvs/atongmu_ros/bin/python',str(ROOT/'evaluate_range.py'),'--case',str(CASE)],check=True)
frames=[json.loads(s) for s in (CASE/'route_evidence/frames.jsonl').read_text().splitlines()]
if len(frames)>1:
    manifest=['ffconcat version 1.0']
    for i,frame in enumerate(frames):
        duration=frames[i+1]['stamp']-frame['stamp'] if i+1<len(frames) else 1.
        manifest.extend(["file '%s'"%(CASE/'route_evidence'/frame['file']), 'duration %.6f'%duration])
    manifest.append("file '%s'"%(CASE/'route_evidence'/frames[-1]['file']))
    path=CASE/'range_evaluation/detections.ffconcat'
    path.write_text('\n'.join(manifest)+'\n')
    subprocess.run(['ffmpeg','-nostdin','-hide_banner','-loglevel','error','-n','-f','concat','-safe','0','-i',str(path),
                    '-vf','fps=10','-c:v','libx264','-pix_fmt','yuv420p',str(CASE/'range_evaluation/detections_1x.mp4')],check=True)
files=[CASE/'observer_video/navigation_1x.mp4',CASE/'range_evaluation/detections_1x.mp4']
(CASE/'range_evaluation/VIDEO_SHA256.json').write_text(json.dumps({str(p.relative_to(CASE)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files if p.exists()},indent=2))
(CASE/'range_evaluation/EXPERIMENT_RESULT.json').write_bytes((ROOT/'EXPERIMENT_RESULT.json').read_bytes())
with tarfile.open(ROOT/'far_range_20261127_01.tar.gz','w:gz') as archive:
    for name in ('range_evaluation','route_evidence','observer_video/navigation_1x.mp4','observer_video/ENCODING_REPORT.json',
                 'truth_trajectory.jsonl','STATIC_FIXTURES.json','CASE.json','REQUESTED_ROUTE.json','VISIBLE_DEMO_RESULT.json',
                 'RUNNER_RESULT.json','missions'):
        path=CASE/name
        if path.exists():
            archive.add(path,arcname=name)
(ROOT/'FINALIZED.json').write_text(json.dumps(dict(archive=str(ROOT/'far_range_20261127_01.tar.gz'),complete=True)))
print('Finalized evidence, video and trajectory')
