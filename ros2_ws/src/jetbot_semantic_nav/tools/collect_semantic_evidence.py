"""Create exclusive, checksummed selected-case backups after sessions stop."""
import argparse
import hashlib
import io
import json
import tarfile
from pathlib import Path

p=argparse.ArgumentParser();p.add_argument('--artifacts',required=True,type=Path);p.add_argument('--output',required=True,type=Path)
p.add_argument('cases',nargs='+');a=p.parse_args();a.output.mkdir(exist_ok=False);archives=[]
for name in a.cases:
    case=a.artifacts/name
    if not(case/'RUNNER_RESULT.json').is_file():raise RuntimeError('Case still active or unfinalized: '+name)
    files=[f for f in case.iterdir()if f.is_file()and f.suffix in('.json','.jsonl','.log','.yaml','.sha256','.usda','.png')]
    for folder in('source','map','semantic_tasks'):
        files.extend(f for f in(case/folder).rglob('*')if f.is_file()and '__pycache__'not in f.parts)
    files.extend(f for f in(case/'local_costmap_diagnostic').glob('*.npz')if f.is_file())
    if(case/'GATE_REJECTED_PREDICTION.npz').is_file():files.append(case/'GATE_REJECTED_PREDICTION.npz')
    files=sorted(set(files));manifest=dict(case=name,scope='Top-level metadata/logs, source/map/task records, optional bounded local costmap diagnostics and first gate rejection NPZ; raw sensor NPZ and observer JPG sequence remain on cloud',files=[])
    target=a.output/(name+'_selected.tar.gz')
    with tarfile.open(target,'x:gz',compresslevel=3)as archive:
        for path in files:
            relative=str(path.relative_to(case));data=path.read_bytes()
            manifest['files'].append(dict(path=relative,bytes=len(data),sha256=hashlib.sha256(data).hexdigest()))
            info=tarfile.TarInfo(relative);info.size=len(data);archive.addfile(info,io.BytesIO(data))
        data=json.dumps(manifest,ensure_ascii=False,indent=2).encode()
        info=tarfile.TarInfo('SELECTED_MANIFEST.json');info.size=len(data);archive.addfile(info,io.BytesIO(data))
    archives.append(dict(file=target.name,bytes=target.stat().st_size,sha256=hashlib.sha256(target.read_bytes()).hexdigest(),members=len(manifest['files'])))
with(a.output/'ARCHIVES.json').open('x')as f:json.dump(dict(archives=archives),f,indent=2);f.write('\n')
print(json.dumps(archives,indent=2))
