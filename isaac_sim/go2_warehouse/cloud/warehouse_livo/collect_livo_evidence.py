"""Selected evidence from finished cases; bulk CDR/NPZ and weights stay cloud-only."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil
import tarfile


def digest(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda:f.read(1024*1024),b''):h.update(block)
    return h.hexdigest()


def main():
    p=argparse.ArgumentParser();p.add_argument('--cases',nargs='+',required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    root=Path('/home/admin/go2-warehouse');target=a.output.resolve()
    if target.parent!=root/'artifacts' or target.exists():raise SystemExit('New direct artifacts child required')
    cases=[]
    for name in a.cases:
        if Path(name).name!=name or not name.startswith(('livo_rgb_','moving_')):raise ValueError('Invalid case')
        c=root/'artifacts'/name
        if not (c/'RUNNER_RESULT.json').is_file():
            audit=c/'POST_STOP_AUDIT.json'
            if not audit.is_file() or json.loads(audit.read_text()).get('remaining_case_processes')!=[]:
                raise ValueError('Active or missing navigation case')
        for src in c.glob('comparison_source_*'):
            if src.is_dir():
                tag=src.name.removeprefix('comparison_source_')
                if not (c/('LIVO_COMPARISON_RESULT_'+tag+'.json')).exists() and not (tag=='01' and (c/'LIVO_COMPARISON_RESULT.json').exists()):
                    raise ValueError('Comparison still active: '+str(c))
        cases.append(c)
    target.mkdir();summary=[]
    for c in cases:
        out=target/c.name;out.mkdir()
        full=sorted(f for f in c.rglob('*') if f.is_file() and '__pycache__' not in f.parts)
        (out/'complete_cloud_files.sha256').write_text(''.join(digest(f)+'  '+str(f.relative_to(c))+'\n' for f in full))
        exclusions=[];source_verified=0
        for row in (c/'source_files.sha256').read_text().splitlines():
            h,n=row.split('  ',1)
            if digest(c/n)!=h:raise ValueError('Changed executed source: '+n)
            source_verified+=1
        for f in full:
            rel=f.relative_to(c)
            if f.name.startswith('sensors.sqlite3') or f.suffix in ['.npz','.pt','.pth'] or ('follow_video' in rel.parts and f.name.startswith('frame_') and f.suffix=='.jpg'):
                exclusions.append(str(rel));continue
            dst=out/rel;dst.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(f,dst)
        logs=out/'external_component_logs';logs.mkdir()
        for name in re.findall(r'^Log: (.+)$',(c/'runner.log').read_text(),re.M):
            f=Path(name)
            if f.parent!=root/'logs':raise ValueError('Unexpected external log')
            if f.is_file():shutil.copy2(f,logs/f.name)
        config=json.loads((c/'CASE.json').read_text());m=Path(config['map'])
        maps=out/'map_reference';maps.mkdir()
        for f in [m,m.parent/'map.pgm',m.parent/'route.json',m.parent/'safety_boxes.json',m.parent/'topology_proposal.json']:
            if f.is_file():shutil.copy2(f,maps/f.name)
        summary.append(dict(case=c.name,source_files_verified=source_verified,excluded_bulk=exclusions,
                            runner=json.loads((c/('RUNNER_RESULT.json' if (c/'RUNNER_RESULT.json').exists() else 'POST_STOP_AUDIT.json')).read_text())))
    provenance=target/'runtime_provenance';provenance.mkdir()
    for name in ['PROVENANCE.json','SOURCE_PINS.json','integration.patch','ADDITIONAL_LIBRARIES.json','files.sha256','complete_files.sha256']:
        src=root/'nav_stage2/deps/fastlivo_trial_20261022'/name
        if src.is_file():shutil.copy2(src,provenance/name)
    metadata=target/'experiment_metadata';metadata.mkdir()
    for pattern in ['livo_selection_20261022*.json','livo_20261022_source_protection_*.json']:
        for src in sorted((root/'artifacts').glob(pattern)):
            if src.is_file():shutil.copy2(src,metadata/src.name)
    (target/'BACKUP_SCOPE.json').write_text(json.dumps(dict(cases=summary,
        scope='selected text sources, telemetry, accuracy exports, image samples, logs and provenance',
        cloud_only='complete ROS CDR, native point arrays and duplicate weights; full SHA manifests included',
        not_included='full Isaac image/assets; existing local stage1 weights/runtime library bundles are separate'),indent=2)+'\n')
    files=sorted(f for f in target.rglob('*') if f.is_file())
    (target/'selected_files.sha256').write_text(''.join(digest(f)+'  '+str(f.relative_to(target))+'\n' for f in files))
    archive=target.with_suffix('.tar.gz')
    with tarfile.open(archive,'w:gz') as t:
        for f in [*files,target/'selected_files.sha256']:t.add(f,arcname=f.relative_to(target),recursive=False)
    print(json.dumps(dict(archive=str(archive),bytes=archive.stat().st_size,sha256=digest(archive),selected_files=len(files),cases=summary),indent=2),flush=True)


if __name__=='__main__':main()
