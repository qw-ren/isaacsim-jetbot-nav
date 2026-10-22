"""Selected local-backup package from finished immutable LIO navigation cases.

Full ROS CDR SQLite and native NPZ remain on cloud with SHA manifests. All
executed sources, parameters, summaries, IMU CSV, odometry and analysis exports
are included. Does not archive active cases or copy weights/assets/installers.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil
import tarfile


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda:f.read(1024*1024),b''):
            h.update(block)
    return h.hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--cases',nargs='+',required=True)
    parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args()
    root = Path('/home/admin/go2-warehouse')
    target = args.output.resolve()
    if target.parent != root/'artifacts' or target.exists():
        raise SystemExit('Require a new direct child of cloud artifacts')
    sources = []
    for name in args.cases:
        if Path(name).name!=name or not name.startswith('lio_nav_'):
            raise SystemExit('Invalid case name')
        p = root/'artifacts'/name
        if not (p/'RUNNER_RESULT.json').is_file():
            raise SystemExit('Case still active or missing: '+name)
        sources.append(p)
    target.mkdir()
    summary = []
    for source in sources:
        destination = target/source.name
        destination.mkdir()
        verified = 0
        for row in (source/'source_files.sha256').read_text().splitlines():
            h,name = row.split('  ',1)
            if digest(source/name)!=h:
                raise RuntimeError('Executed source changed: '+str(source/name))
            verified += 1
        files = sorted(p for p in source.rglob('*') if p.is_file() and '__pycache__' not in p.parts)
        manifest = ''.join(digest(p)+'  '+str(p.relative_to(source))+'\n' for p in files)
        (destination/'complete_cloud_files.sha256').write_text(manifest)
        excluded = []
        for p in files:
            relative = p.relative_to(source)
            if p.suffix=='.npz' or p.name.startswith('sensors.sqlite3'):
                excluded.append(str(relative))
                continue
            out = destination/relative
            out.parent.mkdir(parents=True,exist_ok=True)
            shutil.copy2(p,out)
        logs = destination/'external_component_logs'
        logs.mkdir()
        for name in re.findall(r'^Log: (.+)$',(source/'runner.log').read_text(),re.M):
            p = Path(name)
            if p.parent!=root/'logs':
                raise RuntimeError('Unexpected component log path')
            if p.is_file():shutil.copy2(p,logs/p.name)
        case = json.loads((source/'CASE.json').read_text())
        runtime = root/case['runtime']
        snapshot = destination/'runtime_source'
        for name in ['isaac_go2_ros2.py','ros2/go2_ros2_bridge.py','go2/go2_sensors.py',
                     'frame20ms_points.py','native_imu_adapter.py','route_trial_adapter.py','plane_audit.py','rtx_time_audit.py',
                     'noisy_route_adapter.py','imu_noise_model.py','live_rtx_points.py','NAV_COPY.json','NOISE_COPY.json']:
            p = runtime/name
            if p.is_file():
                out = snapshot/name
                out.parent.mkdir(parents=True,exist_ok=True)
                shutil.copy2(p,out)
        reference = destination/'map_reference'
        reference.mkdir()
        map_file = Path(case.get('map',str(root/'nav_stage2/maps/warehouse_dev_v2/map.yaml')))
        for p in [map_file,map_file.parent/'map.pgm',map_file.parent/'route.json']:
            if p.is_file():shutil.copy2(p,reference/p.name)
        shutil.copy2(root/'artifacts/m4_open_a_20261018/height_clipped/static_boxes.json',reference/'safety_boxes.json')
        summary.append(dict(case=source.name,cloud_files=len(files),source_files_verified=verified,
                            excluded_bulk=excluded,runner=json.loads((source/'RUNNER_RESULT.json').read_text())))
    for package in ['fastlio_reliable_20261021']:
        src = root/'nav_stage2/deps'/package
        if src.exists():
            out = target/'runtime_provenance'/package
            out.mkdir(parents=True)
            for name in ['PROVENANCE.json','reliable_transport.patch','files.sha256']:
                shutil.copy2(src/name,out/name)
    scope = dict(cases=summary,scope='selected reproducibility and telemetry; full CDR/native points remain cloud-only',
                 excludes=['full sensor SQLite','native raw NPZ','Isaac assets/image','weights and installer archives'])
    (target/'BACKUP_SCOPE.json').write_text(json.dumps(scope,indent=2)+'\n')
    files = sorted(p for p in target.rglob('*') if p.is_file())
    (target/'selected_files.sha256').write_text(''.join(digest(p)+'  '+str(p.relative_to(target))+'\n' for p in files))
    archive = target.with_suffix('.tar.gz')
    with tarfile.open(archive,'w:gz') as t:
        for p in [*files,target/'selected_files.sha256']:
            t.add(p,arcname=p.relative_to(target),recursive=False)
    print(json.dumps(dict(archive=str(archive),bytes=archive.stat().st_size,sha256=digest(archive),selected_files=len(files),cases=summary),indent=2),flush=True)


if __name__ == '__main__':
    main()
