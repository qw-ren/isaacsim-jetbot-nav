"""Archive this migration's actual files and supplemental backend; not a cloud image."""
import argparse,hashlib,json,shutil,tarfile
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();root=a.root.resolve();assert root==Path('/home/admin/jetbot-warehouse');assert a.output.parent==Path('/home/admin/go2-warehouse/artifacts')and not a.output.exists()
def digest(p):
 h=hashlib.sha256()
 with p.open('rb')as f:
  for chunk in iter(lambda:f.read(1024*1024),b''):h.update(chunk)
 return h.hexdigest()
active=[]
for proc in Path('/proc').glob('[0-9]*/cmdline'):
 try:
  args=[s.decode(errors='replace')for s in proc.read_bytes().split(b'\0')if s]
  if any(Path(s).name in ['run_sim.py','supervise.py','nav_odometry.py','velocity_gate.py','check_navigation.py','check_motion.py','audit_inputs.py','jetbot_audit_inputs.py']for s in args)and any(str(root)in s for s in args):active.append(proc.parent.name)
 except OSError:pass
assert not active,active
bundle=root/'estimator_runtime_backup';bundle.mkdir(exist_ok=False);(bundle/'bin').mkdir();(bundle/'lib').mkdir();shared=Path('/home/admin/go2-warehouse/nav_stage2/deps');primary=shared/'fastlivo_trial_20261022';binary=primary/'bin/fastlivo_mapping';assert digest(binary)=='8a8e61dd543befa5f9da26bdf2674af804f7d2d5a81542d9f1fb7a9cf11e958f';shutil.copy2(binary,bundle/'bin/fastlivo_mapping');shadow=[]
for src in [primary/'lib',shared/'fastlio_eval_20261019/lib']:
 for f in sorted(src.iterdir()):
  if not f.is_file():continue
  dst=bundle/'lib'/f.name
  if dst.exists():shadow.append(dict(name=f.name,second_source=str(f),identical=digest(f)==digest(dst)));continue
  shutil.copy2(f,dst,follow_symlinks=True)
for name in ['PROVENANCE.json','SOURCE_PINS.json','integration.patch','ADDITIONAL_LIBRARIES.json','LIBRARIES.json','files.sha256','complete_files.sha256']:
 if (primary/name).is_file():shutil.copy2(primary/name,bundle/name)
shutil.copytree(primary/'source',bundle/'source',ignore=shutil.ignore_patterns('.git','__pycache__','build','install','log','Log','PCD','devel'),symlinks=False)
(bundle/'LIBRARY_PRECEDENCE.json').write_text(json.dumps(dict(first_library_root=str(primary/'lib'),second_library_root=str(shared/'fastlio_eval_20261019/lib'),shadowed=shadow),indent=2)+'\n')
history=[]
for c in sorted((root/'artifacts').iterdir()):
 if not c.is_dir():continue
 row=dict(case=c.name)
 for name in ['RUNNER_RESULT.json','LAUNCH_FAILURE.json','INPUT_FORMAT_INVALID.json','MOTION_RESULT.json','NAVIGATION_RESULT.json','ODOM_FAULT.json','INPUT_AUDIT.json','INPUT_AUDIT_external_01.json','RESULT.json']:
  if (c/name).is_file():
   d=json.loads((c/name).read_text());row[name]={k:d[k]for k in ['passed','exit_code','error','phase','native_status','checks','scope','domain']if k in d}
 history.append(row)
(root/'EXPERIMENT_HISTORY.json').write_text(json.dumps(history,indent=2)+'\n')
(root/'BACKUP_SCOPE.json').write_text(json.dumps(dict(scope='actual JetBot source, original USD, resolved copies, prior map, all local trial evidence, pinned FAST-LIVO binary/source and extra libraries',excluded='Isaac Sim / ROS / CUDA / PyTorch installation and full NVIDIA asset library; shared base environment still required',active_workers=active,power_operation=False),indent=2)+'\n')
manifest=root/'BACKUP_FILES.sha256';assert not manifest.exists();files=sorted(f for f in root.rglob('*')if f.is_file());manifest.write_text(''.join(digest(f)+'  '+str(f.relative_to(root))+'\n'for f in files))
with tarfile.open(a.output,'w:gz',dereference=True)as archive:
 for f in [*files,manifest]:archive.add(f,arcname=str(f.relative_to(root)),recursive=False)
result=dict(archive=str(a.output),bytes=a.output.stat().st_size,sha256=digest(a.output),files=len(files),scope='BACKUP_SCOPE.json');result_file=a.output.with_name(a.output.name.replace('.tar.gz','_RESULT.json'));result_file.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
