#!/usr/bin/env python3
"""Run offline component checks in separate processes, without ROS or Isaac launch."""
import argparse,ast,json,os,re,subprocess,sys,xml.etree.ElementTree as ET
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
SUITES=[
 ('navigation_visual_map',['-m','unittest','discover','-s','ros2_ws/src/jetbot_semantic_nav/test','-q']),
 ('perception',['perception/tests/run_all.py']),
 ('capture_transform',['-m','unittest','discover','-s','deployment/server_demo','-p','test_capture_route.py','-q']),
 ('warehouse_config',['isaac_sim/warehouse/tests/test_config.py']),
 ('warehouse_determinism',['isaac_sim/warehouse/tests/test_determinism.py']),
 ('warehouse_negative',['isaac_sim/warehouse/tests/test_validator_negative.py'])]

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',required=True,type=Path);a=p.parse_args()
 a.output=a.output.resolve();a.output.mkdir(parents=True,exist_ok=False)
 env=os.environ.copy();env['PYTHONPATH']=os.pathsep.join([str(ROOT/'perception'),str(ROOT/'ros2_ws/src/jetbot_semantic_nav'),env.get('PYTHONPATH','')])
 result=dict(scope='offline source merge and component regression; no camera inference or Isaac navigation',python=sys.executable,suites=[],checks={})
 for name,args in SUITES:
  r=subprocess.run([sys.executable,*args],cwd=ROOT,env=env,capture_output=True,text=True)
  text=r.stdout+r.stderr;(a.output/(name+'.log')).write_text(text)
  m=re.search(r'(\d+) passed, (\d+) failed',text)or re.search(r'Ran (\d+) tests',text)
  row=dict(name=name,exit_code=r.returncode,passed=r.returncode==0,checks_reported=int(m[1])if m else None,
           skipped_asset_checks=3 if 'SKIP source_root absent'in text else 0)
  result['suites'].append(row);print(json.dumps(row),flush=True)
  if r.returncode:result['checks']['component_failure']=name;break
 tracked=subprocess.check_output(['git','ls-files','-z'],cwd=ROOT).decode().strip('\0').split('\0')
 syntax=[]
 for name in tracked:
  if name.endswith('.py'):ast.parse((ROOT/name).read_text(),filename=name);syntax.append(name)
 result['checks']['python_syntax_files']=len(syntax)
 packages={}
 for name in tracked:
  if name.endswith('/package.xml'):
   pkg=ET.parse(ROOT/name).getroot().findtext('name')
   if pkg in packages:raise ValueError('Duplicate ROS package: '+pkg)
   packages[pkg]=name
 result['checks']['unique_ros_packages']=packages
 shell=[name for name in tracked if name.endswith('.sh')]
 for name in shell:subprocess.run(['bash','-n',str(ROOT/name)],check=True)
 result['checks']['shell_syntax_files']=len(shell)
 generated=[]
 for label in ['first','repeat']:
  target=a.output/('scene_1001_medium_'+label+'.json')
  r=subprocess.run([sys.executable,'isaac_sim/warehouse/scripts/make_scene.py','--seed','1001','--density','medium','--out',str(target)],cwd=ROOT,env=env,capture_output=True,text=True)
  (a.output/('scene_'+label+'.log')).write_text(r.stdout+r.stderr)
  if r.returncode:raise RuntimeError('Scene generation failed; see log')
  generated.append(json.loads(target.read_text()))
 first,second=generated
 result['checks']['scene_reproducible']=first==second
 result['checks']['scene_layout_hash']=first['layout_hash']
 result['checks']['scene_counts']={kind:sum(o['kind']==kind for o in first['objects'])for kind in ('A','B')}
 result['checks']['scene_matches_upstream_example']=first['layout_hash']==json.loads((ROOT/'isaac_sim/warehouse/scenes/examples/scene_1001_medium.json').read_text())['layout_hash']
 result['checks']['scene_blocked_aisles']=first['blocked_aisles']
 result['passed']=(len(result['suites'])==len(SUITES)and all(r['passed']for r in result['suites'])and first==second and result['checks']['scene_matches_upstream_example'])
 result['checks_reported']=sum(r['checks_reported']or 0 for r in result['suites'])
 (a.output/'RESULT.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
 print(json.dumps(dict(passed=result['passed'],checks_reported=result['checks_reported'],output=str(a.output))))
 return 0 if result['passed']else 1

if __name__=='__main__':raise SystemExit(main())
