"""Submit one immutable native-goal request to an existing bounded operator case. No ROS/velocity publisher."""
import argparse,json,math,os,sys,fcntl,time,hashlib
from pathlib import Path
if os.geteuid()==0:os.execvp('runuser',['runuser','-u','admin','--','python3',str(Path(__file__).resolve()),*sys.argv[1:]])
p=argparse.ArgumentParser();p.add_argument('--case',type=Path,required=True);p.add_argument('--id',required=True);p.add_argument('--x',type=float,required=True);p.add_argument('--y',type=float,required=True);p.add_argument('--yaw',type=float,required=True);p.add_argument('--timeout',type=float,default=180.);a=p.parse_args();case=a.case.resolve()
assert case.parent==Path('/home/admin/jetbot-warehouse/artifacts')and case.is_dir(), 'Not an owned JetBot case'
assert 0<len(a.id)<=64 and a.id.replace('_','').isalnum(), 'Use a new alphanumeric/underscore mission id'
assert all(math.isfinite(v)for v in[a.x,a.y,a.yaw,a.timeout])and 0<a.timeout<=1800
cfg=json.loads((case/'CASE.json').read_text());assert cfg.get('operator_enabled'), 'Case is not an operator session'
assert not(case/'RUNNER_RESULT.json').exists(), 'Session already finished'
ready=json.loads((case/'OPERATOR_READY.json').read_text());assert ready['ready']and ready['case']==str(case)
args=Path('/proc/'+str(ready['pid'])+'/cmdline').read_bytes().decode().split('\0');assert str(case)in args and any(Path(v).name=='operator_session.py'for v in args), 'Stale operator session'
with(case/'submit.lock').open('a')as lock:
 fcntl.flock(lock,fcntl.LOCK_EX);status=json.loads((case/'OPERATOR_STATUS.json').read_text());assert status['state']=='IDLE', 'Operator is not idle: '+status['state'];directory=case/'mission_requests';pending=[v for v in directory.glob('*.json')if not(case/'missions'/v.stem/'RESULT.json').exists()];assert not pending, 'Another mission is queued or running'
 target=directory/(a.id+'.json');assert not target.exists()and not(case/'missions'/a.id).exists(), 'Mission id already used'
 request=dict(id=a.id,goal=[a.x,a.y,a.yaw],timeout_wall_s=a.timeout,submitted_epoch_s=time.time(),submit_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),native_action='NavigateToPose',truth_input=False,velocity_publisher=False)
 temporary=directory/(a.id+'.'+str(os.getpid())+'.tmp')
 with temporary.open('x')as f:json.dump(request,f,indent=2);f.write('\n')
 temporary.replace(target)
print(json.dumps(dict(queued=True,request=str(target),result=str(case/'missions'/a.id/'RESULT.json'),ssh_disconnect_does_not_cancel=True),indent=2))
