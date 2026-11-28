"""Bounded native Nav2 approach trial with synchronized RGB-D and observer video."""
import json
import math
import os
from pathlib import Path
import subprocess
import time

ROOT = Path('/home/admin/atongmu_deploy_20261120')
CODE = Path('/home/admin/jetbot-warehouse/code_deploy_20261120')
TOOLS = ROOT / 'far_range_20261127'
CASE = Path('/home/admin/jetbot-warehouse/artifacts/far_range_20261127_01')


def run(*args):
    subprocess.run(list(map(str, args)), check=True)


def read(path):
    try:
        return json.loads(path.read_text())
    except (FileNotFoundError, json.JSONDecodeError):
        return {}


def wait(predicate, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if (CASE/'RUNNER_RESULT.json').exists():
            raise RuntimeError('simulation ended; inspect runner results')
        if (CASE/'ODOM_FAULT.json').exists() or read(CASE/'GATE_PROGRESS.json').get('fault'):
            raise RuntimeError('odometry/velocity gate fault')
        value = predicate()
        if value:
            return value
        time.sleep(1)
    raise TimeoutError('bounded experiment wait expired')


def main():
    if os.geteuid() == 0:
        os.execvp('runuser', ['runuser','-u','admin','--','python3',str(Path(__file__).resolve())])
    if CASE.exists():
        raise FileExistsError(CASE)
    cfg = read(CODE/'warehouse_vision_record_20261120.json')
    cfg.update(timeout_wall_s=1800, scope='Known USD box approach in existing north rack aisle; truth only for offline evaluation',
               static_fixtures=[dict(kind='A',center=[-8.,23.])], obstacles=[])
    cfg['visualization'] = dict(headless=False,follow=True,record_video=True)
    # The first trial recorded controller_probe for diagnosis; its 600-wall-second
    # cap ended the supervisor before the final mission. Future repeats omit this
    # optional score recorder. Safety gate and localization probes remain enabled.
    cfg['extensions']['controller_probe'] = False
    config = TOOLS/'experiment_config.json'
    config.write_text(json.dumps(cfg,indent=2))
    report = dict(passed=False, missions=[], case=str(CASE), error=None)
    capture = None
    try:
        run('bash',CODE/'manage_jetbot.sh','start',CASE.name,'600','operator','stationary',config)
        run('tmux','new-session','-d','-s','atongmu-vision-20261120',
            'bash '+str(ROOT/'start_vision.sh')+' > '+str(TOOLS/'vision.log')+' 2>&1')
        capture_log = (TOOLS/'capture.log').open('w')
        env = os.environ.copy()
        env['ROS_DOMAIN_ID'] = '51'
        capture = subprocess.Popen(['/home/admin/venvs/atongmu_ros/bin/python',str(TOOLS/'capture_route.py'),
            '--case',str(CASE),'--interval','1.','--timeout','1900','--raw-depth'],stdout=capture_log,stderr=subprocess.STDOUT,env=env)
        def ready():
            return (read(CASE/'OPERATOR_READY.json') and read(CASE/'OPERATOR_STATUS.json').get('state')=='IDLE'
                    and read(CASE/'PROGRESS.json').get('sim_s',0)>=10)
        wait(ready,360)
        # Previously catalogued north rack aisle at x=-8. Target remains ahead at y=23.
        goals = [(0,7,math.pi/2),(-8,7,math.pi),(-8,10,math.pi/2),
                 (-8,14,math.pi/2),(-8,18,math.pi/2),(-8,20,math.pi/2)]
        (CASE/'REQUESTED_ROUTE.json').write_text(json.dumps(dict(frame_id='map',waypoints=goals,
            source='existing warehouse_aisle_video43 route; box approach stops 3m before declared target'),indent=2))
        for i,(x,y,yaw) in enumerate(goals):
            mid='range_%02d'%i
            run('python3',CODE/'submit_goal.py','--case',CASE,'--id',mid,'--x',x,'--y',y,'--yaw',yaw,'--timeout','360')
            result=wait(lambda:read(CASE/'missions'/mid/'RESULT.json'),375)
            report['missions'].append(result)
            (CASE/'EXPERIMENT_PROGRESS.json').write_text(json.dumps(report,indent=2))
            if not result.get('passed'):
                raise RuntimeError('native navigation failed: '+mid)
            wait(ready,20)
            # Static dwell for observations; wall wait is bounded and pose remains Nav2 controlled.
            start=read(CASE/'PROGRESS.json').get('sim_s',0)
            wait(lambda:read(CASE/'PROGRESS.json').get('sim_s',0)>=start+4,60)
        report['passed']=True
    except Exception as exc:
        report['error']=str(exc)
    finally:
        if CASE.exists():
            (CASE/'VISIBLE_DEMO_RESULT.json').write_text(json.dumps(report,indent=2))
        if capture:
            try:
                report['capture_exit']=capture.wait(timeout=20)
            except subprocess.TimeoutExpired:
                capture.terminate()
                report['capture_exit']=capture.wait(timeout=10)
        run('bash',ROOT/'stop_demo.sh')
        for _ in range(45):
            if (CASE/'RUNNER_RESULT.json').exists():
                break
            time.sleep(1)
        if (CASE/'observer_video/frames.csv').exists():
            try:
                run('python3',TOOLS/'encode_observer_video.py','--case',CASE,'--speed','1')
            except Exception as exc:
                report['encoding_error']=str(exc)
        (TOOLS/'EXPERIMENT_RESULT.json').write_text(json.dumps(report,indent=2))
    return 0 if report['passed'] else 1


if __name__=='__main__':
    raise SystemExit(main())
