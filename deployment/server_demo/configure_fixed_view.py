#!/usr/bin/env python3
"""Opt in to a fixed display camera without changing robot sensors or controls."""
import json
from pathlib import Path
import shutil

root = Path('/home/admin/jetbot-warehouse/code_deploy_20261120')
manager=root/'manage_jetbot.sh'
text=manager.read_text()
backup=manager.with_suffix('.sh.before_fixed_20261126')
if not backup.exists(): shutil.copy2(manager,backup)
text=text.replace("c['visualization']=dict(headless=headless,", "c['visualization']=dict(fixed_eye=view.get('fixed_eye'),fixed_target=view.get('fixed_target'),headless=headless,") if "fixed_eye=view.get('fixed_eye')" not in text else text
usd='/home/admin/isaacsim/extscache/omni.usd.libs-1.0.1+d02c707b.lx64.r.cp310'
needle=' export ROS_DOMAIN_ID=51\n'
if '# USD bootstrap for fresh tmux servers' not in text:
    text=text.replace(needle,needle+' # USD bootstrap for fresh tmux servers\n export PYTHONPATH="'+usd+':${PYTHONPATH:-}"\n export LD_LIBRARY_PATH="'+usd+'/bin:${LD_LIBRARY_PATH:-}"\n')
manager.write_text(text)
path = root/'run_sim.py'
source = path.read_text()
old = 'eye,target=observer_follow.update(position,yaw,float(world.current_time),observer_boxes+fixture.display_boxes)'
new = "eye,target=(view['fixed_eye'],view['fixed_target']) if view.get('fixed_eye') else observer_follow.update(position,yaw,float(world.current_time),observer_boxes+fixture.display_boxes)"
if old in source:
    backup = path.with_suffix('.py.before_fixed_20261126')
    if not backup.exists(): shutil.copy2(path,backup)
    source = source.replace(old,new)
    source = source.replace("follow=bool(view.get('follow',True)),", "follow=bool(view.get('follow',True)) and not bool(view.get('fixed_eye')),fixed_eye=view.get('fixed_eye'),fixed_target=view.get('fixed_target'),")
    compile(source,str(path),'exec')
    path.write_text(source)
elif new not in source:
    raise RuntimeError('Unexpected simulator source; inspect before patching')
for name in ('warehouse_vision_record_20261120.json','warehouse_vision_gui_20261120.json'):
    path=root/name
    backup=path.with_suffix('.json.before_fixed_20261126')
    if not backup.exists(): shutil.copy2(path,backup)
    data=json.loads(path.read_text())
    # Stay below the roof and on the route side of the pillar at y=3.4..3.8.
    data['visualization'].update(fixed_eye=[-1.5,9.,4.],fixed_target=[-1.5,6.7,.2])
    data['max_linear_m_s']=.4
    data['parameter_overrides'].update({
        'controller_server/ros__parameters/FollowPath/vx_samples':41,
        'controller_server/ros__parameters/FollowPath/vtheta_samples':31,
        'controller_server/ros__parameters/FollowPath/trans_stopped_velocity':.02})
    path.write_text(json.dumps(data,indent=2)+'\n')
print('Fixed display view configured; source and configuration backups preserved')
