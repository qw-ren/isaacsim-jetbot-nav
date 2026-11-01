#!/usr/bin/env bash
set -euo pipefail
if [[ $(id -u) -eq 0 ]];then exec runuser -u admin -- bash "$0" "$@";fi
jetbot_root=/home/admin/jetbot-warehouse
jetbot_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
case ${1:-status} in
start)
 [[ ${2:?unique id} =~ ^[A-Za-z0-9_]+$ ]] || exit 2
 if tmux has-session -t =jetbot-fastlivo-task 2>/dev/null || pgrep -u "$(id -u)" -f '(^|[ /])(isaac_go2_ros2|run_sim)[.]py([ ]|$)' >/dev/null;then echo 'Existing simulation; inspect first';exit 1;fi
 jetbot_case="$jetbot_root/artifacts/$2"
 mkdir -p "$jetbot_root/artifacts"
 export PYTHONPATH="/home/admin/isaacsim/extscache/omni.usd.libs-1.0.1+d02c707b.lx64.r.cp310:${PYTHONPATH:-}"
 export LD_LIBRARY_PATH="/home/admin/isaacsim/extscache/omni.usd.libs-1.0.1+d02c707b.lx64.r.cp310/bin:${LD_LIBRARY_PATH:-}"
 jetbot_scale=$(python3 - "${6:-}" <<'PY_SCALE'
import json,sys
print(json.load(open(sys.argv[1])).get('robot_scale',1.)if sys.argv[1]else 1.)
PY_SCALE
)
 /home/admin/isaacsim/kit/python/bin/python3 "$jetbot_dir/prepare_scene.py" --source "$jetbot_root/source_scene/jetbot_sim.usd" --case "$jetbot_case" --assets /home/admin/isaacsim_assets/Assets/Isaac/4.5/Isaac --robot-scale "$jetbot_scale"
 mkdir "$jetbot_case/source"
 cp "$jetbot_dir/"*.py "$jetbot_case/source/"
 python3 "$jetbot_dir/validate_actuator.py" --source "$jetbot_case/source/sensor_bridge.py" --output "$jetbot_case/ACTUATOR_WATCHDOG_VALIDATION.json" > "$jetbot_case/actuator_validation.log"
 cp "$jetbot_dir/extension_contract.json" "$jetbot_dir/semantic_places.json" "$jetbot_case/source/"
 cp "$jetbot_dir/manage_jetbot.sh" "$jetbot_dir/nav2.yaml" "$jetbot_dir/livo_base.yaml" "$jetbot_dir/navigate.xml" "$jetbot_dir/navigate_through.xml" "$jetbot_case/source/"
 python3 - "$jetbot_case" "${3:-35}" "${4:-sensors}" "${5:-stationary}" "${6:-}" <<'PY'
from pathlib import Path
import hashlib,json,sys,shutil
p=Path(sys.argv[1]);duration=float(sys.argv[2]);assert 0<duration<=600
assert sys.argv[3]in ('sensors','livo','nav2','operator') and sys.argv[4]in ('stationary','motion')
assert not(sys.argv[3]in ('nav2','operator')and sys.argv[4]=='motion'), 'Nav2 gate and manual motion must not publish together'
(p/'CASE.json').write_text(json.dumps(dict(duration_sim_s=duration,ros_domain_id=51,simulation_only=True,backend='FAST-LIVO2 migration',initial_mode='native operator mission session'if sys.argv[3]=='operator'else'native Nav2 configured route'if sys.argv[3]=='nav2'else sys.argv[4],backend_enabled=sys.argv[3]in ('livo','nav2','operator'),motion_enabled=sys.argv[4]=='motion',nav2_enabled=sys.argv[3]in ('nav2','operator'),operator_enabled=sys.argv[3]=='operator'),indent=2)+'\n')
if sys.argv[5]:
 navigation=json.loads(Path(sys.argv[5]).read_text());assert 0<len(navigation['goals'])<=20
 assert all(len(g)==3 and all(isinstance(v,(float,int)) and __import__('math').isfinite(v) for v in g) for g in navigation['goals'])
 assert 0<float(navigation.get('timeout_wall_s',120))<=1800
 c=json.loads((p/'CASE.json').read_text());c['navigation']=navigation;c['runtime_wall_s']=max(600.,float(navigation.get('timeout_wall_s',120))+100.);(p/'CASE.json').write_text(json.dumps(c,indent=2)+'\n')
if sys.argv[3]in ('nav2','operator'):
 source=Path(json.loads((p/'CASE.json').read_text()).get('navigation',{}).get('prior_map_dir','/home/admin/jetbot-warehouse/map_prior_20261023_01')).resolve()
 assert source==Path('/home/admin/jetbot-warehouse/map_prior_20261023_01')or source.parent==Path('/home/admin/jetbot-warehouse/map_candidates'), 'Only original or explicit isolated prior candidates'
 proof=json.loads((source/'MAP_PROVENANCE.json').read_text());assert proof['preflight_passed'];shutil.copytree(source,p/'map')
c=json.loads((p/'CASE.json').read_text())
if c.get('navigation',{}).get('localization_prior_dir'):
 assert float(c['navigation'].get('robot_scale',1.))==3., 'Packaged localization prior requires the 3x JetBot configuration'
 localization=Path(c['navigation']['localization_prior_dir']).resolve()
 assert localization.parent==Path('/home/admin/jetbot-warehouse/map_candidates'), 'Isolated localization candidates only'
 proof=json.loads((localization/'MAP_PROVENANCE.json').read_text())
 assert proof.get('source_geometry_unchanged')is True and proof.get('truth_or_trajectory_input')is False
 assert proof.get('type')=='visible CAD mesh surface projection for localization only'
 assert 0<proof['occupied_cells']<proof['grid_size'][0]*proof['grid_size'][1]<8_000_000
 assert proof['height_band_chassis_m']==[.03,.40]
 # Validate the base scene/prior before declared static fixtures are applied.
 assert hashlib.sha256((p/'jetbot_resolved.usda').read_bytes()).hexdigest()==proof['source_scene_sha256'], 'Localization prior does not match the prepared base scene'
 assert hashlib.sha256((p/'map/map.pgm').read_bytes()).hexdigest()==proof['reference_map_sha256'], 'Localization prior does not match the base planning map'
 shutil.copytree(localization,p/'localization_map')
c=json.loads((p/'CASE.json').read_text());joint=json.loads((p/'JOINT_LIMITS.json').read_text());variant=json.loads((p/'ROBOT_VARIANT.json').read_text());scale=variant['scale'];c['robot_variant']=variant;c['motion_limits']=dict(max_linear_m_s=min(float(c.get('navigation',{}).get('max_linear_m_s',.37)),joint['straight_limit_m_s']),max_angular_rad_s=.6,wheel_radius_m=joint['wheel_radius_m'],wheel_base_m=joint['wheel_base_m'],footprint_radius_m=(variant['footprint_bound_radius_m']if scale!=1 else .16),scan_clearance_m=(variant['footprint_bound_radius_m']+.06 if scale!=1 else .20),max_wheel_rad_s=min(v['max_rad_s']for v in joint['joints']),joint_limits_modified=False)
if scale!=1:
 nav=p/'source/nav2.yaml';text=nav.read_text();radius=c['motion_limits']['scan_clearance_m'];speed=c['motion_limits']['max_linear_m_s']
 for before,after in [('max_vel_x: 0.37',f'max_vel_x: {speed}'),('max_speed_xy: 0.37',f'max_speed_xy: {speed}'),('robot_radius: 0.16',f'robot_radius: {radius}'),('inflation_radius: 0.55','inflation_radius: 0.75'),('forward_prune_distance: 1.0','forward_prune_distance: 2.0'),('acc_lim_x: 0.40','acc_lim_x: 0.80'),('decel_lim_x: -0.40','decel_lim_x: -1.0')]:text=text.replace(before,after)
 nav.write_text(text)
 bt=p/'source/navigate_through.xml';bt.write_text(bt.read_text().replace('radius="0.60"','radius="0.80"'))
view=dict(c.get('navigation',{}).get('visualization',{}));headless=view.get('headless',False);assert type(headless)is bool
record=view.get('record_video',False);assert type(record)is bool
c['visualization']=dict(headless=headless,follow=view.get('follow',True),record_video=record,observer_camera='/World/JetbotObserverCamera')
if not headless:
 sys.path.insert(0,str(p/'source'));from desktop_env import discover_desktop
 c['visualization'].update(discover_desktop())
(p/'CASE.json').write_text(json.dumps(c,indent=2)+'\n')
(p/'source_files.sha256').write_text(''.join(hashlib.sha256(f.read_bytes()).hexdigest()+'  '+str(f.relative_to(p))+'\n'for f in sorted((p/'source').glob('*'))if f.is_file()))
PY
 printf -v jetbot_cmd '%q ' bash "$jetbot_case/source/manage_jetbot.sh" __run "$jetbot_case"
 printf -v jetbot_log '%q' "$jetbot_case/runner.log"
 tmux new-session -d -s jetbot-fastlivo-task "$jetbot_cmd > $jetbot_log 2>&1"
 echo "$jetbot_case";;
__run)
 jetbot_case=$2
 set +u
 source /home/admin/go2-warehouse/nav_stage2/cloud/warehouse_stage2/nav_env.sh
 set -u
 export ROS_DOMAIN_ID=51
 /home/admin/isaacsim/python.sh "$jetbot_case/source/prepare_fixture_prior.py" --case "$jetbot_case"
 exec python3 "$jetbot_case/source/supervise.py" --case "$jetbot_case";;
stop)
 python3 - "$jetbot_root" <<'PY_STOP'
import json,os,signal,subprocess,sys,time
from pathlib import Path
root=Path(sys.argv[1]);v=subprocess.run(['tmux','list-panes','-t','jetbot-fastlivo-task','-F','#{pane_pid}'],capture_output=True,text=True)
if v.returncode:print('No managed JetBot session');raise SystemExit(0)
pane=int(v.stdout.strip());pending=[pane];matches=[]
while pending:
 candidate=pending.pop()
 try:
  args=Path(f'/proc/{candidate}/cmdline').read_bytes().decode().split('\0')
  descendants=Path(f'/proc/{candidate}/task/{candidate}/children').read_text().split();pending.extend(map(int,descendants))
 except FileNotFoundError:continue
 if any(Path(x).name=='supervise.py'for x in args):matches.append((candidate,args))
assert len(matches)==1, 'Expected exactly one owned supervisor beneath tmux pane'
pid,args=matches[0]
case=Path(args[args.index('--case')+1]).resolve();assert case.parent==root/'artifacts'
(case/'STOP_REQUESTED.json').write_text(json.dumps(dict(time=time.time(),requested_normal_stop=True,power_operation=False))+'\n');os.kill(pid,signal.SIGINT);print('Normal stop requested',case)
PY_STOP
 ;;
status)
 tmux list-panes -t jetbot-fastlivo-task -F '#{session_name} pid=#{pane_pid} command=#{pane_current_command}' 2>/dev/null || true;;
*) echo 'start UNIQUE_ID SIM_SECONDS [sensors|livo|nav2|operator] [stationary|motion] | status | stop';exit 2;;
esac
