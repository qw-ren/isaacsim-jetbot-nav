#!/usr/bin/env bash
# Independent LIO odometry -> existing AMCL/Nav2 and unchanged velocity gate.
set -eo pipefail
if [[ $(id -u) -eq 0 ]]; then exec runuser -u admin -- bash "$0" "$@"; fi
go2_lio_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
go2_root=/home/admin/go2-warehouse
go2_session=go2-lio-nav-task
case ${1:-help} in
 start)
  go2_id=${2:-$(date +%Y%m%d_%H%M%S)}
  go2_noise=${3:-clean};go2_route=${4:-short}
  [[ "$go2_id" =~ ^[A-Za-z0-9_-]+$ ]] || exit 2
  [[ "$go2_noise" == clean || "$go2_noise" == noise ]] || exit 2
  [[ "$go2_route" == short || "$go2_route" == long ]] || exit 2
  for go2_s in "$go2_session" go2-mainline-task go2-warehouse go2-scan-m1 go2-localization-m2 go2-obstacle-plan-m3 go2-velocity-guard go2-navigation-m1; do
   if tmux has-session -t "=$go2_s" 2>/dev/null; then echo "Existing session $go2_s; refusing duplicate"; exit 1; fi
  done
  if pgrep -u "$(id -u)" -f '(^|[ /])isaac_go2_ros2[.]py([ ]|$)' >/dev/null; then echo 'Existing simulator process';exit 1;fi
  go2_case="$go2_root/artifacts/lio_nav_$go2_id"
  test ! -e "$go2_case"
  if [[ "$go2_noise" == noise && ! -d "$go2_root/motion_lio_noise_20261021" ]];then
   python3 "$go2_lio_dir/prepare_noise_nav_copy.py" --target "$go2_root/motion_lio_noise_20261021"
  fi
  python3 - "$go2_case" "$go2_lio_dir" "$go2_noise" "$go2_route" <<'PY'
from pathlib import Path
import ast,hashlib,json,shutil,sys
p=Path(sys.argv[1]);s=Path(sys.argv[2]);(p/'simulation').mkdir(parents=True,exist_ok=False)
for name in ['warehouse_lio','warehouse_stage2']:
 shutil.copytree(s.parent/name,p/'source'/name,ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
for name,variable in [('make_localization_params.py','pose'),('seed_amcl_pose.py','d')]:
 v=p/'source/warehouse_stage2'/name;t=v.read_text();a=f"{variable}['source'].startswith('SLAM_')";assert t.count(a)==1;t=t.replace(a,f"{variable}['source'].startswith(('SLAM_', 'CONFIGURED_CAD_'))");ast.parse(t);v.write_text(t)
v=p/'source/warehouse_stage2/start_localization.sh';t=v.read_text();a='"$go2_dir/seed_amcl_pose.py"';assert t.count(a)==1;t=t.replace(a,'"$go2_dir/../warehouse_lio/seed_lio_localization.py"');a='"${go2_params%.yaml}_seed.json"';assert t.count(a)==1;t=t.replace(a,'"'+str(p/'localization_seed.json')+'"');v.write_text(t)
v=p/'source/warehouse_stage2/start_velocity_guard.sh';t=v.read_text();a='"$go2_script_dir/velocity_guard.py"';assert t.count(a)==1;t=t.replace(a,'"$go2_script_dir/../warehouse_lio/lio_map_velocity_guard.py"');v.write_text(t)
files=sorted(f for f in (p/'source').rglob('*') if f.is_file());(p/'source_files.sha256').write_text(''.join(hashlib.sha256(f.read_bytes()).hexdigest()+'  '+str(f.relative_to(p))+'\n' for f in files))
(p/'initial_pose.json').write_text(json.dumps(dict(source='CONFIGURED_CAD_scan_projection_known_spawn',x=2.2,y=-1.,yaw=0.))+'\n')
noise,route=sys.argv[3:5];r=Path('/home/admin/go2-warehouse');map_dir=r/'nav_stage2/maps'/('warehouse_lio_cad_v2_20261021' if route=='long' else 'warehouse_dev_v2')
assert (map_dir/'map.yaml').is_file()
binary=r/'nav_stage2/deps/fastlio_reliable_20261021/bin/fastlio_mapping';assert binary.is_file()
v=dict(runtime='motion_lio_noise_20261021' if noise=='noise' else 'motion_lio_nav_20261021',spawn=[2.,-1.,0.],odom_anchor=[2.,-1.,.4],goal=[5.,-1.],imu_noise_enabled=noise=='noise',imu_noise_seed=20261021,map=str(map_dir/'map.yaml'),route=str(map_dir/'route.json') if route=='long' else None,map_source='USD auxiliary CAD prior, not SLAM',scene='warehouse shelves only; no added obstacles',stable_defaults_modified=False,fastlio_binary=str(binary),fastlio_binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),reliable_lidar=True,guard_geometry_frame='map via AMCL; original guard limits unchanged')
if route=='long':v['planned_route_m']=json.loads((map_dir/'route.json').read_text())['planned_route_m']
(p/'CASE.json').write_text(json.dumps(v,indent=2)+'\n')
PY
  printf -v go2_cmd '%q ' timeout -s INT 1800 bash "$go2_case/source/warehouse_lio/run_nav_trial.sh" __run "$go2_case"
  printf -v go2_log '%q' "$go2_case/runner.log"
  tmux new-session -d -s "$go2_session" "$go2_cmd > $go2_log 2>&1"
  echo "$go2_case";;
 status)
  tmux list-panes -t "$go2_session:0" -F '#{session_name} pid=#{pane_pid} command=#{pane_current_command}' 2>/dev/null || true;;
 stop)
  python3 "$go2_lio_dir/request_trial_stop.py";;
 __run)
  go2_case=$2
  go2_lio_dir="$go2_case/source/warehouse_lio"
  go2_stage2="$go2_case/source/warehouse_stage2"
  mapfile -t go2_values < <(python3 - "$go2_case/CASE.json" <<'PY'
import json,sys
x=json.load(open(sys.argv[1]));print(x['runtime']);print(x['map']);print(x['route'] or '');print('True' if x['imu_noise_enabled'] else 'False')
PY
)
  go2_runtime="$go2_root/${go2_values[0]}";go2_map=${go2_values[1]};go2_route=${go2_values[2]};go2_noise=${go2_values[3]}
  go2_stops=();go2_children=()
  go2_cleanup() {
   go2_exit=$?
   trap - EXIT INT TERM
   for ((go2_i=${#go2_stops[@]}-1;go2_i>=0;go2_i--));do bash "${go2_stops[go2_i]}" stop || true;done
   for go2_pid in "${go2_children[@]}";do kill -INT -- "-$go2_pid" 2>/dev/null || true;done
   for ((go2_i=0;go2_i<50;go2_i++));do
    go2_alive=0
    for go2_pid in "${go2_children[@]}";do if kill -0 "$go2_pid" 2>/dev/null;then go2_alive=1;fi;done
    if [[ "$go2_alive" == 0 ]];then break;fi
    sleep .2
   done
   for go2_pid in "${go2_children[@]}";do if kill -0 "$go2_pid" 2>/dev/null;then kill -TERM -- "-$go2_pid" 2>/dev/null || true;fi;done
   for ((go2_i=0;go2_i<100;go2_i++));do
    if ! tmux has-session -t '=go2-warehouse' 2>/dev/null;then break;fi
    sleep .2
   done
   python3 - "$go2_case" "$go2_exit" <<'PY'
import datetime,json,subprocess,sys
from pathlib import Path
s=subprocess.run(['tmux','list-sessions','-F','#{session_name}'],capture_output=True,text=True).stdout.splitlines()
r=dict(exit_code=int(sys.argv[2]),time=datetime.datetime.now().astimezone().isoformat(),power_operation=False,remaining_sessions=[x for x in s if x!='go2-lio-nav-task'])
(Path(sys.argv[1])/'RUNNER_RESULT.json').write_text(json.dumps(r,indent=2)+'\n')
PY
   exit "$go2_exit"
  }
  trap go2_cleanup EXIT
  trap 'exit 130' INT TERM
  go2_start() { bash "$1" start;go2_stops+=("$1"); }
  source "$go2_stage2/nav_env.sh"
  bash "$go2_runtime/cloud/warehouse_stage1/manage_warehouse.sh" start --spawn-x 2 --spawn-y -1 --run-seconds 1650 --artifact-dir "$go2_case/simulation" freq=50 sensor.enable_camera=False camera_follow=False ++publish_map_to_odom=False ++lio_frame_trial_gate="$go2_root/artifacts/lio_plane_20261019/frame20ms_trial_gate.json" ++imu_noise_enabled="$go2_noise" ++imu_noise_seed=20261021
  go2_stops+=("$go2_runtime/cloud/warehouse_stage1/manage_warehouse.sh")
  go2_record_args=()
  if [[ -z "$go2_route" ]];then go2_record_args+=(--include-nav-cloud);fi
  setsid "$GO2_ISAAC_ROOT/python.sh" "$go2_lio_dir/record_lio_evaluation.py" --output "$go2_case/recording" "${go2_record_args[@]}" > "$go2_case/recording.log" 2>&1 &
  go2_children+=("$!")
  "$GO2_ISAAC_ROOT/python.sh" "$go2_lio_dir/wait_sensor_warmup.py" --output "$go2_case/sensor_warmup"
  setsid env LD_LIBRARY_PATH="$go2_root/nav_stage2/deps/fastlio_eval_20261019/lib:$LD_LIBRARY_PATH" "$go2_root/nav_stage2/deps/fastlio_reliable_20261021/bin/fastlio_mapping" --ros-args --params-file "$go2_lio_dir/fastlio_frame20ms_trial.yaml" -p reliable_lidar:=true -r __node:=fastlio_navigation_trial -r /tf:=/lio/evaluation_tf -r /tf_static:=/lio/evaluation_tf_static -r /Odometry:=/lio/odometry > "$go2_case/fastlio.log" 2>&1 &
  go2_children+=("$!")
  setsid "$GO2_ISAAC_ROOT/python.sh" "$go2_lio_dir/lio_nav_odometry.py" --output "$go2_case/estimated_odom" --anchor-xyz 2 -1 .4 > "$go2_case/estimated_odom.log" 2>&1 &
  go2_children+=("$!")
  GO2_SCAN_PARAMS="$go2_stage2/scan_converter_live.yaml" go2_start "$go2_stage2/manage_scan.sh"
  GO2_SAVED_MAP="$go2_map" GO2_INITIAL_POSE="$go2_case/initial_pose.json" go2_start "$go2_stage2/manage_localization.sh"
  python3 "$go2_lio_dir/wait_trial_ready.py" "$go2_case/localization_seed.json" --timeout 140
  GO2_PLAN_PARAMS="$go2_stage2/nav_obstacle_plan_safe.yaml" go2_start "$go2_stage2/manage_obstacle_plan.sh"
  GO2_GUARD_BOXES="$go2_root/artifacts/m4_open_a_20261018/height_clipped/static_boxes.json" go2_start "$go2_stage2/manage_velocity_guard.sh"
  GO2_NAV_PARAMS="$go2_stage2/nav_obstacle_region_arrival.yaml" GO2_NAV_BT="$go2_stage2/navigate_valid_path.xml" go2_start "$go2_stage2/manage_navigation.sh"
  if [[ -n "$go2_route" ]];then go2_target_args=(--route "$go2_route");else go2_target_args=(--goal 5 -1);fi
  "$GO2_ISAAC_ROOT/python.sh" "$go2_lio_dir/check_lio_navigation.py" --output "$go2_case/navigation" --map "$go2_map" "${go2_target_args[@]}" --adapter-output "$go2_case/estimated_odom";;
 *) echo 'Usage: run_nav_trial.sh start [unique_case_id] [clean|noise] [short|long] | status | stop';;
esac
