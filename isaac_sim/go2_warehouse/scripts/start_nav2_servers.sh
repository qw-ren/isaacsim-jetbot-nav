#!/usr/bin/env bash
set -e

# 一键启动 Nav2 核心节点并激活生命周期（map, planner, controller, behavior_server, bt_navigator）
# 使用方法：
#   chmod +x scripts/start_nav2_servers.sh
#   ./scripts/start_nav2_servers.sh

ROS_SETUP="/opt/ros/humble/setup.bash"
PARAMS="/home/liswnux/isaac-go2-ros2/nav2/nav2_params.yaml"
MAP_YAML="/home/liswnux/isaac-go2-ros2/map/sim_map.yaml"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
LOGDIR="${REPO_ROOT}/logs"
RUNDIR="${REPO_ROOT}/run"

mkdir -p "$LOGDIR" "$RUNDIR"

source "$ROS_SETUP"

wait_for_node() {
  name="$1"
  timeout_s=${2:-10}
  interval=0.5
  end=$(( $(date +%s) + timeout_s ))
  while [ $(date +%s) -lt $end ]; do
    if ros2 node list 2>/dev/null | grep -q "$name"; then
      return 0
    fi
    sleep $interval
  done
  return 1
}

lifecycle_state() {
  node_name="$1"
  ros2 lifecycle get "/${node_name}" 2>/dev/null | tail -n 1 | awk '{print $1}'
}

start_and_activate() {
  node_exec="$1"
  node_name="$2"
  extra_args="$3"
  log="${LOGDIR}/${node_name}.log"

  echo "Starting ${node_name} ..."
  # keep logs visible in the terminal and mirror them to a file
  eval "stdbuf -oL -eL ${node_exec} ${extra_args} 2>&1 | tee -a \"${log}\" &"
  pid=$!
  echo "$pid" > "${RUNDIR}/${node_name}.pid"

  # wait for node to register
  if wait_for_node "$node_name" 15; then
    state="$(lifecycle_state "$node_name")"
    if [ "$state" = "active" ]; then
      echo "${node_name} is already active, skipping lifecycle transitions."
    else
      if [ "$state" != "inactive" ] && [ "$state" != "unconfigured" ] && [ -n "$state" ]; then
        echo "${node_name} is in state ${state}, trying configure -> activate."
      fi
      echo "Configuring ${node_name} ..."
      ros2 lifecycle set /${node_name} configure
      echo "Activating ${node_name} ..."
      ros2 lifecycle set /${node_name} activate
    fi
  else
    echo "Warning: ${node_name} did not appear in ros2 node list (see ${log})"
  fi
}

cleanup_stale_nav2() {
  patterns=(
    'nav2_map_server map_server --ros-args -p yaml_filename:=/home/liswnux/isaac-go2-ros2/map/sim_map.yaml'
    'nav2_planner planner_server --ros-args --params-file /home/liswnux/isaac-go2-ros2/nav2/nav2_params.yaml'
    'nav2_controller controller_server --ros-args --params-file /home/liswnux/isaac-go2-ros2/nav2/nav2_params.yaml'
    'nav2_behaviors behavior_server --ros-args --params-file /home/liswnux/isaac-go2-ros2/nav2/nav2_params.yaml'
    'nav2_bt_navigator bt_navigator --ros-args --params-file /home/liswnux/isaac-go2-ros2/nav2/nav2_params.yaml'
  )

  for pattern in "${patterns[@]}"; do
    pkill -TERM -f "$pattern" 2>/dev/null || true
  done
  sleep 2
  for pattern in "${patterns[@]}"; do
    pkill -KILL -f "$pattern" 2>/dev/null || true
  done
}

cleanup_stale_nav2

# map_server (map yaml)
start_and_activate "ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=${MAP_YAML} -p use_sim_time:=true" "map_server"

# planner_server
start_and_activate "ros2 run nav2_planner planner_server --ros-args --params-file ${PARAMS} -p use_sim_time:=true" "planner_server"

# controller_server (includes local_costmap)
start_and_activate "ros2 run nav2_controller controller_server --ros-args --params-file ${PARAMS} -p use_sim_time:=true" "controller_server"

# behavior_server
start_and_activate "ros2 run nav2_behaviors behavior_server --ros-args --params-file ${PARAMS} -p use_sim_time:=true" "behavior_server"

# bt_navigator
start_and_activate "ros2 run nav2_bt_navigator bt_navigator --ros-args --params-file ${PARAMS} -p use_sim_time:=true" "bt_navigator"

# summary
echo "\n=== Lifecycle states ==="
for n in map_server planner_server controller_server behavior_server bt_navigator; do
  echo "--- $n ---"
  ros2 lifecycle get /$n 2>/dev/null || echo "(not available)"
done

echo "Logs: ${LOGDIR}  PIDs: ${RUNDIR}"

echo "Done. Keep this terminal open to watch logs. Ctrl+C here stops the Nav2 stack."

wait

