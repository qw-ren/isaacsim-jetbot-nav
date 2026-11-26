#!/usr/bin/env bash
set -euo pipefail
if [[ $(id -u) -eq 0 ]]; then exec runuser -u admin -- bash "$0" "$@"; fi
ROOT=/home/admin/atongmu_deploy_20261120
id=${1:?usage: launch_demo.sh NEW_CASE_ID [gui|record]}
mode=${2:-record}
[[ "$id" =~ ^[A-Za-z0-9_]+$ && ${#id} -le 64 && ( "$mode" == gui || "$mode" == record ) ]]
[[ ! -e /home/admin/jetbot-warehouse/artifacts/$id ]] || { echo 'Case already exists; use a new ID'; exit 2; }
for session in jetbot-fastlivo-task atongmu-vision-20261120 "atongmu-demo-$id"; do
    if tmux has-session -t "=$session" 2>/dev/null; then
        echo "Existing session $session; inspect or stop it first"; exit 2
    fi
done
printf -v command '%q ' bash "$ROOT/demo_tools/demo_video.sh" "$id" "$mode"
tmux new-session -d -s "atongmu-demo-$id" "$command > $ROOT/logs/$id.demo.log 2>&1"
printf 'Started %s demonstration: %s\nLog: %s/logs/%s.demo.log\n' "$mode" "$id" "$ROOT" "$id"
if [[ "$mode" == record ]]; then
    printf 'MP4 after completion: /home/admin/jetbot-warehouse/artifacts/%s/observer_video/navigation_1x.mp4\n' "$id"
fi
