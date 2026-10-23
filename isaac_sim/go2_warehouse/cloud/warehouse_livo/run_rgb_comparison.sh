#!/usr/bin/env bash
set -eo pipefail
if [[ $(id -u) -eq 0 ]];then exec runuser -u admin -- bash "$0" "$@";fi
go2_here=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
go2_case=${2:?Captured case directory}
go2_tag=${3:-01}
go2_begin=${4:-3}
[[ "$go2_tag" =~ ^[0-9]+$ ]] || exit 2
case ${1:-} in
 start)
  test -f "$go2_case/RUNNER_RESULT.json"
  if tmux has-session -t '=go2-livo-comparison' 2>/dev/null;then echo 'Comparison already active';exit 1;fi
  python3 - "$go2_case" "$go2_here" "$go2_tag" <<'PY'
from pathlib import Path
import hashlib,shutil,sys
c=Path(sys.argv[1]);source=Path(sys.argv[2]).parent;p=c/('comparison_source_'+sys.argv[3])
p.mkdir(exist_ok=False)
for name in ['warehouse_lio','warehouse_livo','warehouse_stage2']:
 shutil.copytree(source/name,p/name,ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
(c/('comparison_source_'+sys.argv[3]+'.sha256')).write_text(''.join(hashlib.sha256(f.read_bytes()).hexdigest()+'  '+str(f.relative_to(c))+'\n' for f in sorted(p.rglob('*')) if f.is_file()))
PY
  printf -v go2_command '%q ' timeout -s INT 1800 bash "$go2_case/comparison_source_$go2_tag/warehouse_livo/run_rgb_comparison.sh" __run "$go2_case" "$go2_tag" "$go2_begin"
  printf -v go2_log '%q' "$go2_case/comparison_$go2_tag.log"
  tmux new-session -d -s go2-livo-comparison "$go2_command > $go2_log 2>&1"
  echo "Comparison started: $go2_case";;
 __run)
  go2_finish() {
   go2_exit=$?
   trap - EXIT INT TERM
   python3 - "$go2_case" "$go2_exit" "$go2_tag" <<'PY'
from pathlib import Path
import datetime,json,sys
(Path(sys.argv[1])/('LIVO_COMPARISON_RESULT_'+sys.argv[3]+'.json')).write_text(json.dumps(dict(exit_code=int(sys.argv[2]),time=datetime.datetime.now().astimezone().isoformat(),truth_replayed=False,power_operation=False),indent=2)+'\n')
PY
   exit "$go2_exit"
  }
  trap go2_finish EXIT
  trap 'exit 130' INT TERM
  bash "$go2_here/run_livo_replay.sh" "$go2_case" "$go2_case/replay_livo_$go2_tag" 1 0 "$go2_begin"
  bash "$go2_here/run_lio_matched_replay.sh" "$go2_case/recording/sensors.sqlite3" "$go2_case/replay_lio_$go2_tag" /lio/imu "$go2_begin"
  bash "$go2_here/run_livo_replay.sh" "$go2_case" "$go2_case/replay_livo_no_image_$go2_tag" 0 0 "$go2_begin"
  source "$go2_here/../warehouse_stage2/nav_env.sh"
  "$GO2_ISAAC_ROOT/python.sh" "$go2_here/analyze_livo_comparison.py" --case "$go2_case" --livo "$go2_case/replay_livo_$go2_tag" --lio "$go2_case/replay_lio_$go2_tag" --ablation "$go2_case/replay_livo_no_image_$go2_tag" --output "$go2_case/evaluation_livo_$go2_tag";;
 *) echo 'Usage: run_rgb_comparison.sh start CASE';exit 2;;
esac
