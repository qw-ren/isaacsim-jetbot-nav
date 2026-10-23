#!/usr/bin/env bash
# Only missing native-input replays; preserve existing historical noisy outputs.
set -eo pipefail
if [[ $(id -u) -eq 0 ]];then exec runuser -u admin -- bash "$0" "$@";fi
go2_here=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
go2_case=${2:?Captured case};go2_noisy=${3:?Historical noisy tag};go2_tag=${4:-native_01}
[[ "$go2_tag" =~ ^native_[0-9]+$ ]] || exit 2
case ${1:-} in
 start)
  test -f "$go2_case/RUNNER_RESULT.json"
  for go2_session in go2-lio-nav-task go2-warehouse go2-livo-comparison;do
   if tmux has-session -t "=$go2_session" 2>/dev/null;then echo "Existing $go2_session; run sequentially";exit 1;fi
  done
  python3 - "$go2_case" "$go2_here" "$go2_tag" <<'PY'
import hashlib,shutil,sys
from pathlib import Path
c=Path(sys.argv[1]);src=Path(sys.argv[2]).parent;out=c/('comparison_source_'+sys.argv[3]);out.mkdir(exist_ok=False)
for n in ['warehouse_lio','warehouse_livo','warehouse_stage2']:shutil.copytree(src/n,out/n,ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
(c/('comparison_source_'+sys.argv[3]+'.sha256')).write_text(''.join(hashlib.sha256(f.read_bytes()).hexdigest()+'  '+str(f.relative_to(c))+'\n' for f in sorted(out.rglob('*')) if f.is_file()))
PY
  printf -v go2_cmd '%q ' timeout -s INT 1200 bash "$go2_case/comparison_source_$go2_tag/warehouse_livo/run_native_history_comparison.sh" __run "$go2_case" "$go2_noisy" "$go2_tag"
  printf -v go2_log '%q' "$go2_case/comparison_$go2_tag.log"
  tmux new-session -d -s go2-livo-comparison "$go2_cmd > $go2_log 2>&1";;
 __run)
  go2_finish() {
   go2_exit=$?;trap - EXIT INT TERM
   python3 - "$go2_case" "$go2_tag" "$go2_exit" <<'PY'
from pathlib import Path
import datetime,json,sys
(Path(sys.argv[1])/('LIVO_COMPARISON_RESULT_'+sys.argv[2]+'.json')).write_text(json.dumps(dict(exit_code=int(sys.argv[3]),time=datetime.datetime.now().astimezone().isoformat(),truth_replayed=False,new_noise_added=False,power_operation=False),indent=2)+'\n')
PY
   exit "$go2_exit"
  }
  trap go2_finish EXIT;trap 'exit 130' INT TERM
  bash "$go2_here/run_livo_replay.sh" "$go2_case" "$go2_case/replay_livo_$go2_tag" 1 0 5 /evaluation/imu_ideal
  bash "$go2_here/run_lio_matched_replay.sh" "$go2_case/recording/sensors.sqlite3" "$go2_case/replay_lio_$go2_tag" /evaluation/imu_ideal 5
  source "$go2_here/../warehouse_stage2/nav_env.sh"
  "$GO2_ISAAC_ROOT/python.sh" "$go2_here/analyze_imu_pair.py" --case "$go2_case" --noisy-tag "$go2_noisy" --native-tag "$go2_tag" --output "$go2_case/evaluation_imu_pair_$go2_tag";;
 *) exit 2;;
esac
