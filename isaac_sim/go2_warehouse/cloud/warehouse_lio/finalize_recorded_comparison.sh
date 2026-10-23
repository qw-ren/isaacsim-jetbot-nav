#!/usr/bin/env bash
# Complete analysis after the already-running paired replay; never rerun it.
set -eo pipefail
if [[ $(id -u) -eq 0 ]];then exec runuser -u admin -- bash "$0" "$@";fi
go2_case=${1:?Provide the finished live-capture case path}
go2_finish() {
 go2_exit=$?
 trap - EXIT INT TERM
 python3 - "$go2_case" "$go2_exit" "$0" <<'PY'
from pathlib import Path
import datetime,hashlib,json,sys
p=Path(sys.argv[1])/'COMPARISON_RESULT.json'
with p.open('x') as f:
 json.dump(dict(exit_code=int(sys.argv[2]),time=datetime.datetime.now().astimezone().isoformat(),power_operation=False,
               continuation_source_sha256=hashlib.sha256(Path(sys.argv[3]).read_bytes()).hexdigest(),
               note='Existing ideal/noisy replays reused; original worker bookkeeping quoting error is preserved in comparison.log'),f,indent=2)
PY
 exit "$go2_exit"
}
trap go2_finish EXIT
trap 'exit 130' INT TERM
python3 - "$go2_case" <<'PY'
from pathlib import Path
import subprocess,sys,time
p=Path(sys.argv[1]);assert (p/'RUNNER_RESULT.json').is_file()
until=time.monotonic()+1000
while time.monotonic()<until:
 active=subprocess.run(['tmux','has-session','-t','=go2-lio-comparison'],capture_output=True).returncode==0
 if not active:break
 time.sleep(.5)
else:raise RuntimeError('Existing replay worker did not finish')
assert (p/'replay_ideal/SUMMARY.json').is_file() and (p/'replay_noisy/SUMMARY.json').is_file(),'Replay result missing; do not invent comparison'
PY
python3 - "$go2_case" <<'PY'
import json,sys
from pathlib import Path
p=Path(sys.argv[1]);a=json.loads((p/'replay_ideal/SUMMARY.json').read_text());b=json.loads((p/'replay_noisy/SUMMARY.json').read_text())
assert not a['error'] and not b['error']
assert a['point_input_sha256']==b['point_input_sha256']
assert a['imu_timestamps_sha256']==b['imu_timestamps_sha256']
with (p/'PAIRED_INPUT_VERIFIED.json').open('x') as f:
 json.dump(dict(point_input_sha256=a['point_input_sha256'],imu_timestamps_sha256=a['imu_timestamps_sha256'],identical_lidar_and_sampling_schedule=True),f,indent=2)
PY
source "$go2_case/comparison_source/warehouse_stage2/nav_env.sh"
"$GO2_ISAAC_ROOT/python.sh" "$go2_case/comparison_source/warehouse_lio/analyze_lio_dataset.py" \
 --database "$go2_case/recording/sensors.sqlite3" --output "$go2_case/evaluation_long_01" \
 --replay "$go2_case/replay_ideal/odometry.json" --replay "$go2_case/replay_noisy/odometry.json" \
 > "$go2_case/evaluation_long_01.log" 2>&1
