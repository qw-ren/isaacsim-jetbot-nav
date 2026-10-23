"""Stop one verified supervised trial through its normal cleanup, never power off."""
import json
import os
from pathlib import Path
import signal
import sys
import time

ROOT=Path('/home/admin/go2-warehouse/artifacts')


def supervisor_case(argv):
    if len(argv)!=4 or Path(argv[0]).name!='bash' or argv[2]!='__run':return None
    case=Path(argv[3])
    if case.parent!=ROOT or not case.name.startswith(('lio_nav_','livo_rgb_','moving_')):return None
    if Path(argv[1])!=case/'source/warehouse_lio/run_nav_trial.sh':return None
    return case


def foreground_targets(records,group,case):
    return [r for r in records if r['group']==group and Path(r['argv'][0]).name.startswith('python3')
            and any(arg.startswith(str(case)+'/') for arg in r['argv'][1:])]


def processes():
    rows=[]
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():continue
        try:
            if entry.stat().st_uid!=os.getuid():continue
            argv=[s.decode() for s in (entry/'cmdline').read_bytes().split(b'\0') if s]
            if argv:rows.append(dict(pid=int(entry.name),group=os.getpgid(int(entry.name)),argv=argv))
        except (FileNotFoundError,ProcessLookupError,PermissionError,UnicodeError):continue
    return rows


def main():
    rows=processes();bosses=[(r,supervisor_case(r['argv'])) for r in rows if supervisor_case(r['argv'])]
    if not bosses:print('No supervised trial is active.');return 0
    if len(bosses)!=1:raise RuntimeError('Multiple supervisors; refusing ambiguous stop')
    boss,case=bosses[0]
    if (case/'RUNNER_RESULT.json').exists():print('Trial already completed.');return 0
    selected=[]
    for _ in range(10):
        selected=foreground_targets(processes(),boss['group'],case)
        if selected:break
        time.sleep(.2)
    if len(selected)>1:raise RuntimeError('Multiple foreground Python processes; refusing ambiguous stop')
    target=selected[0] if selected else None
    # Revalidate PID identity immediately before signalling; asynchronous recorder,
    # estimator and simulator groups are never selected as foreground workers.
    now={r['pid']:r for r in processes()}.get(target['pid']) if target else None
    if target and (now is None or now['argv']!=target['argv'] or now['group']!=target['group']):
        raise RuntimeError('Process changed; no signal sent')
    request=dict(requested_wall=time.time(),case=str(case),method='TERM verified foreground worker, then normal supervisor cleanup' if selected else 'no foreground: wait for existing cleanup without signalling supervisor',target_pid=target['pid'] if target else None,power_operation=False)
    (case/'STOP_REQUEST.json').write_text(json.dumps(request,indent=2)+'\n')
    if target:os.kill(target['pid'],signal.SIGTERM)
    deadline=time.monotonic()+45
    while time.monotonic()<deadline:
        if (case/'RUNNER_RESULT.json').exists():
            print('Normal supervisor cleanup completed: '+str(case));return 0
        time.sleep(.25)
    raise RuntimeError('Stop requested; final cleanup is not yet confirmed')


if __name__=='__main__':raise SystemExit(main())
