"""Command/cancel client: UUID correlation, discovery wait, no robot control."""
import argparse
import json
import signal
import time
import uuid
from pathlib import Path
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from .config import load
from .core import CaseEventReader

TERMINAL={'SUCCEEDED','FAILED','NAVIGATION_FAILED','CANCELLED','TARGET_INVALID','STOPPED',
          'NOT_FOUND','UNREACHABLE','INVALID_COMMAND','BUSY','NOT_READY','DUPLICATE',
          'DRY_RUN_COMPLETE','SAFETY_CLEANUP_FAILED','ALIGNMENT_FAILED',
          'REFERENCE_NOT_FOUND','AMBIGUOUS_REFERENCE','AMBIGUOUS'}


def main():
    p=argparse.ArgumentParser();p.add_argument('text',nargs='?',default='');p.add_argument('--case',required=True,type=Path)
    p.add_argument('--id',default=None);p.add_argument('--cancel',action='store_true');p.add_argument('--dry-run',action='store_true')
    p.add_argument('--timeout',type=float,default=900.);p.add_argument('--cancel-after',type=float)
    p.add_argument('--case-log-only',action='store_true',help='Read task acknowledgements from the shared case log; commands still use ROS')
    args=p.parse_args()
    if args.cancel and not args.id:p.error('--cancel requires --id of the active task')
    _,cfg=load(args.case);task=args.id or uuid.uuid4().hex
    rclpy.init();node=Node('semantic_command_cli_'+uuid.uuid4().hex[:8]);done=[];navigating=[]
    seen=set();events=None
    def consume(row,transport):
        if row.get('task_id')!=task:return
        key=(row.get('wall'),row['state'])
        if key in seen:return
        seen.add(key)
        print(json.dumps(dict(row,ack_transport=transport),ensure_ascii=False),flush=True)
        if row['state']=='NAVIGATING':navigating.append(time.monotonic())
        if row['state']in TERMINAL:done.append(row)
    def poll_events():
        if events is not None:
            for row in events.read():consume(row,'case_event_log')
    if not args.case_log_only:
        node.create_subscription(String,cfg['topics']['status'],lambda m:consume(json.loads(m.data),'ros'),10)
    pub=node.create_publisher(String,cfg['topics']['cancel'if args.cancel else'command'],10)
    cancel=node.create_publisher(String,cfg['topics']['cancel'],10)
    interrupted=[]
    signal.signal(signal.SIGINT,lambda *_:interrupted.append(True))
    try:
        end=time.monotonic()+10
        while pub.get_subscription_count()==0 and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
        if not pub.get_subscription_count():raise TimeoutError('Semantic manager unavailable')
        # Allow the reciprocal status subscription to finish discovery too.
        deadline=time.monotonic()+.5
        while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.05)
        # Cursor is established BEFORE sending; old task IDs cannot be mistaken for this response.
        try:events=CaseEventReader(args.case/'semantic_tasks.jsonl')
        except FileNotFoundError:
            if args.case_log_only:raise
        row=dict(version=1,task_id=task)
        if not args.cancel:row.update(text=args.text,dry_run=args.dry_run)
        pub.publish(String(data=json.dumps(row,ensure_ascii=False)));end=time.monotonic()+args.timeout;sent=False
        while not done and time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.05)
            poll_events()
            if not sent and(interrupted or(args.cancel_after is not None and navigating and time.monotonic()-navigating[0]>=args.cancel_after)):
                cancel.publish(String(data=json.dumps(dict(version=1,task_id=task))));sent=True
        if not done:
            # An explicit timeout requests cancellation, then allows cleanup confirmation.
            cancel.publish(String(data=json.dumps(dict(version=1,task_id=task))))
            end=time.monotonic()+10
            while not done and time.monotonic()<end:
                rclpy.spin_once(node,timeout_sec=.05);poll_events()
        if not done:raise TimeoutError('No terminal acknowledgement; inspect gate/session before any new task')
        return 0 if done[-1]['state']in {'SUCCEEDED','DRY_RUN_COMPLETE','CANCELLED'}else 1
    finally:
        if events is not None:events.close()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':raise SystemExit(main())
