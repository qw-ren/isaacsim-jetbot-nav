"""Bounded read-only local costmap service snapshots. No publishers or truth input."""
import argparse
import json
import signal
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from nav2_msgs.srv import GetCostmap

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--case',type=Path,required=True)
a=p.parse_args()
output=a.case/'local_costmap_diagnostic';output.mkdir(exist_ok=False)
rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
node=Node('jetbot_local_costmap_probe')
client=node.create_client(GetCostmap,'/local_costmap/get_costmap')
stop=False;future=None;count=0;started=time.monotonic();requested=0.;error=None
def shutdown(*_):
    global stop
    stop=True
signal.signal(signal.SIGINT,shutdown);signal.signal(signal.SIGTERM,shutdown)
try:
    while not stop and time.monotonic()-started<600 and not (a.case/'RUNNER_RESULT.json').exists():
        if future is not None and future.done():
            msg=future.result().map;info=msg.metadata
            grid=np.asarray(msg.data,dtype=np.uint8).reshape(info.size_y,info.size_x)
            with (output/f'{count:05d}.npz').open('xb') as f:
                np.savez_compressed(f,data=grid,frame=np.array(msg.header.frame_id),
                    origin=np.array([info.origin.position.x,info.origin.position.y]),
                    origin_quaternion=np.array([info.origin.orientation.x,info.origin.orientation.y,info.origin.orientation.z,info.origin.orientation.w]),
                    resolution=np.array(info.resolution),
                    stamp=np.array(info.update_time.sec+info.update_time.nanosec/1e9),
                    received_wall=np.array(time.monotonic()))
            count+=1;future=None
        if future is None and time.monotonic()-requested>=.5 and client.service_is_ready():
            requested=time.monotonic();future=client.call_async(GetCostmap.Request())
        rclpy.spin_once(node,timeout_sec=.05)
except Exception as exc:
    error=repr(exc)
    raise
finally:
    with (a.case/'LOCAL_COSTMAP_PROBE_RESULT.json').open('x') as f:
        json.dump(dict(snapshots=count,error=error,wall_s=time.monotonic()-started,scope=__doc__),f,indent=2)
    node.destroy_node();rclpy.shutdown()
