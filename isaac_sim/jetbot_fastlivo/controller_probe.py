"""Read-only, opt-in DWB scores and live controller parameter capture.
Bounded to 64 MiB sampled scores + 32 MiB selections / 600 wall seconds.
No command or TF publishers.
Early-terminated candidate scores can be incomplete with short-circuit scoring.
"""
import argparse
import json
import signal
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import GetParameters, ListParameters
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rosidl_runtime_py.convert import message_to_ordereddict
from dwb_msgs.msg import LocalPlanEvaluation

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--case', type=Path, required=True)
a = p.parse_args()
rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
n = Node('jetbot_controller_probe', parameter_overrides=[Parameter('use_sim_time', value=True)])
list_client = n.create_client(ListParameters, '/controller_server/list_parameters')
get_client = n.create_client(GetParameters, '/controller_server/get_parameters')
started = time.monotonic()
stop = False
count = 0
size = 0
last_stamp = {}
truncated = False
subscriptions = {}
listing = None
getting = None
error = None
parameter_saved = False
stream = (a.case / 'controller_evaluations.jsonl').open('x')
selected_stream = (a.case / 'controller_selected.jsonl').open('x')
selected_bytes = 0
selected_count = 0
selected_truncated = False


def shutdown(*_):
    global stop
    stop = True


def evaluation(msg, topic):
    global count, size, truncated, selected_bytes, selected_count, selected_truncated
    # Every received selection, including intervals omitted by the full-score sampler.
    if 0 <= msg.best_index < len(msg.twists) and not selected_truncated:
        best = msg.twists[msg.best_index]
        selected = json.dumps(dict(topic=topic, wall=time.monotonic(),
            stamp=msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
            frame=msg.header.frame_id, best_index=msg.best_index, total=best.total,
            scores=[[item.name, item.scale, item.raw_score] for item in best.scores],
            trajectory=message_to_ordereddict(best.traj)), separators=(',', ':')) + '\n'
        if selected_bytes + len(selected.encode()) <= 32 * 1024 * 1024:
            selected_stream.write(selected)
            selected_bytes += len(selected.encode())
            selected_count += 1
            if selected_count % 20 == 0:
                selected_stream.flush()
        else:
            selected_truncated = True
    if truncated:
        return
    stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
    if stamp - last_stamp.get(topic, -1e9) < 0.19:
        return
    last_stamp[topic] = stamp
    candidates = [dict(velocity=[score.traj.velocity.x, score.traj.velocity.y,
                                 score.traj.velocity.theta], total=score.total,
                       scores=[[item.name, item.scale, item.raw_score] for item in score.scores])
                  for score in msg.twists]
    line = json.dumps(dict(topic=topic, stamp=stamp, frame=msg.header.frame_id,
                           best_index=msg.best_index, worst_index=msg.worst_index,
                           candidates=candidates), separators=(',', ':')) + '\n'
    encoded_size = len(line.encode())
    if size + encoded_size > 64 * 1024 * 1024:
        truncated = True
        return
    stream.write(line)
    count += 1
    size += encoded_size
    if count % 20 == 0:
        stream.flush()


signal.signal(signal.SIGINT, shutdown)
signal.signal(signal.SIGTERM, shutdown)
try:
    while not stop and time.monotonic() - started < 600:
        # Discover the actual resolved topic, rather than assuming a namespace.
        for topic, types in n.get_topic_names_and_types():
            if 'dwb_msgs/msg/LocalPlanEvaluation' in types and topic not in subscriptions:
                subscriptions[topic] = n.create_subscription(
                    LocalPlanEvaluation, topic,
                    lambda msg, topic=topic: evaluation(msg, topic), qos_profile_sensor_data)
        if listing is None and list_client.service_is_ready() and get_client.service_is_ready():
            listing = list_client.call_async(ListParameters.Request())
        if listing is not None and listing.done() and getting is None:
            names = listing.result().result.names
            if 'FollowPath.min_speed_theta' not in names:
                listing = None
            else:
                request = GetParameters.Request()
                request.names = names
                getting = (names, get_client.call_async(request))
        if getting is not None and getting[1].done() and not parameter_saved:
            names, future = getting
            data = dict(zip(names, [message_to_ordereddict(value) for value in future.result().values]))
            with (a.case / 'CONTROLLER_RUNTIME_PARAMETERS.json').open('x') as output:
                json.dump(dict(node='/controller_server', received_sim_s=n.get_clock().now().nanoseconds / 1e9,
                               parameters=data), output, indent=2)
            parameter_saved = True
        rclpy.spin_once(n, timeout_sec=0.1)
except Exception as exc:
    error = repr(exc)
    raise
finally:
    stream.close()
    selected_stream.close()
    (a.case / 'CONTROLLER_PROBE_RESULT.json').write_text(json.dumps(dict(
        messages=count, bytes=size, truncated=truncated, topics=list(subscriptions),
        selected_messages=selected_count, selected_bytes=selected_bytes,
        selected_truncated=selected_truncated,
        runtime_parameters_saved=parameter_saved, error=error,
        scope='read-only diagnostic; score completeness depends on DWB short-circuit setting',
        elapsed_wall_s=time.monotonic() - started), indent=2) + '\n')
    n.destroy_node()
    rclpy.shutdown()
