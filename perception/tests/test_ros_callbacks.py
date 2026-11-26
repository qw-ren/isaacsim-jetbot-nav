"""Execute the real ROS callbacks with fake ROS I/O and the real geometry pipeline.

These tests do not validate DDS, cv_bridge ABI, rclpy executors or Isaac Sim.
"""
import importlib.util
import json
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace as NS
from unittest.mock import Mock, patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from atongmu_perception import PerceptionConfig, PerceptionPipeline
from atongmu_perception.types import Detection2D


class FakeTime:
    @staticmethod
    def from_msg(stamp):
        return stamp


class FakeTFError(Exception):
    pass


def load_callbacks():
    stubs = {}
    ports = {
        "rclpy": {}, "rclpy.node": {"Node": object},
        "rclpy.time": {"Time": FakeTime}, "rclpy.qos": {"qos_profile_sensor_data": object()},
        "rclpy.clock": {"Clock": Mock(), "ClockType": NS(STEADY_TIME=1)},
        "cv_bridge": {"CvBridge": object},
        "geometry_msgs.msg": {"TransformStamped": object},
        "sensor_msgs.msg": {"CameraInfo": object, "Image": object, "LaserScan": object},
        "std_msgs.msg": {"String": NS},
        "visualization_msgs.msg": {"Marker": object, "MarkerArray": object},
        "tf2_ros": {"Buffer": object, "TransformException": FakeTFError, "TransformListener": object},
    }
    for name, attrs in ports.items():
        module = ModuleType(name)
        module.__dict__.update(attrs)
        stubs[name] = module
    path = Path(__file__).resolve().parents[2] / "ros2_ws/src/atongmu_perception_ros/atongmu_perception_ros/perception_node.py"
    spec = importlib.util.spec_from_file_location("offline_perception_node", path)
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, stubs):
        spec.loader.exec_module(module)
    return module


ROS = load_callbacks()


def image(sec=1, encoding="rgb8", frame="optical"):
    return NS(header=NS(stamp=NS(sec=sec, nanosec=0), frame_id=frame),
              width=640, height=480, encoding=encoding,
              pixels=np.zeros((480, 640, 3)) if encoding == "rgb8" else np.full((480, 640), 2.))


def fixture(require_map=True):
    node = ROS.PerceptionNode.__new__(ROS.PerceptionNode)
    cfg = PerceptionConfig()
    cfg.tracker.min_hits = 1
    cfg.fusion.enabled = False
    detector = NS(detect=lambda image, stamp: [Detection2D(0, "box", .9, (280, 200, 360, 280), stamp)])
    node.pipeline = PerceptionPipeline(cfg, detector=detector)
    node.camera_frame, node.base_frame, node.map_frame = "optical", "chassis", "map"
    node.laser_frame = "chassis"
    node.last_stamp, node.latest_scan = None, None
    node.latest_info = NS(width=640, height=480)
    node.require_map, node.publish_legacy, node.use_scan = require_map, False, False
    node.use_tf_extrinsics = True
    node.session_id, node.sequence, node.scan_slop = "offline", 0, .06
    node._warned = set()
    node.get_logger = lambda: Mock()
    node.get_parameter = lambda name: NS(value=True)
    node.bridge = NS(imgmsg_to_cv2=lambda msg, desired_encoding: msg.pixels)
    node.det_pub, node.pose_pub, node.planar_pub = Mock(), Mock(), Mock()
    node.marker_pub = None
    from collections import deque
    node.pending_frames = deque(maxlen=5)
    node.tf_wait_sec = .25
    node.tf_buffer = Mock()

    def lookup(target, source, stamp):
        if target == "chassis":
            return cfg.extrinsics.T_base_camera()
        if target == "map":
            t = np.eye(4)
            t[0, 3] = 10.
            return t
        raise AssertionError("unexpected TF lookup")
    node.tf_buffer.lookup_transform.side_effect = lookup
    return node


def execute(node, sec=1, encoding="32FC1"):
    depth = image(sec, encoding)
    if encoding == "16UC1":
        depth.pixels *= 1000
    # Matrices stand in for TransformStamped messages; all real geometry runs.
    with patch.object(ROS, "transform_to_matrix", side_effect=lambda value: value):
        node.image_depth_cb(image(sec), depth)


def payload(node):
    return json.loads(node.planar_pub.publish.call_args.args[0].data)


def test_callback_queries_capture_time_and_preserves_map_frame():
    node = fixture()
    execute(node)
    data = payload(node)
    assert data["frame_id"] == "map" and data["objects"]
    assert len(data["objects"][0]["position_xy"]) == 2
    for call in node.tf_buffer.lookup_transform.call_args_list:
        assert call.args[2].sec == 1  # never latest Time(0)
    node.pose_pub.publish.assert_not_called()


def test_callback_metres_and_millimetres_give_same_xy():
    a, b = fixture(), fixture()
    execute(a)
    execute(b, encoding="16UC1")
    assert payload(a)["objects"][0]["position_xy"] == payload(b)["objects"][0]["position_xy"]


def test_callback_map_missing_never_uses_odom_as_map():
    node = fixture(require_map=False)
    def lookup(target, source, stamp):
        if target == "map":
            raise FakeTFError("map unavailable")
        assert target == "chassis"
        return node.pipeline.cfg.extrinsics.T_base_camera()
    node.tf_buffer.lookup_transform.side_effect = lookup
    execute(node)
    assert payload(node)["frame_id"] == "chassis"


def test_callback_rejects_depth_from_different_frame():
    node = fixture()
    node.image_depth_cb(image(), image(1, "32FC1", frame="wrong"))
    node.planar_pub.publish.assert_not_called()


def test_callback_clock_reset_latches_and_drops_later_images():
    node = fixture()
    execute(node, 2)
    execute(node, 1)
    execute(node, 3)
    assert node.planar_pub.publish.call_count == 1


def test_callback_late_tf_is_retried_at_original_image_time():
    node = fixture()
    lookup = node.tf_buffer.lookup_transform.side_effect
    node.tf_buffer.lookup_transform.side_effect = FakeTFError("TF arrives later")
    execute(node)
    node.planar_pub.publish.assert_not_called()
    node.tf_buffer.lookup_transform.side_effect = lookup
    with patch.object(ROS, "transform_to_matrix", side_effect=lambda value: value):
        node._retry_pending()
    assert payload(node)["stamp"] == 1.


def test_callback_rejects_unknown_depth_encoding():
    node = fixture()
    execute(node, encoding="8UC1")
    node.planar_pub.publish.assert_not_called()


def test_callback_expired_tf_wait_is_dropped_without_publishing():
    node = fixture()
    node.tf_buffer.lookup_transform.side_effect = FakeTFError("no TF")
    execute(node)
    assert len(node.pending_frames) == 1
    with patch.object(ROS.time, "monotonic", return_value=node.pending_frames[0][2] + .251):
        node._retry_pending()
    assert not node.pending_frames
    node.planar_pub.publish.assert_not_called()


def test_callback_does_not_publish_duplicate_image():
    node = fixture()
    execute(node, 1)
    execute(node, 1)
    assert node.planar_pub.publish.call_count == 1


def test_callback_zero_start_time_does_not_prevent_later_images():
    node = fixture()
    execute(node, 0)
    assert node.last_stamp is None
    execute(node, 1)
    assert node.planar_pub.publish.call_count == 1


def test_callback_map_required_does_not_publish_chassis_coordinates():
    node = fixture()
    original = node.tf_buffer.lookup_transform.side_effect
    def lookup(target, source, stamp):
        if target == "map":
            raise FakeTFError("map missing")
        return original(target, source, stamp)
    node.tf_buffer.lookup_transform.side_effect = lookup
    execute(node)
    node.planar_pub.publish.assert_not_called()
    assert len(node.pending_frames) == 1


def test_callback_rotated_map_transform_has_correct_xy_axes():
    local, rotated = fixture(require_map=False), fixture()
    original = local.tf_buffer.lookup_transform.side_effect
    def lookup(target, source, stamp):
        if target == "map":
            raise FakeTFError("local only")
        return original(target, source, stamp)
    local.tf_buffer.lookup_transform.side_effect = lookup
    execute(local)
    x, y = payload(local)["objects"][0]["position_xy"]
    def rotate(target, source, stamp):
        if target == "map":
            return np.array([[0., -1, 0, 10], [1, 0, 0, 5], [0, 0, 1, 0], [0, 0, 0, 1]])
        return original(target, source, stamp)
    rotated.tf_buffer.lookup_transform.side_effect = rotate
    execute(rotated)
    assert np.allclose(payload(rotated)["objects"][0]["position_xy"], [10 - y, 5 + x])


def test_callback_mismatched_rgb_depth_timestamps_are_rejected():
    node = fixture()
    node.image_depth_cb(image(1), image(2, "32FC1"))
    node.planar_pub.publish.assert_not_called()


def test_callback_queued_frames_publish_in_order_once_tf_arrives():
    node = fixture()
    original = node.tf_buffer.lookup_transform.side_effect
    node.tf_buffer.lookup_transform.side_effect = FakeTFError("delayed")
    execute(node, 1)
    execute(node, 2)
    node.tf_buffer.lookup_transform.side_effect = original
    with patch.object(ROS, "transform_to_matrix", side_effect=lambda value: value):
        node._retry_pending()
    data = [json.loads(call.args[0].data) for call in node.planar_pub.publish.call_args_list]
    assert [item["stamp"] for item in data] == [1., 2.]
    assert [item["sequence"] for item in data] == [1, 2]
    assert not node.pending_frames
