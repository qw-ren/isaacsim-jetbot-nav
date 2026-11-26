"""Real probe decoder with padded/endian sensor buffers; ROS imports are stubs."""
import importlib.util
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace as NS
from unittest.mock import patch

import numpy as np


def load_probe():
    ports = {"rclpy": {}, "rclpy.node": {"Node": object},
             "rclpy.qos": {"HistoryPolicy": object, "QoSProfile": object, "ReliabilityPolicy": object},
             "sensor_msgs.msg": {"CameraInfo": object, "Image": object, "LaserScan": object},
             "tf2_msgs.msg": {"TFMessage": object}}
    modules = {}
    for name, attrs in ports.items():
        mod = ModuleType(name);mod.__dict__.update(attrs);modules[name] = mod
    path = Path(__file__).resolve().parents[2] / "ros2_ws/tools/perception_probe.py"
    spec = importlib.util.spec_from_file_location("offline_probe", path)
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, modules):
        spec.loader.exec_module(module)
    return module.decode_2d


decode = load_probe()


def test_probe_float_depth_with_row_padding():
    raw = np.array([[1, 2, 99], [3, 4, 99]], dtype="<f4")
    msg = NS(encoding="32FC1", width=2, height=2, step=12, data=raw.tobytes(), is_bigendian=False)
    assert np.array_equal(decode(msg), [[1, 2], [3, 4]])


def test_probe_uint16_depth_is_one_channel():
    raw = np.array([[1000, 2000], [3000, 4000]], dtype="<u2")
    msg = NS(encoding="16UC1", width=2, height=2, step=4, data=raw.tobytes(), is_bigendian=False)
    assert np.array_equal(decode(msg), raw)


def test_probe_big_endian_depth():
    raw = np.array([[1.5, 2.5]], dtype=">f4")
    msg = NS(encoding="32FC1", width=2, height=1, step=8, data=raw.tobytes(), is_bigendian=True)
    assert np.array_equal(decode(msg), [[1.5, 2.5]])


def test_probe_rejects_short_rows_and_truncated_data():
    for step, data in [(4, bytes(4)), (8, bytes(7))]:
        msg = NS(encoding="32FC1", width=2, height=1, step=step, data=data)
        assert decode(msg) is None
