"""Planar wire contract and regression checks for truthful frame labels."""
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from atongmu_perception.config import PerceptionConfig
from atongmu_perception.pipeline import PerceptionPipeline
from atongmu_perception.planar import json_finite, planar_observations
from atongmu_perception.types import Detection2D, Localization, ObjectPose3D, PerceptionResult
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "ros2_ws" / "tools"))
from check_planar_stream import validate_message


def sample():
    loc = Localization(2.0, (0, 0, 2), method="depth", xyz_base=(2, 0, .2))
    obj = ObjectPose3D(0, "box", .9, (2, 0, .2), frame_id="chassis",
                       localization=loc, stamp=1.5, track_id=7)
    return PerceptionResult(objects=[obj], stamp=1.5, frame_id="chassis")


def test_planar_has_only_xy_and_stable_session_scoped_id():
    result = sample()
    a = planar_observations(result, "run1", 1)
    b = planar_observations(result, "run1", 2)
    c = planar_observations(result, "run2", 1)
    obj = a["objects"][0]
    assert obj["position_xy"] == [2., 0.]
    assert obj["position_reference"] == "visible_surface"
    assert "dimensions" not in obj and "orientation_xyzw" not in obj
    assert obj["object_id"] == b["objects"][0]["object_id"]
    assert obj["object_id"] != c["objects"][0]["object_id"]
    assert a["source"] == "vision" and a["frame_id"] == "chassis"
    json.dumps(a, allow_nan=False)


def test_planar_rejects_unconfirmed_invalid_and_mixed_frames():
    for field, value in [("track_id", None), ("position", (float("nan"), 0, 0)),
                         ("frame_id", "map"), ("confidence", float("inf")), ("stamp", 2.)]:
        result = sample()
        setattr(result.objects[0], field, value)
        assert planar_observations(result, "run", 1)["objects"] == []


def test_planar_ground_is_explicit_estimate():
    result = sample()
    result.objects[0].localization.method = "ground"
    assert planar_observations(result, "run", 1)["objects"][0]["position_reference"] == "ground_contact_estimate"


def test_json_unavailable_values_are_null():
    data = json_finite({"size": [1., float("nan"), float("inf")], "x": -float("inf")})
    assert json.loads(json.dumps(data, allow_nan=False)) == {"size": [1., None, None], "x": None}


def test_server_checker_accepts_wire_payload_and_rejects_3d_or_invalid_xy():
    data = planar_observations(sample(), "run", 1)
    validate_message(json.loads(json.dumps(data, allow_nan=False)), "chassis")
    for key, value in [("position_xy", [1., float("nan")]), ("position_xy", [1., 2., 3.]),
                       ("dimensions", [1., 1., 1.]), ("object_id", "wrong-id")]:
        invalid = json.loads(json.dumps(data))
        invalid["objects"][0][key] = value
        try:
            validate_message(invalid, "chassis")
        except ValueError:
            pass
        else:
            raise AssertionError("invalid planar observation accepted: %s" % key)


def test_server_checker_rejects_mislabeled_frame_and_zero_stamp():
    for key, value in [("frame_id", "map"), ("stamp", 0.), ("stamp", float("inf"))]:
        data = planar_observations(sample(), "run", 1)
        data[key] = value
        try:
            validate_message(data, "chassis")
        except ValueError:
            pass
        else:
            raise AssertionError("invalid header accepted: %s" % key)


def test_producer_rejects_zero_stamp_and_invalid_sequence():
    for stamp, sequence in [(0., 1), (1.5, 1.5), (1.5, True)]:
        result = sample()
        result.stamp = stamp
        try:
            planar_observations(result, "run", sequence)
        except ValueError:
            pass
        else:
            raise AssertionError("producer accepted invalid header")


def test_producer_filters_invalid_tracks_and_methods():
    for field, value in [("track_id", -1), ("track_id", 1.5), ("class_name", "")]:
        result = sample()
        setattr(result.objects[0], field, value)
        assert planar_observations(result, "run", 1)["objects"] == []
    result = sample()
    result.objects[0].localization.method = "unknown"
    assert planar_observations(result, "run", 1)["objects"] == []


def test_pipeline_respects_actual_target_and_base_frames_without_boxes():
    cfg = PerceptionConfig()
    cfg.tracker.enabled = False
    cfg.fusion.enabled = False
    pipe = PerceptionPipeline(cfg, detector=object())
    det = Detection2D(0, "box", .9, (280, 200, 360, 280), stamp=1.5)
    depth = np.full((480, 640), 2.)
    # A planar run must never call the optional 3D size estimator.
    def no_boxes(*args):
        raise AssertionError("size estimation was invoked")
    pipe.localizer.estimate_size = no_boxes
    local = pipe.process(depth=depth, detections=[det], stamp=1.5,
                         base_frame="chassis", estimate_sizes=False)
    assert local.frame_id == local.objects[0].frame_id == "chassis"
    transform = np.eye(4)
    transform[0, 3] = 10.
    for target in ("odom", "warehouse_map"):
        result = pipe.process(depth=depth, detections=[det], stamp=1.5,
                              T_map_base=transform, output_frame=target, estimate_sizes=False)
        assert result.frame_id == result.objects[0].frame_id == target
        assert abs(result.objects[0].position[0] - local.objects[0].position[0] - 10.) < 1e-9
        assert result.objects[0].size_estimate is None
