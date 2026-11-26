"""目标跟踪模块的单元测试。

不依赖 pytest 也能跑：python tests/run_all.py
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from atongmu_perception.config import PerceptionConfig, TrackerConfig  # noqa: E402
from atongmu_perception.tracker import IouTracker, bbox_iou, iou_matrix  # noqa: E402
from atongmu_perception.types import Detection2D  # noqa: E402


def _det(x1, y1, x2, y2, class_name="person", conf=0.9, stamp=0.0):
    return Detection2D(
        class_id=0,
        class_name=class_name,
        confidence=conf,
        bbox=(float(x1), float(y1), float(x2), float(y2)),
        stamp=stamp,
    )


def _tracker(**kwargs):
    cfg = TrackerConfig(**kwargs)
    return IouTracker(cfg)


# --------------------------------------------------------------------------- #
# 基础几何
# --------------------------------------------------------------------------- #
def test_bbox_iou_values():
    a = (0.0, 0.0, 10.0, 10.0)
    assert abs(bbox_iou(a, a) - 1.0) < 1e-9
    assert bbox_iou(a, (20.0, 20.0, 30.0, 30.0)) == 0.0
    # 水平移动一半：交集 50，并集 150
    assert abs(bbox_iou(a, (5.0, 0.0, 15.0, 10.0)) - 50.0 / 150.0) < 1e-9


def test_iou_matrix_shape_and_empty():
    m = iou_matrix([(0, 0, 10, 10), (20, 20, 30, 30)], [(0, 0, 10, 10)])
    assert m.shape == (2, 1)
    assert abs(m[0, 0] - 1.0) < 1e-9 and m[1, 0] == 0.0
    assert iou_matrix([], [(0, 0, 1, 1)]).shape == (0, 1)
    assert iou_matrix([(0, 0, 1, 1)], []).shape == (1, 0)


# --------------------------------------------------------------------------- #
# 关联行为
# --------------------------------------------------------------------------- #
def test_id_stable_across_frames():
    tracker = _tracker(min_hits=2, velocity_prediction=False)
    ids1 = tracker.update([_det(100, 100, 200, 300)], stamp=1.0)
    ids2 = tracker.update([_det(105, 102, 205, 302)], stamp=1.1)
    ids3 = tracker.update([_det(110, 104, 210, 304)], stamp=1.2)
    assert ids1 == ids2 == ids3
    assert len(set(ids1 + ids2 + ids3)) == 1
    # 第 1 帧未确认，第 2 帧起确认
    assert tracker.get(ids1[0]).hits == 3
    assert tracker.is_confirmed(ids1[0])


def test_min_hits_gate():
    tracker = _tracker(min_hits=3, velocity_prediction=False)
    tid = tracker.update([_det(100, 100, 200, 300)], stamp=1.0)[0]
    assert not tracker.is_confirmed(tid)
    tracker.update([_det(101, 100, 201, 300)], stamp=1.1)
    assert not tracker.is_confirmed(tid)
    tracker.update([_det(102, 100, 202, 300)], stamp=1.2)
    assert tracker.is_confirmed(tid)


def test_new_object_gets_new_id():
    tracker = _tracker(velocity_prediction=False)
    id_a = tracker.update([_det(0, 0, 100, 200)], stamp=1.0)[0]
    ids = tracker.update([_det(2, 0, 102, 200), _det(400, 0, 500, 200)], stamp=1.1)
    assert ids[0] == id_a
    assert ids[1] != id_a


def test_assignment_is_order_independent():
    tracker = _tracker(velocity_prediction=False)
    left = _det(0, 0, 100, 200)
    right = _det(200, 0, 300, 200)
    id_left, id_right = tracker.update([left, right], stamp=1.0)
    # 下一帧检测顺序反转，但位置没变，ID 应跟着位置走
    ids = tracker.update(
        [_det(202, 0, 302, 200), _det(2, 0, 102, 200)], stamp=1.1
    )
    assert ids[1] == id_left
    assert ids[0] == id_right


def test_class_aware_prevents_cross_class_match():
    tracker = _tracker(class_aware=True, velocity_prediction=False)
    person_id = tracker.update([_det(100, 100, 200, 300, "person")], stamp=1.0)[0]
    chair_id = tracker.update([_det(100, 100, 200, 300, "chair")], stamp=1.1)[0]
    assert chair_id != person_id


def test_class_agnostic_can_match_same_box():
    tracker = _tracker(class_aware=False, min_hits=2, velocity_prediction=False)
    tid = tracker.update([_det(100, 100, 200, 300, "person")], stamp=1.0)[0]
    # 允许跨类别时，同一位置的错误分类也沿用原轨迹（ID 不跳）
    assert tracker.update([_det(100, 100, 200, 300, "chair")], stamp=1.1)[0] == tid


def test_track_expires_and_id_not_reused():
    tracker = _tracker(max_age=2, min_hits=1, velocity_prediction=False)
    old_id = tracker.update([_det(0, 0, 100, 200)], stamp=1.0)[0]
    for k in range(3):
        tracker.update([], stamp=1.1 + 0.1 * k)
    assert tracker.get(old_id) is None
    new_id = tracker.update([_det(0, 0, 100, 200)], stamp=2.0)[0]
    assert new_id > old_id


def test_moving_object_keeps_id_with_prediction():
    tracker = _tracker(iou_threshold=0.4, min_hits=1, velocity_prediction=True)
    tid = None
    for k in range(6):
        box = (100 + 40 * k, 100, 200 + 40 * k, 300)
        ids = tracker.update([_det(*box)], stamp=1.0 + 0.1 * k)
        if tid is None:
            tid = ids[0]
        assert ids[0] == tid


def test_reset_restarts_ids():
    tracker = _tracker(velocity_prediction=False)
    first = tracker.update([_det(0, 0, 100, 200)], stamp=1.0)[0]
    tracker.reset()
    assert tracker.update([_det(0, 0, 100, 200)], stamp=2.0)[0] == first
    assert tracker.tracks[0].track_id == 1


# --------------------------------------------------------------------------- #
# 与主流程集成
# --------------------------------------------------------------------------- #
class _FakeDetector:
    """pipeline 在传入 detections 时不会调用它，这里只为避免加载 YOLO。"""

    def detect(self, image, stamp=0.0):  # pragma: no cover - 不应被调用
        return []


def test_pipeline_assigns_track_id_after_min_hits():
    cfg = PerceptionConfig()
    cfg.tracker = TrackerConfig(enabled=True, min_hits=2, velocity_prediction=False)
    from atongmu_perception.pipeline import PerceptionPipeline

    pipeline = PerceptionPipeline(cfg, detector=_FakeDetector())
    det = _det(280, 330, 360, 420, "chair")

    first = pipeline.process(detections=[det], stamp=1.0)
    second = pipeline.process(detections=[_det(282, 331, 362, 421, "chair")], stamp=1.1)
    assert len(first.objects) == 1 and len(second.objects) == 1
    assert first.objects[0].track_id is None       # 第一帧未确认
    assert second.objects[0].track_id is not None  # 第二帧起暴露稳定 ID
    assert second.objects[0].to_dict()["track_id"] == second.objects[0].track_id


def test_pipeline_can_disable_tracking():
    cfg = PerceptionConfig()
    cfg.tracker = TrackerConfig(enabled=False)
    from atongmu_perception.pipeline import PerceptionPipeline

    pipeline = PerceptionPipeline(cfg, detector=_FakeDetector())
    det = _det(280, 330, 360, 420, "chair")
    for k in range(3):
        result = pipeline.process(detections=[det], stamp=1.0 + 0.1 * k)
        assert result.objects[0].track_id is None
