"""跨帧目标跟踪：IoU + 匈牙利匹配（SORT 的简化版）。

输入每帧的 Detection2D，输出与检测一一对应的稳定 ``track_id``，让语义导航能说
「跟着刚才那个人」。本模块只做数据关联，不掺任何定位逻辑：

- 同类别的框才允许互相匹配（``class_aware``），避免人/椅子互换 ID；
- 用轨迹速度做 bbox 平移预测，弥补低帧率（~8 FPS）下 IoU 快速衰减；
- 连续丢失超过 ``max_age`` 帧的轨迹删除，ID 单调递增、绝不复用；
- 新轨迹累计命中 ``min_hits`` 帧才算「确认」，供 pipeline 决定是否对外暴露 ID。

只依赖 numpy；装了 scipy 用匈牙利最优匹配，没装时自动退化为贪心匹配。
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np

from .config import TrackerConfig
from .types import Detection2D

BBox = Tuple[float, float, float, float]
_DISALLOWED = 1e6


# --------------------------------------------------------------------------- #
# 基础几何 / 匹配
# --------------------------------------------------------------------------- #
def bbox_iou(a: BBox, b: BBox) -> float:
    """两个 (x1, y1, x2, y2) 框的交并比，范围 [0, 1]。"""
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    iw = max(0.0, min(ax2, bx2) - max(ax1, bx1))
    ih = max(0.0, min(ay2, by2) - max(ay1, by1))
    inter = iw * ih
    if inter <= 0.0:
        return 0.0
    area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
    area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
    union = area_a + area_b - inter
    return float(inter / union) if union > 0.0 else 0.0


def iou_matrix(boxes_a: Sequence[BBox], boxes_b: Sequence[BBox]) -> np.ndarray:
    """形状 (len(a), len(b)) 的 IoU 矩阵；空输入返回对应形状的零矩阵。"""
    out = np.zeros((len(boxes_a), len(boxes_b)), dtype=np.float64)
    for i, a in enumerate(boxes_a):
        for j, b in enumerate(boxes_b):
            out[i, j] = bbox_iou(a, b)
    return out


def _greedy_assignment(cost: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """按代价从低到高贪心配对，作为没有 scipy 时的兜底。"""
    rows, cols = [], []
    if cost.size == 0:
        return np.asarray(rows, dtype=int), np.asarray(cols, dtype=int)
    used_r, used_c = set(), set()
    for flat in np.argsort(cost, axis=None):
        r, c = divmod(int(flat), cost.shape[1])
        if r in used_r or c in used_c or cost[r, c] >= _DISALLOWED * 0.5:
            continue
        used_r.add(r)
        used_c.add(c)
        rows.append(r)
        cols.append(c)
    return np.asarray(rows, dtype=int), np.asarray(cols, dtype=int)


def _linear_assignment(cost: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """匈牙利匹配（scipy 可用时），返回 (行索引, 列索引)。"""
    if cost.size == 0:
        return np.asarray([], dtype=int), np.asarray([], dtype=int)
    try:
        from scipy.optimize import linear_sum_assignment
    except Exception:  # pragma: no cover - 无 scipy 的机器
        return _greedy_assignment(cost)
    return linear_sum_assignment(cost)


def _center(bbox: BBox) -> Tuple[float, float]:
    x1, y1, x2, y2 = bbox
    return ((x1 + x2) * 0.5, (y1 + y2) * 0.5)


# --------------------------------------------------------------------------- #
# 轨迹与跟踪器
# --------------------------------------------------------------------------- #
@dataclass
class Track:
    """一条活跃轨迹。bbox 是「最近一次真实观测」，速度单位像素/秒。"""

    track_id: int
    class_name: str
    bbox: BBox
    confidence: float
    stamp: float = 0.0
    hits: int = 1                      # 累计命中帧数
    age: int = 1                       # 存活帧数
    time_since_update: int = 0         # 距上次命中的帧数
    velocity: Tuple[float, float] = (0.0, 0.0)  # 像素/秒 (vx, vy)
    confirmed: bool = False

    def to_dict(self) -> dict:
        return {
            "track_id": int(self.track_id),
            "class_name": self.class_name,
            "bbox": [round(float(v), 1) for v in self.bbox],
            "confidence": round(float(self.confidence), 4),
            "hits": int(self.hits),
            "age": int(self.age),
            "time_since_update": int(self.time_since_update),
            "confirmed": bool(self.confirmed),
        }


class IouTracker:
    """多目标跟踪器。每帧调用一次 :meth:`update`。"""

    def __init__(self, cfg: Optional[TrackerConfig] = None) -> None:
        self.cfg = cfg if cfg is not None else TrackerConfig()
        self.tracks: List[Track] = []
        self._next_id = 1
        self._last_stamp: Optional[float] = None

    # ------------------------------------------------------------------ #
    def reset(self) -> None:
        """清空所有轨迹（切换视频源时调用）。ID 也从头开始。"""
        self.tracks = []
        self._next_id = 1
        self._last_stamp = None

    def is_confirmed(self, track_id: int) -> bool:
        for track in self.tracks:
            if track.track_id == track_id:
                return track.confirmed
        return False

    def get(self, track_id: int) -> Optional[Track]:
        for track in self.tracks:
            if track.track_id == track_id:
                return track
        return None

    def update(self, detections: Sequence[Detection2D], stamp: float = 0.0) -> List[int]:
        """关联本帧检测与已有轨迹。

        返回与 ``detections`` 等长、同序的 track_id 列表；每个检测都会分到 ID
        （老目标沿用旧 ID，新目标分配新 ID）。
        """
        dt = self._step_dt(stamp)
        n_trk, n_det = len(self.tracks), len(detections)

        pred_boxes = [self._predict(track, dt) for track in self.tracks]
        ious = iou_matrix(pred_boxes, [det.bbox for det in detections])

        cost = 1.0 - ious
        if n_trk > 0 and n_det > 0:
            if self.cfg.class_aware:
                for i, track in enumerate(self.tracks):
                    for j, det in enumerate(detections):
                        if track.class_name != det.class_name:
                            cost[i, j] = _DISALLOWED
            # 类内关联也要求 IoU 达到门限，低于门限的对视为不可匹配
            cost[ious < self.cfg.iou_threshold] = _DISALLOWED

        rows, cols = _linear_assignment(cost)

        track_ids: List[Optional[int]] = [None] * n_det
        matched_tracks = set()
        for r, c in zip(rows.tolist(), cols.tolist()):
            if ious[r, c] < self.cfg.iou_threshold:
                continue
            track = self.tracks[r]
            det = detections[c]
            if self.cfg.class_aware and track.class_name != det.class_name:
                continue  # 匈牙利在「禁止匹配」的大代价下仍可能配对，这里再兜一次
            self._update_track(track, det, dt)
            track_ids[c] = track.track_id
            matched_tracks.add(r)

        # 未命中的轨迹：用预测框续命，超时后删除
        for i, track in enumerate(self.tracks):
            if i in matched_tracks:
                continue
            track.bbox = pred_boxes[i]
            track.time_since_update += 1
            track.age += 1
            track.stamp = stamp
            track.confirmed = track.confirmed or track.hits >= self.cfg.min_hits

        # 新目标：分配新 ID
        for j, det in enumerate(detections):
            if track_ids[j] is not None:
                continue
            track = Track(
                track_id=self._next_id,
                class_name=det.class_name,
                bbox=det.bbox,
                confidence=det.confidence,
                stamp=stamp,
                hits=1,
                age=1,
                time_since_update=0,
                velocity=(0.0, 0.0),
                confirmed=self.cfg.min_hits <= 1,
            )
            self._next_id += 1
            self.tracks.append(track)
            track_ids[j] = track.track_id

        self.tracks = [
            track for track in self.tracks if track.time_since_update <= self.cfg.max_age
        ]

        # 走到这里，每条检测都必有 ID
        return [int(v) for v in track_ids]

    # ------------------------------------------------------------------ #
    def _step_dt(self, stamp: float) -> float:
        """由时间戳推算帧间隔；时间戳不可用时按 15 FPS 估计。"""
        dt = None
        if stamp and np.isfinite(stamp) and stamp > 0.0:
            if self._last_stamp is not None and stamp > self._last_stamp:
                dt = min(float(stamp - self._last_stamp), 0.5)
            self._last_stamp = float(stamp)
        return dt if dt is not None else 1.0 / 15.0

    def _predict(self, track: Track, dt: float) -> BBox:
        if not self.cfg.velocity_prediction:
            return track.bbox
        vx, vy = track.velocity
        if abs(vx) < 1e-9 and abs(vy) < 1e-9:
            return track.bbox
        x1, y1, x2, y2 = track.bbox
        dx, dy = vx * dt, vy * dt
        return (x1 + dx, y1 + dy, x2 + dx, y2 + dy)

    def _update_track(self, track: Track, det: Detection2D, dt: float) -> None:
        old_cx, old_cy = _center(track.bbox)
        new_cx, new_cy = _center(det.bbox)
        if dt > 1e-6:
            vx, vy = (new_cx - old_cx) / dt, (new_cy - old_cy) / dt
            # 指数平滑，避免检测框抖动让速度来回跳
            alpha = float(np.clip(self.cfg.velocity_smoothing, 0.0, 1.0))
            vx = (1.0 - alpha) * track.velocity[0] + alpha * vx
            vy = (1.0 - alpha) * track.velocity[1] + alpha * vy
            speed = float(np.hypot(vx, vy))
            if speed > self.cfg.max_speed_px_s > 0:
                scale = self.cfg.max_speed_px_s / speed
                vx, vy = vx * scale, vy * scale
            track.velocity = (vx, vy)

        track.bbox = det.bbox
        track.confidence = det.confidence
        track.stamp = det.stamp if det.stamp else track.stamp
        track.hits += 1
        track.age += 1
        track.time_since_update = 0
        track.confirmed = track.confirmed or track.hits >= self.cfg.min_hits


__all__ = ["Track", "IouTracker", "bbox_iou", "iou_matrix"]
