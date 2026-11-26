"""感知模块的数据结构。

约定（与 docs/perception_interface.md 保持一致）：
- 所有长度单位为米，角度为弧度，时间为秒。
- 图像坐标 (u, v) 为像素坐标，原点在左上角，u 向右、v 向下。
- 相机光学坐标系遵循 ROS 约定：x 向右、y 向下、z 向前（光轴）。
- 机器人/世界坐标系遵循 ROS 约定：x 向前、y 向左、z 向上。
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Tuple


@dataclass
class Detection2D:
    """L1 输出：图像平面上的一个目标框。"""

    class_id: int
    class_name: str
    confidence: float
    bbox: Tuple[float, float, float, float]  # (x1, y1, x2, y2) 像素
    stamp: float = 0.0

    @property
    def center(self) -> Tuple[float, float]:
        x1, y1, x2, y2 = self.bbox
        return ((x1 + x2) * 0.5, (y1 + y2) * 0.5)

    @property
    def bottom_center(self) -> Tuple[float, float]:
        """框底边中点，近似目标与地面的接触点，用于单目地面约束测距。"""
        x1, _, x2, y2 = self.bbox
        return ((x1 + x2) * 0.5, y2)

    @property
    def size_px(self) -> Tuple[float, float]:
        x1, y1, x2, y2 = self.bbox
        return (max(0.0, x2 - x1), max(0.0, y2 - y1))

    @property
    def area_px(self) -> float:
        w, h = self.size_px
        return w * h

    def to_dict(self) -> dict:
        return {
            "class_id": int(self.class_id),
            "class_name": self.class_name,
            "confidence": round(float(self.confidence), 4),
            "bbox": [round(float(v), 1) for v in self.bbox],
            "stamp": round(float(self.stamp), 6),
        }


@dataclass
class Localization:
    """L2/L3 输出：单个目标的几何估计（相机坐标系 + 可选的机器人坐标系）。"""

    range_m: float                       # 目标到机器人的水平距离（米）
    xyz_camera: Tuple[float, float, float]
    method: str = "none"                 # depth | ground | lidar | lidar+depth | none
    n_points: int = 0                    # 参与估计的像素/激光点数
    xyz_base: Optional[Tuple[float, float, float]] = None  # 机器人坐标系
    yaw_base: Optional[float] = None     # 目标相对机器人朝向（弧度，逆时针为正）

    def to_dict(self) -> dict:
        d = {
            "range_m": round(float(self.range_m), 3),
            "xyz_camera": [round(float(v), 3) for v in self.xyz_camera],
            "method": self.method,
            "n_points": int(self.n_points),
        }
        if self.xyz_base is not None:
            d["xyz_base"] = [round(float(v), 3) for v in self.xyz_base]
        if self.yaw_base is not None:
            d["yaw_base"] = round(float(self.yaw_base), 4)
        return d


@dataclass
class ObjectPose3D:
    """L4 输出：发给下游（语义导航 / RL）的一条目标记录。"""

    detection_id: int
    class_name: str
    confidence: float
    position: Tuple[float, float, float]  # 世界/地图坐标系下的 (x, y, z)
    frame_id: str = "map"
    localization: Optional[Localization] = None
    size_estimate: Optional[Tuple[float, float, float]] = None
    stamp: float = 0.0
    track_id: Optional[int] = None

    def to_dict(self) -> dict:
        d = {
            "detection_id": int(self.detection_id),
            "class_name": self.class_name,
            "confidence": round(float(self.confidence), 4),
            "position": [round(float(v), 3) for v in self.position],
            "frame_id": self.frame_id,
            "stamp": round(float(self.stamp), 6),
        }
        if self.localization is not None:
            d["localization"] = self.localization.to_dict()
        if self.size_estimate is not None:
            d["size_estimate"] = [round(float(v), 3) for v in self.size_estimate]
        if self.track_id is not None:
            d["track_id"] = int(self.track_id)
        return d


@dataclass
class PerceptionResult:
    """一帧的完整感知结果。"""

    detections: List[Detection2D] = field(default_factory=list)
    objects: List[ObjectPose3D] = field(default_factory=list)
    stamp: float = 0.0
    latency_ms: float = 0.0
    frame_id: str = "map"
    camera_frame_id: str = "camera_link"
    image_size: Optional[Tuple[int, int]] = None  # (width, height)

    def to_dict(self) -> dict:
        d = {
            "stamp": round(float(self.stamp), 6),
            "frame_id": self.frame_id,
            "camera_frame_id": self.camera_frame_id,
            "latency_ms": round(float(self.latency_ms), 2),
            "detections": [det.to_dict() for det in self.detections],
            "objects": [obj.to_dict() for obj in self.objects],
        }
        if self.image_size is not None:
            d["image_size"] = [int(self.image_size[0]), int(self.image_size[1])]
        return d

    def count_by_class(self) -> dict:
        counts: dict = {}
        for det in self.detections:
            counts[det.class_name] = counts.get(det.class_name, 0) + 1
        return counts


def nearest_object(result: PerceptionResult, class_name: str) -> Optional[ObjectPose3D]:
    """按类别挑出最近的一个目标，供语义导航使用。"""
    candidates = [o for o in result.objects if o.class_name == class_name]
    if not candidates:
        return None
    return min(
        candidates,
        key=lambda o: (o.localization.range_m if o.localization else float("inf")),
    )


__all__ = [
    "Detection2D",
    "Localization",
    "ObjectPose3D",
    "PerceptionResult",
    "nearest_object",
]
