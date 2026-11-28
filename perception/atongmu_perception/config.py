"""感知模块配置：与 perception/configs/perception.yaml 一一对应。

所有标定参数都可以在 Isaac Sim 里通过 CameraInfo 和 TF 自动获取；
这里的数值只是离线 demo 的默认值，接入仿真后应以 /camera/*/camera_info 和 TF 为准。
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Sequence

import numpy as np

from .geometry import CameraIntrinsics, camera_pose_transform, invert_transform, make_transform, rotation_about_axis


@dataclass
class CameraConfig:
    fx: float = 554.26
    fy: float = 554.26
    cx: float = 320.0
    cy: float = 240.0
    width: int = 640
    height: int = 480
    depth_scale: float = 1.0          # 深度图单位 -> 米（Isaac Sim 的 depth 通常是米，故为 1.0）
    depth_min: float = 0.05
    depth_max: float = 10.0

    def intrinsics(self) -> CameraIntrinsics:
        return CameraIntrinsics(self.fx, self.fy, self.cx, self.cy, self.width, self.height)

    @classmethod
    def from_dict(cls, d: Optional[dict]) -> "CameraConfig":
        d = d or {}
        return cls(**{k: d[k] for k in d if hasattr(cls, k)})


@dataclass
class ExtrinsicsConfig:
    """相机 / 雷达在机器人基座系下的安装位姿。

    camera_pitch_deg 为正表示镜头向下俯视；lidar_yaw_deg 为正表示逆时针（左转）。
    """

    camera_position: Sequence[float] = (0.0, 0.0, 0.35)
    camera_pitch_deg: float = 0.0
    camera_yaw_deg: float = 0.0
    camera_roll_deg: float = 0.0
    lidar_position: Sequence[float] = (0.0, 0.0, 0.20)
    lidar_yaw_deg: float = 0.0
    lidar_pitch_deg: float = 0.0

    def T_base_camera(self) -> np.ndarray:
        d2r = np.pi / 180.0
        return camera_pose_transform(
            position_xyz=self.camera_position,
            pitch=self.camera_pitch_deg * d2r,
            yaw=self.camera_yaw_deg * d2r,
            roll=self.camera_roll_deg * d2r,
        )

    def T_base_lidar(self) -> np.ndarray:
        d2r = np.pi / 180.0
        R = (
            rotation_about_axis("z", self.lidar_yaw_deg * d2r)
            @ rotation_about_axis("y", self.lidar_pitch_deg * d2r)
        )
        return make_transform(self.lidar_position, rotation=R)

    def T_camera_lidar(self) -> np.ndarray:
        """激光点（laser 系）-> 相机系。"""
        return invert_transform(self.T_base_camera()) @ self.T_base_lidar()

    @property
    def camera_height(self) -> float:
        return float(self.camera_position[2])

    @classmethod
    def from_dict(cls, d: Optional[dict]) -> "ExtrinsicsConfig":
        d = d or {}
        known = {k: d[k] for k in d if hasattr(cls, k)}
        if "camera_position" in known:
            known["camera_position"] = tuple(float(v) for v in known["camera_position"])
        if "lidar_position" in known:
            known["lidar_position"] = tuple(float(v) for v in known["lidar_position"])
        return cls(**known)


@dataclass
class DetectorConfig:
    weights: str = "weights/yolo26s.pt"
    device: str = "cpu"
    imgsz: int = 640
    conf: float = 0.35
    iou: float = 0.5
    half: bool = False
    max_det: int = 100
    # 只保留这些类别名（COCO 名）。空列表 = 保留全部 80 类。
    classes: List[str] = field(
        default_factory=lambda: [
            "person",
            "chair",
            "dining table",
            "couch",
            "bottle",
            "cup",
            "backpack",
            "potted plant",
            "laptop",
        ]
    )

    @classmethod
    def from_dict(cls, d: Optional[dict]) -> "DetectorConfig":
        d = d or {}
        # 注意：不能写 hasattr(cls, k)。classes 用的是 field(default_factory=...)，
        # 类上没有同名属性，hasattr 恒为 False，yaml 里的 classes 会被静默丢掉。
        from dataclasses import fields

        valid = {f.name for f in fields(cls)}
        return cls(**{k: d[k] for k in d if k in valid})


@dataclass
class LocalizerConfig:
    shrink: float = 0.25            # bbox 内部收缩比例，避开背景渗色
    depth_percentile: float = 20.0  # 深度采样分位数，偏向近处
    min_valid_pixels: int = 8
    max_range: float = 12.0
    # 没有深度图时的兜底方案
    use_ground_plane_fallback: bool = True
    ground_plane_pixel: str = "bottom_center"  # bottom_center | center

    @classmethod
    def from_dict(cls, d: Optional[dict]) -> "LocalizerConfig":
        d = d or {}
        return cls(**{k: d[k] for k in d if hasattr(cls, k)})


@dataclass
class FusionConfig:
    enabled: bool = True
    bbox_margin_px: int = 12        # 关联时把框往外扩几个像素
    min_points: int = 3             # 少于这个激光点数就不采信
    depth_window_ratio: float = 0.35  # 用深度限制激光点的纵深范围（±35%）
    max_range: float = 12.0
    scan_angle_offset_deg: float = 0.0  # 雷达角度 0 方向与 laser x 轴的偏差

    @classmethod
    def from_dict(cls, d: Optional[dict]) -> "FusionConfig":
        d = d or {}
        return cls(**{k: d[k] for k in d if hasattr(cls, k)})


@dataclass
class TrackerConfig:
    """跨帧目标跟踪参数（IoU + 匈牙利匹配）。"""

    enabled: bool = True
    iou_threshold: float = 0.3     # 低于这个 IoU 不认为是同一个目标
    max_age: int = 15              # 连续丢失多少帧后删除轨迹（8 FPS 约 2 秒）
    min_hits: int = 2              # 累计命中多少帧后才对外暴露 track_id
    class_aware: bool = True       # 只允许同类别目标互相匹配
    velocity_prediction: bool = True
    velocity_smoothing: float = 0.3  # 新速度的权重，越小越平滑
    max_speed_px_s: float = 4000.0   # 速度上限，防止误匹配把轨迹甩飞

    @classmethod
    def from_dict(cls, d: Optional[dict]) -> "TrackerConfig":
        d = d or {}
        return cls(**{k: d[k] for k in d if hasattr(cls, k)})


@dataclass
class PerceptionConfig:
    camera: CameraConfig = field(default_factory=CameraConfig)
    extrinsics: ExtrinsicsConfig = field(default_factory=ExtrinsicsConfig)
    detector: DetectorConfig = field(default_factory=DetectorConfig)
    localizer: LocalizerConfig = field(default_factory=LocalizerConfig)
    fusion: FusionConfig = field(default_factory=FusionConfig)
    tracker: TrackerConfig = field(default_factory=TrackerConfig)

    @classmethod
    def from_dict(cls, d: dict) -> "PerceptionConfig":
        d = d or {}
        return cls(
            camera=CameraConfig.from_dict(d.get("camera")),
            extrinsics=ExtrinsicsConfig.from_dict(d.get("extrinsics")),
            detector=DetectorConfig.from_dict(d.get("detector")),
            localizer=LocalizerConfig.from_dict(d.get("localizer")),
            fusion=FusionConfig.from_dict(d.get("fusion")),
            tracker=TrackerConfig.from_dict(d.get("tracker")),
        )

    @classmethod
    def from_yaml(cls, path: str) -> "PerceptionConfig":
        import yaml

        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        return cls.from_dict(data)


__all__ = [
    "CameraConfig",
    "ExtrinsicsConfig",
    "DetectorConfig",
    "LocalizerConfig",
    "FusionConfig",
    "TrackerConfig",
    "PerceptionConfig",
]
