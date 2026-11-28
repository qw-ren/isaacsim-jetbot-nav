"""阿童木课题 · 视觉感知模块

分层：
    L1 语义理解   detector.YoloDetector       图像 -> Detection2D
    L2 几何定位   localizer.Localizer         框+深度 -> 相机系/基座系坐标
    L3 融合       fusion.LidarCameraFusion    激光点 <-> 检测框 数据关联
    L4 标准接口   types / pipeline            统一的数据结构与 ROS2 话题

只依赖 numpy/opencv；YOLO 相关依赖在 detector 内部延迟导入。
"""
from .config import (
    CameraConfig,
    DetectorConfig,
    ExtrinsicsConfig,
    FusionConfig,
    LocalizerConfig,
    PerceptionConfig,
    TrackerConfig,
)
from .detector import YoloDetector
from .fusion import FusionResult, LidarCameraFusion
from .geometry import CameraIntrinsics, camera_pose_transform, make_transform
from .localizer import Localizer
from .pipeline import PerceptionPipeline
from .tracker import IouTracker, Track, bbox_iou, iou_matrix
from .types import Detection2D, Localization, ObjectPose3D, PerceptionResult, nearest_object

__version__ = "0.1.0"

__all__ = [
    "CameraConfig",
    "CameraIntrinsics",
    "Detection2D",
    "DetectorConfig",
    "ExtrinsicsConfig",
    "FusionConfig",
    "FusionResult",
    "LidarCameraFusion",
    "Localization",
    "Localizer",
    "ObjectPose3D",
    "PerceptionConfig",
    "PerceptionPipeline",
    "PerceptionResult",
    "YoloDetector",
    "camera_pose_transform",
    "make_transform",
    "nearest_object",
]
