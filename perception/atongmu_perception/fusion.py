"""L3 融合层：激光点云投影到图像，与检测框做数据关联。

为什么需要它：
- 深度相机对玻璃、黑色物体、远距离目标容易失效，激光雷达在这些情况下仍然可靠；
- 激光雷达只有几何、没有语义，YOLO 补上了「这是什么」；
- 两者关联后，每个目标同时拥有「类别」和「高置信度距离」。

2D 单线激光的做法：把 scan 点投到图像上，落在检测框内、且纵深在合理窗口内的点视为同一目标，
取其深度中位数作为该目标的距离估计。
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional, Sequence

import numpy as np

from .config import CameraConfig, ExtrinsicsConfig, FusionConfig
from .geometry import project_points, transform_points
from .types import Detection2D


@dataclass
class FusionResult:
    detection_index: int
    n_points: int
    xyz_camera: np.ndarray
    range_m: float
    median_depth: float


class LidarCameraFusion:
    def __init__(
        self,
        camera: CameraConfig,
        extrinsics: ExtrinsicsConfig,
        cfg: FusionConfig,
        T_camera_laser: Optional[np.ndarray] = None,
    ) -> None:
        self.camera = camera
        self.extrinsics = extrinsics
        self.cfg = cfg
        self.K = camera.intrinsics().K
        # 4x4 矩阵可直接由 TF 得到，覆盖配置里的安装位姿
        self.T_camera_laser = (
            np.asarray(T_camera_laser, dtype=np.float64)
            if T_camera_laser is not None
            else extrinsics.T_camera_lidar()
        )

    # ------------------------------------------------------------------ #
    def scan_points_camera(
        self,
        ranges: Sequence[float],
        angle_min: float,
        angle_increment: float,
        angle_offset: Optional[float] = None,
    ) -> np.ndarray:
        """把一帧 LaserScan 转成相机系三维点。

        ranges: 距离数组（米），无效值为 inf/nan 会被剔除。
        angle_min / angle_increment: 与 sensor_msgs/LaserScan 同义。
        """
        ranges = np.asarray(ranges, dtype=np.float64)
        n = ranges.size
        if n == 0:
            return np.zeros((0, 3), dtype=np.float64)
        offset = np.deg2rad(self.cfg.scan_angle_offset_deg) if angle_offset is None else float(angle_offset)
        angles = angle_min + np.arange(n, dtype=np.float64) * angle_increment + offset
        valid = np.isfinite(ranges) & (ranges > 1e-3) & (ranges <= self.cfg.max_range)
        if not np.any(valid):
            return np.zeros((0, 3), dtype=np.float64)
        r = ranges[valid]
        a = angles[valid]
        pts_laser = np.stack([r * np.cos(a), r * np.sin(a), np.zeros_like(r)], axis=1)
        return transform_points(self.T_camera_laser, pts_laser)

    # ------------------------------------------------------------------ #
    def associate(
        self,
        detections: Sequence[Detection2D],
        scan_points_camera: np.ndarray,
        rough_depth: Optional[Sequence[Optional[float]]] = None,
    ) -> Dict[int, FusionResult]:
        """为每个检测框找出与其关联的激光点。

        rough_depth: 可选，每个检测框的粗略光轴深度（通常来自深度图），
                     用来把关联限制在 ±depth_window_ratio 的纵深范围内，避免把背景点算进来。
        返回 {detection_index: FusionResult}，没有关联上的框不会出现在结果里。
        """
        if scan_points_camera is None or len(scan_points_camera) == 0 or not detections:
            return {}
        pts = np.asarray(scan_points_camera, dtype=np.float64).reshape(-1, 3)
        uv, z, in_front = project_points(pts, self.K)
        h, w = self.camera.height, self.camera.width
        inside = (
            in_front
            & (uv[:, 0] >= 0)
            & (uv[:, 0] < w)
            & (uv[:, 1] >= 0)
            & (uv[:, 1] < h)
            & (z > self.camera.depth_min)
            & (z < self.cfg.max_range)
        )
        if not np.any(inside):
            return {}
        pts_i, uv_i, z_i = pts[inside], uv[inside], z[inside]

        results: Dict[int, FusionResult] = {}
        margin = float(self.cfg.bbox_margin_px)
        for idx, det in enumerate(detections):
            x1, y1, x2, y2 = det.bbox
            in_box = (
                (uv_i[:, 0] >= x1 - margin)
                & (uv_i[:, 0] <= x2 + margin)
                & (uv_i[:, 1] >= y1 - margin)
                & (uv_i[:, 1] <= y2 + margin)
            )
            if rough_depth is not None and idx < len(rough_depth):
                ref = rough_depth[idx]
                if ref is not None and np.isfinite(ref) and ref > 0:
                    ratio = float(self.cfg.depth_window_ratio)
                    in_box &= (z_i >= ref * (1.0 - ratio)) & (z_i <= ref * (1.0 + ratio))
            if int(in_box.sum()) < self.cfg.min_points:
                continue
            sel = pts_i[in_box]
            z_sel = z_i[in_box]
            # 单线雷达在目标表面通常是一段连续点；取中位数深度对应的一小段点做代表
            med_z = float(np.median(z_sel))
            band = np.abs(z_sel - med_z) <= max(0.05, 0.02 * med_z)
            band_pts = sel[band] if int(band.sum()) >= self.cfg.min_points else sel
            center = np.median(band_pts, axis=0)
            center[2] = med_z
            results[idx] = FusionResult(
                detection_index=idx,
                n_points=int(band_pts.shape[0]),
                xyz_camera=center,
                range_m=float(np.hypot(center[0], center[1])),
                median_depth=med_z,
            )
        return results


__all__ = ["FusionResult", "LidarCameraFusion"]
