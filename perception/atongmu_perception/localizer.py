"""L2 几何定位层：把 2D 框 + 深度图 变成相机系 / 基座系下的三维位置。

三条路径按优先级：
1. 深度图可用 -> 在框内采样深度，反投影得到三维点（最准，依赖 RGB-D）；
2. 只有 RGB   -> 地面约束：假设框底边落在地面，射线与 z=0 平面求交（兜底，目标悬空时会偏大）；
3. 激光关联   -> 由 fusion 模块给出（在 pipeline 中做二次精化）。

一个目标最终只输出一个 Localization，method 字段标明来源，方便下游判断置信度。
"""
from __future__ import annotations

from typing import Optional, Tuple

import numpy as np

from .config import CameraConfig, ExtrinsicsConfig, LocalizerConfig
from .geometry import (
    backproject_pixels,
    bbox_region,
    bbox_size_estimate,
    ground_plane_point,
    invert_transform,
    sample_depth_in_bbox,
    transform_points,
)
from .types import Detection2D, Localization


class Localizer:
    def __init__(
        self,
        camera: CameraConfig,
        extrinsics: ExtrinsicsConfig,
        cfg: LocalizerConfig,
        T_base_camera: Optional[np.ndarray] = None,
    ) -> None:
        self.camera = camera
        self.extrinsics = extrinsics
        self.cfg = cfg
        self.K = camera.intrinsics().K
        # T_base_camera 可由 TF 直接给出，覆盖配置里手工填的安装位姿
        self.T_base_camera = np.asarray(T_base_camera, dtype=np.float64) if T_base_camera is not None else extrinsics.T_base_camera()
        self.R_base_camera = self.T_base_camera[:3, :3]
        self.T_camera_base = invert_transform(self.T_base_camera)
        self.camera_position = tuple(float(v) for v in self.T_base_camera[:3, 3])

    # ------------------------------------------------------------------ #
    def localize(
        self,
        det: Detection2D,
        depth: Optional[np.ndarray] = None,
        depth_scale: Optional[float] = None,
    ) -> Localization:
        depth_localization = self._from_depth(det, depth, depth_scale)
        if depth_localization is not None:
            return depth_localization
        if self.cfg.use_ground_plane_fallback:
            ground = self._from_ground_plane(det)
            if ground is not None:
                return ground
        return Localization(range_m=float("nan"), xyz_camera=(float("nan"),) * 3, method="none")

    # ------------------------------------------------------------------ #
    def _from_depth(
        self,
        det: Detection2D,
        depth: Optional[np.ndarray],
        depth_scale: Optional[float],
    ) -> Optional[Localization]:
        if depth is None:
            return None
        scale = self.camera.depth_scale if depth_scale is None else float(depth_scale)
        depth_m = np.asarray(depth, dtype=np.float64) * scale
        z = sample_depth_in_bbox(
            depth_m,
            det.bbox,
            shrink=self.cfg.shrink,
            min_depth=self.camera.depth_min,
            max_depth=min(self.camera.depth_max, self.cfg.max_range),
            percentile=self.cfg.depth_percentile,
            min_valid_pixels=self.cfg.min_valid_pixels,
        )
        if z is None:
            return None
        # 用框内收缩区域的中心像素 + 采样深度反投影，避开框边缘的背景像素
        x1, y1, x2, y2 = bbox_region(det.bbox, self.cfg.shrink)
        u, v = (x1 + x2) * 0.5, (y1 + y2) * 0.5
        xyz_cam = backproject_pixels(np.array([[u, v]]), np.array([z]), self.K)[0]
        n_points = _count_valid_pixels(depth_m, det.bbox, self.cfg, self.camera.depth_min, self.cfg.max_range)
        return self._build(xyz_cam, method="depth", n_points=n_points)

    def _from_ground_plane(self, det: Detection2D) -> Optional[Localization]:
        pixel = det.bottom_center if self.cfg.ground_plane_pixel == "bottom_center" else det.center
        out = ground_plane_point(
            pixel,
            self.K,
            self.R_base_camera,
            self.camera_position,
            self.cfg.max_range,
        )
        if out is None:
            return None
        point_base, rng = out
        xyz_cam = transform_points(self.T_camera_base, point_base)
        # 地面点的高度为 0，投影到相机系后 z 分量即光轴距离，但这里直接用基座系结果
        return Localization(
            range_m=float(rng),
            xyz_camera=tuple(float(v) for v in xyz_cam),
            method="ground",
            n_points=0,
            xyz_base=tuple(float(v) for v in point_base),
            yaw_base=float(np.arctan2(point_base[1], point_base[0])),
        )

    # ------------------------------------------------------------------ #
    def _build(self, xyz_cam: np.ndarray, method: str, n_points: int) -> Localization:
        point_base = transform_points(self.T_base_camera, xyz_cam)
        rng = float(np.hypot(point_base[0], point_base[1]))
        return Localization(
            range_m=rng,
            xyz_camera=tuple(float(v) for v in xyz_cam),
            method=method,
            n_points=int(n_points),
            xyz_base=tuple(float(v) for v in point_base),
            yaw_base=float(np.arctan2(point_base[1], point_base[0])),
        )

    def estimate_size(self, det: Detection2D, loc: Localization) -> Optional[Tuple[float, float, float]]:
        """由框尺寸 + 深度估计目标宽高（深向尺寸无法从单视角得到，填 NaN）。"""
        if loc.method not in ("depth", "lidar", "lidar+depth") or not np.isfinite(loc.xyz_camera[2]):
            return None
        w, h = bbox_size_estimate(det.bbox, loc.xyz_camera[2], self.K)
        return (float(w), float(h), float("nan"))

    def merge_lidar(self, loc: Localization, lidar_xyz_cam: np.ndarray, n_points: int) -> Localization:
        """用激光测量替换/融合深度测距结果，并重算基座系坐标。"""
        if loc.method == "none":
            return self._build(lidar_xyz_cam, method="lidar", n_points=n_points)
        # 深度与激光都可信时取两者光轴深度的加权平均（激光权重更高：不受纹理和光照影响）
        z_depth = float(loc.xyz_camera[2])
        z_lidar = float(lidar_xyz_cam[2])
        z_fused = 0.35 * z_depth + 0.65 * z_lidar if z_depth > 0 and z_lidar > 0 else z_lidar
        # 保持像素方向不变，只替换深度
        scale = z_fused / z_lidar if z_lidar > 0 else 1.0
        fused = np.asarray(lidar_xyz_cam, dtype=np.float64) * scale
        fused[2] = z_fused
        return self._build(fused, method="lidar+depth", n_points=n_points)


def _count_valid_pixels(
    depth_m: np.ndarray,
    bbox,
    cfg: LocalizerConfig,
    depth_min: float,
    depth_max: float,
) -> int:
    """统计框内有效深度像素数，作为该次测距的「证据量」随结果一起输出。"""
    h, w = depth_m.shape
    x1, y1, x2, y2 = bbox_region(bbox, cfg.shrink)
    x1, y1 = max(0, min(x1, w - 1)), max(0, min(y1, h - 1))
    x2, y2 = max(x1 + 1, min(x2, w)), max(y1 + 1, min(y2, h))
    patch = depth_m[y1:y2, x1:x2]
    valid = np.isfinite(patch) & (patch >= depth_min) & (patch <= depth_max)
    return int(valid.sum())


__all__ = ["Localizer"]
