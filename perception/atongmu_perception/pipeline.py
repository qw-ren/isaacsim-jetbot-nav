"""感知主流程：detector -> localizer -> lidar fusion -> ObjectPose3D。

建议的调用方式见 perception/README.md。ROS2 节点只是在这个类外面套一层订阅/发布。
"""
from __future__ import annotations

import time
from typing import List, Optional, Sequence, Tuple

import numpy as np

from .config import PerceptionConfig
from .detector import YoloDetector
from .fusion import LidarCameraFusion
from .geometry import CameraIntrinsics, transform_points
from .localizer import Localizer
from .tracker import IouTracker
from .types import Detection2D, ObjectPose3D, PerceptionResult


class PerceptionPipeline:
    def __init__(
        self,
        cfg: PerceptionConfig,
        detector: Optional[YoloDetector] = None,
        verbose: bool = False,
    ) -> None:
        self.cfg = cfg
        self.verbose = verbose
        self.intrinsics: CameraIntrinsics = cfg.camera.intrinsics()
        self._T_base_camera: Optional[np.ndarray] = None
        self._T_camera_laser: Optional[np.ndarray] = None
        self.localizer = Localizer(cfg.camera, cfg.extrinsics, cfg.localizer)
        self.fusion = LidarCameraFusion(cfg.camera, cfg.extrinsics, cfg.fusion) if cfg.fusion.enabled else None
        self.tracker = IouTracker(cfg.tracker) if cfg.tracker.enabled else None
        self.detector = detector if detector is not None else YoloDetector.from_config(cfg.detector)
        self.n_frames = 0
        self.last_result: Optional[PerceptionResult] = None

    # ------------------------------------------------------------------ #
    def set_intrinsics(self, intrinsics: CameraIntrinsics) -> None:
        """用真实 CameraInfo 覆盖配置里的内参（接入 Isaac Sim 后必须调用一次）。"""
        self.intrinsics = intrinsics
        self.cfg.camera = type(self.cfg.camera)(
            fx=intrinsics.fx,
            fy=intrinsics.fy,
            cx=intrinsics.cx,
            cy=intrinsics.cy,
            width=intrinsics.width,
            height=intrinsics.height,
            depth_scale=self.cfg.camera.depth_scale,
            depth_min=self.cfg.camera.depth_min,
            depth_max=self.cfg.camera.depth_max,
        )
        self._rebuild_geometry()

    def set_transforms(
        self,
        T_base_camera: Optional[np.ndarray] = None,
        T_camera_laser: Optional[np.ndarray] = None,
    ) -> None:
        """用 TF 查到的真实外参覆盖配置（T_base_camera: 相机光学系->base_link，

        T_camera_laser: laser->相机光学系）。传 None 表示沿用配置里的值。
        """
        self._T_base_camera = T_base_camera
        self._T_camera_laser = T_camera_laser
        self._rebuild_geometry()

    def _rebuild_geometry(self) -> None:
        self.localizer = Localizer(
            self.cfg.camera, self.cfg.extrinsics, self.cfg.localizer, T_base_camera=self._T_base_camera
        )
        if self.fusion is not None:
            self.fusion = LidarCameraFusion(
                self.cfg.camera, self.cfg.extrinsics, self.cfg.fusion, T_camera_laser=self._T_camera_laser
            )

    # ------------------------------------------------------------------ #
    def process(
        self,
        image: Optional[np.ndarray] = None,
        depth: Optional[np.ndarray] = None,
        depth_scale: Optional[float] = None,
        scan: Optional[Tuple[Sequence[float], float, float]] = None,
        T_map_base: Optional[np.ndarray] = None,
        stamp: float = 0.0,
        detections: Optional[List[Detection2D]] = None,
        base_frame: str = "base_link",
        output_frame: str = "map",
        estimate_sizes: bool = True,
    ) -> PerceptionResult:
        """处理一帧。

        image:      BGR 图像；如果传了 detections 则可以不传图像。
        depth:      深度图（与图像同尺寸），单位由 depth_scale 指定。
        scan:       (ranges, angle_min, angle_increment)，与 sensor_msgs/LaserScan 一致。
        T_map_base: 4x4，机器人基座系 -> 地图系，用于把局部坐标变成世界坐标。
        """
        t0 = time.perf_counter()
        if detections is None:
            if image is None:
                raise ValueError("process() 需要 image 或 detections 二者之一")
            detections = self.detector.detect(image, stamp=stamp)

        # 跟踪在 2D 框上做，和定位是否成功无关；ID 只对「确认过」的轨迹暴露
        if self.tracker is not None:
            track_ids: List[Optional[int]] = list(self.tracker.update(detections, stamp=stamp))
        else:
            track_ids = [None] * len(detections)

        localizations = [self.localizer.localize(det, depth, depth_scale) for det in detections]

        if self.fusion is not None and scan is not None and len(detections) > 0:
            ranges, angle_min, angle_increment = scan
            pts_cam = self.fusion.scan_points_camera(ranges, angle_min, angle_increment)
            rough = [
                (loc.xyz_camera[2] if loc.method == "depth" and np.isfinite(loc.xyz_camera[2]) else None)
                for loc in localizations
            ]
            fused = self.fusion.associate(detections, pts_cam, rough_depth=rough)
            for idx, item in fused.items():
                localizations[idx] = self.localizer.merge_lidar(
                    localizations[idx], item.xyz_camera, item.n_points
                )

        objects: List[ObjectPose3D] = []
        for idx, (det, loc) in enumerate(zip(detections, localizations)):
            if loc.xyz_base is None:
                continue  # 完全无法定位的目标不输出，避免下游拿到 NaN
            p_base = np.asarray(loc.xyz_base, dtype=np.float64)
            if T_map_base is not None:
                p_map = transform_points(T_map_base, p_base)
                frame_id = output_frame
            else:
                p_map = p_base
                frame_id = base_frame
            track_id = track_ids[idx]
            if track_id is not None and self.tracker is not None and not self.tracker.is_confirmed(track_id):
                track_id = None
            objects.append(
                ObjectPose3D(
                    detection_id=idx,
                    class_name=det.class_name,
                    confidence=det.confidence,
                    position=tuple(float(v) for v in p_map),
                    frame_id=frame_id,
                    localization=loc,
                    size_estimate=self.localizer.estimate_size(det, loc) if estimate_sizes else None,
                    stamp=stamp,
                    track_id=track_id,
                )
            )

        img_hw = None
        if image is not None:
            img_hw = (int(image.shape[1]), int(image.shape[0]))

        result = PerceptionResult(
            detections=detections,
            objects=objects,
            stamp=stamp,
            latency_ms=(time.perf_counter() - t0) * 1000.0,
            frame_id=output_frame if T_map_base is not None else base_frame,
            image_size=img_hw,
        )
        self.n_frames += 1
        self.last_result = result
        if self.verbose:
            print(
                "[perception] frame=%d dets=%d objs=%d latency=%.1fms"
                % (self.n_frames, len(detections), len(objects), result.latency_ms)
            )
        return result


__all__ = ["PerceptionPipeline"]
