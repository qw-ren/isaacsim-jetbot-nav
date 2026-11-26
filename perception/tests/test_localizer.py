"""L2 定位与 L3 融合的单元测试。"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from atongmu_perception.config import CameraConfig, ExtrinsicsConfig, FusionConfig, LocalizerConfig  # noqa: E402
from atongmu_perception.fusion import LidarCameraFusion  # noqa: E402
from atongmu_perception.localizer import Localizer  # noqa: E402
from atongmu_perception.types import Detection2D  # noqa: E402


def _make_localizer(camera_height=0.35, pitch_deg=0.0) -> Localizer:
    cam = CameraConfig(width=640, height=480)
    ext = ExtrinsicsConfig(camera_position=(0.0, 0.0, camera_height), camera_pitch_deg=pitch_deg)
    return Localizer(cam, ext, LocalizerConfig())


def test_depth_localization_center_object():
    loc = _make_localizer()
    det = Detection2D(0, "chair", 0.9, (260.0, 200.0, 380.0, 280.0))
    depth = np.full((480, 640), 9.0)
    depth[200:280, 260:380] = 4.0
    out = loc.localize(det, depth)
    assert out.method == "depth"
    assert abs(out.xyz_camera[2] - 4.0) < 1e-6
    # 相机在基座系 (0,0,0.35)，正前方 4m -> 基座系 (4,0,0.35)
    assert abs(out.xyz_base[0] - 4.0) < 1e-6
    assert abs(out.xyz_base[1]) < 1e-6
    assert abs(out.xyz_base[2] - 0.35) < 1e-6
    assert abs(out.range_m - 4.0) < 1e-6


def test_depth_ignores_far_background():
    loc = _make_localizer()
    # 框覆盖物体和身后的墙，物体只占中间一小块
    det = Detection2D(0, "person", 0.9, (200.0, 100.0, 440.0, 400.0))
    depth = np.full((480, 640), 9.0)
    depth[180:320, 280:360] = 3.5
    out = loc.localize(det, depth)
    assert out.method == "depth"
    assert abs(out.xyz_camera[2] - 3.5) < 1e-6


def test_ground_plane_fallback_without_depth():
    loc = _make_localizer(camera_height=0.35)
    det = Detection2D(0, "chair", 0.9, (280.0, 250.0, 360.0, 400.0))
    out = loc.localize(det, depth=None)
    assert out.method == "ground"
    assert out.xyz_base is not None
    assert abs(out.xyz_base[2]) < 1e-9  # 地面点
    assert out.range_m > 1.0


def test_merge_lidar_blends_and_reports_method():
    loc = _make_localizer()
    det = Detection2D(0, "person", 0.9, (280.0, 100.0, 360.0, 300.0))
    depth = np.full((480, 640), 4.0)
    out = loc.localize(det, depth)
    lidar_pt = np.array([0.0, 0.0, 3.8])
    merged = loc.merge_lidar(out, lidar_pt, n_points=12)
    assert merged.method == "lidar+depth"
    # 0.35*4.0 + 0.65*3.8 = 3.87
    assert abs(merged.xyz_camera[2] - 3.87) < 1e-6
    assert merged.n_points == 12
    assert abs(merged.xyz_base[0] - 3.87) < 1e-6  # 基座系坐标已重算


def test_merge_lidar_when_depth_unavailable():
    loc = _make_localizer()
    det = Detection2D(0, "person", 0.9, (280.0, 100.0, 360.0, 300.0))
    empty = loc.localize(det, depth=None)
    empty.method = "none"  # 模拟两条路径都失败
    merged = loc.merge_lidar(empty, np.array([0.0, 0.0, 5.0]), n_points=8)
    assert merged.method == "lidar"
    assert abs(merged.range_m - 5.0) < 1e-6


def test_no_localization_returns_nan():
    loc = _make_localizer()
    loc.cfg.use_ground_plane_fallback = False
    out = loc.localize(Detection2D(0, "cup", 0.5, (10.0, 10.0, 30.0, 30.0)), depth=None)
    assert out.method == "none"
    assert np.isnan(out.range_m)


def _make_fusion() -> LidarCameraFusion:
    cam = CameraConfig(width=640, height=480)
    ext = ExtrinsicsConfig(camera_position=(0.0, 0.0, 0.35), lidar_position=(0.0, 0.0, 0.20))
    return LidarCameraFusion(cam, ext, FusionConfig())


def test_lidar_scan_projection_and_association():
    fusion = _make_fusion()
    angles = np.linspace(-np.pi, np.pi, 720, endpoint=False)
    ranges = np.full(720, 8.0)
    ranges[350:370] = 3.0  # 正前方一小段：3m 处的墙面
    pts = fusion.scan_points_camera(ranges, float(angles[0]), float(angles[1] - angles[0]))
    assert pts.shape[1] == 3 and len(pts) > 0

    # 3m 处的点在雷达高度 0.2，投到图像上位于中心偏下
    det = Detection2D(0, "box", 0.9, (300.0, 250.0, 340.0, 290.0))
    out = fusion.associate([det], pts, rough_depth=[3.0])
    assert 0 in out
    assert abs(out[0].median_depth - 3.0) < 0.1
    assert out[0].n_points >= 3


def test_lidar_depth_window_rejects_wrong_cluster():
    fusion = _make_fusion()
    angles = np.linspace(-np.pi, np.pi, 720, endpoint=False)
    ranges = np.full(720, 3.0)
    pts = fusion.scan_points_camera(ranges, float(angles[0]), float(angles[1] - angles[0]))
    det = Detection2D(0, "box", 0.9, (300.0, 250.0, 340.0, 290.0))
    # 深度图说目标在 8m，而激光看到的是 3m 的别的东西 -> 不应关联
    assert fusion.associate([det], pts, rough_depth=[8.0]) == {}
    # 不限制纵深时应能关联上
    assert 0 in fusion.associate([det], pts, rough_depth=None)


def test_empty_scan_is_safe():
    fusion = _make_fusion()
    assert fusion.associate([Detection2D(0, "box", 0.9, (300.0, 250.0, 340.0, 290.0))], np.zeros((0, 3))) == {}
    assert fusion.scan_points_camera([], 0.0, 0.01).shape == (0, 3)


def test_pipeline_tf_override_changes_alignment():
    from atongmu_perception import PerceptionConfig, PerceptionPipeline
    from atongmu_perception.geometry import camera_pose_transform, make_transform

    cfg = PerceptionConfig()
    cfg.fusion.enabled = False
    pipeline = PerceptionPipeline(cfg, detector=object())  # 这里不跑检测

    det = Detection2D(0, "chair", 0.9, (280.0, 200.0, 360.0, 280.0))
    depth = np.full((480, 640), 4.0)

    cfg_position = pipeline.process(detections=[det], depth=depth).objects[0].position
    assert abs(cfg_position[0] - 4.0) < 1e-6  # 配置：相机在 (0,0,0.35)

    # TF 说相机其实在车头前方 1m、高 0.5m
    pipeline.set_transforms(T_base_camera=camera_pose_transform((1.0, 0.0, 0.5)))
    tf_position = pipeline.process(detections=[det], depth=depth).objects[0].position
    assert abs(tf_position[0] - 5.0) < 1e-6
    assert abs(tf_position[2] - 0.5) < 1e-6

    # 再叠加地图位姿，输出应转到 map 系
    result = pipeline.process(detections=[det], depth=depth, T_map_base=make_transform((10.0, 0.0, 0.0)))
    assert result.frame_id == "map"
    assert abs(result.objects[0].position[0] - 15.0) < 1e-6
