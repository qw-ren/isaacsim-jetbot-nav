"""几何模块的单元测试。

不依赖 pytest 也能跑：python tests/run_all.py
装了 pytest 也可以：pytest perception/tests
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from atongmu_perception.geometry import (  # noqa: E402
    CameraIntrinsics,
    bbox_region,
    camera_pose_transform,
    ground_plane_point,
    invert_transform,
    make_transform,
    matrix_to_quaternion_xyzw,
    pixels_to_rays,
    project_points,
    quaternion_xyzw_to_matrix,
    sample_depth_in_bbox,
    transform_points,
)

K = CameraIntrinsics(554.26, 554.26, 320.0, 240.0, 640, 480)


def test_intrinsics_matrix():
    Km = K.K
    assert Km.shape == (3, 3)
    assert abs(Km[0, 0] - 554.26) < 1e-9
    assert CameraIntrinsics.from_matrix(Km, 640, 480).fx == K.fx


def test_project_backproject_roundtrip():
    pixels = np.array([[100.0, 80.0], [320.0, 240.0], [600.0, 450.0]])
    depths = np.array([1.5, 4.0, 9.0])
    pts = np.array(
        [
            [(100 - K.cx) * 1.5 / K.fx, (80 - K.cy) * 1.5 / K.fy, 1.5],
            [0.0, 0.0, 4.0],
            [(600 - K.cx) * 9.0 / K.fx, (450 - K.cy) * 9.0 / K.fy, 9.0],
        ]
    )
    uv, z, valid = project_points(pts, K.K)
    assert np.all(valid)
    assert np.allclose(uv, pixels, atol=1e-6)
    assert np.allclose(z, depths)


def test_camera_pose_orientation():
    T = camera_pose_transform((0.0, 0.0, 0.35), pitch=0.0)
    R = T[:3, :3]
    assert abs(np.linalg.det(R) - 1.0) < 1e-9
    assert np.allclose(R @ R.T, np.eye(3), atol=1e-9)
    # 相机光轴 (0,0,1) 在基座系里应指向 +x
    assert np.allclose(R @ np.array([0, 0, 1.0]), [1.0, 0.0, 0.0], atol=1e-9)
    # 图像中心像素的射线 -> 正前方，且位于安装高度
    ray = pixels_to_rays(np.array([[K.cx, K.cy]]), K.K)[0]
    assert np.allclose(R @ ray, [1.0, 0.0, 0.0], atol=1e-9)


def test_pitch_down_tilts_optical_axis():
    T = camera_pose_transform((0.0, 0.0, 0.5), pitch=np.deg2rad(10.0))
    axis_base = T[:3, :3] @ np.array([0.0, 0.0, 1.0])
    # 低头时，光轴应同时有 +x 和 -z 分量，且与水平面夹角为 -10°
    assert axis_base[0] > 0.9
    assert axis_base[2] < 0.0
    assert abs(np.deg2rad(-10.0) - np.arctan2(axis_base[2], axis_base[0])) < 1e-6


def test_transform_inverse_and_points():
    T = make_transform([1.0, 2.0, 3.0], quaternion_xyzw=(0.0, 0.0, np.sin(np.pi / 8), np.cos(np.pi / 8)))
    pts = np.array([[0.0, 0.0, 0.0], [1.0, -2.0, 0.5]])
    back = transform_points(invert_transform(T), transform_points(T, pts))
    assert np.allclose(back, pts, atol=1e-9)


def test_quaternion_roundtrip():
    R = camera_pose_transform((0.1, 0.2, 0.3), pitch=0.3, yaw=0.7, roll=0.1)[:3, :3]
    q = matrix_to_quaternion_xyzw(R)
    assert np.allclose(quaternion_xyzw_to_matrix(q), R, atol=1e-9)


def test_ground_plane_point_analytic():
    T = camera_pose_transform((0.0, 0.0, 0.35), pitch=0.0)
    out = ground_plane_point((320.0, 400.0), K.K, T[:3, :3], (0.0, 0.0, 0.35))
    assert out is not None
    point, rng = out
    # 手算：ray_cam=(0,0.2888,1) 归一化后转到基座系 (0.9607,0,-0.2777)，t=0.35/0.2777
    assert abs(point[2]) < 1e-9  # 落在地面
    assert abs(point[0] - 1.2124) < 1e-3
    assert abs(point[1]) < 1e-9
    assert abs(rng - 1.2124) < 1e-3


def test_ground_plane_respects_camera_offset():
    # 相机装在车头前方 0.5m：地面落点必须带上这个偏移（曾经漏加，被本用例抓出来）
    T = camera_pose_transform((0.5, 0.0, 0.35), pitch=0.0)
    point, _ = ground_plane_point((320.0, 400.0), K.K, T[:3, :3], (0.5, 0.0, 0.35))
    assert abs(point[0] - 1.7124) < 1e-3
    assert abs(point[2]) < 1e-9


def test_ground_plane_no_intersection():
    T = camera_pose_transform((0.0, 0.0, 0.35), pitch=0.0)
    # 图像上半部分射线朝上，打不到地面
    assert ground_plane_point((320.0, 200.0), K.K, T[:3, :3], (0.0, 0.0, 0.35)) is None


def test_bbox_region_shrink_inside():
    x1, y1, x2, y2 = bbox_region((100.0, 50.0, 300.0, 250.0), shrink=0.25)
    assert x1 > 100 and x2 < 300 and y1 > 50 and y2 < 250
    # 极小框也要至少返回 1 像素宽高
    x1, y1, x2, y2 = bbox_region((10.0, 10.0, 10.5, 10.5), shrink=0.25)
    assert x2 > x1 and y2 > y1


def test_sample_depth_prefers_object_over_background():
    depth = np.full((480, 640), 9.0)
    depth[150:300, 200:400] = 3.0
    z = sample_depth_in_bbox(depth, (200.0, 150.0, 400.0, 300.0), shrink=0.2)
    assert z is not None and abs(z - 3.0) < 1e-6


def test_sample_depth_ignores_invalid_values():
    depth = np.full((480, 640), 4.0)
    depth[210:230, 250:390] = np.nan   # 中间一块无效
    depth[240:245, 250:390] = np.inf
    depth[255:260, 250:390] = 0.0
    z = sample_depth_in_bbox(depth, (240.0, 200.0, 400.0, 280.0), shrink=0.1, min_valid_pixels=8)
    assert z is not None and abs(z - 4.0) < 1e-6
    # 全是无效值时返回 None，而不是抛异常或给出 NaN
    assert sample_depth_in_bbox(np.full((10, 10), np.nan), (0, 0, 10, 10)) is None


def test_sample_depth_too_few_pixels():
    depth = np.full((480, 640), np.nan)
    depth[240, 320] = 2.0
    assert sample_depth_in_bbox(depth, (300.0, 220.0, 340.0, 260.0), min_valid_pixels=8) is None
