"""几何工具：针孔相机模型、坐标变换、深度采样、地面约束测距。

本模块只依赖 numpy，方便在 Windows 上单独测试，也方便云电脑上的 ROS2 节点复用。
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

import numpy as np


# --------------------------------------------------------------------------- #
# 相机内参
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int = 640
    height: int = 480

    def as_matrix(self) -> np.ndarray:
        return np.array(
            [[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )

    @property
    def K(self) -> np.ndarray:
        return self.as_matrix()

    @classmethod
    def from_matrix(cls, K: Sequence[Sequence[float]], width: int = 640, height: int = 480) -> "CameraIntrinsics":
        K = np.asarray(K, dtype=np.float64)
        return cls(fx=float(K[0, 0]), fy=float(K[1, 1]), cx=float(K[0, 2]), cy=float(K[1, 2]), width=int(width), height=int(height))

    @classmethod
    def from_camera_info(cls, camera_info) -> "CameraIntrinsics":
        """从 sensor_msgs/CameraInfo（或任何带 K/width/height 属性的对象）构造。"""
        K = np.asarray(camera_info.k, dtype=np.float64).reshape(3, 3)
        return cls.from_matrix(K, int(camera_info.width), int(camera_info.height))

    def to_dict(self) -> dict:
        return {
            "fx": round(self.fx, 3),
            "fy": round(self.fy, 3),
            "cx": round(self.cx, 3),
            "cy": round(self.cy, 3),
            "width": int(self.width),
            "height": int(self.height),
        }


# --------------------------------------------------------------------------- #
# 旋转与变换矩阵
# --------------------------------------------------------------------------- #
def quaternion_xyzw_to_matrix(q: Sequence[float]) -> np.ndarray:
    """四元数 (x, y, z, w) -> 3x3 旋转矩阵。兼容 ROS 的 x,y,z,w 顺序。"""
    x, y, z, w = [float(v) for v in q]
    norm = np.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        return np.eye(3)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def matrix_to_quaternion_xyzw(R: np.ndarray) -> Tuple[float, float, float, float]:
    """3x3 旋转矩阵 -> 四元数 (x, y, z, w)。"""
    R = np.asarray(R, dtype=np.float64)
    trace = np.trace(R)
    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return (float(x), float(y), float(z), float(w))


def rotation_about_axis(axis: str, angle: float) -> np.ndarray:
    """绕基座坐标系 x/y/z 轴旋转 angle 弧度（右手定则）。"""
    c, s = np.cos(angle), np.sin(angle)
    if axis == "x":
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=np.float64)
    if axis == "y":
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=np.float64)
    if axis == "z":
        return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=np.float64)
    raise ValueError("axis 必须是 'x'/'y'/'z'，收到: %r" % (axis,))


def yaw_from_matrix(R: np.ndarray) -> float:
    """提取绕 z 轴的偏航角。"""
    R = np.asarray(R, dtype=np.float64)
    return float(np.arctan2(R[1, 0], R[0, 0]))


def make_transform(
    translation: Sequence[float],
    quaternion_xyzw: Optional[Sequence[float]] = None,
    rotation: Optional[np.ndarray] = None,
) -> np.ndarray:
    """构造 4x4 齐次变换矩阵。注意命名：T_target_source 表示把 source 系下的点变到 target 系。"""
    T = np.eye(4, dtype=np.float64)
    if rotation is not None:
        T[:3, :3] = np.asarray(rotation, dtype=np.float64)
    elif quaternion_xyzw is not None:
        T[:3, :3] = quaternion_xyzw_to_matrix(quaternion_xyzw)
    T[:3, 3] = np.asarray(translation, dtype=np.float64).reshape(3)
    return T


def invert_transform(T: np.ndarray) -> np.ndarray:
    T = np.asarray(T, dtype=np.float64)
    R = T[:3, :3]
    t = T[:3, 3]
    T_inv = np.eye(4, dtype=np.float64)
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ t
    return T_inv


def transform_points(T: np.ndarray, points: np.ndarray) -> np.ndarray:
    """T: 4x4；points: (N, 3) -> (N, 3)。"""
    points = np.asarray(points, dtype=np.float64)
    single = points.ndim == 1
    if single:
        points = points.reshape(1, 3)
    T = np.asarray(T, dtype=np.float64)
    out = points @ T[:3, :3].T + T[:3, 3]
    return out[0] if single else out


# ROS 约定的相机光学系 -> 机器人基座系（无俯仰、无偏航时）的旋转。
# 相机 x(右) -> 基座 -y；相机 y(下) -> 基座 -z；相机 z(前) -> 基座 +x。
R_BASE_CAMERA_ZERO = np.array(
    [
        [0.0, 0.0, 1.0],
        [-1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
    ],
    dtype=np.float64,
)


def camera_pose_transform(
    position_xyz: Sequence[float],
    pitch: float = 0.0,
    yaw: float = 0.0,
    roll: float = 0.0,
) -> np.ndarray:
    """由相机在机器人上的安装位姿构造 T_base_camera（相机光学系 -> 基座系）。

    position_xyz: 相机光心在 base_link 下的位置 [x, y, z]，z 为安装高度。
    pitch: 俯仰角（正 = 镜头向下看），弧度。
    yaw:   偏航角（正 = 向左转），弧度。
    roll:  横滚角，弧度。
    """
    R = (
        rotation_about_axis("z", yaw)
        @ rotation_about_axis("y", pitch)
        @ rotation_about_axis("x", roll)
        @ R_BASE_CAMERA_ZERO
    )
    return make_transform(position_xyz, rotation=R)


# --------------------------------------------------------------------------- #
# 投影与反投影
# --------------------------------------------------------------------------- #
def pixels_to_rays(pixels: np.ndarray, K: np.ndarray) -> np.ndarray:
    """像素坐标 (N, 2) -> 相机系下的单位方向向量 (N, 3)。"""
    pixels = np.asarray(pixels, dtype=np.float64).reshape(-1, 2)
    K = np.asarray(K, dtype=np.float64)
    rays = np.stack(
        [
            (pixels[:, 0] - K[0, 2]) / K[0, 0],
            (pixels[:, 1] - K[1, 2]) / K[1, 1],
            np.ones(len(pixels)),
        ],
        axis=1,
    )
    norms = np.linalg.norm(rays, axis=1, keepdims=True)
    return rays / np.maximum(norms, 1e-12)


def backproject_pixels(pixels: np.ndarray, depths: np.ndarray, K: np.ndarray) -> np.ndarray:
    """像素 + 光轴深度 Z -> 相机系三维点 (N, 3)。"""
    pixels = np.asarray(pixels, dtype=np.float64).reshape(-1, 2)
    depths = np.asarray(depths, dtype=np.float64).reshape(-1)
    K = np.asarray(K, dtype=np.float64)
    x = (pixels[:, 0] - K[0, 2]) * depths / K[0, 0]
    y = (pixels[:, 1] - K[1, 2]) * depths / K[1, 1]
    return np.stack([x, y, depths], axis=1)


def project_points(points_cam: np.ndarray, K: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """相机系点 (N, 3) -> 像素 (N, 2)、深度 (N,)、是否在相机前方的布尔掩码。"""
    points_cam = np.asarray(points_cam, dtype=np.float64).reshape(-1, 3)
    K = np.asarray(K, dtype=np.float64)
    z = points_cam[:, 2]
    valid = z > 1e-6
    z_safe = np.where(valid, z, 1.0)
    u = K[0, 0] * points_cam[:, 0] / z_safe + K[0, 2]
    v = K[1, 1] * points_cam[:, 1] / z_safe + K[1, 2]
    return np.stack([u, v], axis=1), z, valid


def project_box_corners(center: Sequence[float], size: Sequence[float], T_cam_box: np.ndarray, K: np.ndarray):
    """把一个轴对齐长方体投影到图像，返回 (u_min, v_min, u_max, v_max)；不可见时返回 None。"""
    center = np.asarray(center, dtype=np.float64)
    sx, sy, sz = [float(v) * 0.5 for v in size]
    corners = np.array(
        [
            [center[0] + dx, center[1] + dy, center[2] + dz]
            for dx in (-sx, sx)
            for dy in (-sy, sy)
            for dz in (-sz, sz)
        ],
        dtype=np.float64,
    )
    pts_cam = transform_points(T_cam_box, corners)
    if np.any(pts_cam[:, 2] <= 1e-3):
        return None
    uv, _, _ = project_points(pts_cam, K)
    return (float(uv[:, 0].min()), float(uv[:, 1].min()), float(uv[:, 0].max()), float(uv[:, 1].max()))


# --------------------------------------------------------------------------- #
# 深度采样
# --------------------------------------------------------------------------- #
def bbox_region(bbox: Sequence[float], shrink: float = 0.25) -> Tuple[int, int, int, int]:
    """把 bbox 收缩后取整，得到 (x1, y1, x2, y2) 整数区域；保证至少 1 像素宽高。"""
    x1, y1, x2, y2 = [float(v) for v in bbox]
    w = max(1.0, x2 - x1)
    h = max(1.0, y2 - y1)
    dx, dy = w * shrink, h * shrink
    x1i, y1i = int(round(x1 + dx)), int(round(y1 + dy))
    x2i, y2i = int(round(x2 - dx)), int(round(y2 - dy))
    if x2i <= x1i:
        x1i, x2i = int(round((x1 + x2) * 0.5)), int(round((x1 + x2) * 0.5)) + 1
    if y2i <= y1i:
        y1i, y2i = int(round((y1 + y2) * 0.5)), int(round((y1 + y2) * 0.5)) + 1
    return x1i, y1i, x2i, y2i


def sample_depth_in_bbox(
    depth: np.ndarray,
    bbox: Sequence[float],
    shrink: float = 0.25,
    min_depth: float = 0.05,
    max_depth: float = 10.0,
    percentile: float = 20.0,
    min_valid_pixels: int = 8,
) -> Optional[float]:
    """在 bbox 内部采样深度，返回稳健的深度估计（默认取 20 分位数，偏向近处目标）。

    只统计在 [min_depth, max_depth] 内的有限值，用于滤掉背景、地面和无效深度。
    有效像素太少时返回 None。
    """
    depth = np.asarray(depth, dtype=np.float64)
    if depth.ndim != 2:
        raise ValueError("depth 必须是二维数组，收到 shape=%r" % (depth.shape,))
    h, w = depth.shape
    x1, y1, x2, y2 = bbox_region(bbox, shrink)
    x1, y1 = max(0, min(x1, w - 1)), max(0, min(y1, h - 1))
    x2, y2 = max(x1 + 1, min(x2, w)), max(y1 + 1, min(y2, h))
    patch = depth[y1:y2, x1:x2].ravel()
    finite = np.isfinite(patch)
    valid = patch[finite & (patch >= min_depth) & (patch <= max_depth)]
    if valid.size < min_valid_pixels:
        return None
    return float(np.percentile(valid, percentile))


def ground_plane_point(
    pixel: Sequence[float],
    K: np.ndarray,
    R_base_camera: np.ndarray,
    camera_position: Sequence[float],
    max_range: float = 12.0,
) -> Optional[Tuple[np.ndarray, float]]:
    """单目地面约束测距：假设该像素落在机器人脚下平面 z=0，返回 (基座系落点, 水平距离)。

    camera_position: 相机光心在 base_link 下的位置 [x, y, z]；只给标量时视为 (0, 0, z)。
    适用场景：Isaac Sim 里只有 RGB 相机、没有深度图时的兜底方案。
    地板不水平或目标悬空（如瓶子放在桌上）时距离会偏大。
    """
    cam_pos = np.asarray(camera_position, dtype=np.float64).reshape(-1)
    if cam_pos.size == 1:
        cam_pos = np.array([0.0, 0.0, float(cam_pos[0])], dtype=np.float64)
    if cam_pos.size != 3:
        raise ValueError("camera_position 必须是 (x, y, z) 或相机高度，收到 %r" % (camera_position,))
    ray_cam = pixels_to_rays(np.asarray(pixel, dtype=np.float64).reshape(1, 2), K)[0]
    ray_base = np.asarray(R_base_camera, dtype=np.float64) @ ray_cam
    if ray_base[2] >= -1e-6:
        return None  # 射线朝上或水平，打不到地面
    t = -cam_pos[2] / ray_base[2]
    if t <= 0.0:
        return None
    p = cam_pos + t * ray_base
    p[2] = 0.0  # 数值上强制落到地面
    rng = float(np.hypot(p[0], p[1]))
    if rng > max_range:
        return None
    return p, rng


def ground_plane_range(
    pixel: Sequence[float],
    K: np.ndarray,
    R_base_camera: np.ndarray,
    camera_position: Sequence[float],
    max_range: float = 12.0,
) -> Optional[float]:
    """ground_plane_point 的简化版，只返回水平距离。"""
    out = ground_plane_point(pixel, K, R_base_camera, camera_position, max_range)
    return None if out is None else out[1]


def bbox_size_estimate(
    bbox: Sequence[float],
    depth_z: float,
    K: np.ndarray,
) -> Tuple[float, float]:
    """由框的像素尺寸和深度粗略估计目标的物理宽高（假设目标正面朝向相机）。"""
    w_px = max(0.0, float(bbox[2]) - float(bbox[0]))
    h_px = max(0.0, float(bbox[3]) - float(bbox[1]))
    return (w_px * depth_z / K[0, 0], h_px * depth_z / K[1, 1])


__all__ = [
    "CameraIntrinsics",
    "R_BASE_CAMERA_ZERO",
    "camera_pose_transform",
    "quaternion_xyzw_to_matrix",
    "matrix_to_quaternion_xyzw",
    "rotation_about_axis",
    "yaw_from_matrix",
    "make_transform",
    "invert_transform",
    "transform_points",
    "pixels_to_rays",
    "backproject_pixels",
    "project_points",
    "project_box_corners",
    "bbox_region",
    "sample_depth_in_bbox",
    "ground_plane_point",
    "ground_plane_range",
    "bbox_size_estimate",
]
