#!/usr/bin/env python
"""L2+L3 最小 Demo：在「真值已知」的合成场景里验证 2D->3D 定位与视觉-激光融合。

为什么要这个脚本：
本地没有 Isaac Sim，但 L2/L3 是纯几何问题，可以用解析射线投射造一个场景：
房间 + 几个已知位置的物体 -> 精确渲染深度图和激光扫描（可加噪声）-> 跑感知模块
-> 和真值逐项对比。这样在拿到 Isaac Sim 之前就能证明几何链路是对的，
报告里也能给出「定位误差 cm 级」这种可量化的结论。

用法（在 perception/ 目录下）：
    python scripts/demo_locate_3d.py
    python scripts/demo_locate_3d.py --noise-depth 0.02 --noise-lidar 0.03
    python scripts/demo_locate_3d.py --pitch-deg 12      # 相机低头 12 度

产物写到 perception/runs/locate3d_<时间戳>/：
    topdown.png   俯视图：真值 vs 估计、激光点、FOV
    depth.png     渲染出的深度图伪彩色
    summary.json  误差表
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

SCRIPT_DIR = Path(__file__).resolve().parent
PERCEPTION_ROOT = SCRIPT_DIR.parent
sys.path.insert(0, str(PERCEPTION_ROOT))

import numpy as np  # noqa: E402

from atongmu_perception import PerceptionConfig, PerceptionPipeline  # noqa: E402
from atongmu_perception.geometry import (  # noqa: E402
    CameraIntrinsics,
    invert_transform,
    pixels_to_rays,
    project_points,
    transform_points,
)
from atongmu_perception.types import Detection2D  # noqa: E402


# --------------------------------------------------------------------------- #
# 场景定义
# --------------------------------------------------------------------------- #
@dataclass
class BoxObject:
    """轴对齐长方体，底面贴地 (z: 0 -> height)。"""

    class_name: str
    center_xy: Tuple[float, float]
    size_xy: Tuple[float, float]   # (x 方向进深, y 方向宽度)
    height: float

    @property
    def fx_min(self) -> float:
        return self.center_xy[0] - self.size_xy[0] * 0.5

    @property
    def fy_min(self) -> float:
        return self.center_xy[1] - self.size_xy[1] * 0.5

    @property
    def fy_max(self) -> float:
        return self.center_xy[1] + self.size_xy[1] * 0.5


@dataclass
class Room:
    x_min: float = -1.0
    x_max: float = 9.0
    y_min: float = -3.0
    y_max: float = 3.0
    height: float = 3.0


def default_objects() -> List[BoxObject]:
    return [
        BoxObject("person", (4.0, 0.5), (0.5, 0.5), 1.70),
        BoxObject("chair", (2.5, -1.1), (0.55, 0.55), 0.90),
        BoxObject("bottle", (1.6, 0.75), (0.12, 0.12), 0.30),
    ]


# --------------------------------------------------------------------------- #
# 解析射线投射
# --------------------------------------------------------------------------- #
def raycast_depth(
    K: CameraIntrinsics,
    T_base_camera: np.ndarray,
    objects: Sequence[BoxObject],
    room: Room,
) -> np.ndarray:
    """渲染深度图（光轴深度，单位米）。只渲染正对相机的可见面，足够做几何验证。"""
    h, w = K.height, K.width
    uu, vv = np.meshgrid(np.arange(w, dtype=np.float64), np.arange(h, dtype=np.float64))
    pixels = np.stack([uu.ravel(), vv.ravel()], axis=1)
    rays_cam = pixels_to_rays(pixels, K.K)              # (N,3) 相机系单位方向
    R = T_base_camera[:3, :3]
    rays_base = rays_cam @ R.T                          # (N,3)
    origin = T_base_camera[:3, 3]

    dx, dy, dz = rays_base[:, 0], rays_base[:, 1], rays_base[:, 2]
    t_best = np.full(len(pixels), np.inf)
    z_min = 0.0
    z_max = room.height

    def update(t: np.ndarray) -> None:
        np.minimum(t_best, t, out=t_best)

    def plane_t(coord_origin: float, plane: float, denom: np.ndarray) -> np.ndarray:
        with np.errstate(divide="ignore", invalid="ignore"):
            t = (plane - coord_origin) / denom
        return np.where(t > 1e-6, t, np.inf)

    # 地面 / 天花板
    update(plane_t(origin[2], z_min, dz))
    update(plane_t(origin[2], z_max, dz))
    # 四面墙
    update(plane_t(origin[0], room.x_max, dx))
    update(plane_t(origin[0], room.x_min, dx))
    update(plane_t(origin[1], room.y_max, dy))
    update(plane_t(origin[1], room.y_min, dy))

    # 物体：只用其朝向相机的 x 方向前表面
    for obj in objects:
        t_face = plane_t(origin[0], obj.fx_min, dx)
        px = origin[0] + t_face * dx
        py = origin[1] + t_face * dy
        pz = origin[2] + t_face * dz
        hit = (
            np.isfinite(t_face)
            & (py >= obj.fy_min)
            & (py <= obj.fy_max)
            & (pz >= z_min)
            & (pz <= obj.height)
        )
        update(np.where(hit, t_face, np.inf))

    with np.errstate(invalid="ignore"):
        depth = t_best * rays_cam[:, 2]                  # 光轴深度 = t * cos(theta)
    depth = np.where(np.isfinite(depth) & (depth > 0), depth, np.inf)
    return depth.reshape(h, w)


def project_base_points(points_base: np.ndarray, K: CameraIntrinsics, T_base_camera: np.ndarray):
    T_cam_base = invert_transform(T_base_camera)
    pts_cam = transform_points(T_cam_base, points_base)
    uv, z, valid = project_points(pts_cam, K.K)
    return uv, z, valid


def object_front_bbox(obj: BoxObject, K: CameraIntrinsics, T_base_camera: np.ndarray) -> Optional[Tuple[float, float, float, float]]:
    """物体前表面投影出的像素包围盒。"""
    corners = np.array(
        [
            [obj.fx_min, y, z]
            for y in (obj.fy_min, obj.fy_max)
            for z in (0.0, obj.height)
        ],
        dtype=np.float64,
    )
    uv, _, valid = project_base_points(corners, K, T_base_camera)
    if not np.all(valid):
        return None
    x1, y1 = float(uv[:, 0].min()), float(uv[:, 1].min())
    x2, y2 = float(uv[:, 0].max()), float(uv[:, 1].max())
    x1, x2 = max(0.0, x1), min(K.width - 1.0, x2)
    y1, y2 = max(0.0, y1), min(K.height - 1.0, y2)
    if x2 - x1 < 3 or y2 - y1 < 3:
        return None
    return (x1, y1, x2, y2)


def raycast_lidar(
    objects: Sequence[BoxObject],
    room: Room,
    angles: np.ndarray,
    lidar_height: float,
) -> np.ndarray:
    """2D 单线激光：在 lidar 高度上对房间墙体和物体footprint做射线求交。"""
    c, s = np.cos(angles), np.sin(angles)
    t_best = np.full(angles.shape, np.inf)

    def plane(dist: float, denom: np.ndarray) -> np.ndarray:
        with np.errstate(divide="ignore", invalid="ignore"):
            t = dist / denom
        return np.where(t > 1e-6, t, np.inf)

    t_best = np.minimum(t_best, plane(room.x_max, c))
    t_best = np.minimum(t_best, plane(room.x_min, c))
    t_best = np.minimum(t_best, plane(room.y_max, s))
    t_best = np.minimum(t_best, plane(room.y_min, s))

    for obj in objects:
        if obj.height < lidar_height:
            continue  # 比雷达扫描平面低的物体扫不到
        # 与深度图保持一致：只用物体朝向机器人的前表面
        t = plane(obj.fx_min, c)
        py = t * s
        hit = np.isfinite(t) & (py >= obj.fy_min) & (py <= obj.fy_max)
        t_best = np.minimum(t_best, np.where(hit, t, np.inf))

    return t_best


# --------------------------------------------------------------------------- #
# 真值
# --------------------------------------------------------------------------- #
def true_front_point_along_pixel(pixel, obj: BoxObject, K: CameraIntrinsics, T_base_camera: np.ndarray) -> np.ndarray:
    """该像素射线打到物体前表面上的真实三维点。"""
    ray_cam = pixels_to_rays(np.asarray(pixel, dtype=np.float64).reshape(1, 2), K.K)[0]
    ray_base = T_base_camera[:3, :3] @ ray_cam
    origin = T_base_camera[:3, 3]
    t = (obj.fx_min - origin[0]) / ray_base[0]
    return origin + t * ray_base


# --------------------------------------------------------------------------- #
def main() -> int:
    ap = argparse.ArgumentParser(description="合成场景下的 RGB-D 3D 定位 + 激光融合验证")
    ap.add_argument("--config", default=str(PERCEPTION_ROOT / "configs" / "perception.yaml"))
    ap.add_argument("--noise-depth", type=float, default=0.01, help="深度噪声标准差（米）")
    ap.add_argument("--noise-lidar", type=float, default=0.02, help="激光噪声标准差（米）")
    ap.add_argument("--pitch-deg", type=float, default=0.0, help="相机俯仰角（正=低头）")
    ap.add_argument("--height", type=float, default=0.35, help="相机安装高度（米）")
    ap.add_argument("--no-dropout", action="store_true", help="不模拟深度无效像素")
    ap.add_argument("--out", default=None)
    ap.add_argument("--tol", type=float, default=0.10, help="误差阈值（米），超过则返回非零退出码")
    args = ap.parse_args()

    cfg = PerceptionConfig.from_yaml(args.config)
    cfg.camera.width, cfg.camera.height = 640, 480
    cfg.extrinsics.camera_position = (0.0, 0.0, args.height)
    cfg.extrinsics.camera_pitch_deg = args.pitch_deg
    cfg.extrinsics.lidar_position = (0.0, 0.0, 0.20)

    K = cfg.camera.intrinsics()
    T_base_camera = cfg.extrinsics.T_base_camera()
    room = Room()
    objects = default_objects()

    out_dir = Path(args.out) if args.out else PERCEPTION_ROOT / "runs" / ("locate3d_%s" % time.strftime("%Y%m%d_%H%M%S"))
    out_dir.mkdir(parents=True, exist_ok=True)

    # ---------------- 1. 造数据 ----------------
    depth_clean = raycast_depth(K, T_base_camera, objects, room)
    rng = np.random.default_rng(20261029)
    depth = depth_clean + rng.normal(0.0, args.noise_depth, size=depth_clean.shape)
    valid_mask = np.isfinite(depth_clean)
    if not args.no_dropout:
        dropout = rng.random(depth_clean.shape) < 0.02
        depth[dropout] = np.nan
    depth[~valid_mask] = np.inf

    angles = np.linspace(-np.pi, np.pi, 720, endpoint=False)
    lidar_ranges = raycast_lidar(objects, room, angles, cfg.extrinsics.lidar_position[2])
    lidar_ranges = lidar_ranges + rng.normal(0.0, args.noise_lidar, size=lidar_ranges.shape)

    # ---------------- 2. 造「完美检测框」（模仿 YOLO 的输出） ----------------
    detections: List[Detection2D] = []
    kept_objects: List[BoxObject] = []
    for obj in objects:
        bbox = object_front_bbox(obj, K, T_base_camera)
        if bbox is None:
            print("[warn] %s 不在视野内，跳过" % obj.class_name)
            continue
        detections.append(Detection2D(class_id=-1, class_name=obj.class_name, confidence=0.9, bbox=bbox))
        kept_objects.append(obj)

    pipeline = PerceptionPipeline(cfg, detector=None)
    pipeline.detector = None  # 本脚本不需要真实检测器

    # ---------------- 3. A/B/C 三种条件分别跑 ----------------
    def run(depth_input, scan_input):
        return pipeline.process(
            detections=[Detection2D(d.class_id, d.class_name, d.confidence, d.bbox) for d in detections],
            depth=depth_input,
            scan=scan_input,
            stamp=0.0,
        )

    res_rgbd = run(depth, None)                                            # RGB-D 定位
    res_ground = run(None, None)                                           # 仅 RGB（地面约束）
    res_fused = run(depth, (lidar_ranges, float(angles[0]), float(angles[1] - angles[0])))  # 视觉+激光

    # ---------------- 4. 误差统计 ----------------
    rows = []
    worst = 0.0
    for idx, obj in enumerate(kept_objects):
        center_px = detections[idx].center
        gt_front = true_front_point_along_pixel(center_px, obj, K, T_base_camera)
        gt_range_centerpx = float(np.hypot(gt_front[0], gt_front[1]))

        per_method = {}
        for tag, result in (("rgbd", res_rgbd), ("rgb_ground", res_ground), ("fused", res_fused)):
            obj_pose = next((o for o in result.objects if o.detection_id == idx), None)
            if obj_pose is None or obj_pose.localization is None:
                per_method[tag] = None
                continue
            est = np.asarray(obj_pose.localization.xyz_base, dtype=np.float64)
            err = float(np.linalg.norm(est[:2] - gt_front[:2]))
            worst = max(worst, err)
            per_method[tag] = {
                "est_xy": [round(float(est[0]), 3), round(float(est[1]), 3)],
                "est_range_m": round(float(obj_pose.localization.range_m), 3),
                "true_range_m": round(gt_range_centerpx, 3),
                "err_m": round(err, 3),
                "err_cm": round(err * 100.0, 1),
                "method": obj_pose.localization.method,
                "n_points": int(obj_pose.localization.n_points),
                "true_xy": [round(float(gt_front[0]), 3), round(float(gt_front[1]), 3)],
            }
        rows.append({"class_name": obj.class_name, "bbox": [round(v, 1) for v in detections[idx].bbox], **per_method})

    # ---------------- 5. 打印 + 保存 ----------------
    print("\n场景: 房间 %.0fx%.0fm, 相机高 %.2fm, pitch %.0f°, 深度噪声 %.0fmm, 激光噪声 %.0fmm"
          % (room.x_max - room.x_min, room.y_max - room.y_min, args.height, args.pitch_deg,
             args.noise_depth * 1000, args.noise_lidar * 1000))
    header = "%-8s %-10s %-9s %-9s %-8s %-7s" % ("物体", "方法", "真实距离", "估计距离", "误差cm", "点数")
    print("\n--- RGB-D 深度定位 ---")
    print(header)
    for r in rows:
        m = r.get("rgbd")
        if m:
            print("%-8s %-10s %8.3fm %8.3fm %7.1f %7d" % (r["class_name"], m["method"], m["true_range_m"], m["est_range_m"], m["err_cm"], m["n_points"]))
    print("\n--- 纯 RGB 地面约束（兜底方案）---")
    for r in rows:
        m = r.get("rgb_ground")
        if m:
            print("%-8s %-10s %8.3fm %8.3fm %7.1f %7d" % (r["class_name"], m["method"], m["true_range_m"], m["est_range_m"], m["err_cm"], m["n_points"]))
    print("\n--- 视觉 + 激光融合 ---")
    for r in rows:
        m = r.get("fused")
        if m:
            print("%-8s %-10s %8.3fm %8.3fm %7.1f %7d" % (r["class_name"], m["method"], m["true_range_m"], m["est_range_m"], m["err_cm"], m["n_points"]))

    summary = {
        "scene": {"room": vars(room), "camera_height": args.height, "pitch_deg": args.pitch_deg,
                  "noise_depth_m": args.noise_depth, "noise_lidar_m": args.noise_lidar},
        "objects": rows,
        "worst_err_m": round(worst, 3),
    }
    (out_dir / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")

    # 图
    try:
        import cv2

        from atongmu_perception.viz import depth_to_color

        cv2.imwrite(str(out_dir / "depth.png"), depth_to_color(np.where(np.isfinite(depth), depth, np.inf), max_depth=10.0))
    except Exception as exc:  # pragma: no cover
        print("[warn] 深度图保存失败: %s" % exc)

    try:
        plot_topdown(out_dir / "topdown.png", room, objects, res_fused, K, T_base_camera, args, lidar_ranges, angles)
    except Exception as exc:  # pragma: no cover
        print("[warn] 俯视图保存失败: %s" % exc)

    print("\n结果目录: %s" % out_dir)
    if worst > args.tol:
        print("[FAIL] 最大误差 %.1f cm 超过阈值 %.1f cm" % (worst * 100, args.tol * 100))
        return 2
    print("[OK] 最大定位误差 %.1f cm（阈值 %.1f cm）" % (worst * 100, args.tol * 100))
    return 0


def plot_topdown(path, room, objects, result, K, T_base_camera, args, lidar_ranges, angles) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon, Rectangle

    fig, ax = plt.subplots(figsize=(10, 5.2), dpi=140)

    # 房间
    ax.add_patch(Rectangle((room.x_min, room.y_min), room.x_max - room.x_min, room.y_max - room.y_min,
                           fill=False, edgecolor="#444", linewidth=1.5))

    # 激光点
    lx = lidar_ranges * np.cos(angles)
    ly = lidar_ranges * np.sin(angles)
    ax.scatter(lx, ly, s=2, c="#9aa5b1", label="lidar scan", zorder=1)

    # FOV
    half_hfov = np.arctan(K.width / 2.0 / K.fx)
    fov = Polygon([[0, 0], [8 * np.cos(half_hfov), 8 * np.sin(half_hfov)],
                   [8 * np.cos(-half_hfov), 8 * np.sin(-half_hfov)]],
                  closed=True, color="#4c78a8", alpha=0.08, label="camera FOV")
    ax.add_patch(fov)

    # 物体真值
    for obj in objects:
        ax.add_patch(Rectangle((obj.fx_min, obj.fy_min), obj.size_xy[0], obj.size_xy[1],
                               facecolor="#d9d9d9", edgecolor="#666", zorder=2))
        ax.text(obj.center_xy[0], obj.center_xy[1], obj.class_name, fontsize=8, ha="center", va="center", zorder=5)

    # 估计结果
    for pose in result.objects:
        if pose.localization is None or pose.localization.xyz_base is None:
            continue
        p = pose.localization.xyz_base
        ax.plot(p[0], p[1], "o", color="#e45756", markersize=7, zorder=6)
        ax.annotate("%s %.2fm" % (pose.class_name, pose.localization.range_m),
                    (p[0], p[1]), textcoords="offset points", xytext=(6, 6), fontsize=8, color="#b23a48")

    ax.plot(0, 0, "s", color="#111", markersize=8, label="robot")
    ax.set_aspect("equal")
    ax.set_xlabel("x forward [m]")
    ax.set_ylabel("y left [m]")
    ax.set_title("Top-down: fused perception vs ground truth")
    ax.grid(alpha=0.25)
    ax.legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    fig.savefig(str(path))
    plt.close(fig)


if __name__ == "__main__":
    raise SystemExit(main())
