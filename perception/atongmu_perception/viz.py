"""可视化工具：把感知结果画到图像上，用于离线 demo、调试和汇报截图。"""
from __future__ import annotations

from typing import Tuple

import numpy as np

from .types import PerceptionResult

# 类别 -> BGR 颜色（不用随机数，保证每次画出来颜色一致，方便写进汇报材料）
_PALETTE = [
    (60, 76, 231),    # 红
    (74, 195, 139),   # 紫
    (219, 152, 52),   # 蓝
    (96, 187, 224),   # 橙
    (128, 176, 96),   # 绿
    (180, 105, 255),  # 粉
    (54, 67, 244),    # 深红
    (40, 39, 214),    # 深橙
]


def class_color(class_name: str) -> Tuple[int, int, int]:
    # 用 crc32 而不是内置 hash()，后者每个进程的随机种子不同，颜色会变
    import zlib

    return _PALETTE[zlib.crc32(class_name.encode("utf-8")) % len(_PALETTE)]


def draw_result(
    image: np.ndarray,
    result: PerceptionResult,
    show_range: bool = True,
    show_method: bool = False,
    thickness: int = 2,
) -> np.ndarray:
    """在图像副本上画出检测框和距离。返回新图像。"""
    import cv2

    canvas = image.copy()
    loc_by_id = {o.detection_id: o for o in result.objects}
    for idx, det in enumerate(result.detections):
        color = class_color(det.class_name)
        x1, y1, x2, y2 = [int(round(v)) for v in det.bbox]
        cv2.rectangle(canvas, (x1, y1), (x2, y2), color, thickness)

        obj = loc_by_id.get(idx)
        track_tag = ""
        if obj is not None and obj.track_id is not None:
            track_tag = "#%d " % obj.track_id
        label = "%s%s %.2f" % (track_tag, det.class_name, det.confidence)
        loc = obj.localization if obj is not None else None
        if show_range and loc is not None and np.isfinite(loc.range_m):
            label += " %.2fm" % loc.range_m
            if show_method:
                label += " [%s/%d]" % (loc.method, loc.n_points)

        ty = max(12, y1 - 6)
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(canvas, (x1, ty - th - 4), (x1 + tw + 4, ty + 2), color, -1)
        cv2.putText(canvas, label, (x1 + 2, ty - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

        if obj is not None and obj.localization is not None and obj.localization.xyz_base is not None:
            u, v = det.center
            cv2.circle(canvas, (int(u), int(v)), 3, color, -1)

    hud = "dets=%d  %.1f ms" % (len(result.detections), result.latency_ms)
    cv2.rectangle(canvas, (0, 0), (230, 26), (30, 30, 30), -1)
    cv2.putText(canvas, hud, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (230, 230, 230), 1, cv2.LINE_AA)
    return canvas


def depth_to_color(depth: np.ndarray, max_depth: float = 8.0) -> np.ndarray:
    """深度图转伪彩色，无效值（0/inf）显示为黑色。"""
    import cv2

    d = np.asarray(depth, dtype=np.float32)
    valid = np.isfinite(d) & (d > 1e-3)
    norm = np.zeros_like(d)
    if np.any(valid):
        norm[valid] = np.clip(d[valid] / max_depth, 0.0, 1.0)
    color = cv2.applyColorMap((255.0 * (1.0 - norm)).astype(np.uint8), cv2.COLORMAP_TURBO)
    color[~valid] = (0, 0, 0)
    return color


def side_by_side(left: np.ndarray, right: np.ndarray, gap: int = 8) -> np.ndarray:
    """水平拼接两幅图，高度不一致时补齐。"""
    h = max(left.shape[0], right.shape[0])

    def _pad(img):
        if img.shape[0] == h:
            return img
        pad = np.zeros((h - img.shape[0], img.shape[1], img.shape[2]), dtype=img.dtype)
        return np.vstack([img, pad])

    return np.hstack([_pad(left), np.zeros((h, gap, 3), dtype=left.dtype), _pad(right)])


__all__ = ["class_color", "draw_result", "depth_to_color", "side_by_side"]
