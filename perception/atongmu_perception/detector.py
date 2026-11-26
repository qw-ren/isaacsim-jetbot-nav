"""L1 语义理解层：YOLO26 目标检测封装。

设计要点：
- 延迟导入 ultralytics，保证几何与融合模块在没有深度学习环境的机器上也能测试。
- 输出统一为 Detection2D，坐标已还原到原始图像像素。
- 类别白名单按「类别名」配置，避免上游换权重后 class_id 错位。
"""
from __future__ import annotations

import os
import time
from pathlib import Path
from typing import List, Optional, Sequence

import numpy as np

from .config import DetectorConfig
from .types import Detection2D


def resolve_weights_path(weights: str) -> str:
    """把配置里的相对权重路径解析到 perception/weights/ 下，避免依赖当前工作目录。

    如果文件不存在且没有目录成分，原样返回，交给 ultralytics 自动下载。
    """
    if not weights or os.path.isabs(weights) or os.path.exists(weights):
        return weights
    candidate = Path(__file__).resolve().parent.parent / weights
    if candidate.exists():
        return str(candidate)
    return weights


class YoloDetector:
    """Ultralytics YOLO 的薄封装（YOLO26 / YOLOv8 / YOLO11 通用）。"""

    def __init__(
        self,
        weights: str = "yolo26s.pt",
        device: str = "cpu",
        imgsz: int = 640,
        conf: float = 0.35,
        iou: float = 0.5,
        half: bool = False,
        max_det: int = 100,
        classes: Optional[Sequence[str]] = None,
        verbose: bool = False,
    ) -> None:
        try:
            from ultralytics import YOLO
        except ImportError as exc:  # pragma: no cover - 只在缺依赖时触发
            raise ImportError(
                "需要 ultralytics：pip install ultralytics。"
                "如果只用几何/融合功能，可以直接向 PerceptionPipeline 传入检测结果。"
            ) from exc

        self.weights = weights
        self.device = device
        self.imgsz = int(imgsz)
        self.conf = float(conf)
        self.iou = float(iou)
        self.half = bool(half) and device != "cpu"
        self.max_det = int(max_det)
        self.verbose = verbose

        self.model = YOLO(resolve_weights_path(weights))
        self.names = {int(k): str(v) for k, v in self.model.names.items()}
        self._name_to_id = {v: k for k, v in self.names.items()}
        self.class_ids = self._resolve_class_ids(classes)
        self.last_latency_ms = 0.0

    def _resolve_class_ids(self, classes: Optional[Sequence[str]]) -> Optional[List[int]]:
        if not classes:
            return None
        ids, unknown = [], []
        for name in classes:
            key = str(name).strip()
            if key in self._name_to_id:
                ids.append(self._name_to_id[key])
            else:
                unknown.append(key)
        if unknown:
            # 不静默失败：类别名写错会让整条链路的语义筛选悄悄失效。
            raise ValueError(
                "配置中的类别 %r 不在权重 %s 的类别表里。可用类别: %s"
                % (unknown, self.weights, sorted(self._name_to_id))
            )
        return ids

    @classmethod
    def from_config(cls, cfg: DetectorConfig) -> "YoloDetector":
        return cls(
            weights=cfg.weights,
            device=cfg.device,
            imgsz=cfg.imgsz,
            conf=cfg.conf,
            iou=cfg.iou,
            half=cfg.half,
            max_det=cfg.max_det,
            classes=cfg.classes,
        )

    def warmup(self, image_size=(480, 640)) -> None:
        """跑一次空推理，避免第一帧把模型加载时间算进延迟。"""
        dummy = np.zeros((image_size[0], image_size[1], 3), dtype=np.uint8)
        self.detect(dummy)

    def detect(self, image_bgr: np.ndarray, stamp: float = 0.0) -> List[Detection2D]:
        """image_bgr: HxWx3 BGR/uint8。返回原图坐标下的检测框。"""
        if image_bgr is None or image_bgr.size == 0:
            return []
        t0 = time.perf_counter()
        kwargs = dict(
            source=image_bgr,
            imgsz=self.imgsz,
            conf=self.conf,
            iou=self.iou,
            device=self.device,
            max_det=self.max_det,
            classes=self.class_ids,
            verbose=False,
        )
        if self.half:
            kwargs["half"] = True  # YOLO26 起该参数改名 quantize，仅在需要时传，避免弃用警告
        results = self.model.predict(**kwargs)
        self.last_latency_ms = (time.perf_counter() - t0) * 1000.0

        detections: List[Detection2D] = []
        if not results:
            return detections
        boxes = results[0].boxes
        if boxes is None or len(boxes) == 0:
            return detections
        xyxy = boxes.xyxy.detach().cpu().numpy()
        confs = boxes.conf.detach().cpu().numpy()
        clss = boxes.cls.detach().cpu().numpy().astype(int)
        for (x1, y1, x2, y2), c, k in zip(xyxy, confs, clss):
            detections.append(
                Detection2D(
                    class_id=int(k),
                    class_name=self.names.get(int(k), str(k)),
                    confidence=float(c),
                    bbox=(float(x1), float(y1), float(x2), float(y2)),
                    stamp=stamp,
                )
            )
        detections.sort(key=lambda d: d.confidence, reverse=True)
        return detections


__all__ = ["YoloDetector"]
