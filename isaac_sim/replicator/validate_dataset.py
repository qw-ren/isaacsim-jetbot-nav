#!/usr/bin/env python3
"""校验 Replicator 生成的 YOLO 数据集（普通 Python 即可，不需要 Isaac Sim）。

在把数据带下云电脑/开始训练前跑一遍，能提前发现：
- 图片和标签数量对不上；
- 类别 id 超出 data.yaml 的 names；
- 归一化坐标越界、宽高为 0；
- 大量空标签（相机可能没看到障碍物）。

用法：
    python isaac_sim/replicator/validate_dataset.py \
        --data datasets/atongmu_obstacles/data.yaml --draw 20
"""
from __future__ import annotations

import argparse
import math
import random
from collections import Counter
from pathlib import Path
from typing import Dict, List, Tuple

import yaml

IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png"}


def load_names(data_path: Path) -> List[str]:
    with open(data_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    names = data.get("names")
    if isinstance(names, dict):
        return [str(names[k]) for k in sorted(names, key=lambda x: int(x))]
    if isinstance(names, list):
        return [str(n) for n in names]
    raise SystemExit("data.yaml 里没有有效的 names 字段")


def split_dirs(data_path: Path, data: dict, split: str) -> Tuple[Path, Path]:
    root = Path(data.get("path") or data_path.parent)
    if not root.is_absolute():
        root = (data_path.parent / root).resolve()
    image_rel = data.get(split)
    if not image_rel:
        raise SystemExit("data.yaml 缺少 %s 字段" % split)
    images_dir = (root / image_rel).resolve()
    labels_dir = images_dir.parent.parent / "labels" / images_dir.name
    return images_dir, labels_dir


def parse_label_file(path: Path, num_classes: int) -> Tuple[List[Tuple[int, float, float, float, float]], List[str]]:
    boxes: List[Tuple[int, float, float, float, float]] = []
    errors: List[str] = []
    text = path.read_text(encoding="utf-8").strip()
    if not text:
        return boxes, errors
    for line_no, line in enumerate(text.splitlines(), 1):
        parts = line.split()
        if len(parts) != 5:
            errors.append("%s:%d 字段数不是 5" % (path.name, line_no))
            continue
        try:
            raw_cls = float(parts[0])
            if not math.isfinite(raw_cls) or not raw_cls.is_integer():
                raise ValueError("class id must be an integer")
            cls = int(raw_cls)
            cx, cy, w, h = (float(v) for v in parts[1:])
        except ValueError:
            errors.append("%s:%d 无法解析为数字" % (path.name, line_no))
            continue
        if not (0 <= cls < num_classes):
            errors.append("%s:%d 类别 id=%d 超出 names 范围" % (path.name, line_no, cls))
            continue
        if not all(math.isfinite(v) for v in (cx, cy, w, h)):
            errors.append("%s:%d 坐标含非有限数" % (path.name, line_no))
            continue
        if w <= 0 or h <= 0:
            errors.append("%s:%d 宽或高为 0" % (path.name, line_no))
            continue
        if not all(-1e-4 <= v <= 1.0 + 1e-4 for v in (cx, cy, w, h)):
            errors.append("%s:%d 归一化坐标越界 (%.3f %.3f %.3f %.3f)" % (path.name, line_no, cx, cy, w, h))
            continue
        if min(cx - w / 2, cy - h / 2) < -1e-4 or max(cx + w / 2, cy + h / 2) > 1.0001:
            errors.append("%s:%d 框边界超出图像" % (path.name, line_no))
            continue
        boxes.append((cls, cx, cy, w, h))
    return boxes, errors


def draw_boxes(image_path: Path, boxes, names: List[str], out_path: Path) -> None:
    import cv2
    import numpy as np
    # 用 imdecode/imencode 而不是 imread/imwrite，兼容 Windows 中文路径
    try:
        image = cv2.imdecode(np.fromfile(str(image_path), dtype=np.uint8), cv2.IMREAD_COLOR)
    except Exception:
        image = None
    if image is None:
        return
    h, w = image.shape[:2]
    for cls, cx, cy, bw, bh in boxes:
        x1 = int(max(0, (cx - bw / 2) * w))
        y1 = int(max(0, (cy - bh / 2) * h))
        x2 = int(min(w - 1, (cx + bw / 2) * w))
        y2 = int(min(h - 1, (cy + bh / 2) * h))
        color = (0, 200, 0)
        cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
        label = names[cls] if cls < len(names) else str(cls)
        cv2.putText(image, label, (x1, max(15, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    ok, buf = cv2.imencode(out_path.suffix or ".jpg", image)
    if ok:
        buf.tofile(str(out_path))


def main() -> int:
    ap = argparse.ArgumentParser(description="校验 YOLO 数据集")
    ap.add_argument("--data", required=True, help="data.yaml 路径")
    ap.add_argument("--draw", type=int, default=0, help="再可视化 N 张带框的图")
    ap.add_argument("--out", default=None, help="可视化输出目录（默认 data 同级 validation/）")
    ap.add_argument("--seed", type=int, default=42)
    args = ap.parse_args()

    data_path = Path(args.data).expanduser().resolve()
    if not data_path.exists():
        raise SystemExit("找不到 %s" % data_path)
    with open(data_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    names = load_names(data_path)
    out_dir = Path(args.out) if args.out else data_path.parent / "validation"

    total_boxes = 0
    total_empty = 0
    class_counts: Counter = Counter()
    all_errors: List[str] = []
    samples: List[Tuple[Path, list]] = []

    for split in ("train", "val"):
        images_dir, labels_dir = split_dirs(data_path, data, split)
        if not images_dir.exists():
            all_errors.append("%s 图像目录不存在：%s" % (split, images_dir))
            continue

        images = sorted(p for p in images_dir.iterdir() if p.suffix.lower() in IMAGE_SUFFIXES)
        print("[%s] %s：%d 张图，标签目录 %s" % (split, images_dir.name, len(images), labels_dir))
        if not images:
            all_errors.append("%s 没有图像" % split)

        for image_path in images:
            label_path = labels_dir / (image_path.stem + ".txt")
            if not label_path.exists():
                all_errors.append("缺少标签：%s" % label_path)
                continue
            boxes, errors = parse_label_file(label_path, len(names))
            all_errors.extend(errors)
            total_boxes += len(boxes)
            if not boxes:
                total_empty += 1
            for cls, *_ in boxes:
                class_counts[names[cls]] += 1
            if args.draw and len(samples) < args.draw and boxes:
                samples.append((image_path, boxes))

    total_images = sum(
        len([p for p in split_dirs(data_path, data, s)[0].iterdir() if p.suffix.lower() in IMAGE_SUFFIXES])
        for s in ("train", "val")
        if split_dirs(data_path, data, s)[0].exists()
    )

    print("\n类别：%s" % names)
    print("图像总数：%d，框总数：%d，空标签帧：%d (%.1f%%)" % (
        total_images, total_boxes, total_empty,
        100.0 * total_empty / max(1, total_images),
    ))
    for name in names:
        print("  %-16s %d" % (name, class_counts.get(name, 0)))

    if all_errors:
        print("\n发现 %d 个问题（前 20 条）：" % len(all_errors))
        for err in all_errors[:20]:
            print("  -", err)
    else:
        print("\n结构校验通过。")

    if args.draw and samples:
        for image_path, boxes in samples:
            draw_boxes(image_path, boxes, names, out_dir / ("%s_%s" % (image_path.parent.name, image_path.name)))
        print("可视化样例写到：%s" % out_dir)

    return 1 if all_errors else 0


if __name__ == "__main__":
    raise SystemExit(main())
