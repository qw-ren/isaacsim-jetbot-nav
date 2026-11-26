#!/usr/bin/env python
"""训练障碍物检测模型（ultralytics YOLO）。

数据集由 isaac_sim/replicator/gen_dataset.py 生成，目录里自带 data.yaml。

用法（在 perception/ 下）：
    python scripts/train_yolo.py --data ../datasets/atongmu_obstacles/data.yaml
    python scripts/train_yolo.py --data ... --epochs 80 --batch 16 --device 0

训练完成后：
1. 把输出的 best.pt 复制到 perception/weights/；
2. 在 configs/perception.yaml 里改 detector.weights 和 detector.classes
   （classes 必须与 data.yaml 的 names 完全一致，否则 detector 会直接报错，不会静默出错）。
"""
from __future__ import annotations

import argparse
from pathlib import Path

from ultralytics import YOLO


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="训练障碍物检测模型")
    ap.add_argument("--data", required=True, help="dataset 的 data.yaml")
    ap.add_argument("--base", default="yolo26s.pt", help="预训练权重（COCO）")
    ap.add_argument("--epochs", type=int, default=80)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--batch", type=int, default=16)
    ap.add_argument("--device", default=None, help="cpu / 0 / 0,1；不填让 ultralytics 自动选")
    ap.add_argument("--project", default="runs/train")
    ap.add_argument("--name", default="obstacles")
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--resume", action="store_true")
    ap.add_argument("--seed", type=int, default=42)
    return ap.parse_args()


def main() -> int:
    args = parse_args()
    data_path = Path(args.data).expanduser().resolve()
    if not data_path.exists():
        raise SystemExit("找不到 data.yaml：%s" % data_path)

    model = YOLO(args.base)
    train_kwargs = dict(
        data=str(data_path),
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        project=args.project,
        name=args.name,
        workers=args.workers,
        seed=args.seed,
        resume=args.resume,
        plots=True,
        verbose=True,
    )
    if args.device:
        train_kwargs["device"] = args.device

    model.train(**train_kwargs)

    best = Path(args.project) / args.name / "weights" / "best.pt"
    if best.exists():
        print("\n训练完成，best 权重：%s" % best.resolve())
        print("下一步：复制到 perception/weights/，并更新 configs/perception.yaml 的 detector 段。")
    else:
        print("\n训练结束，但没有找到 %s，请检查 ultralytics 输出。" % best)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
