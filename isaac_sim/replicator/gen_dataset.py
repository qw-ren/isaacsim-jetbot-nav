#!/usr/bin/env python3
"""用 Isaac Sim Replicator 生成障碍物检测数据集（YOLO 格式）。

目标不是刷高精度，而是把「合成数据 → YOLO 训练 → ROS2 部署」这条流程跑通：
- 障碍物在场地内随机撒布、随机朝向；
- 相机在场地外圈随机距离/高度/视角，灯光方向每帧变化；
- Replicator 自动输出 2D 框，脚本直接写成 YOLO 的 ``images/`` + ``labels/`` + ``data.yaml``。

运行环境：**Isaac Sim 4.5 自带的 Python**（Standalone Workflow），不是系统 Python。
    Linux:   ./python.sh isaac_sim/replicator/gen_dataset.py --config ...
    Windows: python.bat isaac_sim\\replicator\\gen_dataset.py --config ...

先用 ``--check`` 验证环境和配置，再正式生成：
    ./python.sh .../gen_dataset.py --config ... --check

⚠️ 类别/资产清单待与航硕确认，见 configs/dataset_config.yaml 顶部说明。
"""
from __future__ import annotations

import argparse
import json
import math
import os
import random
import shutil
import sys
from pathlib import Path
from typing import Any, Dict, Iterable, List, Sequence, Tuple

# Isaac Sim 4.5 用 isaacsim 包；旧版本回退到 omni.isaac.kit。
try:
    from isaacsim import SimulationApp  # type: ignore
except ImportError:  # pragma: no cover - 仅旧版本
    from omni.isaac.kit import SimulationApp  # type: ignore

IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png"}


def log(msg: str) -> None:
    """SimulationApp.close() 可能吞掉未 flush 的 stdout，关键节点显式 flush。"""
    print(msg, flush=True)


# --------------------------------------------------------------------------- #
# 自定义 Writer：把 Replicator 的 bbox 标注直接写成 YOLO txt
# --------------------------------------------------------------------------- #
def _build_writer_class():
    """延迟到 SimulationApp 启动后再导入 omni，保证独立进程里能正常工作。"""
    from pathlib import Path as _Path

    import numpy as np
    from PIL import Image
    from omni.replicator.core import Writer

    class YoloDatasetWriter(Writer):
        def __init__(
            self,
            output_dir: str,
            class_names: Sequence[str],
            max_occlusion: float = 0.95,
            image_format: str = "jpg",
        ) -> None:
            self.output_dir = _Path(output_dir)
            self.images_dir = self.output_dir / "images"
            self.labels_dir = self.output_dir / "labels"
            self.images_dir.mkdir(parents=True, exist_ok=True)
            self.labels_dir.mkdir(parents=True, exist_ok=True)

            self.class_names = [str(c) for c in class_names]
            self.name_to_id = {name: i for i, name in enumerate(self.class_names)}
            self.max_occlusion = float(max_occlusion)
            self.image_format = str(image_format).lower().lstrip(".")

            # 需要哪些 annotator：RGB + 2D 紧凑框
            self.annotators = ["rgb", "bounding_box_2d_tight"]
            self.version = "1.0"
            self._frame_id = 0
            self.written = 0
            self.skipped_classes: Dict[str, int] = {}

        # ------------------------------------------------------------------ #
        def _class_name(self, semantic_id: int, id_to_labels: Dict[Any, Any]) -> str:
            label = id_to_labels.get(str(semantic_id))
            if label is None:
                label = id_to_labels.get(semantic_id)
            if isinstance(label, dict):
                return str(label.get("class") or label.get("class_name") or "")
            return str(label) if label is not None else ""

        def write(self, data: dict) -> None:
            rgb = data.get("rgb")
            if rgb is None:
                return
            image = np.asarray(rgb)
            if image.ndim != 3 or image.shape[2] < 3:
                return
            image = np.ascontiguousarray(image[:, :, :3])
            height, width = int(image.shape[0]), int(image.shape[1])

            stem = "%06d" % self._frame_id
            image_path = self.images_dir / (stem + "." + self.image_format)
            if self.image_format in ("jpg", "jpeg"):
                Image.fromarray(image).save(image_path, quality=95)
            else:
                Image.fromarray(image).save(image_path)

            lines: List[str] = []
            bbox_data = data.get("bounding_box_2d_tight")
            if self._frame_id == 0:
                log("[writer] data keys=%s bbox_type=%s" % (list(data.keys()), type(bbox_data).__name__))
            if bbox_data is not None:
                # annotator 输出形如 {"data": 结构化数组, "info": {"idToLabels": {...}}}
                if isinstance(bbox_data, dict):
                    boxes = bbox_data.get("data")
                    id_to_labels = dict(bbox_data.get("info", {}).get("idToLabels", {}))
                else:
                    boxes = bbox_data
                    id_to_labels = {}

                if self._frame_id == 0:
                    log(
                        "[writer] bbox dtype=%s rows=%d idToLabels=%s"
                        % (
                            getattr(boxes, "dtype", None),
                            0 if boxes is None else len(boxes),
                            id_to_labels,
                        )
                    )

                for row in boxes if boxes is not None else []:
                    semantic_id = int(row["semanticId"])
                    class_name = self._class_name(semantic_id, id_to_labels)
                    class_id = self.name_to_id.get(class_name)
                    if class_id is None:
                        # 例如 UNLABELLED/背景，或配置里没列的类别
                        if class_name:
                            self.skipped_classes[class_name] = self.skipped_classes.get(class_name, 0) + 1
                        continue

                    try:
                        occlusion = float(row["occlusionRatio"])
                    except (KeyError, ValueError, TypeError):
                        occlusion = 0.0
                    if occlusion > self.max_occlusion:
                        continue

                    x1 = max(0.0, min(float(row["x_min"]), width - 1.0))
                    y1 = max(0.0, min(float(row["y_min"]), height - 1.0))
                    x2 = max(0.0, min(float(row["x_max"]), width - 1.0))
                    y2 = max(0.0, min(float(row["y_max"]), height - 1.0))
                    if x2 <= x1 or y2 <= y1:
                        continue

                    cx = ((x1 + x2) * 0.5) / width
                    cy = ((y1 + y2) * 0.5) / height
                    bw = (x2 - x1) / width
                    bh = (y2 - y1) / height
                    lines.append(
                        "%d %.6f %.6f %.6f %.6f" % (class_id, cx, cy, bw, bh)
                    )

            # 没有目标的帧也写空 txt：YOLO 需要负样本抑制误检
            (self.labels_dir / (stem + ".txt")).write_text(
                "\n".join(lines) + ("\n" if lines else ""), encoding="utf-8"
            )
            self._frame_id += 1
            self.written += 1

    return YoloDatasetWriter


# --------------------------------------------------------------------------- #
# 配置与工具
# --------------------------------------------------------------------------- #
def load_config(path: Path) -> dict:
    import yaml

    with open(path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f) or {}
    cfg["_config_path"] = str(path.resolve())
    return cfg


def resolve_path(value: str, base_dir: Path) -> Path:
    p = Path(os.path.expandvars(os.path.expanduser(str(value))))
    if not p.is_absolute():
        p = (base_dir / p).resolve()
    return p


def collect_class_names(cfg: dict) -> List[str]:
    names: List[str] = []
    for spec in cfg.get("obstacles", []):
        name = str(spec.get("class_name", "")).strip()
        if not name:
            raise ValueError("obstacles[] 里有条目的 class_name 为空")
        if name not in names:
            names.append(name)
    if not names:
        raise ValueError("配置里没有任何障碍物类别，检查 obstacles 字段")
    return names


def _sample_scale(spec: dict) -> Tuple[float, float, float]:
    lo, hi = spec.get("scale", [[0.2, 0.2, 0.2], [0.6, 0.6, 0.6]])
    return tuple(
        float(random.uniform(float(lo[i]), float(hi[i]))) for i in range(3)
    )  # type: ignore[return-value]


def _prim_from_item(item, label: str):
    """ReplicatorItem -> USD prim。create 节点的 outputs:prims 不一定可用，带名字兜底。"""
    import omni.usd

    try:
        for prims in item.get_output_prims().values():
            if prims:
                return prims[0]
    except Exception:
        pass
    stage = omni.usd.get_context().get_stage()
    for prim in stage.Traverse():
        if prim.GetName() == label:
            return prim
    raise RuntimeError("无法从 ReplicatorItem 获取 prim：%s" % label)


def _look_at_matrix(eye, target, up=None):
    """USD 相机（朝 -Z）的 look-at 变换矩阵，行向量约定。"""
    from pxr import Gf

    up = up or Gf.Vec3d(0.0, 0.0, 1.0)
    z_axis = -(target - eye).GetNormalized()
    x_axis = up.GetCross(z_axis)
    if x_axis.GetLength() < 1e-6:
        x_axis = Gf.Vec3d(1.0, 0.0, 0.0).GetCross(z_axis)
    x_axis = x_axis.GetNormalized()
    y_axis = z_axis.GetCross(x_axis).GetNormalized()
    matrix = Gf.Matrix4d(1.0)
    matrix.SetRow(0, Gf.Vec4d(x_axis[0], x_axis[1], x_axis[2], 0.0))
    matrix.SetRow(1, Gf.Vec4d(y_axis[0], y_axis[1], y_axis[2], 0.0))
    matrix.SetRow(2, Gf.Vec4d(z_axis[0], z_axis[1], z_axis[2], 0.0))
    matrix.SetRow(3, Gf.Vec4d(eye[0], eye[1], eye[2], 1.0))
    return matrix


def _create_obstacle(rep, spec: dict, class_name: str, name: str):
    semantics = [("class", class_name)]
    primitive = str(spec.get("primitive", "")).lower()
    if primitive == "cube":
        return rep.create.cube(semantics=semantics, name=name)
    if primitive == "cylinder":
        return rep.create.cylinder(semantics=semantics, name=name)
    if primitive == "cone":
        return rep.create.cone(semantics=semantics, name=name)
    if primitive == "sphere":
        return rep.create.sphere(semantics=semantics, name=name)

    usd_path = spec.get("usd")
    if not usd_path:
        raise ValueError(
            "类别 %r 既没有 primitive 也没有 usd，无法生成；"
            "资产到位前请先用 primitive 模式。" % class_name
        )
    usd_path = os.path.expandvars(os.path.expanduser(str(usd_path)))
    if not os.path.exists(usd_path):
        raise FileNotFoundError("找不到 USD 资产：%s" % usd_path)
    return rep.create.from_usd(usd_path, semantics=semantics)


# --------------------------------------------------------------------------- #
# 生成主流程
# --------------------------------------------------------------------------- #
def run_generation(cfg: dict, args: argparse.Namespace, sim_app, class_names: List[str]) -> Path:
    import numpy as np
    import omni.replicator.core as rep
    import omni.usd
    from pxr import Gf, Usd, UsdGeom

    seed = int(cfg.get("seed", 42))
    random.seed(seed)
    np.random.seed(seed)

    out_dir = resolve_path(cfg["output_dir"], Path(cfg["_config_path"]).parent)
    # 每次生成前清空旧目录，避免上一次的帧混进来
    if out_dir.exists() and any(out_dir.iterdir()):
        if not args.overwrite:
            raise SystemExit(
                "输出目录非空：%s\n确认要覆盖请加 --overwrite，或换一个 output_dir。" % out_dir
            )
        shutil.rmtree(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    log("[replicator] out_dir ready: %s" % out_dir)

    # ---- 场景 ---------------------------------------------------------- #
    omni.usd.get_context().new_stage()
    stage = omni.usd.get_context().get_stage()
    if not stage.GetPrimAtPath("/World").IsValid():
        UsdGeom.Xform.Define(stage, "/World")
    log("[replicator] stage ready")

    scene = cfg.get("scene", {})
    # 注意：rep.create.plane 是 UsdGeom.Plane（XZ 竖直面），在 Z-up 场景里会变成一堵墙。
    # 用一块压扁的立方体当地面：顶面正好在 z=0。
    ground_size = float(scene.get("ground_size", 30.0))
    ground = rep.create.cube(
        scale=(ground_size, ground_size, 0.05),
        position=(0.0, 0.0, -0.025),
        name="Ground",
    )

    light_cfg = scene.get("light", {})
    rep.create.light(
        light_type="Dome", intensity=float(light_cfg.get("dome_intensity", 900.0))
    )
    distant = rep.create.light(
        light_type="Distant",
        intensity=float(light_cfg.get("distant_intensity", 1200.0)),
        name="DistantLight",
    )

    # ---- 相机 ---------------------------------------------------------- #
    cam_cfg = scene.get("camera", {})
    camera = rep.create.camera(
        focal_length=float(cam_cfg.get("focal_length", 24.0)), name="Camera"
    )
    camera_prim = _prim_from_item(camera, "Camera")

    def reset_camera_child_rotation() -> None:
        """create.camera 会给子 Camera prim 自动加 (90,0,90)（Z-up 补偿），并可能被
        modify.pose 的延迟节点重新写回；预热后清一次，保证只有父 Xform 的矩阵生效。"""
        for child in Usd.PrimRange(camera_prim):
            if child.GetTypeName() == "Camera":
                UsdGeom.Xformable(child).ClearXformOpOrder()

    # ---- 障碍物 -------------------------------------------------------- #
    # 注意：Replicator 的 randomizer 函数体只在建图时执行一次，帧间随机由 OGN 节点完成。
    # 这里要跑「最小间距」这类 Python 逻辑，所以直接持有 prim，逐帧写 USD 变换。
    objects: List[Tuple[Any, Tuple[float, float, float], float]] = []
    for spec in cfg.get("obstacles", []):
        class_name = str(spec["class_name"]).strip()
        count = int(spec.get("count", 1))
        for idx in range(count):
            name = "%s_%02d" % (class_name, idx)
            item = _create_obstacle(rep, spec, class_name, name)
            prim = _prim_from_item(item, name)
            if spec.get("usd"):
                # Official assets can carry class tags on nested mesh parts.
                # Give the reference one canonical task label, avoiding both
                # composite labels (barrier,cone) and part-level duplicates.
                from isaacsim.core.utils.semantics import remove_all_semantics, add_update_semantics

                remove_all_semantics(prim, recursive=True)
                add_update_semantics(prim, class_name)
            scale = _sample_scale(spec)
            if spec.get("z_offset") is not None:
                z_offset = float(spec["z_offset"])
            elif spec.get("primitive"):
                # primitive 的几何中心在原点，默认抬到半高贴地；sign 这类悬空物体显式给 z_offset
                z_offset = 0.5 * scale[2]
            else:
                z_offset = 0.0
            objects.append((prim, scale, z_offset))

    if not objects:
        raise SystemExit("没有生成任何障碍物，检查 obstacles 配置")
    log("[replicator] %d obstacles created" % len(objects))

    area = scene.get("area", [[-3.5, -3.5], [3.5, 3.5]])
    min_gap = float(scene.get("min_gap", 0.45))
    light_prim = _prim_from_item(distant, "DistantLight")

    def set_matrix(prim, matrix) -> None:
        UsdGeom.Xformable(prim).MakeMatrixXform().Set(matrix)

    def randomize_layout() -> None:
        placed: List[Tuple[float, float]] = []
        for prim, scale, z_offset in objects:
            x = y = 0.0
            for _ in range(80):
                x = random.uniform(float(area[0][0]), float(area[1][0]))
                y = random.uniform(float(area[0][1]), float(area[1][1]))
                if all(
                    (x - px) ** 2 + (y - py) ** 2 >= min_gap * min_gap
                    for px, py in placed
                ):
                    break
            placed.append((x, y))
            yaw = random.uniform(0.0, 360.0)
            matrix = Gf.Matrix4d(1.0)
            matrix = matrix * Gf.Matrix4d(1.0).SetScale(Gf.Vec3d(*[float(v) for v in scale]))
            matrix = matrix * Gf.Matrix4d(1.0).SetRotate(
                Gf.Rotation(Gf.Vec3d(0.0, 0.0, 1.0), yaw)
            )
            matrix = matrix * Gf.Matrix4d(1.0).SetTranslate(Gf.Vec3d(x, y, z_offset))
            set_matrix(prim, matrix)

    def randomize_camera() -> None:
        distance = random.uniform(*[float(v) for v in cam_cfg.get("distance", [4.5, 7.5])])
        theta = random.uniform(0.0, 2.0 * math.pi)
        height = random.uniform(*[float(v) for v in cam_cfg.get("height", [0.35, 1.6])])
        target_z = random.uniform(
            *[float(v) for v in cam_cfg.get("look_at_height", [0.1, 0.8])]
        )
        eye = Gf.Vec3d(distance * math.cos(theta), distance * math.sin(theta), height)
        # 注视点在场内抖动，抖动幅度跟场地大小走（别固定 ±1 m）
        half_x = 0.5 * abs(float(area[1][0]) - float(area[0][0]))
        half_y = 0.5 * abs(float(area[1][1]) - float(area[0][1]))
        target = Gf.Vec3d(
            random.uniform(-0.6 * half_x, 0.6 * half_x),
            random.uniform(-0.6 * half_y, 0.6 * half_y),
            target_z,
        )
        set_matrix(camera_prim, _look_at_matrix(eye, target))

    def randomize_light() -> None:
        tilt = random.uniform(-60.0, 60.0)
        azimuth = random.uniform(0.0, 360.0)
        rotation = Gf.Rotation(Gf.Vec3d(1.0, 0.0, 0.0), tilt) * Gf.Rotation(
            Gf.Vec3d(0.0, 1.0, 0.0), azimuth
        )
        set_matrix(light_prim, Gf.Matrix4d(1.0).SetRotate(rotation))

    # ---- Writer + 逐帧生成 ---------------------------------------------- #
    width, height = [int(v) for v in cfg.get("resolution", [640, 480])]
    num_frames = int(args.num_frames or cfg.get("num_frames", 1000))

    log("[replicator] creating render product %dx%d" % (width, height))
    render_product = rep.create.render_product(camera, (width, height))
    registry = getattr(rep, "WriterRegistry", None) or rep.writers  # 4.5 两种别名都兼容
    writer = registry.get("YoloDatasetWriter")
    writer.initialize(
        output_dir=str(out_dir),
        class_names=class_names,
        max_occlusion=float(cfg.get("max_occlusion", 0.95)),
        image_format=str(cfg.get("image_format", "jpg")),
    )
    writer.attach([render_product])

    # 预热：让材质/光照/RTX 管线稳定，这几帧不写盘
    for _ in range(8):
        sim_app.update()
    reset_camera_child_rotation()

    log(
        "[replicator] frames=%d  resolution=%dx%d  classes=%s\n[replicator] output=%s"
        % (num_frames, width, height, class_names, out_dir)
    )

    for frame in range(num_frames):
        randomize_layout()
        randomize_camera()
        randomize_light()
        rep.orchestrator.step()
        if frame == 0:
            cw = UsdGeom.Xformable(camera_prim).ComputeLocalToWorldTransform(0)
            log(
                "[debug] camera wrapper %s t=%s -Z=%s"
                % (camera_prim.GetPath(), cw.ExtractTranslation(), cw.TransformDir(Gf.Vec3d(0, 0, -1)))
            )
            for child in Usd.PrimRange(camera_prim):
                if child.GetTypeName() == "Camera":
                    ccw = UsdGeom.Xformable(child).ComputeLocalToWorldTransform(0)
                    log(
                        "[debug] camera child %s t=%s -Z=%s"
                        % (child.GetPath(), ccw.ExtractTranslation(), ccw.TransformDir(Gf.Vec3d(0, 0, -1)))
                    )
            for prim, _, _ in objects[:3]:
                tw = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0).ExtractTranslation()
                log("[debug] %s -> %s" % (prim.GetPath(), tw))
        if frame == 0 or (frame + 1) % 50 == 0 or frame + 1 == num_frames:
            log(
                "[replicator] frame %d/%d  written=%d"
                % (frame + 1, num_frames, getattr(writer, "written", 0))
            )
    log("[replicator] generation loop done")

    writer.detach()
    render_product.destroy()

    skipped = getattr(writer, "skipped_classes", {})
    if skipped:
        log("[replicator] 以下类别的框被忽略（不在 class_names 中）：%s" % skipped)
    log("[replicator] 已写 %d 帧" % getattr(writer, "written", 0))
    return out_dir


def finalize_dataset(out_dir: Path, class_names: List[str], cfg: dict, seed: int) -> None:
    """按比例划分 train/val，并生成 YOLO 的 data.yaml。"""
    images_dir = out_dir / "images"
    labels_dir = out_dir / "labels"

    pairs: List[Tuple[Path, Path]] = []
    for img in sorted(images_dir.iterdir()):
        if img.is_file() and img.suffix.lower() in IMAGE_SUFFIXES:
            pairs.append((img, labels_dir / (img.stem + ".txt")))
    if len(pairs) < 2:
        raise SystemExit(
            "只生成了 %d 帧，无法划分 train/val；至少用 --num-frames 2 以上先验证流程。"
            % len(pairs)
        )

    ratio = float(cfg.get("train_val_split", 0.85))
    ratio = min(max(ratio, 0.1), 0.95)
    random.Random(seed).shuffle(pairs)
    n_train = max(1, min(len(pairs) - 1, int(round(len(pairs) * ratio)))) if len(pairs) > 1 else len(pairs)

    for split in ("train", "val"):
        (images_dir / split).mkdir(parents=True, exist_ok=True)
        (labels_dir / split).mkdir(parents=True, exist_ok=True)

    for idx, (img, label) in enumerate(pairs):
        split = "train" if idx < n_train else "val"
        shutil.move(str(img), str(images_dir / split / img.name))
        target_label = labels_dir / split / (img.stem + ".txt")
        if label.exists():
            shutil.move(str(label), str(target_label))
        else:
            target_label.write_text("", encoding="utf-8")

    import yaml

    data = {
        "path": str(out_dir.resolve()),
        "train": "images/train",
        "val": "images/val",
        "nc": len(class_names),
        "names": {i: name for i, name in enumerate(class_names)},
    }
    with open(out_dir / "data.yaml", "w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)

    info = {
        "classes": class_names,
        "num_images": len(pairs),
        "num_train": n_train,
        "num_val": len(pairs) - n_train,
        "seed": seed,
        "config": cfg.get("_config_path"),
        "resolution": cfg.get("resolution", [640, 480]),
    }
    with open(out_dir / "dataset_info.json", "w", encoding="utf-8") as f:
        json.dump(info, f, ensure_ascii=False, indent=2)

    log(
        "[dataset] %d 张图：train=%d val=%d\n[dataset] data.yaml: %s"
        % (len(pairs), n_train, len(pairs) - n_train, out_dir / "data.yaml")
    )


# --------------------------------------------------------------------------- #
def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    here = Path(__file__).resolve().parent
    ap = argparse.ArgumentParser(description="Isaac Sim Replicator → YOLO 障碍物数据集")
    ap.add_argument(
        "--config",
        default=str(here / "configs" / "dataset_config.yaml"),
        help="数据集配置文件（默认 configs/dataset_config.yaml）",
    )
    ap.add_argument("--num-frames", type=int, default=None, help="覆盖配置里的 num_frames")
    ap.add_argument("--output", default=None, help="覆盖配置里的 output_dir")
    ap.add_argument("--overwrite", action="store_true", help="输出目录非空时允许清空")
    ap.add_argument("--gui", action="store_true", help="打开 Isaac Sim 窗口（默认无头）")
    ap.add_argument("--check", action="store_true", help="只检查环境和配置，不生成数据")
    return ap.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    import faulthandler

    faulthandler.enable()  # 原生崩溃（段错误）时打印 Python 栈
    args = parse_args(argv)
    log("[replicator] argv=%s" % (list(sys.argv[1:]) if argv is None else list(argv),))
    config_path = Path(args.config)
    if not config_path.exists():
        raise SystemExit("配置文件不存在：%s" % config_path)
    cfg = load_config(config_path)
    if args.output:
        cfg["output_dir"] = args.output
    class_names = collect_class_names(cfg)

    log("[replicator] starting SimulationApp")
    simulation_app = SimulationApp({"headless": not args.gui})
    try:
        import omni.replicator.core as rep

        log("[check] isaacsim SimulationApp OK")
        log("[check] replicator version: %s" % getattr(rep, "__version__", "unknown"))
        log("[check] classes: %s" % (class_names,))
        log("[check] output: %s" % resolve_path(cfg["output_dir"], config_path.parent))
        log("[check] args.check=%s" % args.check)
        if args.check:
            return 0

        registry = getattr(rep, "WriterRegistry", None) or rep.writers
        registry.register(_build_writer_class())
        log("[replicator] writer registered")
        out_dir = run_generation(cfg, args, simulation_app, class_names)
        # 让渲染队列收尾，再落盘 data.yaml
        for _ in range(3):
            simulation_app.update()
        # 必须放在 close() 之前：SimulationApp.close() 会直接结束进程
        finalize_dataset(out_dir, class_names, cfg, int(cfg.get("seed", 42)))
    except BaseException as exc:  # close() 前先把真实错误打出来
        import traceback

        traceback.print_exc()
        log("[replicator] FAILED: %r" % (exc,))
        return 1
    finally:
        sys.stdout.flush()
        sys.stderr.flush()
        simulation_app.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
