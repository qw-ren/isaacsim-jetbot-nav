#!/usr/bin/env python
"""L1 最小 Demo：对一张图片 / 一段视频 / 摄像头跑 YOLO26 检测并可视化。

用法（在 perception/ 目录下）：
    python scripts/demo_detect.py                          # 用 ultralytics 自带样例图
    python scripts/demo_detect.py --source path/to.mp4     # 视频
    python scripts/demo_detect.py --source 0               # 摄像头
    python scripts/demo_detect.py --probe-cameras          # 摄像头打不开时先跑这个体检
    python scripts/demo_detect.py --source 0 --backend dshow   # 指定视频后端
    python scripts/demo_detect.py --no-filter               # 不按类别白名单过滤

摄像头在 Windows 上默认走 Media Foundation，部分内置摄像头（如 Acer HD User Facing）
会「能打开但读不出帧」。open_camera() 默认 auto，会自动兜底到 DirectShow。

输出写到 perception/runs/detect_<时间戳>/：标注图/视频 + summary.json
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from collections import Counter
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
PERCEPTION_ROOT = SCRIPT_DIR.parent
sys.path.insert(0, str(PERCEPTION_ROOT))

import cv2  # noqa: E402

from atongmu_perception import PerceptionConfig, PerceptionPipeline  # noqa: E402
from atongmu_perception.camera import gui_available, open_camera, probe_cameras  # noqa: E402
from atongmu_perception.detector import YoloDetector  # noqa: E402
from atongmu_perception.viz import draw_result  # noqa: E402

IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


def default_sample_image() -> str:
    try:
        import ultralytics

        candidate = Path(ultralytics.__file__).parent / "assets" / "bus.jpg"
        if candidate.exists():
            return str(candidate)
    except ImportError:
        pass
    return ""


def parse_args() -> argparse.Namespace:
    here = Path(__file__).resolve()
    p = argparse.ArgumentParser(description="YOLO26 目标检测最小 Demo")
    p.add_argument("--source", default=None, help="图片/视频路径，或摄像头编号（默认用样例图）")
    p.add_argument("--config", default=str(PERCEPTION_ROOT / "configs" / "perception.yaml"))
    p.add_argument("--weights", default=None, help="覆盖配置里的权重路径")
    p.add_argument("--device", default=None, help="cpu / cuda:0")
    p.add_argument("--imgsz", type=int, default=None)
    p.add_argument("--conf", type=float, default=None)
    p.add_argument("--no-filter", action="store_true", help="不做类别白名单过滤")
    p.add_argument("--max-frames", type=int, default=0, help="视频最多处理多少帧，0 = 全部")
    p.add_argument("--duration", type=float, default=0.0, help="最多运行多少秒后自动停止，0 = 不限（摄像头常用）")
    p.add_argument("--verbose", action="store_true", help="每帧打印检测到的类别、置信度和耗时")
    p.add_argument("--video-fps", type=float, default=0.0,
                   help="录制视频的帧率，0=用输入源帧率。摄像头处理慢于 30fps 时设成实际速率（如 8）画面才是正常速度")
    p.add_argument("--out", default=None, help="输出目录")
    p.add_argument("--show", dest="show", action="store_true", default=None, help="实时显示窗口（摄像头默认开启）")
    p.add_argument("--no-show", dest="show", action="store_false", help="不显示窗口，只写视频文件")
    p.add_argument("--probe-cameras", action="store_true", help="逐个试 0-4 号摄像头，报告哪个能出画面后退出")
    p.add_argument("--backend", default="auto", choices=["auto", "default", "dshow", "msmf"],
                   help="摄像头后端，auto 会依次尝试 default/dshow/msmf")
    return p.parse_args()


def build_detector(args: argparse.Namespace):
    cfg = PerceptionConfig.from_yaml(args.config)
    if args.no_filter:
        cfg.detector.classes = []
    if args.weights:
        cfg.detector.weights = args.weights
    if args.device:
        cfg.detector.device = args.device
    if args.imgsz:
        cfg.detector.imgsz = args.imgsz
    if args.conf is not None:
        cfg.detector.conf = args.conf
    detector = YoloDetector.from_config(cfg.detector)
    pipeline = PerceptionPipeline(cfg, detector=detector)
    return cfg, detector, pipeline


def main() -> int:
    args = parse_args()
    if args.probe_cameras:
        usable = probe_cameras()
        if not usable:
            print("\n没有可用摄像头。按顺序检查：")
            print("  1) 笔记本有没有物理防窥挡片 / 相机快捷键（Acer 多为 Fn+F6），指示灯亮不亮")
            print("  2) 设备管理器 → 照相机 → ACER HD User Facing 有没有黄色感叹号")
            print("  3) 关掉独占摄像头的程序：抖音直播伴侣、微信视频、腾讯会议、OBS")
            print("  4) 打开系统自带「相机」App，能出画面说明硬件正常，是 OpenCV 后端问题")
            return 1
        idx, backend = usable[0]
        print("\n用这个参数重跑: python scripts/demo_detect.py --source %d --backend %s" % (idx, backend))
        return 0
    source = args.source if args.source is not None else default_sample_image()
    if source == "":
        print("找不到默认样例图，请用 --source 指定输入")
        return 1
    is_camera = str(source).isdigit()

    out_dir = Path(args.out) if args.out else PERCEPTION_ROOT / "runs" / ("detect_%s" % time.strftime("%Y%m%d_%H%M%S"))
    out_dir.mkdir(parents=True, exist_ok=True)

    cfg, detector, pipeline = build_detector(args)
    print("[demo] weights=%s device=%s imgsz=%d conf=%.2f" % (cfg.detector.weights, cfg.detector.device, cfg.detector.imgsz, cfg.detector.conf))
    print("[demo] 类别白名单: %s" % (cfg.detector.classes or "全部 80 类"))
    print("[demo] 输入: %s" % ("摄像头 %s" % source if is_camera else source))

    # ---------------- 单张图片 ----------------
    if not is_camera and Path(source).suffix.lower() in IMAGE_SUFFIXES and Path(source).is_file():
        image = cv2.imread(str(source))
        if image is None:
            print("[demo] 读不到图片: %s" % source)
            return 1
        t0 = time.perf_counter()
        result = pipeline.process(image=image, stamp=time.time())
        dt = (time.perf_counter() - t0) * 1000.0
        # 这张图没有真实深度和内参，只展示检测结果，不显示距离（距离见 demo_locate_3d.py）
        vis = draw_result(image, result, show_range=False)
        out_img = out_dir / ("%s_annotated.jpg" % Path(source).stem)
        cv2.imwrite(str(out_img), vis)

        counts = Counter(d.class_name for d in result.detections)
        print("[demo] 检测到 %d 个目标，耗时 %.1f ms（含预处理/后处理）" % (len(result.detections), dt))
        for det in result.detections:
            print("        %-12s conf=%.3f bbox=(%.0f, %.0f, %.0f, %.0f)" % (det.class_name, det.confidence, *det.bbox))
        summary = {
            "source": str(source),
            "n_detections": len(result.detections),
            "class_counts": dict(counts),
            "latency_ms_total": round(dt, 2),
            "latency_ms_model": round(detector.last_latency_ms, 2),
            "result": result.to_dict(),
        }
        (out_dir / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
        print("[demo] 标注图: %s" % out_img)
        print("[demo] 结果 JSON: %s" % (out_dir / "summary.json"))
        return 0

    # ---------------- 视频 / 摄像头 ----------------
    show = args.show if args.show is not None else is_camera
    if show and not gui_available():
        print("[demo] ⚠ 当前 OpenCV 弹不出窗口：装的是 opencv-python-headless（GUI: NONE），")
        print("       imshow 会抛 'The function is not implemented'，进程会在第一帧后直接退出。")
        print("       本次改为只录视频，结束后打开 annotated.mp4 就能看到动态效果。")
        print("       想要实时看画面，用无窗口版预览: python scripts/live_preview.py --source %s" % source)
        show = False
    if is_camera:
        cap, backend_used = open_camera(int(source), args.backend)
        if cap is None:
            print("[demo] 摄像头 %s 打不开（%s）" % (
                source, "已依次试过 default/dshow/msmf" if args.backend == "auto" else "后端 %s" % args.backend))
            print("        先跑一次体检: python scripts/demo_detect.py --probe-cameras")
            print("        也可能是被独占了（抖音直播伴侣/微信视频/腾讯会议），或隐私开关被关")
            return 1
        print("[demo] 摄像头 %s 已打开，后端 = %s" % (source, backend_used))
    else:
        cap, backend_used = cv2.VideoCapture(str(source)), None
    if not cap.isOpened():
        print("[demo] 打不开输入: %s" % source)
        return 1
    fps_in = cap.get(cv2.CAP_PROP_FPS) or 30.0
    if fps_in != fps_in or fps_in <= 1.0:  # NaN 或异常值
        fps_in = 30.0

    if show:
        print("[demo] 已打开预览窗口：按 q 或 Esc 退出，也可以 Ctrl+C")
        print("[demo] 注意：画面会同时录到 %s/annotated.mp4" % out_dir.name)

    writer = None  # 用第一帧的真实尺寸初始化，避免 cap 报的尺寸和实际画面不一致
    write_ok = True
    frame_idx, total_ms, model_ms = 0, 0.0, 0.0
    conf_sum = 0.0
    fps_ema = 0.0
    all_counts = Counter()
    t_start = time.perf_counter()
    if args.verbose:
        print("\n%-6s %-46s %8s" % ("帧", "检测结果（类别 置信度）", "耗时"))
        print("-" * 64)
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            t0 = time.perf_counter()
            result = pipeline.process(image=frame, stamp=time.time())
            dt_ms = (time.perf_counter() - t0) * 1000.0
            total_ms += dt_ms
            model_ms += detector.last_latency_ms
            frame_idx += 1
            for det in result.detections:
                all_counts[det.class_name] += 1
                conf_sum += det.confidence

            if args.verbose:
                items = "  ".join("%s %.2f" % (d.class_name, d.confidence) for d in result.detections)
                print("%-6d %-46s %6.0f ms" % (frame_idx, items[:46] or "-", dt_ms), flush=True)

            vis = draw_result(frame, result, show_range=False)

            # 左上角 HUD：帧率 + 当前目标数，录像和窗口里都有
            inst_fps = 1000.0 / dt_ms if dt_ms > 0 else 0.0
            fps_ema = inst_fps if frame_idx == 1 else 0.85 * fps_ema + 0.15 * inst_fps
            hud = "%.1f FPS   %d obj" % (fps_ema, len(result.detections))
            cv2.putText(vis, hud, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0), 4, cv2.LINE_AA)
            cv2.putText(vis, hud, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (80, 255, 120), 2, cv2.LINE_AA)

            if writer is None:
                h, w = vis.shape[:2]
                writer = cv2.VideoWriter(
                    str(out_dir / "annotated.mp4"), cv2.VideoWriter_fourcc(*"mp4v"),
                    args.video_fps if args.video_fps > 0 else fps_in, (w, h)
                )
                if not writer.isOpened():
                    print("[demo] ⚠ 视频编码器不可用（mp4v），本次不保存视频")
                    write_ok = False
            if write_ok:
                writer.write(vis)

            if show:
                try:
                    cv2.imshow("perception (press q to quit)", vis)
                    key = cv2.waitKey(1) & 0xFF
                    if key in (ord("q"), 27):
                        break
                except cv2.error as exc:  # 运行中才发现 GUI 不可用，降级为只录视频
                    print("\n[demo] ⚠ 弹窗失败，改为只录视频: %s" % str(exc).splitlines()[0])
                    show = False

            if not args.verbose and frame_idx % 30 == 0:
                print("  frame %4d   %.1f ms/frame" % (frame_idx, total_ms / frame_idx))
            if args.max_frames and frame_idx >= args.max_frames:
                break
            if args.duration and (time.perf_counter() - t_start) >= args.duration:
                print("\n[demo] 已运行 %.0f 秒，按 --duration 停止" % args.duration)
                break
    except KeyboardInterrupt:
        print("\n[demo] 收到 Ctrl+C，保存结果中...")
    finally:
        cap.release()
        if writer is not None:
            writer.release()
        if show:
            cv2.destroyAllWindows()

    wall = time.perf_counter() - t_start
    if frame_idx == 0:
        print("[demo] 没有读到任何帧")
        return 1
    n_det = sum(all_counts.values())
    summary = {
        "source": str(source),
        "frames": frame_idx,
        "wall_time_s": round(wall, 2),
        "fps_wall": round(frame_idx / wall, 2) if wall > 0 else None,
        "avg_total_ms": round(total_ms / frame_idx, 2),
        "avg_model_ms": round(model_ms / frame_idx, 2),
        "class_counts": dict(all_counts),
        "n_detections": n_det,
        "avg_confidence": round(conf_sum / n_det, 4) if n_det else None,
    }
    (out_dir / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    print("[demo] 共 %d 帧，平均 %.1f ms/帧（推理 %.1f ms），整体 %.2f FPS" % (
        frame_idx, total_ms / frame_idx, model_ms / frame_idx, frame_idx / wall if wall > 0 else 0))
    if all_counts:
        print("[demo] 类别统计: %s" % dict(all_counts))
        print("[demo] 共 %d 个目标，平均置信度 %.3f" % (n_det, conf_sum / n_det if n_det else 0.0))
    else:
        print("[demo] 全程没检测到白名单内的目标（换个角度或降低 --conf 试试）")
    print("[demo] 输出: %s  (annotated.mp4 / summary.json)" % out_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
