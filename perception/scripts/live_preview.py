#!/usr/bin/env python
"""视觉感知实时监控页：左边实时画面，右边逐目标置信度与累计统计。

两种用法，取决于机器上有没有图形界面：

* 有窗口（``cv2.imshow`` 可用）：直接用 ``demo_detect.py --source 0`` 看原生窗口；
* 没有窗口（``opencv-python-headless``、云电脑无显示器、远程 SSH）：用本脚本，
  浏览器里看，不依赖任何图形库。

用法（在 perception/ 目录下）：
    python scripts/live_preview.py --source 0                   # 摄像头
    python scripts/live_preview.py --source runs/test_pan.mp4   # 视频文件
    python scripts/live_preview.py --source 0 --port 8080

浏览器会自动打开 http://127.0.0.1:<port>/ ，终端按 Ctrl+C 结束。
默认只监听 127.0.0.1；要让同一局域网的其他机器看，用 ``--host 0.0.0.0``。
"""
from __future__ import annotations

import argparse
import json
import socketserver
import sys
import threading
import time
import webbrowser
from collections import Counter
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Optional, Tuple

SCRIPT_DIR = Path(__file__).resolve().parent
PERCEPTION_ROOT = SCRIPT_DIR.parent
sys.path.insert(0, str(PERCEPTION_ROOT))

import cv2  # noqa: E402

from atongmu_perception import PerceptionConfig, PerceptionPipeline  # noqa: E402
from atongmu_perception.camera import gui_available, open_camera  # noqa: E402
from atongmu_perception.detector import YoloDetector  # noqa: E402
from atongmu_perception.viz import draw_result  # noqa: E402

UI_PAGE = r"""<!doctype html>
<html lang="zh">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>阿童木 · 视觉感知监控</title>
<style>
  :root {
    --bg: #0a0b0d;
    --panel: #131519;
    --panel-2: #191c21;
    --line: #23262d;
    --text: #e6e8ec;
    --muted: #868c97;
    --accent: #4ade80;
    --accent-dim: #1f6b45;
    --warn: #f59e0b;
    --mono: ui-monospace, "Cascadia Mono", Consolas, monospace;
  }
  * { box-sizing: border-box; }
  html, body { height: 100%; }
  body {
    margin: 0; background: var(--bg); color: var(--text);
    font: 14px/1.5 system-ui, "Microsoft YaHei", sans-serif;
  }
  header {
    display: flex; align-items: center; gap: 12px;
    padding: 12px 20px; border-bottom: 1px solid var(--line); background: var(--panel);
  }
  header h1 { margin: 0; font-size: 15px; font-weight: 600; letter-spacing: .02em; }
  .dot { width: 8px; height: 8px; border-radius: 50%; background: #555; transition: all .2s; }
  .dot.on { background: var(--accent); box-shadow: 0 0 9px var(--accent); }
  .dot.off { background: #ef4444; box-shadow: 0 0 9px #ef4444; }
  .meta { margin-left: auto; font: 12px/1.5 var(--mono); color: var(--muted); }
  main {
    display: grid; grid-template-columns: minmax(0, 1fr) 340px;
    gap: 16px; padding: 16px; height: calc(100vh - 53px);
  }
  @media (max-width: 900px) {
    main { grid-template-columns: 1fr; height: auto; }
  }
  .stage {
    position: relative;
    display: flex; align-items: center; justify-content: center;
    background: #000; border: 1px solid var(--line); border-radius: 8px; overflow: hidden;
    min-height: 240px;
  }
  .stage img { max-width: 100%; max-height: 100%; display: block; }
  .wait {
    position: absolute; inset: 0; display: flex; align-items: center; justify-content: center;
    background: #000; color: var(--muted); font-size: 13px;
    transition: opacity .35s;
  }
  .wait.hide { opacity: 0; pointer-events: none; }
  aside { display: flex; flex-direction: column; gap: 12px; min-height: 0; overflow-y: auto; }
  .card { background: var(--panel); border: 1px solid var(--line); border-radius: 8px; padding: 12px 14px; }
  .card h2 {
    margin: 0 0 10px; font-size: 11px; font-weight: 600; letter-spacing: .08em;
    text-transform: uppercase; color: var(--muted);
  }
  .kpis { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; }
  .kpi { background: var(--panel-2); border-radius: 6px; padding: 10px 12px; }
  .kpi .v { font: 600 22px/1.2 var(--mono); color: var(--text); }
  .kpi .k { font-size: 11px; color: var(--muted); margin-top: 2px; }
  .det { padding: 8px 0; border-bottom: 1px solid var(--line); }
  .det:last-child { border-bottom: none; }
  .det .row { display: flex; align-items: baseline; gap: 8px; }
  .det .cls { font-weight: 600; }
  .det .conf { margin-left: auto; font: 600 13px/1 var(--mono); color: var(--accent); }
  .det .box { font: 11px/1.4 var(--mono); color: var(--muted); }
  .bar { height: 3px; background: #262a31; border-radius: 2px; margin-top: 5px; overflow: hidden; }
  .bar i { display: block; height: 100%; background: var(--accent); border-radius: 2px; }
  .counts { display: flex; flex-wrap: wrap; gap: 6px; }
  .chip {
    font: 12px/1 var(--mono); color: var(--muted);
    border: 1px solid var(--line); border-radius: 20px; padding: 5px 10px;
  }
  .chip b { color: var(--text); font-weight: 600; }
  .empty { color: var(--muted); font-size: 13px; padding: 4px 0; }
  .path { font: 11px/1.55 var(--mono); color: var(--muted); word-break: break-all; }
</style>
</head>
<body>
<header>
  <span class="dot" id="conn"></span>
  <h1>阿童木 · 视觉感知监控</h1>
  <span class="meta" id="meta">连接中…</span>
</header>
<main>
  <div class="stage">
    <img id="stream" src="/stream" alt="实时画面">
    <div class="wait" id="wait">正在等待第一帧…</div>
  </div>
  <aside>
    <div class="card">
      <h2>运行指标</h2>
      <div class="kpis">
        <div class="kpi"><div class="v" id="fps">—</div><div class="k">FPS</div></div>
        <div class="kpi"><div class="v" id="lat">—</div><div class="k">耗时 ms</div></div>
        <div class="kpi"><div class="v" id="nobj">—</div><div class="k">当前目标</div></div>
        <div class="kpi"><div class="v" id="nframe">—</div><div class="k">已处理帧</div></div>
      </div>
    </div>
    <div class="card">
      <h2>当前目标（按置信度）</h2>
      <div id="dets"><div class="empty">暂无目标</div></div>
    </div>
    <div class="card">
      <h2>累计检测统计</h2>
      <div class="counts" id="counts"><span class="empty">暂无数据</span></div>
    </div>
    <div class="card">
      <h2>录制文件</h2>
      <div class="path" id="rec">—</div>
    </div>
  </aside>
</main>
<script>
  const $ = (id) => document.getElementById(id);
  const fmt = (v, n) => (v === null || v === undefined) ? "—" : Number(v).toFixed(n);

  function renderDets(list) {
    const box = $("dets");
    if (!list || !list.length) {
      box.innerHTML = '<div class="empty">暂无目标</div>';
      return;
    }
    list.sort((a, b) => b.conf - a.conf);
    box.innerHTML = list.map(d => {
      const pct = Math.max(0, Math.min(1, d.conf)) * 100;
      const b = d.bbox || [0, 0, 0, 0];
      return '<div class="det">'
        + '<div class="row"><span class="cls">' + d.class + '</span>'
        + '<span class="conf">' + fmt(d.conf, 3) + '</span></div>'
        + '<div class="box">bbox ' + b.map(v => Math.round(v)).join(", ") + '</div>'
        + '<div class="bar"><i style="width:' + pct.toFixed(1) + '%"></i></div>'
        + '</div>';
    }).join("");
  }

  function renderCounts(counts) {
    const box = $("counts");
    const keys = Object.keys(counts || {});
    if (!keys.length) {
      box.innerHTML = '<span class="empty">暂无数据</span>';
      return;
    }
    keys.sort((a, b) => counts[b] - counts[a]);
    box.innerHTML = keys.map(k =>
      '<span class="chip">' + k + ' <b>' + counts[k] + '</b></span>'
    ).join("");
  }

  async function tick() {
    try {
      const res = await fetch("/stats.json", { cache: "no-store" });
      const s = await res.json();
      $("conn").className = "dot on";
      if (s.frame > 0) $("wait").classList.add("hide");
      $("fps").textContent = fmt(s.fps, 1);
      $("lat").textContent = fmt(s.latency_ms, 0);
      $("nobj").textContent = s.detections ? s.detections.length : 0;
      $("nframe").textContent = s.frame || 0;
      const size = Array.isArray(s.image_size) ? s.image_size.join(" × ") : "—";
      $("meta").textContent = (s.source || "—") + "  ·  " + size + "  ·  平均置信度 " + fmt(s.avg_conf, 3);
      $("rec").textContent = s.record || "未录制";
      renderDets(s.detections);
      renderCounts(s.counts);
    } catch (e) {
      $("conn").className = "dot off";
    }
    setTimeout(tick, 300);
  }
  $("stream").addEventListener("error", () => {
    const w = $("wait");
    w.classList.remove("hide");
    w.textContent = "画面流断了，刷新页面重连";
  });
  tick();
</script>
</body>
</html>
"""


class _PreviewServer(ThreadingHTTPServer):
    """跳过 HTTPServer.server_bind() 里的 socket.getfqdn()。

    那个调用会对主机名做反向解析，中文计算机名在 Python 3.8 下会返回非 UTF-8
    字节，``gethostbyaddr`` 直接抛 UnicodeDecodeError，服务根本起不来。
    """

    def server_bind(self) -> None:
        socketserver.TCPServer.server_bind(self)
        host, port = self.server_address[:2]
        self.server_name = str(host)
        self.server_port = port


class FrameHolder:
    """主线程发布最新一帧 JPEG 和一份统计，HTTP 线程各取所需。"""

    def __init__(self) -> None:
        self._cond = threading.Condition()
        self._jpeg: Optional[bytes] = None
        self._stats: dict = {}
        self._seq = 0

    def publish(self, jpeg: bytes, stats: dict) -> None:
        with self._cond:
            self._jpeg = jpeg
            self._stats = stats
            self._seq += 1
            self._cond.notify_all()

    def wait_first(self, timeout: float = 60.0) -> Tuple[Optional[bytes], int]:
        """等第一帧就绪。模型加载要几秒，期间别给浏览器回应，否则它先看到空响应。"""
        with self._cond:
            self._cond.wait_for(lambda: self._jpeg is not None, timeout=timeout)
            return self._jpeg, self._seq

    def wait_next(self, last_seq: int, timeout: float = 5.0) -> Tuple[Optional[bytes], int]:
        with self._cond:
            self._cond.wait_for(lambda: self._seq != last_seq, timeout=timeout)
            return self._jpeg, self._seq

    def snapshot(self) -> dict:
        with self._cond:
            return self._stats


def make_handler(holder: FrameHolder):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args) -> None:  # 别把每个请求都刷到终端
            pass

        def _send(self, body: bytes, content_type: str) -> None:
            self.send_response(200)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self) -> None:
            path = self.path.split("?")[0]
            if path in ("/", "/index.html"):
                self._send(UI_PAGE.encode("utf-8"), "text/html; charset=utf-8")
            elif path == "/stats.json":
                payload = json.dumps(holder.snapshot(), ensure_ascii=False).encode("utf-8")
                self._send(payload, "application/json; charset=utf-8")
            elif path == "/stream":
                self._stream()
            elif path == "/favicon.ico":
                self.send_response(204)
                self.end_headers()
            else:
                self.send_error(404)

        def _stream(self) -> None:
            print("[preview] 浏览器已连接画面流")
            # 先等第一帧再回响应头：这之前浏览器显示的是「加载中」，
            # 而不是一个内容为空的 200 导致图片裂开。
            first, seq0 = holder.wait_first()
            if first is None:
                self.send_error(503, "no frame yet")
                return
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            last_seq = seq0 - 1
            try:
                while True:
                    jpg, seq = holder.wait_next(last_seq)
                    if jpg is None or seq == last_seq:
                        continue  # 等超时且没有新帧，别重复推
                    last_seq = seq
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % len(jpg))
                    self.wfile.write(jpg)
                    self.wfile.write(b"\r\n")
            except (BrokenPipeError, ConnectionResetError, OSError):
                return  # 浏览器关掉了页面

    return Handler


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="感知实时监控页（浏览器版，不依赖 OpenCV GUI）")
    p.add_argument("--source", default="0", help="摄像头编号或视频路径，默认 0")
    p.add_argument("--config", default=str(PERCEPTION_ROOT / "configs" / "perception.yaml"))
    p.add_argument("--weights", default=None)
    p.add_argument("--device", default=None)
    p.add_argument("--conf", type=float, default=None)
    p.add_argument("--backend", default="auto", choices=["auto", "default", "dshow", "msmf"])
    p.add_argument("--host", default="127.0.0.1", help="监听地址，0.0.0.0 表示允许局域网访问")
    p.add_argument("--port", type=int, default=8080)
    p.add_argument("--quality", type=int, default=80, help="JPEG 质量 1-100")
    p.add_argument("--loop", action="store_true", help="视频文件播完自动重播")
    p.add_argument("--no-browser", action="store_true", help="不自动打开浏览器")
    p.add_argument("--no-save", action="store_true", help="不录制视频（默认会把带框的画面存成 mp4）")
    p.add_argument("--out", default=None, help="录制输出目录，默认 perception/runs/preview_<时间戳>/")
    p.add_argument("--record-fps", type=float, default=10.0,
                   help="录制帧率，按实际处理速度（约 7-10 FPS）设置，否则回放会快进")
    return p.parse_args()


def main() -> int:
    args = parse_args()
    is_camera = str(args.source).isdigit()

    # 先打开输入源：打不开就在这里报错退出，别先开出一个永远空白的页面。
    if is_camera:
        cap, backend = open_camera(int(args.source), args.backend)
        if cap is None:
            print("[preview] 摄像头 %s 打不开，先跑 python scripts/demo_detect.py --probe-cameras" % args.source)
            return 1
        print("[preview] 摄像头 %s 已打开，后端 = %s" % (args.source, backend))
    else:
        cap = cv2.VideoCapture(args.source)
        if not cap.isOpened():
            print("[preview] 打不开输入: %s" % args.source)
            return 1

    cfg = PerceptionConfig.from_yaml(args.config)
    if args.weights:
        cfg.detector.weights = args.weights
    if args.device:
        cfg.detector.device = args.device
    if args.conf is not None:
        cfg.detector.conf = args.conf
    detector = YoloDetector.from_config(cfg.detector)
    pipeline = PerceptionPipeline(cfg, detector=detector)

    holder = FrameHolder()
    server = _PreviewServer((args.host, args.port), make_handler(holder))
    threading.Thread(target=server.serve_forever, daemon=True).start()

    shown_host = "127.0.0.1" if args.host in ("", "0.0.0.0") else args.host
    url = "http://%s:%d/" % (shown_host, args.port)
    print("[preview] 监控页地址: %s" % url)
    if args.host == "0.0.0.0":
        print("[preview] 注意：已监听所有网卡，局域网内其他机器也能访问这个画面")
    print("[preview] OpenCV 窗口可用: %s（可用的话也可以直接跑 demo_detect.py 看原生窗口）" % gui_available())

    out_dir = Path(args.out) if args.out else PERCEPTION_ROOT / "runs" / ("preview_%s" % time.strftime("%Y%m%d_%H%M%S"))
    video_path = out_dir / "annotated.mp4"
    if not args.no_save:
        out_dir.mkdir(parents=True, exist_ok=True)
        print("[preview] 录制输出: %s" % video_path)
    print("[preview] 浏览器里看画面，终端按 Ctrl+C 结束")
    if not args.no_browser:
        webbrowser.open(url)

    fps_ema = 0.0
    frames = 0
    writer = None
    pushed_bytes = 0
    conf_sum = 0.0
    all_counts: Counter = Counter()
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                if args.loop and not is_camera:
                    cap.release()
                    cap = cv2.VideoCapture(args.source)
                    continue
                break

            t0 = time.perf_counter()
            result = pipeline.process(image=frame, stamp=time.time())
            vis = draw_result(frame, result, show_range=False)
            dt = time.perf_counter() - t0
            inst = 1.0 / dt if dt > 0 else 0.0
            fps_ema = inst if frames == 0 else 0.85 * fps_ema + 0.15 * inst
            frames += 1
            conf_sum += sum(d.confidence for d in result.detections)
            all_counts.update(d.class_name for d in result.detections)

            hud = "%.1f FPS   %d obj" % (fps_ema, len(result.detections))
            cv2.putText(vis, hud, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0), 4, cv2.LINE_AA)
            cv2.putText(vis, hud, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (80, 255, 120), 2, cv2.LINE_AA)

            if not args.no_save:
                if writer is None:
                    vh, vw = vis.shape[:2]
                    writer = cv2.VideoWriter(str(video_path), cv2.VideoWriter_fourcc(*"mp4v"),
                                             args.record_fps, (vw, vh))
                    if not writer.isOpened():
                        print("[preview] ⚠ 视频编码器不可用，本次不录制", flush=True)
                        writer = None
                        args.no_save = True
                if writer is not None:
                    writer.write(vis)

            ok_enc, buf = cv2.imencode(".jpg", vis, [int(cv2.IMWRITE_JPEG_QUALITY), args.quality])
            if not ok_enc:
                continue

            n_det = sum(all_counts.values())
            h, w = vis.shape[:2]
            stats = {
                "frame": frames,
                "fps": round(fps_ema, 2),
                "latency_ms": round(dt * 1000.0, 1),
                "image_size": [w, h],
                "source": str(args.source),
                "detections": [
                    {
                        "class": d.class_name,
                        "conf": round(d.confidence, 3),
                        "bbox": [round(v, 1) for v in d.bbox],
                    }
                    for d in result.detections
                ],
                "counts": dict(all_counts),
                "avg_conf": round(conf_sum / n_det, 3) if n_det else None,
                "record": None if args.no_save else str(video_path),
            }
            pushed_bytes += len(buf.tobytes())
            holder.publish(buf.tobytes(), stats)
            if frames % 30 == 0:
                print("[preview] 第 %d 帧  %.1f FPS  %d 个目标  已编码 %.0f KB"
                      % (frames, fps_ema, len(result.detections), pushed_bytes / 1024.0), flush=True)
    except KeyboardInterrupt:
        print("\n[preview] 收到 Ctrl+C，结束")
    finally:
        if cap is not None:
            cap.release()
        if writer is not None:
            writer.release()
        server.shutdown()

    print("[preview] 共处理 %d 帧" % frames)
    if writer is not None:
        print("[preview] 录像已保存: %s" % video_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
