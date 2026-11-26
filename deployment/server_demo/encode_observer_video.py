#!/usr/bin/env python3
"""Encode actual observer frames using simulation-time intervals, not wall time."""
import argparse
import csv
import json
import math
from pathlib import Path
import statistics
import subprocess


def prepare_manifest(frame_dir, playback_speed=1.):
    root = Path(frame_dir).resolve()
    if not math.isfinite(playback_speed) or playback_speed <= 0:
        raise ValueError("playback speed must be positive")
    frames = []
    with (root / "frames.csv").open(newline="") as stream:
        for row in csv.DictReader(stream):
            path = (root / row["file"]).resolve()
            if path.parent != root or not path.is_file() or not path.stat().st_size:
                raise ValueError("missing frame or unsafe frame path")
            stamp = float(row["requested_sim_s"])
            if not math.isfinite(stamp):
                raise ValueError("invalid frame stamp")
            frames.append((stamp, path))
    frames.sort(key=lambda row: row[0])
    if len(frames) < 2:
        raise ValueError("need at least two recorded frames")
    gaps = [b[0] - a[0] for a, b in zip(frames, frames[1:])]
    if any(gap <= 0 for gap in gaps):
        raise ValueError("frame timestamps must strictly increase")
    durations = gaps + [statistics.median(gaps)]
    lines = ["ffconcat version 1.0"]
    for (_, path), duration in zip(frames, durations):
        escaped = str(path).replace("'", "'\\''")
        lines.extend(["file '%s'" % escaped, "duration %.9f" % (duration / playback_speed)])
    lines.append("file '%s'" % str(frames[-1][1]).replace("'", "'\\''"))
    return "\n".join(lines) + "\n", dict(frames=len(frames),
        simulation_duration_s=sum(durations), expected_video_duration_s=sum(durations) / playback_speed,
        playback_speed=playback_speed, timebase="recorded simulation timestamps")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--case", type=Path, required=True)
    ap.add_argument("--speed", type=float, default=1.)
    args = ap.parse_args()
    root = args.case / "observer_video"
    manifest, report = prepare_manifest(root, args.speed)
    path = root / "frames.ffconcat"
    path.write_text(manifest, encoding="utf-8")
    output = root / ("navigation_%gx.mp4" % args.speed)
    if output.exists():
        raise FileExistsError(output)
    subprocess.run(["ffmpeg", "-nostdin", "-hide_banner", "-loglevel", "warning", "-n", "-f", "concat",
                    "-safe", "0", "-i", str(path), "-vf", "fps=25,scale=trunc(iw/2)*2:trunc(ih/2)*2",
                    "-an", "-c:v", "libx264", "-preset", "fast", "-crf", "23",
                    "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(output)], check=True)
    info = subprocess.run(["ffprobe", "-v", "error", "-show_entries", "format=duration,size",
                           "-of", "json", str(output)], capture_output=True, text=True, check=True)
    report.update(output=str(output), ffprobe=json.loads(info.stdout))
    (root / "ENCODING_REPORT.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
