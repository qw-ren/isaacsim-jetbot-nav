import importlib.util
from pathlib import Path
import tempfile

path = Path(__file__).resolve().parents[2] / "deployment/server_demo/encode_observer_video.py"
spec = importlib.util.spec_from_file_location("encode_observer_video", path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_video_uses_simulation_gaps_and_playback_speed():
    with tempfile.TemporaryDirectory() as temp:
        root = Path(temp)
        for name in ["a.jpg", "b.jpg", "c.jpg"]:
            (root / name).write_bytes(b"frame")
        (root / "frames.csv").write_text("frame,requested_sim_s,completed_sim_s,file\n0,1,2,a.jpg\n1,1.2,9,b.jpg\n2,1.8,10,c.jpg\n")
        manifest, info = module.prepare_manifest(root, 2.)
        assert "duration 0.100000000" in manifest and "duration 0.300000000" in manifest
        assert abs(info["expected_video_duration_s"] - .6) < 1e-9


def test_video_rejects_frame_paths_outside_recording_folder():
    with tempfile.TemporaryDirectory() as temp:
        root = Path(temp)
        (root / "frames.csv").write_text("frame,requested_sim_s,completed_sim_s,file\n0,1,2,../outside.jpg\n")
        try:
            module.prepare_manifest(root)
        except ValueError:
            pass
        else:
            raise AssertionError("unsafe frame path accepted")
