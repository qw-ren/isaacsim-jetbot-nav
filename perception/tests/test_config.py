"""配置加载的单元测试。

不依赖 pytest 也能跑：python tests/run_all.py

回归用例：DetectorConfig.classes 用 field(default_factory=...)，类上没有同名属性，
早期用 hasattr(cls, k) 过滤时 yaml 里的 classes 会被静默丢掉（2026-11-12 发现）。
"""
from __future__ import annotations

import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from atongmu_perception.config import DetectorConfig, PerceptionConfig  # noqa: E402

YAML = """
detector:
  weights: weights/obstacles_best.pt
  device: cuda:0
  conf: 0.4
  classes:
    - shelf
    - wall
    - box
"""


def test_detector_classes_from_dict() -> None:
    cfg = DetectorConfig.from_dict({"classes": ["shelf", "person"], "conf": 0.5})
    assert cfg.classes == ["shelf", "person"], cfg.classes
    assert cfg.conf == 0.5


def test_detector_classes_from_yaml() -> None:
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "perception.yaml"
        path.write_text(YAML, encoding="utf-8")
        cfg = PerceptionConfig.from_yaml(str(path))
    assert cfg.detector.classes == ["shelf", "wall", "box"], cfg.detector.classes
    assert cfg.detector.weights == "weights/obstacles_best.pt"
    assert cfg.detector.device == "cuda:0"
    assert cfg.detector.conf == 0.4


def test_unknown_keys_ignored() -> None:
    cfg = DetectorConfig.from_dict({"classes": ["box"], "not_a_field": 1})
    assert cfg.classes == ["box"]
    assert not hasattr(cfg, "not_a_field")
