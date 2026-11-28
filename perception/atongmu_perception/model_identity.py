"""Bind runtime observations to the exact checkpoint and class ID order."""
import hashlib
import importlib.metadata
import json
from pathlib import Path


def model_identity(weights, names):
    path = Path(weights).resolve()
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    classes = [names[i] for i in range(len(names))]
    manifest_path = path.with_suffix(".manifest.json")
    if manifest_path.exists():
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        if manifest["sha256"] != digest or manifest["classes"] != classes:
            raise ValueError("checkpoint SHA256 or class order differs from manifest")
    return dict(weights_sha256=digest, classes=classes,
                inference_version="atongmu.planar.v1",
                runtime={p: importlib.metadata.version(p) for p in ("ultralytics", "torch")})
