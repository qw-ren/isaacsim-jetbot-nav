import importlib.util
from pathlib import Path
import tempfile

ROOT = Path(__file__).resolve().parents[2]


def load(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / "isaac_sim/replicator" / (name + ".py"))
    module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module


def test_yolo_labels_reject_fractional_ids_nan_and_outside_edges():
    parse = load("validate_dataset").parse_label_file
    with tempfile.TemporaryDirectory() as temp:
        path = Path(temp) / "labels.txt"
        for line in ["0.5 .5 .5 .2 .2", "0 nan .5 .2 .2", "0 .95 .5 .2 .2", "inf .5 .5 .2 .2"]:
            path.write_text(line)
            boxes, errors = parse(path, 8)
            assert not boxes and errors, line
        path.write_text("2 .5 .5 .2 .2")
        assert parse(path, 8) == ([(2, .5, .5, .2, .2)], [])


def test_asset_pilot_preserves_ids_and_excludes_ambiguous_sign():
    module = load("prepare_asset_pilot")
    classes = ["box"] * 4 + ["pallet"] * 2 + ["barrier", "cart"]
    registry = dict(source_root="/assets", assets={key: dict(verified=True, dims_m=[1, 1, 1],
                     path=key + ".usd", **{"class": kind}) for key, kind in zip(module.ASSETS, classes)})
    template = dict(obstacles=[], scene={})
    cfg = module.build_config(template, registry, "/tmp/pilot")
    assert template == dict(obstacles=[], scene={})
    names = list(dict.fromkeys(row["class_name"] for row in cfg["obstacles"]))
    assert names == module.CLASSES
    assert cfg["output_dir"] == "/tmp/pilot"
    assert cfg["obstacles"][8]["usd"] == "/assets/box_A.usd"
    assert cfg["pilot_only"] and cfg["coverage_pending"] == ["shelf", "wall", "sign", "person"]
    assert all("wet" not in row.get("usd", "") for row in cfg["obstacles"])
