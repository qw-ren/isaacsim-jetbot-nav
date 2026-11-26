import copy
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from atongmu_perception.planar_map import PlanarObjectStore


def packet(session="run", sequence=1, stamp=10.):
    return dict(schema="atongmu.planar_observations", version=1, source="vision", frame_id="map",
                session_id=session, sequence=sequence, stamp=stamp,
                objects=[dict(object_id="vision:%s:1" % session, track_id=1, class_name="box",
                              confidence=.9, position_xy=[2., 3.], position_reference="visible_surface",
                              localization_method="depth", range_m=2.)])


def test_store_updates_one_track_and_keeps_only_xy():
    store = PlanarObjectStore()
    store.update(packet(), 10., 100.)
    p = packet(sequence=2, stamp=11.)
    p["objects"][0]["position_xy"] = [4., 5.]
    out = store.update(p, 11., 101.)
    assert len(out["objects"]) == 1 and out["objects"][0]["position_xy"] == [4., 5.]
    assert "dimensions" not in out["objects"][0]


def test_store_expires_when_sim_pauses():
    store = PlanarObjectStore()
    store.update(packet(), 10., 100.)
    assert not store.snapshot(10., 106.)["objects"]


def test_store_new_session_removes_old_objects():
    store = PlanarObjectStore()
    store.update(packet(), 10., 100.)
    out = store.update(packet(session="restart", stamp=11.), 11., 101.)
    assert [o["object_id"] for o in out["objects"]] == ["vision:restart:1"]


def test_store_batch_validation_is_atomic():
    store = PlanarObjectStore()
    store.update(packet(), 10., 100.)
    before = copy.deepcopy(store.objects)
    bad = packet(sequence=2, stamp=11.)
    bad["objects"] *= 2
    try:
        store.update(bad, 11., 101.)
    except ValueError:
        pass
    else:
        raise AssertionError("duplicate ID accepted")
    assert before == store.objects and store.last_sequence == 1


def test_store_rejects_stale_or_wrong_frame():
    for change in ({"frame_id": "chassis"}, {"stamp": 1.}, {"source": "ground_truth"}):
        store = PlanarObjectStore()
        data = packet();data.update(change)
        try:
            store.update(data, 10., 100.)
        except ValueError:
            pass
        else:
            raise AssertionError("invalid packet accepted")
