"""Short-lived, map-frame XY observation cache; no invented object extent."""
from __future__ import annotations

import math


class PlanarObjectStore:
    def __init__(self, frame_id="map", max_age=5.0):
        if max_age <= 0 or not math.isfinite(max_age):
            raise ValueError("max_age must be positive")
        self.frame_id, self.max_age = frame_id, float(max_age)
        self.session = None
        self.last_sequence = -1
        self.last_stamp = -1.
        self.objects = {}

    def update(self, data, now_sim, now_wall):
        if (data.get("schema") != "atongmu.planar_observations" or data.get("version") != 1
                or data.get("source") != "vision" or data.get("frame_id") != self.frame_id):
            raise ValueError("unexpected planar source/schema/frame")
        stamp, sequence, session = data.get("stamp"), data.get("sequence"), data.get("session_id")
        if (type(stamp) not in (float, int) or not math.isfinite(stamp) or stamp <= 0
                or type(sequence) is not int or sequence < 0
                or not isinstance(session, str) or not session
                or not all(math.isfinite(v) for v in (now_sim, now_wall))
                or not -.1 <= now_sim - stamp <= self.max_age):
            raise ValueError("invalid/stale planar header")
        if session == self.session and (sequence <= self.last_sequence or stamp <= self.last_stamp):
            raise ValueError("duplicate/out-of-order observation")
        if not isinstance(data.get("objects"), list):
            raise ValueError("objects must be a list")
        updates = {}
        for obj in data["objects"]:
            tid, xy = obj.get("track_id"), obj.get("position_xy")
            confidence = obj.get("confidence")
            if (type(tid) is not int or tid < 0 or obj.get("object_id") != "vision:%s:%d" % (session, tid)
                    or not isinstance(xy, list) or len(xy) != 2
                    or not all(type(v) in (int, float) and math.isfinite(v) for v in xy)
                    or type(confidence) not in (int, float) or not math.isfinite(confidence)
                    or not 0 <= confidence <= 1 or not isinstance(obj.get("class_name"), str)
                    or not obj["class_name"] or obj["object_id"] in updates
                    or obj.get("localization_method") not in ("depth", "lidar", "lidar+depth", "ground")):
                raise ValueError("invalid planar object")
            reference = "ground_contact_estimate" if obj["localization_method"] == "ground" else "visible_surface"
            if obj.get("position_reference") != reference:
                raise ValueError("invalid measurement reference")
            previous = self.objects.get(obj["object_id"]) if session == self.session else None
            if previous and previous["class_name"] != obj["class_name"]:
                raise ValueError("track changed class")
            updates[obj["object_id"]] = dict(obj, position_xy=list(xy), last_seen=float(stamp), _received=now_wall)
        # Commit only after validating the entire batch.
        if session != self.session:
            self.objects.clear()
        self.session, self.last_sequence, self.last_stamp = session, sequence, float(stamp)
        self.objects.update(updates)
        return self.snapshot(now_sim, now_wall)

    def snapshot(self, now_sim, now_wall):
        self.objects = {key: obj for key, obj in self.objects.items()
                        if -.1 <= now_sim - obj["last_seen"] <= self.max_age
                        and 0 <= now_wall - obj["_received"] <= self.max_age}
        rows = [{key: value for key, value in obj.items() if key != "_received"}
                for _, obj in sorted(self.objects.items())]
        return dict(schema="atongmu.planar_map", version=1, frame_id=self.frame_id,
                    source="vision", session_id=self.session, stamp=float(now_sim), objects=rows)
