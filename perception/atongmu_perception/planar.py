"""Planar observations: measured XY points, never invented object boxes or goals."""
from __future__ import annotations

import math
from typing import Any

from .types import PerceptionResult


def json_finite(value: Any) -> Any:
    """Replace unavailable floating measurements with JSON null (not NaN)."""
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: json_finite(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [json_finite(item) for item in value]
    return value


def planar_observations(result: PerceptionResult, session_id: str, sequence: int) -> dict:
    """Project measured points to XY. IDs are session-local visual tracks.

    Camera depth locates a visible surface, not the object's centre. Track IDs
    are not guaranteed to survive occlusion. Consumers must associate objects
    and choose reachable approach poses themselves.
    """
    if not session_id or not result.frame_id:
        raise ValueError("session_id and frame_id must be nonempty")
    if (not math.isfinite(result.stamp) or result.stamp <= 0
            or type(sequence) is not int or sequence < 0):
        raise ValueError("invalid simulation stamp or sequence")
    objects = []
    for obj in result.objects:
        loc = obj.localization
        if (type(obj.track_id) is not int or obj.track_id < 0
                or not isinstance(obj.class_name, str) or not obj.class_name
                or loc is None or loc.method not in ("depth", "lidar", "lidar+depth", "ground")
                or obj.frame_id != result.frame_id or obj.stamp != result.stamp):
            continue
        xy = tuple(float(v) for v in obj.position[:2])
        if (len(xy) != 2 or not all(math.isfinite(v) for v in xy)
                or not math.isfinite(obj.confidence) or not 0 <= obj.confidence <= 1
                or not math.isfinite(loc.range_m) or loc.range_m < 0):
            continue
        objects.append({
            "object_id": "vision:%s:%d" % (session_id, obj.track_id),
            "track_id": int(obj.track_id),
            "class_name": obj.class_name,
            "confidence": float(obj.confidence),
            "position_xy": list(xy),
            "position_reference": "ground_contact_estimate" if loc.method == "ground" else "visible_surface",
            "range_m": float(loc.range_m),
            "localization_method": loc.method,
        })
    return {
        "schema": "atongmu.planar_observations",
        "version": 1,
        "source": "vision",
        "session_id": session_id,
        "sequence": int(sequence),
        "stamp": float(result.stamp),
        "time_domain": "ros",
        "frame_id": result.frame_id,
        "objects": objects,
    }
