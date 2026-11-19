"""Explicit map-frame navigation poses, bound to scene and object geometry."""
import copy
import math
from .core import finite_vector


def validate_bindings(document, scene_id):
    if document.get('version') != 1 or document.get('frame_id') != 'map' or document.get('scene_id') != scene_id:
        raise ValueError('Binding version/frame/scene mismatch')
    result = {}
    for row in document['objects']:
        key = row['object_id']
        if not isinstance(key, str) or not key or key in result:
            raise ValueError('Invalid or duplicate bound object ID')
        record = copy.deepcopy(row)
        record['goal'] = finite_vector(row['goal'], 3)
        record['object_position'] = finite_vector(row['object_position'], 3)
        record['object_dimensions'] = finite_vector(row['object_dimensions'], 3)
        record['object_orientation_xyzw'] = finite_vector(row['object_orientation_xyzw'], 4)
        if abs(sum(v*v for v in record['object_orientation_xyzw'])-1)>0.002:
            raise ValueError('Invalid bound quaternion')
        if min(record['object_dimensions']) <= 0:
            raise ValueError('Invalid bound dimensions')
        for key2, default in [('max_position_delta_m', .10), ('max_dimension_delta_m', .05), ('max_rotation_delta_rad', .10)]:
            value = record.get(key2, default)
            if type(value) not in (int, float) or not math.isfinite(value) or not 0 < value <= .20:
                raise ValueError('Binding tolerance invalid')
            record[key2] = value
        result[key] = record
    return result


def bound_goal(bindings, obj):
    row = bindings.get(obj['object_id'])
    if row is None:
        raise ValueError('OBJECT_UNBOUND')
    if math.dist(row['object_position'], obj['position']) > row['max_position_delta_m']:
        raise ValueError('BINDING_STALE_POSITION')
    if max(abs(a-b) for a,b in zip(row['object_dimensions'], obj['dimensions'])) > row['max_dimension_delta_m']:
        raise ValueError('BINDING_STALE_GEOMETRY')
    dot = min(1., abs(sum(a*b for a,b in zip(row['object_orientation_xyzw'], obj['orientation_xyzw']))))
    if 2*math.acos(dot) > row['max_rotation_delta_rad']:
        raise ValueError('BINDING_STALE_ROTATION')
    return list(row['goal'])
