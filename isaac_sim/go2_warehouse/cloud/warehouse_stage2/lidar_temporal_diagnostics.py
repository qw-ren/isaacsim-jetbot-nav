"""Extra raw channels for an isolated, bounded LiDAR provenance experiment."""
import json
from pathlib import Path
import numpy as np

OUT = Path('/home/admin/go2-warehouse/artifacts/m1_lidar_temporal_20261015/sensor')
_seen = set()

def capture_scan(annotator, stamp, env):
    raw = annotator.get_data()
    points = raw['data'].reshape(-1, 3)
    key = round(stamp, 5)
    if key in _seen or not (17.99 <= stamp <= 18.055):
        return points
    _seen.add(key)
    try:
        OUT.mkdir(parents=True, exist_ok=True)
        arrays = {}
        meta = {'stamp': stamp, 'fields': {}}
        def collect(value, prefix):
            if isinstance(value, dict):
                for k,v in value.items():collect(v, prefix+'_'+str(k) if prefix else str(k))
            elif isinstance(value, np.ndarray):
                arrays[prefix] = value.copy()
                meta['fields'][prefix] = {'shape': list(value.shape), 'dtype': str(value.dtype)}
            elif isinstance(value, (str, bool, int, float)):
                meta['fields'][prefix] = value
        collect(raw, '')
        robot = env.unwrapped.scene['unitree_go2'].data
        arrays['root_state_w'] = robot.root_state_w.detach().cpu().numpy().copy()
        object_arrays = [v for k,v in arrays.items() if 'objectid' in k.lower()]
        if object_arrays:
            from omni.syntheticdata._syntheticdata import acquire_syntheticdata_interface
            iface = acquire_syntheticdata_interface()
            meta['id_to_path'] = {str(int(i)): iface.get_uri_from_instance_segmentation_id(int(i)) for v in object_arrays for i in np.unique(v)}
        np.savez_compressed(OUT/f'raw_{key:.5f}.npz', **arrays)
        (OUT/f'raw_{key:.5f}.json').write_text(json.dumps(meta, indent=2)+'\n')
    except Exception as exc:
        OUT.mkdir(parents=True, exist_ok=True)
        (OUT/f'error_{key:.5f}.json').write_text(json.dumps({'error':str(exc),'stamp':stamp})+'\n')
    return points
