"""Bounded current RTX packets for navigation; no LIO/IMU or TF publishers.

Each packet uses the native render-end transform. This is a navigation point
cloud approximation, not a claim of exact per-ray motion compensation.
"""
from collections import deque
from pathlib import Path
import json
import numpy as np
from scipy.spatial.transform import Rotation

_active = None

class LiveRtxPoints:
    def __init__(self, raw, scan_buffer, env, artifacts):
        global _active
        self.raw, self.scan_buffer, self.env = raw, scan_buffer, env.unwrapped
        self.root = Path(artifacts) / 'live_rtx'
        self.root.mkdir(exist_ok=False)
        self.stream = (self.root / 'packets.jsonl').open('x')
        config = Path(artifacts) / 'box_config.json'
        box = json.loads(config.read_text()) if config.exists() else dict(center=[4.,-1.,.5],size=[.6,.6,1.])
        box = box.get('barrier', box)
        self.box_center = np.asarray(box['center'])
        self.box_half = np.asarray(box['size']) / 2
        self.group = deque(maxlen=5)
        self.last_frame = None
        self.last_stamp = None
        self.saved = set()
        _active = self

    def tick(self):
        data = self.raw.get_data()
        if not data or int(data.get('timestampNs', 0)) <= 0:
            return
        frame = int(data['frameId'])
        if frame == self.last_frame:
            return
        self.last_frame = frame
        stamp = float(self.env.sim.current_time)
        if self.last_stamp is not None and abs(stamp-self.last_stamp-.02) > 1e-5:
            self.group.clear()
        self.last_stamp = stamp
        d = np.asarray(data['distances'])
        a = np.deg2rad(data['azimuths'])
        e = np.deg2rad(data['elevations'])
        valid = np.isfinite(d) & np.isfinite(a) & np.isfinite(e) & (d > .3) & (d < 30.)
        d, a, e = d[valid], a[valid], e[valid]
        xyz = np.column_stack((d*np.cos(e)*np.cos(a), d*np.cos(e)*np.sin(a), d*np.sin(e)))
        transform = np.asarray(data['transform']).reshape(4,4)
        world = xyz @ transform[:3,:3] + transform[3,:3]
        self.group.append((stamp, world))
        accumulated = np.concatenate([p for _,p in self.group])
        old = np.asarray(self.scan_buffer.get_data()['data']).reshape(-1,3)
        def hits(points):
            return int(np.sum(np.all(abs(points-self.box_center) < self.box_half+.04,axis=1)))
        row = dict(sim=stamp, frame=frame, packets=len(self.group), points=len(accumulated), raw_box_hits=hits(accumulated), buffer_box_hits=hits(old))
        self.stream.write(json.dumps(row)+'\n')
        if frame % 25 == 0:
            self.stream.flush()
        window = int(stamp)
        if window not in self.saved and window in [3,7,11,15,19,23,27,31]:
            self.saved.add(window)
            np.savez_compressed(self.root/f'points_{window}.npz', fresh_world=accumulated, buffer_world=old, transform=transform)

    def points(self):
        if not self.group:
            return np.empty((0,3), dtype=np.float32)
        world = np.concatenate([p for _,p in self.group])
        state = self.env.scene['unitree_go2'].data.root_state_w[0].detach().cpu().numpy()
        rotation = Rotation.from_quat(state[[4,5,6,3]])
        origin = state[:3]+rotation.apply([.2,0,.2])
        return rotation.inv().apply(world-origin).astype(np.float32)

    def close(self):
        self.stream.close()

def sensor_points(annotator, env, stamp):
    return _active.points()
