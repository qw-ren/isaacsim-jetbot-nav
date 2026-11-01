"""Pair native LiDAR rays with the same render product's clock, without pose inputs."""
import math
import numpy as np

def packet_timing(frame_id,render_times,delta_ns,now,expected_frame_offset=None):
 if not render_times or 'simulationTime'not in render_times or 'frameNumber'not in render_times:return None
 end=float(render_times['simulationTime']);offset=int(render_times['frameNumber'])-int(frame_id);ray_offsets=np.asarray(delta_ns,dtype=np.float64)*1e-9
 if not len(ray_offsets)or not np.isfinite(ray_offsets).all()or ray_offsets.min()<0:return None
 duration=math.ceil(float(ray_offsets.max())/.005)*.005
 if not math.isfinite(end)or not 0<duration<=.041 or end-duration<0 or not -.0051<=now-end<=.10:return None
 if expected_frame_offset is not None and offset!=expected_frame_offset:return None
 return dict(start=end-duration,end=end,ray_offsets=ray_offsets,frame_offset=offset)
