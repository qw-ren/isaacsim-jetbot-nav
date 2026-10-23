"""Longer route diagnostics with bounded raw samples; unchanged point stream semantics."""
import numpy as np
from frame20ms_points import Frame20msTrialAdapter
class RouteTrialAdapter(Frame20msTrialAdapter):
    maximum_frames=6000
    debug_windows=[3,12,25,40,55,70]
    def attach(self):
        super().attach();self.raw_route_windows=set()
    def archive_frame(self,data,index,fid):
        window=int(float(self.env.sim.current_time))
        if window not in [3,12,25,40,55,70] or window in self.raw_route_windows:return
        d=np.asarray(data['distances']);valid=(np.abs(data['azimuths'])<40)&(np.abs(data['elevations'])<15)&(d>.3)&(d<30)
        if not valid.any():return
        super().archive_frame(data,index,fid);self.raw_route_windows.add(window)
