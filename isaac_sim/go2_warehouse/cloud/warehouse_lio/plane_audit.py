"""Known static calibration plane, installed before motion; never a navigation task."""
import json
from pathlib import Path
import numpy as np
import omni.usd
from pxr import UsdGeom,UsdPhysics,Gf
from rtx_time_audit import TimeAuditAdapter

PLANE_PATH='/World/LioCalibrationTarget'
PLANE_CENTER=[6.5,-1.,1.5]
PLANE_SIZE=[.1,6.,3.]

def add_calibration_plane(artifacts):
    stage=omni.usd.get_context().get_stage()
    if stage.GetPrimAtPath(PLANE_PATH).IsValid():raise RuntimeError('Calibration target already exists')
    cube=UsdGeom.Cube.Define(stage,PLANE_PATH);cube.CreateSizeAttr(1.)
    cube.AddTranslateOp().Set(Gf.Vec3d(*PLANE_CENTER));cube.AddScaleOp().Set(Gf.Vec3f(*PLANE_SIZE))
    cube.CreateDisplayColorAttr([Gf.Vec3f(.05,.8,.9)])
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim()).CreateCollisionEnabledAttr(True)
    bounds=UsdGeom.BBoxCache(0,[UsdGeom.Tokens.default_]).ComputeWorldBound(cube.GetPrim()).ComputeAlignedRange()
    out=Path(artifacts);out.mkdir(parents=True,exist_ok=True)
    (out/'calibration_plane.json').write_text(json.dumps({'path':PLANE_PATH,'center':PLANE_CENTER,'size':PLANE_SIZE,'min':list(bounds.GetMin()),'max':list(bounds.GetMax()),'purpose':'analytic ray-distance time/geometry calibration; fixed before start; not sudden blockage','guard_includes_target':True},indent=2)+'\n')

class PlaneAuditAdapter(TimeAuditAdapter):
    maximum_frames=1500
    archive_every=1
    def archive_frame(self,data,index,fid):
        distance=np.asarray(data['distances']);mask=(np.abs(data['azimuths'])<40)&(np.abs(data['elevations'])<15)&(distance>.3)&(distance<30)
        if not mask.any():return
        arrays={k:np.asarray(data[k])[mask].copy() for k in ['azimuths','elevations','distances','channels','deltaTimes','intensities','objectIds','ticks']}
        arrays.update({k:np.asarray(data[k]).copy() for k in ['frameId','timestampNs','transform','transformStart']})
        arrays['original_ray_indices']=np.flatnonzero(mask);arrays['original_num_beams']=np.asarray(len(distance))
        np.savez_compressed(self.audit_root/f'frame_{index}_{fid}.npz',**arrays)
