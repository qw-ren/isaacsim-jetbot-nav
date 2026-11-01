"""Conservative collision-AABB development prior for the teammate warehouse, not SLAM."""
import argparse,hashlib,json,math
from pathlib import Path
import numpy as np
from pxr import Usd,UsdGeom,UsdPhysics
p=argparse.ArgumentParser();p.add_argument('--scene',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(exist_ok=False)
stage=Usd.Stage.Open(str(a.scene));cache=UsdGeom.BBoxCache(Usd.TimeCode.Default(),[UsdGeom.Tokens.default_,UsdGeom.Tokens.render,UsdGeom.Tokens.proxy]);warehouse=stage.GetPrimAtPath('/World/full_warehouse');assert warehouse
bounds=cache.ComputeWorldBound(warehouse).ComputeAlignedRange();lo=np.asarray(bounds.GetMin());hi=np.asarray(bounds.GetMax());boxes=[];total=0
for prim in stage.Traverse():
 if not str(prim.GetPath()).startswith('/World/full_warehouse/')or not prim.HasAPI(UsdPhysics.CollisionAPI):continue
 total+=1
 if UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is False:continue
 b=cache.ComputeWorldBound(prim).ComputeAlignedRange();lower=np.array(b.GetMin());upper=np.array(b.GetMax())
 if not np.isfinite(np.r_[lower,upper]).all()or upper[2]<=.035 or lower[2]>=.30:continue
 boxes.append(dict(path=str(prim.GetPath()),min=lower.tolist(),max=upper.tolist()))
resolution=.05;origin=np.floor((lo[:2]-.5)/resolution)*resolution;size=np.ceil((hi[:2]+.5-origin)/resolution).astype(int);assert size.prod()<8_000_000
x=origin[0]+(np.arange(size[0])+.5)*resolution;y=origin[1]+(np.arange(size[1])+.5)*resolution;pixels=np.full((size[1],size[0]),128,np.uint8);inside=(x[None,:]>=lo[0])&(x[None,:]<=hi[0])&(y[:,None]>=lo[1])&(y[:,None]<=hi[1]);pixels[inside]=254
for b in boxes:
 lower,upper=np.array(b['min']),np.array(b['max']);begin=np.maximum(0,np.floor((lower[:2]-origin)/resolution).astype(int));end=np.minimum(size-1,np.floor((upper[:2]-origin)/resolution).astype(int))
 if np.all(end>=begin):pixels[begin[1]:end[1]+1,begin[0]:end[0]+1]=0
(a.output/'map.pgm').write_bytes(f'P5\n{size[0]} {size[1]}\n255\n'.encode()+pixels[::-1].tobytes())
(a.output/'map.yaml').write_text(f'image: map.pgm\nmode: trinary\nresolution: {resolution}\norigin: [{origin[0]}, {origin[1]}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
def clearance(point):
 if not boxes:return None
 b=np.array([[v['min'][:2],v['max'][:2]]for v in boxes]);d=np.maximum(np.maximum(b[:,0]-point,point-b[:,1]),0);return float(np.linalg.norm(d,axis=1).min())
line=[np.array([t,6.])for t in np.linspace(0,1.5,61)];minimum=min(clearance(v)for v in line)
report=dict(type='USD collision AABB prior, not SLAM',source_scene_sha256=hashlib.sha256(a.scene.read_bytes()).hexdigest(),warehouse_collision_prims=total,selected_collision_boxes=len(boxes),height_band_m=[.035,.30],bounds=[lo.tolist(),hi.tolist()],resolution=resolution,grid_size=size.tolist(),known_spawn=[0.,6.,0.],proposed_goal=[1.5,6.,0.],straight_preflight_clearance_m=minimum,preflight_passed=minimum>.24,limitations=['rectangular interior assumed free unless selected collision AABB','conservative projection, not a swept-body certification','not evidence of localization or actual motion'])
(a.output/'MAP_PROVENANCE.json').write_text(json.dumps(report,indent=2)+'\n');(a.output/'collision_boxes.json').write_text(json.dumps(boxes,indent=2)+'\n');print(json.dumps(report,indent=2))
