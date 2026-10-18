"""Conservative collision-mesh bounds clipped to the existing robot height band."""
import os,sys,argparse,json,hashlib
from pathlib import Path
parser=argparse.ArgumentParser();parser.add_argument('--boxes',type=Path,required=True);parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
if not os.environ.get('GO2_USD_CPU_READY'):
 p=next((Path(os.environ['GO2_ISAAC_ROOT'])/'extscache').glob('omni.usd.libs-*'));e=os.environ.copy();e['PYTHONPATH']=str(p)+':'+e.get('PYTHONPATH','');e['LD_LIBRARY_PATH']=str(p/'bin')+':'+e.get('LD_LIBRARY_PATH','');e['GO2_USD_CPU_READY']='1';os.execvpe(sys.executable,[sys.executable,*sys.argv],e)
import numpy as np
from pxr import Usd,UsdGeom,UsdPhysics,Gf
asset=Path(os.environ['GO2_ASSET_ROOT'])/'Isaac/Environments/Simple_Warehouse/warehouse_multiple_shelves.usd';stage=Usd.Stage.CreateInMemory();UsdGeom.SetStageMetersPerUnit(stage,1.);UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z);root=stage.DefinePrim('/World/Warehouse','Xform');root.GetReferences().AddReference(str(asset));cache=UsdGeom.XformCache()
zlo,zhi=.05,.85

def clip(poly,height,above):
 result=[]
 for a,b in zip(poly,poly[1:]+poly[:1]):
  ina=(a[2]>=height) if above else (a[2]<=height);inb=(b[2]>=height) if above else (b[2]<=height)
  if ina:result.append(a)
  if ina!=inb:result.append(a+(height-a[2])/(b[2]-a[2])*(b-a))
 return result

original=json.loads(args.boxes.read_text());refined=[];changes=[];checked_samples=0
for box in original:
 p=stage.GetPrimAtPath(box['path']);approx=UsdPhysics.MeshCollisionAPI(p).GetApproximationAttr().Get() if p.IsValid() else None
 # Non-mesh or non-triangle collision approximations retain their old safe bounds.
 if not p.IsValid() or not p.IsA(UsdGeom.Mesh) or approx not in (None,'none'):
  refined.append(box);continue
 mesh=UsdGeom.Mesh(p);matrix=cache.GetLocalToWorldTransform(p);v=np.array([matrix.Transform(Gf.Vec3d(*map(float,x))) for x in mesh.GetPointsAttr().Get()]);ids=list(mesh.GetFaceVertexIndicesAttr().Get());counts=list(mesh.GetFaceVertexCountsAttr().Get());clipped=[];samples=[];offset=0
 for count in counts:
  poly=[v[i] for i in ids[offset:offset+count]];offset+=count
  q=clip(clip(poly,zlo,True),zhi,False)
  clipped.extend(q)
  # Independent dense edge sampling checks that in-band geometry is retained.
  for a,b in zip(poly,poly[1:]+poly[:1]):
   s=np.linspace(a,b,17);samples.extend(s[(s[:,2]>=zlo)&(s[:,2]<=zhi)])
 if not clipped:
  if samples:raise RuntimeError('Clipping dropped sampled geometry: '+box['path'])
  changes.append({'path':box['path'],'old':box,'new':None});continue
 points=np.asarray(clipped);lo=points.min(axis=0)-1e-5;hi=points.max(axis=0)+1e-5
 if samples:
  s=np.asarray(samples);assert np.all(s>=lo-1e-7) and np.all(s<=hi+1e-7),box['path'];checked_samples+=len(s)
 if not np.all(lo>=np.asarray(box['min'])-1e-3) or not np.all(hi<=np.asarray(box['max'])+1e-3):raise RuntimeError('World bounds mismatch '+box['path'])
 item=dict(box,min=lo.tolist(),max=hi.tolist(),height_clipped=True);refined.append(item)
 if np.max(abs(lo[:2]-np.asarray(box['min'])[:2]))>.001 or np.max(abs(hi[:2]-np.asarray(box['max'])[:2]))>.001:changes.append({'path':box['path'],'old':box,'new':item})
args.output.mkdir(parents=True,exist_ok=False);(args.output/'static_boxes.json').write_text(json.dumps(refined,indent=2)+'\n');(args.output/'changes.json').write_text(json.dumps(changes,indent=2)+'\n');report={'source_sha256':hashlib.sha256(args.boxes.read_bytes()).hexdigest(),'asset_sha256':hashlib.sha256(asset.read_bytes()).hexdigest(),'height_band':[zlo,zhi],'original_boxes':len(original),'refined_boxes':len(refined),'changed_xy_or_removed':len(changes),'independent_edge_samples_retained':checked_samples,'non_mesh_or_convex_collision_fallback':'unchanged original AABB','safety_threshold_changed':False};(args.output/'report.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
