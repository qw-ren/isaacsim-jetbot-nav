"""Read-only USD bounding-box exclusion test for suspicious world returns."""
import os,sys,json,argparse
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--points',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
if not os.environ.get('GO2_USD_CPU_READY'):
 lib=next((Path(os.environ['GO2_ISAAC_ROOT'])/'extscache').glob('omni.usd.libs-*'));e=os.environ.copy();e['PYTHONPATH']=f"{lib}:{e.get('PYTHONPATH','')}";e['LD_LIBRARY_PATH']=f"{lib/'bin'}:{e.get('LD_LIBRARY_PATH','')}";e['GO2_USD_CPU_READY']='1';os.execvpe(sys.executable,[sys.executable,*sys.argv],e)
import numpy as np
from pxr import Usd,UsdGeom
s=Usd.Stage.CreateInMemory();root=s.DefinePrim('/World/Warehouse','Xform');root.GetReferences().AddReference(str(Path(os.environ['GO2_ASSET_ROOT'])/'Isaac/Environments/Simple_Warehouse/warehouse_multiple_shelves.usd'));cache=UsdGeom.BBoxCache(0,[UsdGeom.Tokens.default_,UsdGeom.Tokens.render]);boxes=[]
for prim in s.Traverse():
 if prim.IsA(UsdGeom.Mesh):
  b=cache.ComputeWorldBound(prim).ComputeAlignedRange();lo,hi=np.array(b.GetMin()),np.array(b.GetMax())
  if np.isfinite(np.r_[lo,hi]).all():boxes.append((str(prim.GetPath()),lo,hi))
# Existing simulation-only ground plane is additional to warehouse assets.
boxes.append(('/World/ground/Environment/Geometry',np.array([-150,-150,.0001]),np.array([150,150,.0001])))
results=[]
for entry in json.loads(a.points.read_text()):
 q=np.array(entry['point']);nearest=sorted((float(np.linalg.norm(np.maximum(np.maximum(lo-q,q-hi),0))),name) for name,lo,hi in boxes)[:4]
 results.append({**entry,'nearest_mesh_aabbs':[{'distance_m':d,'path':n} for d,n in nearest],'within_any_mesh_bbox_005m':nearest[0][0]<=.05})
a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps({'mesh_count':len(boxes),'results':results,'limitations':'AABB inclusion is not a mesh-surface hit proof; exclusion applies to this static warehouse and ground, not moving robot or other dynamic assets.'},indent=2)+'\n');print(a.output)
