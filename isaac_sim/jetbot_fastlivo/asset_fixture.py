"""Reuse pinned local USD props, including their authored collision geometry."""
import hashlib,math
from pathlib import Path
from pxr import Usd,UsdGeom,UsdPhysics,Gf
ASSET_ROOT=Path('/home/admin/isaacsim_assets/Assets/Isaac/4.5/Isaac')
ASSETS={'box':'Environments/Simple_Warehouse/Props/SM_CardBoxA_01_467.usd','pallet':'Props/Pallet/pallet.usd','pushcart':'Environments/Simple_Warehouse/Props/SM_PushcartA_02.usd'}
def place_asset(stage,path,kind,center,yaw=0.):
 asset=ASSET_ROOT/ASSETS[kind];assert asset.is_file();wrapper=UsdGeom.Xform.Define(stage,path);content=stage.DefinePrim(path+'/content','Xform');content.GetReferences().AddReference(str(asset))
 cache=UsdGeom.BBoxCache(Usd.TimeCode.Default(),[UsdGeom.Tokens.default_,UsdGeom.Tokens.render,UsdGeom.Tokens.proxy]);b=cache.ComputeWorldBound(content).ComputeAlignedRange();lo=list(b.GetMin());hi=list(b.GetMax());cx,cy=(lo[0]+hi[0])/2,(lo[1]+hi[1])/2;c,s=math.cos(yaw),math.sin(yaw)
 wrapper.AddTranslateOp().Set(Gf.Vec3d(center[0]-c*cx+s*cy,center[1]-s*cx-c*cy,float(center[2]if len(center)>2 else 0.)-lo[2]));wrapper.AddRotateZOp().Set(math.degrees(yaw))
 colliders=[]
 for prim in Usd.PrimRange(content):
  if prim.HasAPI(UsdPhysics.RigidBodyAPI):UsdPhysics.RigidBodyAPI(prim).CreateRigidBodyEnabledAttr(False)
  if prim.HasAPI(UsdPhysics.CollisionAPI):
   UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr(True);colliders.append(str(prim.GetPath()))
 assert colliders,'Asset must have authored collision geometry'
 cache.Clear();b=cache.ComputeWorldBound(wrapper.GetPrim()).ComputeAlignedRange();lo=list(b.GetMin());hi=list(b.GetMax());assert all(math.isfinite(v)for v in lo+hi)
 return dict(path=path,asset=str(asset),asset_sha256=hashlib.sha256(asset.read_bytes()).hexdigest(),min=lo,max=hi,center=[(x+y)/2 for x,y in zip(lo,hi)],size=[y-x for x,y in zip(lo,hi)],yaw_rad=yaw,collision=True,authored_colliders=colliders,procedural_geometry=False)
def placements(event):
 if 'assets'in event:return event['assets']
 x,y=event['center']
 if event['kind']=='A':return [dict(asset='box',center=[x,y,0.])]
 if event['kind']in('B','C'):return [dict(asset='box',center=[x+(.7*(j-2)),y,0.])for j in range(5)]
 raise ValueError('Unknown obstacle class')
def place_event(stage,prefix,event):
 return [place_asset(stage,prefix+'/asset_'+str(i),p['asset'],p['center'],p.get('yaw_rad',0.))for i,p in enumerate(placements(event))]
