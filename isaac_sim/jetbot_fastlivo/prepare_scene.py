"""Resolve a copy of teammate PR4; keep source scene/Go2 runtimes untouched."""
import argparse,hashlib,json,re,shutil,math
from pathlib import Path
from pxr import Sdf,Usd,UsdGeom,UsdPhysics,Gf
p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True);p.add_argument('--case',type=Path,required=True);p.add_argument('--assets',type=Path,required=True);p.add_argument('--robot-scale',type=float,default=1.);a=p.parse_args();assert 1.<=a.robot_scale<=4.;a.case.mkdir(exist_ok=False)
root=Sdf.Layer.FindOrOpen(str(a.source));text=root.ExportToString();assets={'jetbot.usd':a.assets/'Robots/Jetbot/jetbot.usd','XT-32.usd':a.assets/'Sensors/HESAI/XT-32.usd','full_warehouse.usd':a.assets/'Environments/Simple_Warehouse/full_warehouse.usd'}
for ref in re.findall(r'@([^@]+)@',text):
 resolved=assets[Path(ref).name]
 if not resolved.is_file():raise RuntimeError('Missing existing asset '+str(resolved))
 text=text.replace('@'+ref+'@','@'+str(resolved)+'@')
scene=a.case/'jetbot_resolved.usda';scene.write_text(text);stage=Usd.Stage.Open(str(scene))
# A single runtime owns sensors/control/clock/TF. Disable native truth TF and
# open-loop command retention together, preserving original file as provenance.
for path in ['/World/ROS2','/Graphs']:
 prim=stage.GetPrimAtPath(path)
 if prim:prim.SetActive(False)
scale=a.robot_scale;robot=stage.GetPrimAtPath('/World/jetbot');mass_changes=[]
if scale!=1.:
 xform=UsdGeom.Xformable(robot);ops=xform.GetOrderedXformOps();translation_op=next(o for o in ops if o.GetOpType()==UsdGeom.XformOp.TypeTranslate);translation=translation_op.Get();scale_op=next((o for o in ops if o.GetOpType()==UsdGeom.XformOp.TypeScale),None)
 if scale_op is None:scale_op=xform.AddScaleOp()
 else:assert all(abs(v-1)<1e-5 for v in scale_op.Get())
 scale_op.Set(Gf.Vec3f(scale));translation_op.Set(Gf.Vec3d(translation[0],translation[1],translation[2]*scale))
 assert all(abs(v-scale)<1e-6 for v in scale_op.Get()), 'Scale was not authored'
 for prim in Usd.PrimRange(robot):
  if prim.HasAPI(UsdPhysics.MassAPI):
   api=UsdPhysics.MassAPI(prim);mass=api.GetMassAttr().Get()
   if mass and mass>0:api.GetMassAttr().Set(float(mass)*scale**3);mass_changes.append(dict(path=str(prim.GetPath()),before_kg=float(mass),after_kg=float(mass)*scale**3))
   inertia=api.GetDiagonalInertiaAttr().Get()
   if inertia and all(v>0 for v in inertia):api.GetDiagonalInertiaAttr().Set(Gf.Vec3f(*(v*scale**5 for v in inertia)))
cache=UsdGeom.BBoxCache(Usd.TimeCode.Default(),[UsdGeom.Tokens.default_,UsdGeom.Tokens.render]);box=cache.ComputeWorldBound(robot).ComputeAlignedRange();lo,hi=box.GetMin(),box.GetMax();center=UsdGeom.XformCache().GetLocalToWorldTransform(robot).ExtractTranslation()
wheel_bounds=cache.ComputeWorldBound(stage.GetPrimAtPath('/World/jetbot/left_wheel')).ComputeAlignedRange();physical_radius=(wheel_bounds.GetMax()[2]-wheel_bounds.GetMin()[2])/2
xc=UsdGeom.XformCache();left=xc.GetLocalToWorldTransform(stage.GetPrimAtPath('/World/jetbot/left_wheel')).ExtractTranslation();right=xc.GetLocalToWorldTransform(stage.GetPrimAtPath('/World/jetbot/right_wheel')).ExtractTranslation();physical_base=(left-right).GetLength()
radius=math.ceil(max(math.hypot(x-center[0],y-center[1])for x in(lo[0],hi[0])for y in(lo[1],hi[1]))*100)/100
(a.case/'ROBOT_VARIANT.json').write_text(json.dumps(dict(scale=scale,size_m=list(hi-lo),footprint_bound_radius_m=radius,physical_wheel_radius_m=physical_radius,physical_wheel_base_m=physical_base,mass_changes=mass_changes,source_scene_unchanged=True),indent=2)+'\n')
limits=[]
for name in ['left_wheel_joint','right_wheel_joint']:
 attr=stage.GetPrimAtPath('/World/jetbot/chassis/'+name).GetAttribute('physxJoint:maxJointVelocity');value=float(attr.Get());assert math.isfinite(value)and value>0
 limits.append(dict(joint=name,max_deg_s=value,max_rad_s=math.radians(value),sources=[dict(layer=s.layer.identifier,path=str(s.path),default=repr(s.default))for s in attr.GetPropertyStack(Usd.TimeCode.Default())]))
(a.case/'JOINT_LIMITS.json').write_text(json.dumps(dict(joints=limits,wheel_radius_m=(physical_radius if scale!=1 else .03),wheel_base_m=(physical_base if scale!=1 else .1125),straight_limit_m_s=min(v['max_rad_s']for v in limits)*(physical_radius if scale!=1 else .03),robot_scale=scale,joint_limits_modified=False),indent=2)+'\n')
stage.GetRootLayer().Save();shutil.copy2(a.source,a.case/'teammate_original.usd')
(a.case/'SCENE_PROVENANCE.json').write_text(json.dumps(dict(upstream_pr=4,upstream_commit='35fee2debcfb528e2aef86c91f82b06c2fd42869',upstream_sha256=hashlib.sha256(a.source.read_bytes()).hexdigest(),asset_paths={k:str(v)for k,v in assets.items()},disabled_graphs=['/World/ROS2','/Graphs'],reason='exclusive sensor timestamps, optical extrinsics, estimated odom/TF and command watchdog; source unchanged'),indent=2)+'\n')
print(scene)
