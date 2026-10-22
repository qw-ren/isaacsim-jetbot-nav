"""Controlled A/B/C geometry applied on the simulation thread, never teleporting Go2."""
import json
from pathlib import Path

import numpy as np
import omni.usd
from pxr import Gf, UsdGeom, UsdPhysics


class DynamicFixture:
    def __init__(self,env,output):
        self.env=env.unwrapped
        self.root=Path(output)
        self.config=json.loads((self.root/'dynamic_config.json').read_text())
        self.stage=omni.usd.get_context().get_stage()
        self.prefix='/World/Go2DynamicTrial'
        if self.stage.GetPrimAtPath(self.prefix).IsValid():raise RuntimeError('Duplicate dynamic fixture')
        UsdGeom.Xform.Define(self.stage,self.prefix)
        self.last_id=None
        for i,b in enumerate(self.config.get('prior',[])):self.create('prior_'+str(i),b,[.15,.35,.85])
        self.apply(dict(id='initial_open',enabled=False))

    def create(self,name,b,color):
        center,size=np.array(b['center'],float),np.array(b['size'],float)
        if center.shape!=(3,) or size.shape!=(3,) or not np.isfinite([center,size]).all():raise ValueError('Invalid box')
        if not ((size>0).all() and (size<=50).all() and abs(center[2]-size[2]/2)<1e-5):raise ValueError('Invalid box size or height')
        cube=UsdGeom.Cube.Define(self.stage,self.prefix+'/'+name)
        cube.CreateSizeAttr(1.)
        cube.AddTranslateOp().Set(Gf.Vec3d(*center))
        cube.AddScaleOp().Set(Gf.Vec3f(*size))
        cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim()).CreateCollisionEnabledAttr(True)

    def apply(self,request):
        if type(request.get('enabled')) is not bool:raise ValueError('Explicit boolean required')
        enabled=request['enabled'];actual=[];boxes=[]
        for i,b in enumerate(self.config['obstacles']):
            path=self.prefix+'/added_'+str(i)
            if self.stage.GetPrimAtPath(path).IsValid():
                if not self.stage.RemovePrim(path):raise RuntimeError('Cannot remove owned trial obstacle')
            if enabled:self.create('added_'+str(i),b,[1.,.15,.05])
            prim=self.stage.GetPrimAtPath(path)
            collision=bool(UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get()) if prim.IsValid() else False
            if prim.IsValid()!=enabled or collision!=enabled:raise RuntimeError('Physical obstacle application failed')
            actual.append(dict(b,prim=path,present=prim.IsValid(),collision_enabled=collision))
            if enabled:
                c,s=np.array(b['center']),np.array(b['size'])
                boxes.append(dict(min=(c-s/2).tolist(),max=(c+s/2).tolist(),source='actual inserted '+path))
        row=dict(request,kind=self.config['kind'],simulation_stamp_s=float(self.env.sim.current_time),
                 dynamic_boxes=boxes,actual_usd=actual,scope='controlled simulation fixture; no robot pose modification')
        p=self.root/'obstacle_state.json';temporary=p.with_suffix('.tmp')
        temporary.write_text(json.dumps(row,indent=2)+'\n');temporary.replace(p)
        with (self.root/'obstacle_events.jsonl').open('a') as f:f.write(json.dumps(row)+'\n')
        self.last_id=request['id']

    def tick(self):
        p=self.root/'obstacle_request.json'
        if not p.exists():return
        d=json.loads(p.read_text())
        if d['id']!=self.last_id:self.apply(d)
