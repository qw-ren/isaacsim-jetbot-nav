"""Owned warehouse props with real conservative collision envelopes and visible parts."""
import json
from pathlib import Path
import numpy as np
import omni.usd
from pxr import Gf,UsdGeom,UsdPhysics

class DynamicFixture:
    def __init__(self,env,output):
        self.env=env.unwrapped;self.root=Path(output);self.config=json.loads((self.root/'dynamic_config.json').read_text())
        self.stage=omni.usd.get_context().get_stage();self.prefix='/World/Go2DynamicTrial';self.last_id=None
        if self.stage.GetPrimAtPath(self.prefix).IsValid():raise RuntimeError('Duplicate fixture')
        UsdGeom.Xform.Define(self.stage,self.prefix)
        for i,b in enumerate(self.config.get('prior',[])):self.create('prior_'+str(i),dict(b,appearance='rack'))
        self.apply(dict(id='initial_open',enabled=False,active=[]))
    def box(self,path,center,size,color):
        cube=UsdGeom.Cube.Define(self.stage,path);cube.CreateSizeAttr(1.);cube.AddTranslateOp().Set(Gf.Vec3d(*center));cube.AddScaleOp().Set(Gf.Vec3f(*size));cube.CreateDisplayColorAttr([Gf.Vec3f(*color)]);return cube
    def create(self,name,b):
        c,s=np.array(b['center'],float),np.array(b['size'],float)
        if c.shape!=(3,) or s.shape!=(3,) or not np.isfinite([c,s]).all() or np.any(s<=0) or np.any(s>50) or abs(c[2]-s[2]/2)>1e-5:raise ValueError('Bad envelope')
        path=self.prefix+'/'+name;UsdGeom.Xform.Define(self.stage,path)
        collider=self.box(path+'/collision',c,s,[.3,.3,.3]);UsdGeom.Imageable(collider).MakeInvisible();UsdPhysics.CollisionAPI.Apply(collider.GetPrim()).CreateCollisionEnabledAttr(True)
        lo=c-s/2;kind=b.get('appearance','pallet');i=0
        def part(offset,size,color):
            nonlocal i
            self.box(path+'/part_'+str(i),lo+np.array(offset)*s,np.array(size)*s,color);i+=1
        wood=[.42,.25,.10];card=[.65,.45,.23];tape=[.88,.79,.55];metal=[.12,.25,.32]
        if kind=='pallet':
            for x in [.13,.5,.87]:part([x,.5,.06],[.16,.92,.10],wood)
            for y in [.1,.3,.5,.7,.9]:part([.5,y,.15],[.95,.13,.07],wood)
            for x in [.27,.73]:
                for y in [.27,.73]:
                    part([x,y,.48],[.42,.42,.56],card);part([x,y,.765],[.07,.42,.012],tape)
            part([.5,.5,.88],[.8,.7,.23],card);part([.5,.5,.997],[.07,.7,.006],tape)
        elif kind in ['cart','rack']:
            for x in [.10,.90]:
                for y in [.08,.92]:
                    part([x,y,.55],[.06,.045,.90],metal)
                    wheel=UsdGeom.Cylinder.Define(self.stage,path+'/wheel_'+str(i));i+=1
                    wheel.CreateRadiusAttr(float(min(.09,s[2]*.08)));wheel.CreateHeightAttr(.055);wheel.CreateAxisAttr('X');wheel.AddTranslateOp().Set(Gf.Vec3d(*(lo+np.array([x,y,.08])*s)));wheel.CreateDisplayColorAttr([Gf.Vec3f(.04,.04,.04)])
            levels=[.22,.58,.96] if kind=='rack' else [.22,.96]
            for z in levels:part([.5,.5,z],[.96,.96,.035],metal)
            for y in [.22,.5,.78]:
                part([.5,y,.42],[.73,.23,.34],card);part([.5,y,.595],[.08,.23,.008],tape)
            if kind=='rack':
                for y in [.22,.5,.78]:part([.5,y,.76],[.73,.23,.31],[.54,.37,.18])
            else:
                for x in [.1,.9]:part([x,.97,.75],[.045,.035,.45],metal)
        else:raise ValueError('Unknown warehouse appearance')
    def apply(self,request):
        if type(request.get('enabled')) is not bool:raise ValueError('Explicit enabled required')
        active=request.get('active',list(range(len(self.config['obstacles']))) if request['enabled'] else [])
        if len(active)!=len(set(active)) or any(type(i)is not int or i<0 or i>=len(self.config['obstacles']) for i in active):raise ValueError('Invalid active props')
        if bool(active)!=request['enabled']:raise ValueError('Inconsistent active props')
        # Demo obstacles persist after insertion: no vanishing blockers behind the camera.
        if not set(getattr(self,'active',[])).issubset(active):raise ValueError('Showcase may only add obstacles')
        actual=[];boxes=[]
        for i,b in enumerate(self.config['obstacles']):
            path=self.prefix+'/added_'+str(i);present=self.stage.GetPrimAtPath(path).IsValid()
            if i in active and not present:self.create('added_'+str(i),b)
            prim=self.stage.GetPrimAtPath(path+'/collision');collision=bool(UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get()) if prim.IsValid() else False
            if collision!=(i in active):raise RuntimeError('Collision envelope missing')
            actual.append(dict(b,prim=path,present=i in active,collision_enabled=collision))
            if i in active:
                c,s=np.array(b['center']),np.array(b['size']);boxes.append(dict(min=(c-s/2).tolist(),max=(c+s/2).tolist(),source='actual warehouse prop envelope '+path))
        self.active=list(active)
        row=dict(request,kind=self.config['kind'],simulation_stamp_s=float(self.env.sim.current_time),dynamic_boxes=boxes,
                 actual_usd=[v for v in actual if v['present']],all_prop_states=actual,collision_scope='invisible conservative envelope; visible pallet/cart/rack parts generate sensor returns',scope='simulation-thread insertion; no robot teleport')
        p=self.root/'obstacle_state.json';tmp=p.with_suffix('.tmp');tmp.write_text(json.dumps(row,indent=2)+'\n');tmp.replace(p)
        with (self.root/'obstacle_events.jsonl').open('a') as f:f.write(json.dumps(row)+'\n')
        self.last_id=request['id']
    def tick(self):
        p=self.root/'obstacle_request.json'
        if p.exists():
            d=json.loads(p.read_text())
            if d['id']!=self.last_id:self.apply(d)
