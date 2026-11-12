"""Isaac sim-thread USD adapter. No robot pose, localization, TF or velocity IO."""
import json
import hashlib
from pathlib import Path
from .core import VERSION
from .config import load, write_json


class GroundTruthObjectPublisher:
    def __init__(self, case, node):
        from pxr import Usd, UsdGeom
        import omni.usd
        from std_msgs.msg import String
        from asset_fixture import place_asset
        self.case=Path(case);self.node=node;_,self.cfg=load(case)
        if self.cfg['mode']!='ground_truth_oracle':raise ValueError('Oracle requires explicit oracle mode')
        self.Usd,self.UsdGeom=Usd,UsdGeom;self.String=String
        self.stage=omni.usd.get_context().get_stage()
        registration=self.cfg['world_to_map']
        if hashlib.sha256((self.case/'jetbot_resolved.usda').read_bytes()).hexdigest()!=registration['source_scene_sha256']:
            raise ValueError('Scene differs from registered map frame')
        if hashlib.sha256((self.case/'map/map.pgm').read_bytes()).hexdigest()!=registration['planning_map_sha256']:
            raise ValueError('Planning prior differs from registration')
        if abs(UsdGeom.GetStageMetersPerUnit(self.stage)-1.)>1e-8 or UsdGeom.GetStageUpAxis(self.stage)!='Z':
            raise ValueError('Expected meter/Z-up stage')
        self.pub=node.create_publisher(String,self.cfg['topics']['observations'],10)
        self.next=0.;self.seq=0;self.registry=[]
        for row in self.cfg['fixtures']:
            path='/World/SemanticOracle/'+row['id']
            if self.stage.GetPrimAtPath(path):
                raise ValueError('Fixture path already exists: '+path)
            part=place_asset(self.stage,path,row['asset_kind'],row['center'],row.get('yaw_rad',0.))
            self.registry.append(dict(object_id=path,class_name=row['class_name'],prim_path=path,asset=part))
        for row in self.cfg.get('registered_prims',[]):
            if not self.stage.GetPrimAtPath(row['prim_path']):raise ValueError('Missing registered Prim')
            self.registry.append(dict(object_id=row['prim_path'],prim_path=row['prim_path'],class_name=row['class_name']))
        for row in self.cfg.get('registered_groups',[]):
            if not row['prim_paths'] or not all(self.stage.GetPrimAtPath(p)for p in row['prim_paths']):
                raise ValueError('Missing registered group Prim')
            self.registry.append(dict(object_id=row['object_id'],prim_paths=row['prim_paths'],
                class_name=row['class_name'],row_index=row['row_index'],label=row['label']))
        self.log=(self.case/'semantic_oracle.jsonl').open('x')
        write_json(self.case/'SEMANTIC_ORACLE_PROVENANCE.json',dict(
            mode='ground_truth_oracle',source='ground_truth',all_registered_objects=True,
            visibility_filter=False,robot_truth_pose_read=False,localization_input=False,
            static_map_modified=False,registry=self.registry,registration=self.cfg['world_to_map']))

    def tick(self, stamp):
        if stamp<self.next:return
        self.next=stamp+1/self.cfg['observation_hz']
        cache=self.UsdGeom.BBoxCache(self.Usd.TimeCode.Default(),
            [self.UsdGeom.Tokens.default_,self.UsdGeom.Tokens.render,self.UsdGeom.Tokens.proxy])
        xc=self.UsdGeom.XformCache();rows=[]
        for registered in self.registry:
            prims=[self.stage.GetPrimAtPath(p)for p in registered.get('prim_paths',[registered.get('prim_path')])]
            if any(not p or not p.IsActive()for p in prims):continue
            prim=prims[0]
            # Publish conservative WORLD-axis-aligned boxes (identity orientation),
            # obtained live from the full transform hierarchy, not fixture input coordinates.
            bounds=[cache.ComputeWorldBound(p).ComputeAlignedRange()for p in prims]
            lo=[min(float(b.GetMin()[i])for b in bounds)for i in range(3)]
            hi=[max(float(b.GetMax()[i])for b in bounds)for i in range(3)]
            row=dict(object_id=registered['object_id'],class_name=registered['class_name'],
                confidence=1.,position=[(a+b)/2 for a,b in zip(lo,hi)],
                orientation_xyzw=[0.,0.,0.,1.],dimensions=[b-a for a,b in zip(lo,hi)],
                source='ground_truth',geometry='world_axis_aligned_bbox')
            # Actual Prim orientation is audit metadata; collision box remains AABB.
            q=xc.GetLocalToWorldTransform(prim).ExtractRotationQuat()
            row['prim_orientation_wxyz']=[float(q.GetReal()),*map(float,q.GetImaginary())]
            for key in('row_index','label'):
                if key in registered:row[key]=registered[key]
            rows.append(row)
        packet=dict(version=VERSION,frame_id=self.cfg['world_frame'],stamp=float(stamp),
                    sequence=self.seq,source='ground_truth',objects=rows)
        self.seq+=1;self.pub.publish(self.String(data=json.dumps(packet,allow_nan=False)))
        self.log.write(json.dumps(packet,allow_nan=False)+'\n');self.log.flush()

    def close(self):
        self.log.close()
