"""Static localization prior from visible CAD mesh surfaces in the scan band.
Does not edit USD or use simulator truth/sensor trajectories. Planning keeps
its original conservative AABB map. This geometry candidate needs validation.
"""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from PIL import Image, ImageDraw
import yaml
from pxr import Gf, Usd, UsdGeom


def clip_height(poly, height, above):
    out=[]
    for a,b in zip(poly,np.roll(poly,-1,axis=0)):
        ia=a[2]>=height if above else a[2]<=height
        ib=b[2]>=height if above else b[2]<=height
        if ia:out.append(a)
        if ia!=ib:
            fraction=(height-a[2])/(b[2]-a[2]);out.append(a+fraction*(b-a))
    return np.asarray(out)


def build(scene, reference, output, resolution):
    assert .01<=resolution<=.05
    stage=Usd.Stage.Open(str(scene));warehouse=stage.GetPrimAtPath('/World/full_warehouse');assert warehouse
    xc=UsdGeom.XformCache();cache=UsdGeom.BBoxCache(Usd.TimeCode.Default(),[UsdGeom.Tokens.default_,UsdGeom.Tokens.render])
    chassis=stage.GetPrimAtPath('/World/jetbot/chassis');wheel=stage.GetPrimAtPath('/World/jetbot/left_wheel');assert chassis and wheel
    # CAD wheel support on the declared z=0 warehouse floor, never truth z.
    nominal_z=float(xc.GetLocalToWorldTransform(chassis).ExtractTranslation()[2]-cache.ComputeWorldBound(wheel).ComputeAlignedRange().GetMin()[2])
    low,high=nominal_z+.03,nominal_z+.40
    proof=json.loads((reference/'MAP_PROVENANCE.json').read_text());lo,hi=np.array(proof['bounds'])
    origin=np.floor((lo[:2]-.5)/resolution)*resolution;size=np.ceil((hi[:2]+.5-origin)/resolution).astype(int)
    assert size.prod()<8_000_000
    x=origin[0]+(np.arange(size[0])+.5)*resolution;y=origin[1]+(np.arange(size[1])+.5)*resolution
    pixels=np.full((size[1],size[0]),128,dtype=np.uint8)
    pixels[(x[None,:]>=lo[0])&(x[None,:]<=hi[0])&(y[:,None]>=lo[1])&(y[:,None]<=hi[1])]=254
    # Raster in increasing-world-y order, flip only when saving the PGM.
    image=Image.fromarray(pixels);draw=ImageDraw.Draw(image)
    meshes=0;polygons=0;triangles=0;asset_layers=[]
    for prim in Usd.PrimRange(warehouse,Usd.TraverseInstanceProxies()):
        if not prim.IsA(UsdGeom.Mesh):continue
        mesh=UsdGeom.Mesh(prim)
        if str(mesh.ComputeVisibility())=='invisible' or str(mesh.ComputePurpose()) not in ('default','render'):continue
        bound=cache.ComputeWorldBound(prim).ComputeAlignedRange()
        if bound.GetMax()[2]<low or bound.GetMin()[2]>high:continue
        points=np.array(mesh.GetPointsAttr().Get(),dtype=float)
        if not len(points):continue
        matrix=np.array(xc.GetLocalToWorldTransform(prim));homogeneous=np.c_[points,np.ones(len(points))]@matrix
        world=homogeneous[:,:3]/homogeneous[:,3,None]
        assert np.allclose(world[0],xc.GetLocalToWorldTransform(prim).Transform(Gf.Vec3d(*points[0])))
        counts=np.array(mesh.GetFaceVertexCountsAttr().Get(),dtype=int);indices=np.array(mesh.GetFaceVertexIndicesAttr().Get(),dtype=int)
        assert np.all((counts>=3)&(counts<=4)), 'Unsupported n-gon; do not guess triangulation'
        if np.all(counts==3):faces=indices.reshape(-1,3)
        else:
            faces=[];offset=0
            for count in counts:
                ids=indices[offset:offset+count];faces.extend([[ids[0],ids[i],ids[i+1]] for i in range(1,count-1)]);offset+=count
            faces=np.array(faces)
        tri=world[faces];selected=(tri[:,:,2].max(axis=1)>=low)&(tri[:,:,2].min(axis=1)<=high)
        for triangle in tri[selected]:
            polygon=clip_height(triangle,low,True)
            if len(polygon)<2:continue
            polygon=clip_height(polygon,high,False)
            if len(polygon)<2:continue
            coords=np.floor((polygon[:,:2]-origin)/resolution).astype(int)
            vertices=[tuple(map(int,v)) for v in coords]
            if len(vertices)>=3:draw.polygon(vertices,fill=0)
            else:draw.line(vertices,fill=0,width=1)
            polygons+=1
        meshes+=1;triangles+=int(selected.sum())
    output.mkdir(exist_ok=False,parents=True)
    final=np.array(image);Image.fromarray(final[::-1]).save(output/'map.pgm')
    meta=dict(image='map.pgm',mode='trinary',resolution=resolution,origin=[*map(float,origin),0.],negate=0,occupied_thresh=.65,free_thresh=.25)
    (output/'map.yaml').write_text(yaml.safe_dump(meta,sort_keys=False))
    report=dict(type='visible CAD mesh surface projection for localization only',source_scene_sha256=hashlib.sha256(scene.read_bytes()).hexdigest(),
        reference_map_sha256=hashlib.sha256((reference/'map.pgm').read_bytes()).hexdigest(),
        source_geometry_unchanged=True,truth_or_trajectory_input=False,ground_z_m=0.,nominal_chassis_z_from_cad_m=nominal_z,
        height_band_world_m=[low,high],height_band_chassis_m=[.03,.40],resolution=resolution,grid_size=size.tolist(),
        visible_meshes=meshes,band_triangles=triangles,rasterized_polygons=polygons,occupied_cells=int((final==0).sum()),
        preflight_passed=False,limitations=['Localization-only candidate, not a replacement for conservative planning geometry.',
        'Nominal chassis height from CAD wheel support, not dynamic tilt.',
        'Surface projection/rasterization is approximate; mesh/material sensor visibility may differ.',
        'No navigation or precision validation yet.'])
    (output/'MAP_PROVENANCE.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--scene',type=Path,required=True);p.add_argument('--reference-prior',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--resolution',type=float,default=.02);a=p.parse_args();build(a.scene,a.reference_prior,a.output,a.resolution)
