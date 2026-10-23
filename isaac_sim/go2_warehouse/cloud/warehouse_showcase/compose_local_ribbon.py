"""Image-only filled ribbon using captured camera matrices and actual local plan."""
import math
import numpy as np
from PIL import Image, ImageDraw, ImageChops

def compose(image,snapshot):
    local=snapshot.get('local')
    if not local:return image
    body=np.array([v for v in local.get('base_points',[]) if v[0]>=0.],float)
    if len(body)<2:return image
    # Preserve the actual curve shape, but move the display cue .65 m ahead
    # so the ground-painted glyph is not hidden by the robot. This is a
    # presentation offset, not a navigation waypoint or an extended rollout.
    body[:,0]+=.65
    robot=snapshot['robot_world'];w,x,y,z=robot[3:7];yaw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z));co,si=math.cos(yaw),math.sin(yaw)
    points=np.column_stack([robot[0]+co*body[:,0]-si*body[:,1],robot[1]+si*body[:,0]+co*body[:,1]])
    dist=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(points,axis=0),axis=1))];keep=np.r_[True,np.diff(dist)>1e-5];dist,points=dist[keep],points[keep]
    if len(points)<2 or dist[-1]<.08:return image
    q=np.linspace(0,dist[-1],max(3,int(dist[-1]/.03)+1));xy=np.column_stack([np.interp(q,dist,points[:,i]) for i in range(2)])
    tangent=np.gradient(xy,axis=0);tangent/=np.maximum(np.linalg.norm(tangent,axis=1)[:,None],1e-8);normal=np.column_stack([-tangent[:,1],tangent[:,0]])
    view=np.array(snapshot['view']);proj=np.array(snapshot['projection']);width,height=image.size
    def project(points):
        points=np.asarray(points);p=np.column_stack([points,np.ones(len(points))]);clip=p@view@proj
        if np.any(clip[:,3]<=1e-5):return
        ndc=clip[:,:2]/clip[:,3,None];pixels=np.column_stack([(ndc[:,0]+1)*width/2,(1-ndc[:,1])*height/2])
        if not np.isfinite(pixels).all() or np.abs(pixels).max()>100000:return
        return [tuple(v) for v in pixels]
    # One closed outline: shaft and head share the exact base station on the
    # recorded curve, including during turns. Never extrapolate the local plan.
    base_s=dist[-1]-min(.30,dist[-1]*.30)
    shaft_q=np.r_[q[q<base_s],base_s]
    shaft=np.column_stack([np.interp(shaft_q,dist,points[:,i]) for i in range(2)])
    direction=np.gradient(shaft,axis=0)
    direction/=np.maximum(np.linalg.norm(direction,axis=1)[:,None],1e-8)
    sides=np.column_stack([-direction[:,1],direction[:,0]])
    outline=np.vstack([shaft+.15*sides,shaft[-1]+.32*sides[-1],xy[-1],shaft[-1]-.32*sides[-1],(shaft-.15*sides)[::-1]])
    # Warehouse floor is world z=0. A 3 mm paint offset, not body-height geometry.
    pixels=project(np.column_stack([outline,np.full(len(outline),.003)]))
    if pixels is None:return image
    mask=Image.new('L',image.size);ImageDraw.Draw(mask).polygon(pixels,fill=190)
    # Image-only capture has no scene depth. Conservatively protect the whole
    # projected robot envelope; the original dog pixels always remain visible.
    corners=[]
    for bx in [-.60,.42]:
        for by in [-.28,.28]:
            for bz in [0.,robot[2]+.06]:
                corners.append([robot[0]+co*bx-si*by,robot[1]+si*bx+co*by,bz])
    projected=project(corners)
    if projected:
        def cross(o,a,b):return (a[0]-o[0])*(b[1]-o[1])-(a[1]-o[1])*(b[0]-o[0])
        ordered=sorted(set(projected));lower=[];upper=[]
        for point in ordered:
            while len(lower)>=2 and cross(lower[-2],lower[-1],point)<=0:lower.pop()
            lower.append(point)
        for point in reversed(ordered):
            while len(upper)>=2 and cross(upper[-2],upper[-1],point)<=0:upper.pop()
            upper.append(point)
        protected=Image.new('L',image.size);ImageDraw.Draw(protected).polygon(lower[:-1]+upper[:-1],fill=255)
        mask=ImageChops.subtract(mask,protected)
    return Image.composite(Image.new('RGB',image.size,(255,145,0)),image,mask)
