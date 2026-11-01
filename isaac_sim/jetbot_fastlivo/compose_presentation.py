"""Observer-image-only floor ribbon and goal beacon. Never imported by simulation.

The ribbon retains the native local curve, shifted ahead for presentation, as in
our Go2 demonstration. Reference poses only align this display cue with the car.
"""
import math
import numpy as np
from PIL import Image,ImageDraw,ImageChops

def camera_projector(eye,target,size):
 eye,target=np.asarray(eye,float),np.asarray(target,float);forward=target-eye;forward/=np.linalg.norm(forward);right=np.cross(forward,[0.,0.,1.]);right/=np.linalg.norm(right);up=np.cross(right,forward);width,height=size;fx,fy=width*24./36.,height*24./20.25
 def project(points):
  q=np.asarray(points,float)-eye;depth=q@forward
  if np.any(depth<=.03):return None
  pixels=np.column_stack([width/2+fx*(q@right)/depth,height/2-fy*(q@up)/depth])
  if not np.isfinite(pixels).all()or np.abs(pixels).max()>100000:return None
  return [tuple(v)for v in pixels]
 return project

def hull(points):
 points=sorted(set(points))
 if len(points)<3:return points
 def cross(a,b,c):return(b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
 lo=[];hi=[]
 for p in points:
  while len(lo)>=2 and cross(lo[-2],lo[-1],p)<=0:lo.pop()
  lo.append(p)
 for p in reversed(points):
  while len(hi)>=2 and cross(hi[-2],hi[-1],p)<=0:hi.pop()
  hi.append(p)
 return lo[:-1]+hi[:-1]

def occluded(eye,point,boxes):
 eye=np.asarray(eye,float);delta=np.asarray(point,float)-eye
 for box in boxes:
  lo,hi=np.asarray(box['min']),np.asarray(box['max'])
  if hi[2]<.05:continue
  begin,end=.02,.98
  for k in range(3):
   if abs(delta[k])<1e-10:
    if not lo[k]<=eye[k]<=hi[k]:end=-1.;break
   else:
    a,b=sorted([(lo[k]-eye[k])/delta[k],(hi[k]-eye[k])/delta[k]]);begin=max(begin,a);end=min(end,b)
  if begin<end:return True
 return False

def compose(image,eye,target,robot_xyz,robot_yaw,body_points,goal,boxes):
 project=camera_projector(eye,target,image.size);co,si=math.cos(robot_yaw),math.sin(robot_yaw);position=np.asarray(robot_xyz);protected=Image.new('L',image.size);corners=[]
 for x in[-.32,.32]:
  for y in[-.25,.25]:
   for z in[0.,position[2]+.35]:corners.append([position[0]+co*x-si*y,position[1]+si*x+co*y,z])
 points=project(corners)
 if points:ImageDraw.Draw(protected).polygon(hull(points),fill=255)
 arrow=False;beacon=False;local_length=0.;mask=Image.new('L',image.size);body=np.asarray([p for p in body_points if p[0]>=0.],float)
 if len(body)>=2:
  body[:,0]+=1.10;xy=np.column_stack([position[0]+co*body[:,0]-si*body[:,1],position[1]+si*body[:,0]+co*body[:,1]]);distance=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(xy,axis=0),axis=1))];keep=np.r_[True,np.diff(distance)>1e-5];distance,xy=distance[keep],xy[keep];local_length=float(distance[-1])
  if len(xy)>=2 and local_length>=.12:
   head_start=local_length-min(.30,local_length*.30);q=np.r_[np.arange(0,head_start,.03),head_start];shaft=np.column_stack([np.interp(q,distance,xy[:,k])for k in range(2)]);direction=np.gradient(shaft,axis=0);direction/=np.maximum(np.linalg.norm(direction,axis=1)[:,None],1e-9);side=np.column_stack([-direction[:,1],direction[:,0]])
   # A single polygon: head and shaft share the same base, without a seam.
   outline=np.vstack([shaft+.15*side,shaft[-1]+.32*side[-1],xy[-1],shaft[-1]-.32*side[-1],(shaft-.15*side)[::-1]]);pixels=project(np.column_stack([outline,np.full(len(outline),.003)]))
   if pixels:ImageDraw.Draw(mask).polygon(pixels,fill=205);arrow=True
 mask=ImageChops.subtract(mask,protected);image=Image.composite(Image.new('RGB',image.size,(255,145,0)),image,mask)
 goal=np.asarray(goal[:2]);distance_goal=float(np.linalg.norm(position[:2]-goal));height=1.35*min(1.,max(0.,(distance_goal-.6)/.8));pole_top=[goal[0],goal[1],max(.05,height)]
 if not occluded(eye,pole_top,boxes):
  marker=Image.new('RGB',image.size,(235,30,35));alpha=Image.new('L',image.size);draw=ImageDraw.Draw(alpha);angles=np.linspace(0,2*math.pi,50);ring=project([[goal[0]+.42*math.cos(a),goal[1]+.42*math.sin(a),.006]for a in angles]);line=project([[goal[0],goal[1],.006],pole_top])
  if ring and any(0<=x<image.width and 0<=y<image.height for x,y in ring):draw.line(ring+[ring[0]],fill=210,width=4);beacon=True
  if line and height>.10:
   a,b=line
   if 0<=b[0]<image.width and 0<=b[1]<image.height:
    draw.line([a,b],fill=225,width=4);draw.ellipse([b[0]-6,b[1]-6,b[0]+6,b[1]+6],fill=245);draw.polygon([b,(b[0]+17,b[1]+7),(b[0],b[1]+15)],fill=235);beacon=True
  alpha=ImageChops.subtract(alpha,protected);image=Image.composite(marker,image,alpha)
 return image,dict(arrow_drawn=arrow,goal_beacon_drawn=beacon,native_curve_length_m=local_length,goal_distance_m=distance_goal)
