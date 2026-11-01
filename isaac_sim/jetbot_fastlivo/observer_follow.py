"""Display-only level observer: planar smoothing, yaw deadband and rate limit."""
import math

class ObserverFollow:
 def __init__(self):self.position=None;self.yaw=None;self.last_time=None;self.distance=2.8
 def update(self,position,yaw,stamp,boxes=()):
  if self.position is None:
   self.position=[float(position[0]),float(position[1])];self.yaw=float(yaw);self.last_time=float(stamp)
  dt=max(0.,min(.1,float(stamp)-self.last_time));self.last_time=float(stamp)
  alpha=-math.expm1(-dt/.35)
  for i in range(2):self.position[i]+=alpha*(float(position[i])-self.position[i])
  error=math.atan2(math.sin(yaw-self.yaw),math.cos(yaw-self.yaw))
  deadband=math.radians(3.);correction=math.copysign(max(0.,abs(error)-deadband),error)
  step=correction*(-math.expm1(-dt/.65));self.yaw+=max(-.4*dt,min(.4*dt,step))
  x,y=self.position;dx,dy=math.cos(self.yaw),math.sin(self.yaw)
  anchor=[float(position[0]),float(position[1]),.15]
  boxes=[b for b in boxes if all(b['min'][j]-.12-3.5<=anchor[j]<=b['max'][j]+.12+3.5 for j in (0,1)) and b['max'][2]>.03]
  cap=min(2.8,self.distance+(-math.expm1(-dt/.8))*(2.8-self.distance));allowed=0.
  for n in range(28,-1,-1):
   distance=min(cap,n*.1);eye=[x-distance*dx,y-distance*dy,2.6]
   if not any(intersects(anchor,eye,b,.12) for b in boxes):allowed=distance;break
  self.distance=allowed
  lookahead=1.8*self.distance/2.8
  return [x-self.distance*dx,y-self.distance*dy,2.6],[x+lookahead*dx,y+lookahead*dy,.35]

def intersects(start,end,box,padding=0.):
 lower,upper=box['min'],box['max'];lo,hi=0.,1.
 for i in range(3):
  delta=end[i]-start[i];a=lower[i]-padding;b=upper[i]+padding
  if abs(delta)<1e-12:
   if start[i]<a or start[i]>b:return False
  else:
   first,second=(a-start[i])/delta,(b-start[i])/delta
   if first>second:first,second=second,first
   lo=max(lo,first);hi=min(hi,second)
   if lo>hi:return False
 return True
