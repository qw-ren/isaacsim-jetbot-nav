"""Simulation extension contracts; no ROS, truth, model weights or training dependencies."""
import math

class CommandLease:
 def __init__(self):self.last_wall=None;self.last_stamp=None;self.value=None;self.fault=None
 def accept(self,value,stamp,wall,sim):
  if self.fault:return
  if len(value)!=2 or not all(math.isfinite(x) for x in [*value,stamp,wall,sim]):self.fault='NONFINITE_PROPOSAL';return
  if not -.025<=sim-stamp<=.25:self.fault='PROPOSAL_SIM_AGE';return
  if self.last_stamp is not None and stamp<self.last_stamp-1e-6:self.fault='PROPOSAL_TIME_REVERSED';return
  self.value=list(value);self.last_wall=wall;self.last_stamp=stamp
 def output(self,wall,sim):
  if self.fault or self.value is None:return None
  if wall-self.last_wall>.25 or not -.025<=sim-self.last_stamp<=.30:self.fault='SELECTED_SOURCE_TIMEOUT';return None
  return self.value

def resolve_place(catalog,label):
 if not isinstance(label,str)or not label.strip():raise ValueError('Empty semantic destination')
 query=label.strip();matches=[p for p in catalog['places'] if query in [p['id'],p['name'],*p.get('aliases',[])]]
 if len(matches)!=1:raise ValueError('Unknown or ambiguous destination; choose an explicit catalog name')
 place=matches[0];goal=place['approach_pose_map'];assert len(goal)==3 and all(math.isfinite(v)for v in goal)
 if not place.get('approved_approach',False):raise ValueError('Approach pose not approved')
 return place
