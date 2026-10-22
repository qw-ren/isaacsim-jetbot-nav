"""LIO pose to navigation odometry; no simulator, ROS or truth inputs.

The fixed anchor is a declared coordinate convention from the case's initial
configuration. It is never fitted to a truth trajectory or updated in flight.
The native FAST-LIO build publishes pose but leaves twist unset, so velocity is
derived from consecutive estimated poses and expressed in the current body.
"""
import math

import numpy as np
from scipy.spatial.transform import Rotation


def pose_in_map(pose, transform):
    """SE2 map<-odom; static CAD boxes are in map, never arbitrary LIO odom."""
    tx,ty,angle = transform
    x,y,z,yaw,tilt = pose
    c,s = math.cos(angle),math.sin(angle)
    return [tx+c*x-s*y,ty+s*x+c*y,z,yaw+angle,tilt]


def clearance_in_map(pose, boxes, transform, v, w):
    # Exactly the original guard's 1.5s/16-sample constant-command geometry;
    # only the coordinate conversion differs. No safety distance changes.
    x,y,_,yaw,_ = pose_in_map(pose,transform)
    t = np.linspace(0,1.5,16)
    if abs(w)<1e-6:
        points = np.c_[x+v*t*math.cos(yaw),y+v*t*math.sin(yaw)]
    else:
        points = np.c_[x+v/w*(np.sin(yaw+w*t)-math.sin(yaw)),
                       y-v/w*(np.cos(yaw+w*t)-math.cos(yaw))]
    delta = np.maximum(np.maximum(boxes[:,0,None,:]-points,points-boxes[:,1,None,:]),0)
    return float(np.linalg.norm(delta,axis=-1).min())


def align_up(vector):
    """Minimal rotation of measured upward specific force to navigation +Z."""
    unit = np.asarray(vector, dtype=float)
    norm = np.linalg.norm(unit)
    if unit.shape != (3,) or not np.isfinite(unit).all() or norm < 1e-6:
        raise ValueError('Invalid measured gravity vector')
    unit = unit/norm
    target = np.array([0., 0., 1.])
    cross = np.cross(unit, target)
    cosine = float(unit@target)
    if cosine < -.999999:
        return Rotation.from_rotvec([np.pi, 0., 0.])
    skew = np.array([[0.,-cross[2],cross[1]],[cross[2],0.,-cross[0]],[-cross[1],cross[0],0.]])
    return Rotation.from_matrix(np.eye(3)+skew+skew@skew/(1+cosine))


class LioNavConverter:
    def __init__(self, anchor_xyz, anchor_yaw=0.0, gravity_in_lio=None):
        self.anchor = np.asarray(anchor_xyz, dtype=float)
        if self.anchor.shape != (3,) or not np.isfinite(self.anchor).all() or not math.isfinite(anchor_yaw):
            raise ValueError('Invalid configured anchor')
        self.rotation = Rotation.from_euler('z', anchor_yaw)
        if gravity_in_lio is not None:
            self.rotation = self.rotation*align_up(gravity_in_lio)
        self.previous = None

    def update(self, stamp, xyz, xyzw):
        xyz = np.asarray(xyz, dtype=float)
        xyzw = np.asarray(xyzw, dtype=float)
        if xyz.shape != (3,) or xyzw.shape != (4,) or not np.isfinite(np.r_[stamp, xyz, xyzw]).all():
            raise ValueError('Nonfinite or malformed LIO pose')
        norm = np.linalg.norm(xyzw)
        if not .99 <= norm <= 1.01:
            raise ValueError('Invalid LIO quaternion')
        body = Rotation.from_quat(xyzw)
        pose = self.rotation.apply(xyz) + self.anchor
        orientation = self.rotation * body
        old = self.previous
        if old is not None:
            dt = stamp-old[0]
            if not 0.0 < dt <= .30:
                raise ValueError('LIO reset, duplicate time or missing scan interval')
            linear = orientation.inv().apply((pose-old[1])/dt)
            angular = (old[2].inv()*orientation).as_rotvec()/dt
            # A discontinuity far outside this simulation's .8m/s/.6rad/s
            # command bounds is an estimator fault, not a quality benchmark.
            if np.linalg.norm(linear) > 5.0 or np.linalg.norm(angular) > 6.0:
                raise ValueError('LIO pose discontinuity')
            result = dict(stamp=float(stamp), xyz=pose, xyzw=orientation.as_quat(),
                          linear_body=linear, angular_body=angular)
        else:
            result = None  # Never label an unknown initial twist as measured zero.
        self.previous = (float(stamp), pose, orientation)
        return result
