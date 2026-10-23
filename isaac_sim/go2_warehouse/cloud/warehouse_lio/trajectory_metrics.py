"""Truth comparison only: one initial SE3 alignment, never trajectory fitting."""
import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from lio_nav_math import align_up


def unique(rows):
    rows = sorted(rows, key=lambda x:x['stamp'])
    return [r for i,r in enumerate(rows) if i == len(rows)-1 or r['stamp'] != rows[i+1]['stamp']]


def inertial_only(rows, calibration=(3.,5.)):
    """Unaided strapdown baseline, gyro-calibrated during configured standstill.

    No truth, wheel velocity, ZUPT during travel, LIO, magnetometer or GPS input.
    Initial roll/pitch and acceleration norm come from the same IMU samples.
    """
    rows = unique(rows)
    stationary = [r for r in rows if calibration[0] <= r['stamp'] <= calibration[1]]
    if len(stationary) < 100:
        raise ValueError('Insufficient configured stationary IMU calibration window')
    mean_acc = np.mean([r['acc'] for r in stationary],axis=0)
    mean_gyr = np.mean([r['gyro'] for r in stationary],axis=0)
    orientation = align_up(mean_acc)
    scale = 9.81/np.linalg.norm(mean_acc)
    position = np.zeros(3)
    velocity = np.zeros(3)
    output = []
    old = None
    for row in rows:
        if row['stamp'] < calibration[1]:
            continue
        if old is not None:
            dt = row['stamp']-old['stamp']
            if not 0. < dt <= .03:
                raise ValueError('IMU capture gap in strapdown baseline')
            omega = (np.asarray(row['gyro'])+old['gyro'])/2-mean_gyr
            delta = omega*dt
            midpoint = orientation*Rotation.from_rotvec(delta/2)
            acceleration = midpoint.apply((np.asarray(row['acc'])+old['acc'])*scale/2)+[0.,0.,-9.81]
            position = position+velocity*dt+acceleration*dt*dt/2
            velocity = velocity+acceleration*dt
            orientation = orientation*Rotation.from_rotvec(delta)
        output.append(dict(stamp=row['stamp'],xyz=position.tolist(),xyzw=orientation.as_quat().tolist()))
        old = row
    return output, dict(calibration_window_s=calibration, samples=len(stationary),
                        gyro_bias_estimate=mean_gyr.tolist(),acceleration_scale=float(scale),
                        aiding='none after initial stationary calibration',truth_used=False)


def summarize(values):
    v = np.asarray(values)
    if not len(v):
        return None
    return dict(mean=float(v.mean()),p50=float(np.percentile(v,50)),p95=float(np.percentile(v,95)),maximum=float(v.max()))


def evaluate(truth_rows, estimated_rows, begin=5.):
    truth_rows, estimated_rows = unique(truth_rows), unique(estimated_rows)
    tt = np.asarray([r['stamp'] for r in truth_rows])
    valid = [r for r in estimated_rows if max(tt[0],begin) <= r['stamp'] <= tt[-1]]
    if len(valid) < 2:
        raise ValueError('Insufficient overlapping poses')
    t = np.asarray([r['stamp'] for r in valid])
    truth_xyz = np.asarray([r['xyz'] for r in truth_rows])
    tp = np.column_stack([np.interp(t,tt,truth_xyz[:,i]) for i in range(3)])
    tr = Slerp(tt,Rotation.from_quat([r['xyzw'] for r in truth_rows]))(t)
    ep = np.asarray([r['xyz'] for r in valid])
    er = Rotation.from_quat([r['xyzw'] for r in valid])
    anchor = tr[0]*er[0].inv()
    ep = anchor.apply(ep-ep[0])+tp[0]
    er = anchor*er
    error = ep-tp
    xy = np.linalg.norm(error[:,:2],axis=1)
    angle = (tr.inv()*er).magnitude()
    # Use all physical truth samples in the evaluated interval for path length,
    # not an estimator's possibly drifting or sparsely sampled curve.
    physical_distance = np.r_[0.,np.cumsum(np.linalg.norm(np.diff(truth_xyz[:,:2],axis=0),axis=1))]
    sample_distance = np.interp(t,tt,physical_distance)-np.interp(t[0],tt,physical_distance)
    distance = float(sample_distance[-1])
    rpe = {}
    for window in [5.,10.,20.]:
        translational, rotational = [],[]
        for i in range(len(t)):
            j = int(np.searchsorted(sample_distance,sample_distance[i]+window))
            if j >= len(t):
                break
            true_delta = tr[i].inv().apply(tp[j]-tp[i])
            estimated_delta = er[i].inv().apply(ep[j]-ep[i])
            translational.append(float(np.linalg.norm(estimated_delta-true_delta)))
            rotational.append(float(((tr[i].inv()*tr[j]).inv()*(er[i].inv()*er[j])).magnitude()))
        rpe[str(window)+'m'] = dict(translation_m=summarize(translational),rotation_rad=summarize(rotational),segments=len(translational))
    metrics = dict(samples=len(valid),begin_s=float(t[0]),end_s=float(t[-1]),physical_xy_path_m=distance,
        alignment='single initial SE3, no full-trajectory/scale fit',
        xy_error_m=summarize(xy),xyz_error_m=summarize(np.linalg.norm(error,axis=1)),
        z_error_m=summarize(abs(error[:,2])),orientation_error_rad=summarize(angle),
        endpoint_xy_error_m=float(xy[-1]),endpoint_xy_drift_percent=float(100*xy[-1]/distance) if distance>.1 else None,
        relative_pose_error=rpe)
    series = dict(stamp=t.tolist(),truth_xyz=tp.tolist(),aligned_estimated_xyz=ep.tolist(),xy_error_m=xy.tolist(),
                  orientation_error_rad=angle.tolist())
    return metrics,series
