"""Planar scan rotation compensation from raw IMU gyro integration only."""
import numpy as np

def deskew_yaw(xyz,point_times,gyro_history):
 history=np.asarray(gyro_history,dtype=np.float64);times=np.asarray(point_times,dtype=np.float64)
 if len(history)<2 or not len(times)or not np.isfinite(times).all():raise ValueError('Missing IMU timing history')
 if times.min()<history[0,0]-1e-6 or times.max()>history[-1,0]+1e-6:raise ValueError('IMU history does not cover scan')
 reference=float((times.min()+times.max())/2);relative=np.interp(times,history[:,0],history[:,1])-float(np.interp(reference,history[:,0],history[:,1]));co,si=np.cos(relative),np.sin(relative);result=np.asarray(xyz).copy();result[:,0]=co*xyz[:,0]-si*xyz[:,1];result[:,1]=si*xyz[:,0]+co*xyz[:,1]
 return result,reference
