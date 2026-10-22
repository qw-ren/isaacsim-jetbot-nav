"""Standalone figure from recorded trajectories, never a fitted illustration."""
import argparse
import gzip
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--evaluation',type=Path,required=True)
    parser.add_argument('--map',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args()
    report = json.loads((args.evaluation/'metrics.json').read_text())
    with gzip.open(args.evaluation/'trajectories.json.gz','rt') as f:
        data = json.load(f)
    series = data['aligned_series']
    labels = {'online_lio':'Online noisy LIO','replay_ideal':'LIO + ideal IMU',
              'replay_noisy':'LIO + noisy IMU','unaided_imu':'Unaided noisy IMU',
              'unaided_ideal_imu':'Unaided ideal IMU'}
    colors = {'online_lio':'#38965a','replay_ideal':'#2878b5','replay_noisy':'#e17825',
              'unaided_imu':'#bb4444','unaided_ideal_imu':'#9573aa'}
    fig = plt.figure(figsize=(13,8))
    grid = fig.add_gridspec(2,2,width_ratios=[.9,1.4])
    path = fig.add_subplot(grid[:,0])
    position = fig.add_subplot(grid[0,1])
    drift = fig.add_subplot(grid[1,1])
    raster = np.asarray(Image.open(args.map))
    path.imshow(raster,cmap='gray',vmin=0,vmax=254,extent=[-12.5,-12.5+raster.shape[1]*.05,-18.5,-18.5+raster.shape[0]*.05],alpha=.6)
    reference = np.asarray(series['online_lio']['truth_xyz'])
    path.plot(reference[:,0],reference[:,1],'k--',lw=1.2,label='Simulation reference')
    for name in ['replay_ideal','replay_noisy','online_lio']:
        if name not in series:continue
        row = series[name]
        xyz = np.asarray(row['aligned_estimated_xyz'])
        path.plot(xyz[:,0],xyz[:,1],color=colors[name],lw=.85,label=labels[name],alpha=.8)
        position.plot(row['stamp'],100*np.asarray(row['xy_error_m']),color=colors[name],lw=1,label=labels[name])
    for name in ['unaided_imu','unaided_ideal_imu','replay_noisy']:
        if name not in series:continue
        row = series[name]
        error = np.asarray(row['xy_error_m'],dtype=float)
        error[error<=0] = np.nan
        drift.plot(row['stamp'],error,color=colors[name],lw=1,label=labels[name])
    path.set(xlabel='Warehouse x (m)',ylabel='Warehouse y (m)',title='Recorded warehouse path')
    path.set_aspect('equal')
    path.legend(loc='lower left',fontsize=8)
    position.set(xlabel='Simulation time (s)',ylabel='XY position error (cm)',title='LIO accuracy: one initial SE(3) alignment')
    drift.set(xlabel='Simulation time (s)',ylabel='XY position error (m), logarithmic',title='Unaided integration is a baseline, not navigation odometry',yscale='log')
    for ax in [position,drift]:
        ax.grid(alpha=.25)
        ax.legend(fontsize=8)
    distance = report['methods']['online_lio']['physical_xy_path_m']
    fig.suptitle(f'Same recorded LiDAR and paired IMU samples | actual reference path {distance:.1f} m',fontsize=14)
    fig.tight_layout(rect=[0,0,1,.96])
    args.output.parent.mkdir(parents=True,exist_ok=True)
    for suffix in ['.png','.pdf']:
        fig.savefig(args.output.with_suffix(suffix),dpi=180)
    print(args.output.with_suffix('.png'))


if __name__ == '__main__':
    main()
