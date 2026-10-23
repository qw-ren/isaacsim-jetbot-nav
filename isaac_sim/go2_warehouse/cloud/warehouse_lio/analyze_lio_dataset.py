#!/usr/bin/env python3
"""Evaluate saved sensor data; never publishes anything back to the experiment."""
import argparse
import csv
import gzip
import json
from pathlib import Path
import sqlite3
import hashlib
import shutil
import zlib

from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

from trajectory_metrics import evaluate, inertial_only


def load(db, topic, kind):
    output = []
    for stamp,payload in db.execute('SELECT stamp_ns,payload FROM records WHERE topic=? ORDER BY stamp_ns,id',(topic,)):
        m = deserialize_message(zlib.decompress(payload),kind)
        row = dict(stamp=stamp/1e9)
        if kind is Odometry:
            p,q = m.pose.pose.position,m.pose.pose.orientation
            row.update(xyz=[p.x,p.y,p.z],xyzw=[q.x,q.y,q.z,q.w])
        else:
            a,g = m.linear_acceleration,m.angular_velocity
            row.update(acc=[a.x,a.y,a.z],gyro=[g.x,g.y,g.z])
        output.append(row)
    return output


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--database',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--replay',type=Path,action='append',default=[])
    parser.add_argument('--imu-csv',type=Path,help='Explicit complete published-IMU log, only for old best-effort captures')
    args = parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    frozen = args.output/'analysis_source'
    frozen.mkdir()
    for name in ['analyze_lio_dataset.py','trajectory_metrics.py','lio_nav_math.py']:
        shutil.copy2(Path(__file__).with_name(name),frozen/name)
    (args.output/'analysis_source.sha256').write_text(''.join(hashlib.sha256(p.read_bytes()).hexdigest()+'  '+str(p.relative_to(args.output))+'\n' for p in sorted(frozen.iterdir())))
    db = sqlite3.connect(f'file:{args.database}?mode=ro',uri=True)
    truth = load(db,'/truth/odom',Odometry)
    imu = load(db,'/lio/imu',Imu)
    if args.imu_csv:
        with args.imu_csv.open() as f:
            imu = [dict(stamp=float(r['stamp']),acc=[float(r[k]) for k in ['ax','ay','az']],
                        gyro=[float(r[k]) for k in ['gx','gy','gz']]) for r in csv.DictReader(f)]
    lio = load(db,'/lio/odometry',Odometry)
    baseline,calibration = inertial_only(imu)
    methods = dict(online_lio=lio,unaided_imu=baseline)
    ideal = load(db,'/evaluation/imu_ideal',Imu)
    ideal_calibration = None
    if ideal:
        methods['unaided_ideal_imu'],ideal_calibration = inertial_only(ideal)
    for p in args.replay:
        data = json.loads(p.read_text())
        if data['statistics'].get('error'):
            raise ValueError('Failed replay: '+str(p))
        methods[p.parent.name] = data['poses']
    metrics,series = {},{}
    for name,rows in methods.items():
        metrics[name],series[name] = evaluate(truth,rows)
    ratio = metrics['unaided_imu']['xy_error_m']['p95']/max(metrics['online_lio']['xy_error_m']['p95'],1e-12)
    report = dict(methods=metrics,unaided_imu_calibration=calibration,unaided_ideal_calibration=ideal_calibration,
        imu_to_lio_xy_p95_ratio=ratio,
        interpretation='Unaided inertial drift versus external LiDAR aiding; not improvement over simulator truth odometry',
        accuracy_acceptance='report errors relative to map resolution/clearance; no fabricated pass threshold',
        source_database=str(args.database),imu_source=str(args.imu_csv) if args.imu_csv else 'recorded /lio/imu')
    (args.output/'metrics.json').write_text(json.dumps(report,indent=2)+'\n')
    with gzip.open(args.output/'trajectories.json.gz','wt') as f:
        json.dump(dict(raw_truth=truth,raw_imu=imu,raw_ideal_imu=ideal,raw_estimates=methods,aligned_series=series),f)
    print(json.dumps(report,indent=2))


if __name__ == '__main__':
    main()
