#!/usr/bin/env python3
"""Record synchronized visual evidence and measured scan/semantic points in map.

This is an observation atlas, not a replacement SLAM or navigation costmap.
"""
import argparse
import json
import math
from pathlib import Path
import time


def transform_xy(x, y, transform):
    q = transform.rotation
    # Full quaternion rotation, including tilted sensor frames.
    return (transform.translation.x + (1-2*(q.y*q.y+q.z*q.z))*x + 2*(q.x*q.y-q.z*q.w)*y,
            transform.translation.y + 2*(q.x*q.y+q.z*q.w)*x + (1-2*(q.x*q.x+q.z*q.z))*y)


def main():
    import cv2
    import numpy as np
    import rclpy
    from rclpy.node import Node
    from rclpy.time import Time
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image, LaserScan
    from nav_msgs.msg import OccupancyGrid
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformListener, TransformException

    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--case', type=Path, required=True)
    ap.add_argument('--interval', type=float, default=2.)
    ap.add_argument('--timeout', type=float, default=1100.)
    args = ap.parse_args()
    if not math.isfinite(args.interval) or args.interval <= 0:
        raise ValueError('interval must be positive')
    out = args.case / 'route_evidence'
    out.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    node = Node('route_evidence', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
    bridge, buffer = CvBridge(), Buffer()
    listener = TransformListener(buffer, node)
    images, detections, planar, pending_scans = {}, {}, {}, {}
    route, hits, objects = [], set(), {}
    last_image, last_scan = -math.inf, -math.inf
    counts = dict(images=0, scans=0, planar_packets=0, tf_misses=0, map_messages=0)
    streams = {k: (out / (k+'.jsonl')).open('x') for k in ('frames', 'scans', 'observations', 'trajectory')}

    def write(key, data):
        streams[key].write(json.dumps(data, allow_nan=False)+'\n')
        streams[key].flush()

    def stamp(msg):
        return round(msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9, 6)

    def bounded(cache, key, value):
        cache[key] = value
        while len(cache) > 80:
            del cache[min(cache)]

    def render():
        points = [(x*.1,y*.1) for x,y in hits] + [p['xy'] for p in route] + [o['position_xy'] for o in objects.values()]
        if not points:
            return
        lo = np.min(points, axis=0)-1
        hi = np.max(points, axis=0)+1
        scale = min(45., 900./max(hi-lo))
        canvas = np.full((int((hi[1]-lo[1])*scale)+90, int((hi[0]-lo[0])*scale)+40, 3), 248, np.uint8)
        def pixel(p):
            return int((p[0]-lo[0])*scale)+20, int((hi[1]-p[1])*scale)+65
        for x,y in hits:
            cv2.circle(canvas, pixel((x*.1,y*.1)), 1, (110,110,110), -1)
        for a,b in zip(route, route[1:]):
            cv2.line(canvas, pixel(a['xy']), pixel(b['xy']), (220,100,20), 2)
        for obj in objects.values():
            p = pixel(obj['position_xy'])
            cv2.circle(canvas,p,3,(20,90,230),-1)
            cv2.putText(canvas,obj['class_name'],p,cv2.FONT_HERSHEY_SIMPLEX,.35,(20,70,180),1)
        if route:
            for row,label in ((route[0],'START'),(route[-1],'CURRENT')):
                p=pixel(row['xy']); cv2.circle(canvas,p,5,(20,160,30),2)
                cv2.putText(canvas,label,p,cv2.FONT_HERSHEY_SIMPLEX,.4,(0,120,20),1)
        cv2.putText(canvas,'MAP: laser hits (gray), estimated route (blue), vision (red)',(8,20),cv2.FONT_HERSHEY_SIMPLEX,.4,(0,0,0),1)
        cv2.putText(canvas,'Observed surfaces; unverified classes; no free-space inference',(8,40),cv2.FONT_HERSHEY_SIMPLEX,.4,(0,0,0),1)
        if not cv2.imwrite(str(out/'semantic_route.png'), canvas):
            raise RuntimeError('map image write failed')

    def save_pairs():
        nonlocal last_image
        for key in sorted(images.keys() & detections.keys() & planar.keys()):
            if key-last_image < args.interval:
                continue
            packet = planar[key]
            if packet.get('frame_id') != 'map':
                continue
            frame = bridge.imgmsg_to_cv2(images[key], desired_encoding='bgr8').copy()
            for det in detections[key]['detections']:
                x1,y1,x2,y2 = [int(v) for v in det['bbox']]
                cv2.rectangle(frame,(x1,y1),(x2,y2),(0,220,255),1)
                cv2.putText(frame,'%s %.2f'%(det['class_name'],det['confidence']),
                            (x1,max(12,y1-3)),cv2.FONT_HERSHEY_SIMPLEX,.35,(0,220,255),1)
            cv2.putText(frame,'sim %.2fs'%key,(5,frame.shape[0]-8),cv2.FONT_HERSHEY_SIMPLEX,.4,(0,220,255),1)
            name = 'detection_%05d.png'%counts['images']
            if not cv2.imwrite(str(out/name),frame):
                raise RuntimeError('detection image write failed')
            write('frames',dict(file=name,stamp=key,detections=detections[key],planar=packet))
            last_image=key; counts['images']+=1
            render()

    def on_image(msg):
        bounded(images,stamp(msg),msg); save_pairs()

    def on_detection(msg):
        data=json.loads(msg.data); bounded(detections,round(data['stamp'],6),data); save_pairs()

    def on_planar(msg):
        data=json.loads(msg.data)
        if data.get('schema') != 'atongmu.planar_observations' or data.get('frame_id') != 'map':
            return
        write('observations',data); counts['planar_packets']+=1
        for obj in data['objects']:
            # Atlas uses spatial cells, not persistent identity; people remain in raw logs only.
            if obj['class_name']=='person':
                continue
            x,y=obj['position_xy']
            objects[(obj['class_name'],round(x/.25),round(y/.25))]=dict(obj,stamp=data['stamp'])
        bounded(planar,round(data['stamp'],6),data); save_pairs()

    def process_scan(msg):
        nonlocal last_scan
        t=stamp(msg)
        if t-last_scan < .5:
            return True
        try:
            tf=buffer.lookup_transform('map',msg.header.frame_id,Time.from_msg(msg.header.stamp)).transform
            base=buffer.lookup_transform('map','chassis',Time.from_msg(msg.header.stamp)).transform
        except TransformException:
            counts['tf_misses']+=1
            return False
        points=[]
        for i,r in enumerate(msg.ranges):
            if math.isfinite(r) and msg.range_min <= r < msg.range_max:
                a=msg.angle_min+i*msg.angle_increment
                x,y=transform_xy(r*math.cos(a),r*math.sin(a),tf)
                points.append([x,y]); hits.add((round(x/.1),round(y/.1)))
        row=dict(stamp=t,xy=[base.translation.x,base.translation.y])
        route.append(row); write('trajectory',row)
        write('scans',dict(stamp=t,frame_id='map',sensor_frame=msg.header.frame_id,points=points))
        counts['scans']+=1; last_scan=t
        return True

    def on_scan(msg):
        bounded(pending_scans,stamp(msg),msg)

    def retry_scans():
        # TF may arrive after the scan; retry exact acquisition time, never latest TF.
        now=node.get_clock().now().nanoseconds*1e-9
        for key in sorted(pending_scans):
            if now-key>2. or process_scan(pending_scans[key]):
                del pending_scans[key]

    def on_map(msg):
        if msg.header.frame_id != 'map':
            return
        # Preserve source geometry and origin; do not call a prebuilt map online SLAM.
        p,q=msg.info.origin.position,msg.info.origin.orientation
        data=dict(topic='/map',frame_id=msg.header.frame_id,stamp=stamp(msg),
                  resolution=msg.info.resolution,width=msg.info.width,height=msg.info.height,
                  origin=dict(position=[p.x,p.y,p.z],orientation=[q.x,q.y,q.z,q.w]),data=list(msg.data))
        (out/'occupancy_map.json').write_text(json.dumps(data))
        counts['map_messages']+=1

    subscriptions=[node.create_subscription(Image,'/jetbot/image_raw',on_image,qos_profile_sensor_data),
        node.create_subscription(String,'/perception/detections',on_detection,10),
        node.create_subscription(String,'/perception/objects_2d',on_planar,10),
        node.create_subscription(LaserScan,'/jetbot/scan',on_scan,qos_profile_sensor_data),
        node.create_subscription(OccupancyGrid,'/map',on_map,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))]
    timer=node.create_timer(.1,retry_scans)
    deadline=time.monotonic()+args.timeout
    error=None
    try:
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.1)
            if (args.case/'VISIBLE_DEMO_RESULT.json').exists() or (args.case/'RUNNER_RESULT.json').exists():
                break
        else:
            error='capture wall timeout'
    except Exception as exc:
        error=str(exc)
    finally:
        render()
        distance=sum(math.dist(a['xy'],b['xy']) for a,b in zip(route,route[1:]))
        report=dict(**counts,estimated_path_length_m=distance,error=error,
                    passed=error is None and counts['images']>=3 and counts['scans']>=3 and counts['planar_packets']>=3 and distance>2,
                    scope='map-frame observation atlas, not SLAM; scan endpoints only; vision classes unverified')
        (out/'SUMMARY.json').write_text(json.dumps(report,indent=2))
        for stream in streams.values(): stream.close()
        node.destroy_node(); rclpy.shutdown()
    print(json.dumps(report))
    return 0 if report['passed'] else 1


if __name__=='__main__':
    raise SystemExit(main())
