"""CAD-only warehouse prior and long-route proposal from frozen collision data.

This is not SLAM and does not inspect any robot truth trajectory. All supplied
body-height collision boxes remain; only the previously verified clipping is
used, rather than projecting complete 9m-high warehouse structures.
"""
import argparse
import heapq
import json
import math
from pathlib import Path

import numpy as np
from scipy.ndimage import distance_transform_edt


def dijkstra(free, start, spacing):
    distances = np.full(free.shape, np.inf)
    previous = {}
    distances[start] = 0.
    pending = [(0., start)]
    height, width = free.shape
    neighbors = [(a,b,spacing*math.hypot(a,b)) for a in [-1,0,1] for b in [-1,0,1] if a or b]
    while pending:
        value, (y,x) = heapq.heappop(pending)
        if value != distances[y,x]:
            continue
        for dy,dx,cost in neighbors:
            yy,xx = y+dy,x+dx
            if not (0 <= yy < height and 0 <= xx < width and free[yy,xx]):
                continue
            if dy and dx and not (free[y,xx] and free[yy,x]):
                continue
            new = value+cost
            if new < distances[yy,xx]:
                distances[yy,xx] = new
                previous[(yy,xx)] = (y,x)
                heapq.heappush(pending, (new,(yy,xx)))
    return distances, previous


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--metadata', type=Path, required=True)
    parser.add_argument('--boxes', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--goal-clearance', type=float, default=2.1)
    parser.add_argument('--laps',type=int,default=2)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    meta = json.loads(args.metadata.read_text())
    boxes = json.loads(args.boxes.read_text())
    resolution = .05
    origin = np.asarray(meta['origin'][:2])
    height, width = meta['height'], meta['width']
    yy,xx = np.indices((height,width))
    wx = origin[0]+(xx+.5)*resolution
    wy = origin[1]+(yy+.5)*resolution
    scene = meta['source_scene']
    interior = (wx >= scene['bounds_min'][0]) & (wx <= scene['bounds_max'][0]) & (wy >= scene['bounds_min'][1]) & (wy <= scene['bounds_max'][1])
    occupied = np.zeros((height,width), dtype=bool)
    for box in boxes:
        a,b = np.asarray(box['min'][:2]),np.asarray(box['max'][:2])
        lo = np.maximum(np.floor((a-origin)/resolution).astype(int), [0,0])
        hi = np.minimum(np.ceil((b-origin)/resolution).astype(int)+1, [width,height])
        if np.all(hi > lo):
            occupied[lo[1]:hi[1],lo[0]:hi[0]] = True
    pixels = np.where(occupied,0,np.where(interior,254,128)).astype(np.uint8)
    with (args.output/'map.pgm').open('xb') as f:
        f.write(f'P5\n{width} {height}\n255\n'.encode())
        f.write(np.flipud(pixels).tobytes())
    (args.output/'map.yaml').write_text(f'image: map.pgm\nmode: trinary\nresolution: {resolution}\norigin: [{origin[0]}, {origin[1]}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
    distance = distance_transform_edt(interior & ~occupied)*resolution
    stride = 2
    free = (distance >= .85)[::stride,::stride]
    safe_goals = (distance >= args.goal_clearance)[::stride,::stride] & free
    def cell(xy):
        index = np.rint(((np.asarray(xy)-origin)/resolution-.5)/stride).astype(int)
        return int(index[1]),int(index[0])
    def world(index):
        return (origin+(np.asarray(index[::-1])*stride+.5)*resolution).tolist()
    start = cell([2.,-1.])
    assert free[start], 'Configured start is outside proposed safe interior'
    goals = []
    route = []
    current = start
    total = 0.
    assert 1<=args.laps<=3
    for leg in range(3*args.laps):
        i = leg%3
        distances, previous = dijkstra(free,current,resolution*stride)
        reachable = np.isfinite(distances)
        if i < 2:
            target = tuple(np.unravel_index(np.argmax(np.where(reachable & safe_goals,distances,-1.)),distances.shape))
        else:
            yy,xx = np.indices(free.shape)
            near_start = (yy-start[0])**2+(xx-start[1])**2
            target = tuple(np.unravel_index(np.argmin(np.where(reachable & safe_goals,near_start,np.inf)),free.shape))
        if not math.isfinite(distances[target]):
            raise RuntimeError('No connected CAD route')
        segment = [target]
        while segment[-1] != current:
            segment.append(previous[segment[-1]])
        segment.reverse()
        route.extend(world(x) for x in segment[bool(route):])
        goals.append(dict(xy=world(target),planned_segment_m=float(distances[target])))
        total += float(distances[target])
        current = target
    # Return to a safe region rather than an artificially exact parking point.
    # Laps increase observation distance, not an accuracy acceptance threshold.
    result = dict(kind='CAD_collision_height_clipped_prior_NOT_SLAM',
        warehouse_bounds=[scene['bounds_min'],scene['bounds_max']],body_height_band_m=[.05,.85],
        collision_boxes_preserved=len(boxes),start=[2.,-1.],goals=goals,planned_route_m=total,
        route_proposal_clearance_m=.85,endpoint_clearance_m=args.goal_clearance,laps=args.laps,route_xy=route,
        limitations=['rectangular interior floor prior','AABBs remain conservative','proposal is not executed evidence'])
    (args.output/'route.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='route_xy'},indent=2))


if __name__ == '__main__':
    main()
