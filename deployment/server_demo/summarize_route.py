#!/usr/bin/env python3
"""Select spatially separated detection pictures and verify route evidence."""
import argparse
import json
import math
from pathlib import Path


def main():
    import cv2
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--case',type=Path,required=True)
    args=ap.parse_args()
    root=args.case/'route_evidence'
    def rows(name):
        return [json.loads(s) for s in (root/(name+'.jsonl')).read_text().splitlines() if s]
    trajectory,frames=rows('trajectory'),rows('frames')
    selected=[]
    for frame in frames:
        if not trajectory: break
        pose=min(trajectory,key=lambda p: abs(p['stamp']-frame['stamp']))
        if abs(pose['stamp']-frame['stamp'])>.6: continue
        if not selected or math.dist(pose['xy'],selected[-1]['xy'])>=.75:
            selected.append(dict(file=frame['file'],stamp=frame['stamp'],xy=pose['xy']))
    tiles=[]
    for row in selected:
        frame=cv2.imread(str(root/row['file']))
        if frame is None: raise RuntimeError('missing image '+row['file'])
        frame=cv2.resize(frame,(480,360))
        frame=cv2.copyMakeBorder(frame,0,30,0,0,cv2.BORDER_CONSTANT,value=(25,25,25))
        cv2.putText(frame,'map XY (%.2f, %.2f)'%tuple(row['xy']),(8,381),cv2.FONT_HERSHEY_SIMPLEX,.5,(255,255,255),1)
        tiles.append(frame)
    if tiles:
        import numpy as np
        while len(tiles)%3: tiles.append(np.full_like(tiles[0],32))
        sheet=np.vstack([np.hstack(tiles[i:i+3]) for i in range(0,len(tiles),3)])
        if not cv2.imwrite(str(root/'route_detections.jpg'),sheet): raise RuntimeError('sheet write failed')
    missions=json.loads((args.case/'VISIBLE_DEMO_RESULT.json').read_text())
    capture=json.loads((root/'SUMMARY.json').read_text())
    report=dict(navigation_passed=missions['passed'],capture=capture,selected_images=selected,
                evidence_passed=missions['passed'] and capture['passed'] and len(selected)>=3,
                note='Position estimates and unverified detections; not detection accuracy or SLAM validation')
    (root/'REVIEW.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2))
    return 0 if report['evidence_passed'] else 1


if __name__=='__main__':
    raise SystemExit(main())
