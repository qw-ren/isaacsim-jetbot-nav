"""Dependency-free preflight against frozen PGM map; execution is separate."""
import math,re
from pathlib import Path

def validate_points(map_yaml,points,margin=.6):
    meta=Path(map_yaml).read_text();resolution=float(re.search(r'^resolution:\s*(\S+)',meta,re.M)[1]);origin_text=meta.split('origin:',1)[1].split('negate:',1)[0];origin=[float(x) for x in re.findall(r'[-+]?(?:\d+\.\d*|\d+)',origin_text)[:2]]
    image=re.search(r'^image:\s*(\S+)',meta,re.M)[1];raw=(Path(map_yaml).parent/image).read_bytes();tokens=[];pos=0
    while len(tokens)<4:
        while raw[pos:pos+1].isspace():pos+=1
        if raw[pos:pos+1]==b'#':pos=raw.index(b'\n',pos)+1;continue
        end=pos
        while not raw[end:end+1].isspace():end+=1
        tokens.append(raw[pos:end]);pos=end
    assert tokens[0]==b'P5' and tokens[3]==b'255','Only 8-bit P5 supported'
    pos+=2 if raw[pos:pos+2]==b'\r\n' else 1
    width,height=map(int,tokens[1:3]);pixels=raw[pos:];assert len(pixels)==width*height
    for x,y in points:
        cx,cy=(x-origin[0])/resolution,(y-origin[1])/resolution
        count=math.ceil(margin/resolution)+1
        for j in range(math.floor(cy)-count,math.floor(cy)+count+1):
            for i in range(math.floor(cx)-count,math.floor(cx)+count+1):
                if math.hypot(i+.5-cx,j+.5-cy)*resolution>margin:continue
                if not (0<=i<width and 0<=j<height) or pixels[(height-1-j)*width+i]!=254:raise ValueError('Unsafe waypoint '+str([x,y]))
    return dict(waypoints=len(points),clearance_checked_m=margin,basis='frozen PGM only; not execution acceptance')
