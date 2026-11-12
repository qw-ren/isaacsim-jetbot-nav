"""ROS-independent protocol, object lifecycle, query and approach geometry."""
import copy
import json
import math
import re

VERSION = 1
ALIASES = {'cart': ('推车', '手推车', 'pushcart', 'cart'),
           'pallet': ('托盘', 'pallet'), 'box': ('纸箱', '箱子', 'box'),
           'shelf': ('货架', 'shelf'), 'sign': ('标牌', '标志牌', 'sign')}
OBJECT_CLASSES=set(ALIASES)|{'wall','barrier','person'}


class CaseEventReader:
    """Read only complete new events; skip pre-request history and partial trailing writes."""
    def __init__(self,path):
        self.stream=open(path,'rb')
        self.stream.seek(0,2)

    def read(self):
        rows=[]
        while True:
            position=self.stream.tell();line=self.stream.readline()
            if not line or not line.endswith(b'\n'):
                self.stream.seek(position);break
            rows.append(json.loads(line))
        return rows

    def close(self):
        self.stream.close()


def parse_command(text):
    if not isinstance(text, str) or len(text) > 128:
        raise ValueError('指令须为128字以内文本')
    text = re.sub(r'[\s。！!，,]', '', text.lower())
    names = '|'.join(re.escape(n) for aliases in ALIASES.values() for n in sorted(aliases,key=len,reverse=True))
    prefix=r'(?:请)?(?:走到|前往|找到|到|去|找)'
    relation=re.fullmatch(prefix+r'第([一二三四五六七八九十百零两\d]+)排货架(?:旁边|附近|旁)的?(那个|最近的?|最远的?)?('+names+r')(?:那里|处)?',text)
    direct=re.fullmatch(prefix+r'(?:离我)?(最近的?|最远的?)?('+names+r')(?:那里|旁边|附近|处)?',text)
    match=relation or direct
    if not match:
        raise ValueError('支持“到最远的纸箱那里”或“走到第二排货架旁边的那个纸箱”；多个候选请指定最近/最远')
    selection,name=(match.group(2),match.group(3)) if relation else match.groups()
    kind=next(k for k,aliases in ALIASES.items()if name in aliases)
    result=dict(intent='navigate_to_object',object_class=kind,relation='approach',
                selection='farthest_reachable' if selection and selection.startswith('最远') else 'nearest_reachable')
    if relation:
        digits={'零':0,'一':1,'二':2,'两':2,'三':3,'四':4,'五':5,'六':6,'七':7,'八':8,'九':9}
        number=match.group(1)
        if number.isdigit():index=int(number)
        elif number in digits:index=digits[number]
        elif number.count('十')==1 and all(c in digits or c=='十' for c in number):
            left,right=number.split('十');index=(digits.get(left,1)*10+digits.get(right,0)) if len(left)<=1 and len(right)<=1 else 0
        else:index=0
        if not 1<=index<=99:raise ValueError('货架排号须为1至99')
        result.update(relation='near_shelf_row',reference=dict(object_class='shelf',row_index=index))
        if not selection or selection=='那个':result['selection']='unique'
    return result


def finite_vector(value, length):
    if not isinstance(value, list) or len(value) != length:
        raise ValueError('Invalid vector shape')
    if not all(type(v) in (int, float) and math.isfinite(v) for v in value):
        raise ValueError('Nonfinite vector')
    return list(map(float, value))


def validate_observation(row):
    row = copy.deepcopy(row)
    if not isinstance(row.get('object_id'), str) or not 0 < len(row['object_id']) <= 512:
        raise ValueError('Invalid object id')
    if row.get('class_name') not in OBJECT_CLASSES or row.get('source') not in ('ground_truth', 'vision'):
        raise ValueError('Invalid class/source')
    row['position'] = finite_vector(row['position'], 3)
    row['orientation_xyzw'] = finite_vector(row['orientation_xyzw'], 4)
    norm = math.sqrt(sum(v*v for v in row['orientation_xyzw']))
    if abs(norm-1) > .001:
        raise ValueError('Invalid unit quaternion')
    row['orientation_xyzw']=[v/norm for v in row['orientation_xyzw']]
    row['dimensions'] = finite_vector(row['dimensions'], 3)
    if not all(0 < v < 50 for v in row['dimensions']):
        raise ValueError('Invalid dimensions')
    if type(row.get('confidence')) not in (int, float) or not 0 <= row['confidence'] <= 1:
        raise ValueError('Invalid confidence')
    if 'row_index' in row and (row['class_name']!='shelf' or type(row['row_index'])is not int or not 1<=row['row_index']<=99):
        raise ValueError('Invalid shelf row index')
    return row


def multiply_quaternion(a, b):
    x,y,z,w = a; X,Y,Z,W = b
    return [w*X+x*W+y*Z-z*Y, w*Y-x*Z+y*W+z*X,
            w*Z+x*Y-y*X+z*W, w*W-x*X-y*Y-z*Z]


def rotate(q, v):
    return multiply_quaternion(multiply_quaternion(q, [*v, 0.]), [-q[0],-q[1],-q[2],q[3]])[:3]


def transform_object(row, translation, rotation):
    result = copy.deepcopy(row)
    p = rotate(rotation, row['position'])
    result['position'] = [a+b for a,b in zip(p, translation)]
    result['orientation_xyzw'] = multiply_quaternion(rotation, row['orientation_xyzw'])
    return result


class ObjectStore:
    def __init__(self, ttl):
        self.ttl = ttl
        self.objects = {}
        self.last_stamp = -1.

    def update(self, packet, now, wall):
        if packet.get('version') != VERSION or packet.get('frame_id') != 'map':
            raise ValueError('Object store requires v1/map')
        stamp = packet['stamp']
        if type(stamp) not in (int,float) or not math.isfinite(stamp) or not -.1 <= now-stamp <= self.ttl:
            raise ValueError('Stale/future observation')
        if stamp < self.last_stamp:
            raise ValueError('Out-of-order observation; restart after clock reset')
        rows = packet['objects']
        if not isinstance(rows, list) or len(rows) > 1000:
            raise ValueError('Invalid batch')
        rows = [validate_observation(row) for row in rows]
        if len({r['object_id'] for r in rows}) != len(rows):
            raise ValueError('Duplicate id in batch')
        self.last_stamp = stamp
        for row in rows:
            old = self.objects.get(row['object_id'])
            if old and (old['source'] != row['source'] or old['class_name'] != row['class_name']):
                raise ValueError('Stable ID changed source/class')
        for row in rows:
            self.objects[row['object_id']] = dict(row, last_seen=stamp, received_wall=wall, status='active')

    def active(self, now, wall):
        self.objects={key:r for key,r in self.objects.items()
                      if -.1<=now-r['last_seen']<=self.ttl and 0<=wall-r['received_wall']<=self.ttl}
        return [copy.deepcopy(r) for r in self.objects.values()
                if -.1 <= now-r['last_seen'] <= self.ttl and 0 <= wall-r['received_wall'] <= self.ttl]


def yaw_of(row):
    x,y,z,w = row['orientation_xyzw']
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def distance_to_object(point, row):
    dx,dy = point[0]-row['position'][0], point[1]-row['position'][1]
    yaw = yaw_of(row); c,s = math.cos(yaw),math.sin(yaw)
    x,y = c*dx+s*dy, -s*dx+c*dy
    half=planar_half_sizes(row)
    return math.hypot(max(abs(x)-half[0], 0),max(abs(y)-half[1], 0))


def planar_half_sizes(row):
    """Conservative 3D box projection onto XY, expressed in its yaw-only frame."""
    if abs(row['orientation_xyzw'][0])+abs(row['orientation_xyzw'][1])<1e-12:
        return [v/2 for v in row['dimensions'][:2]]
    yaw=yaw_of(row);c,s=math.cos(yaw),math.sin(yaw)
    axes=[rotate(row['orientation_xyzw'],v)for v in([1.,0.,0.],[0.,1.,0.],[0.,0.,1.])]
    return [sum(abs(c*axis[0]+s*axis[1])*size/2 for axis,size in zip(axes,row['dimensions'])),
            sum(abs(-s*axis[0]+c*axis[1])*size/2 for axis,size in zip(axes,row['dimensions']))]


def approaches(row, radius, padding, margin, count=12):
    """Ray/OBB surface intersection + robot disk + explicit approach margin."""
    yaw = yaw_of(row); half = planar_half_sizes(row)
    candidates = []
    for i in range(count):
        a = i*2*math.pi/count
        dx,dy = math.cos(a),math.sin(a)
        edge = min(half[0]/abs(dx) if abs(dx)>1e-8 else float('inf'),
                   half[1]/abs(dy) if abs(dy)>1e-8 else float('inf'))
        distance = edge+radius+padding+margin
        angle = a+yaw
        p = [row['position'][0]+distance*math.cos(angle),
             row['position'][1]+distance*math.sin(angle)]
        # Ray offset at a corner need not provide full Euclidean margin.
        if distance_to_object(p,row)+1e-8 >= radius+padding+margin:
            candidates.append([*p, math.atan2(row['position'][1]-p[1],row['position'][0]-p[0])])
    return candidates


def object_corners(row):
    hx,hy=planar_half_sizes(row);yaw=yaw_of(row);c,s=math.cos(yaw),math.sin(yaw)
    x,y=row['position'][:2]
    return [[x+c*u-s*v,y+s*u+c*v]for u,v in[(-hx,-hy),(hx,-hy),(hx,hy),(-hx,hy)]]


def object_gap(a,b):
    """Minimum XY distance between conservative box projections, zero if intersecting."""
    pa,pb=object_corners(a),object_corners(b)
    separated=False
    for polygon in(pa,pb):
        for u,v in zip(polygon,polygon[1:]+polygon[:1]):
            axis=[u[1]-v[1],v[0]-u[0]]
            sa=[sum(x*y for x,y in zip(p,axis))for p in pa]
            sb=[sum(x*y for x,y in zip(p,axis))for p in pb]
            if max(sa)<min(sb) or max(sb)<min(sa):separated=True
    if not separated:return 0.
    return min([distance_to_object(p,b)for p in pa]+[distance_to_object(p,a)for p in pb])


def resolve_query(objects,parsed,near_distance):
    """Ground class + explicit shelf-row reference + metric relation, never Prim-name matching."""
    candidates=[r for r in objects if r['class_name']==parsed['object_class']]
    audit=dict(relation=parsed['relation'],reference_objects=[],candidate_relations=[])
    if parsed['relation']=='near_shelf_row':
        refs=[r for r in objects if r['class_name']=='shelf' and r.get('row_index')==parsed['reference']['row_index']]
        audit.update(reference_objects=refs,near_surface_distance_m=near_distance)
        if not refs:return [],audit,'REFERENCE_NOT_FOUND'
        # One explicit aggregate object per shelf row; duplicate labels require correction.
        if len(refs)!=1:return [],audit,'AMBIGUOUS_REFERENCE'
        matches=[]
        for row in candidates:
            gap=object_gap(row,refs[0]);audit['candidate_relations'].append(dict(object_id=row['object_id'],surface_gap_m=gap,matched=gap<=near_distance))
            if gap<=near_distance:matches.append(row)
        candidates=matches
    if not candidates:return [],audit,'NOT_FOUND'
    if parsed['selection']=='unique' and len(candidates)!=1:
        audit['clarification']='找到多个匹配物体，请补充最近的、最远的，或更具体的位置'
        return candidates,audit,'AMBIGUOUS'
    return candidates,audit,None


def rank_reachable(ranked,selection):
    """Shortest safe sampled approach per object, then nearest/farthest among objects."""
    best={}
    for row in sorted(ranked,key=lambda r:(r[0],r[1],r[2])):best.setdefault(row[1],row)
    return sorted(best.values(),key=lambda r:((-r[0] if selection=='farthest_reachable' else r[0]),r[1],r[2]))


def proximity_speed_limit(objects,robot_xy,policy,maximum,previous=None):
    """Semantic maneuver cap via native SpeedLimit, with distance hysteresis."""
    props=[r for r in objects if r['class_name']in('box','pallet','cart','barrier')]
    if not props:return policy['near_speed_m_s'],None,'no_active_prop_observations'
    distance=min(distance_to_object(robot_xy,r)for r in props)
    boundary=policy['release_distance_m'] if previous==policy['near_speed_m_s'] else policy['near_distance_m']
    near=distance<boundary
    return (policy['near_speed_m_s'] if near else maximum),distance,('near_registered_prop'if near else'free_cruise')


def native_speed_limit_value(limit,maximum):
    # DWB ignores absolute limits >= base_max_speed_xy; NO_SPEED_LIMIT=0 restores base caps.
    return 0. if limit>=maximum else float(limit)


def sample_path(points, step=.025):
    if not points:
        return
    yield points[0]
    for a,b in zip(points,points[1:]):
        n = max(1,math.ceil(math.dist(a,b)/step))
        for i in range(1,n+1):
            yield [a[0]+(b[0]-a[0])*i/n,a[1]+(b[1]-a[1])*i/n]


def path_length(points):
    return sum(math.dist(a,b) for a,b in zip(points,points[1:]))


def grid_cost(grid, point):
    """nav2_msgs/Costmap, supports rotated metadata origin."""
    origin=grid.metadata.origin; q=origin.orientation
    yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    dx,dy=point[0]-origin.position.x,point[1]-origin.position.y
    c,s=math.cos(yaw),math.sin(yaw)
    x=math.floor((c*dx+s*dy)/grid.metadata.resolution)
    y=math.floor((-s*dx+c*dy)/grid.metadata.resolution)
    if not(0<=x<grid.metadata.size_x and 0<=y<grid.metadata.size_y):
        return 255
    return grid.data[y*grid.metadata.size_x+x]
