"""Spatial hypotheses from measured visual surface points, never object boxes."""
import copy
import math
from atongmu_perception.planar_map import PlanarObjectStore


class VisualMap:
    def __init__(self, min_confidence=.65, association_m=.45, min_hits=3,
                 memory_sim_s=45., memory_wall_s=90., active_s=5.):
        if not (.5 <= min_confidence <= 1 and 0 < association_m <= 1 and
                type(min_hits) is int and min_hits >= 2 and
                0 < active_s <= memory_sim_s <= 120 and 0 < memory_wall_s <= 180):
            raise ValueError('Invalid visual map policy')
        self.raw = PlanarObjectStore(max_age=active_s)
        self.confidence, self.radius, self.min_hits = min_confidence, association_m, min_hits
        self.memory_sim, self.memory_wall, self.active_s = memory_sim_s, memory_wall_s, active_s
        self.rows = {}; self.history = {}; self.aliases = {}; self.counter = 0; self.clock = None; self.fault = False
        self.batches = 0; self.filtered = 0
        self.retired_sessions = set()

    def expire(self, now, wall):
        if not all(type(v) in (int,float) and math.isfinite(v) for v in (now,wall)):
            raise ValueError('Invalid map time')
        if self.clock is not None and now < self.clock - 1e-6:
            self.rows.clear(); self.history.clear(); self.aliases.clear(); self.fault = True
        self.clock = now
        if self.fault:
            raise ValueError('Clock reset: restart visual map session')
        self.rows = {k:r for k,r in self.rows.items()
                     if -.1 <= now-r['last_seen'] <= self.memory_sim and
                     0 <= wall-r['_received'] <= self.memory_wall}
        self.aliases = {k:v for k,v in self.aliases.items() if v in self.rows}

    def update(self, packet, now, wall, observer_xy):
        if len(observer_xy) != 2 or not all(math.isfinite(x) for x in observer_xy):
            raise ValueError('Invalid acquisition-time observer pose')
        # Validate the complete packet before committing spatial associations.
        for obj in packet.get('objects', []):
            distance = obj.get('range_m')
            if type(distance) not in (int,float) or not math.isfinite(distance) or not 0 <= distance <= 12:
                raise ValueError('Invalid measured range')
        self.expire(now, wall)
        previous = self.raw.session
        if packet.get('session_id') in self.retired_sessions:
            raise ValueError('Delayed packet from retired visual session')
        self.raw.update(packet, now, wall)
        if previous != self.raw.session:
            if previous is not None:self.retired_sessions.add(previous)
            self.rows.clear(); self.history.clear(); self.aliases.clear(); self.counter = 0
        used = set()
        for obj in packet['objects']:
            if (obj['confidence'] < self.confidence or obj['class_name'] not in
                    ('shelf','wall','box','pallet','barrier','sign') or
                    obj['position_reference'] != 'visible_surface'):
                self.filtered += 1; continue
            point = obj['position_xy']; track = obj['object_id']
            key = self.aliases.get(track)
            if key in used or (key and math.dist(point,self.rows[key]['position_xy']) > self.radius):
                key = None
            if key is None:
                candidates = sorted((math.dist(point,r['position_xy']),k) for k,r in self.rows.items()
                                    if k not in used and r['class_name'] == obj['class_name'] and
                                    math.dist(point,r['position_xy']) <= self.radius)
                # Reject ambiguous association rather than merging neighbouring objects.
                if len(candidates)>1 and candidates[1][0]-candidates[0][0] < .15:
                    self.filtered += 1; continue
                key = candidates[0][1] if candidates else None
            if key is None:
                self.counter += 1; key = 'vision-map:%s:%d' % (packet['session_id'], self.counter)
                self.rows[key] = dict(object_id=key,class_name=obj['class_name'],position_xy=list(point),
                    first_seen=packet['stamp'],hits=0,position_uncertainty_m=.25,
                    position_reference='visible_surface',source='vision',geometry='measured_point',
                    identity_scope='session_spatial_hypothesis',confidence=0.)
            r = self.rows[key]; delta = math.dist(point,r['position_xy'])
            r['position_xy'] = [a*.7+b*.3 for a,b in zip(r['position_xy'],point)]
            r.update(hits=r['hits']+1,last_seen=packet['stamp'],_received=wall,
                     confidence=max(r['confidence']*.95,obj['confidence']),
                     observed_from_xy=list(observer_xy),localization_method=obj['localization_method'],
                     last_track_id=track,position_uncertainty_m=max(.25,delta))
            self.aliases[track] = key; used.add(key)
        self.batches += 1
        return self.snapshot(now,wall)

    def snapshot(self, now, wall):
        self.expire(now,wall); objects=[]
        for key,r in sorted(self.rows.items()):
            if r['hits'] < self.min_hits or r['last_seen']-r['first_seen'] < .3:
                continue
            row = {k:copy.deepcopy(v) for k,v in r.items() if not k.startswith('_')}
            row['status'] = 'active' if now-r['last_seen'] <= self.active_s else 'memory'
            objects.append(row)
            self.history[key] = copy.deepcopy(row)
        # The full-session evidence map is separate from currently usable targets.
        # Bound RAM; raw measurements remain in the append-only observation log.
        if len(self.history)>4096:
            keep=sorted(self.history,key=lambda k:self.history[k]['last_seen'],reverse=True)[:4096]
            self.history={k:self.history[k] for k in keep}
        return dict(schema='jetbot.visual_object_map',version=1,source='vision',frame_id='map',
                    session_id=self.raw.session,stamp=now,objects=objects,
                    observation_batches=self.batches,filtered_observations=self.filtered,
                    geometry_complete=False,robot_truth_input=False)

    def historical_map(self,now,wall):
        current=self.snapshot(now,wall);active={r['object_id'] for r in current['objects']}
        rows=[]
        for key,r in sorted(self.history.items()):
            row=copy.deepcopy(r);row['current_observation_valid']=key in active
            if key not in active:row['status']='stale'
            rows.append(row)
        return dict(current,schema='jetbot.visual_evidence_map',objects=rows,
                    navigation_input=False,semantics='historical point hypotheses; stale rows require reobservation')


def validate_point(row):
    if (row.get('source') != 'vision' or row.get('geometry') != 'measured_point' or
            row.get('position_reference') != 'visible_surface' or
            not isinstance(row.get('object_id'),str) or not row['object_id'] or
            row.get('class_name') not in ('shelf','wall','box','pallet','barrier','sign')):
        raise ValueError('Invalid visual object')
    for field in ('position_xy','observed_from_xy'):
        if len(row[field]) != 2 or not all(type(v) in (int,float) and math.isfinite(v) for v in row[field]):
            raise ValueError('Invalid planar vector')
    if not .0 <= row['confidence'] <= 1 or not .0 < row['position_uncertainty_m'] <= 1:
        raise ValueError('Invalid confidence/uncertainty')
    if not math.isfinite(row['last_seen']):raise ValueError('Invalid last_seen')
    return copy.deepcopy(row)


def point_approaches(row, radius, padding, margin):
    x,y = row['position_xy']; ox,oy = row['observed_from_xy']
    if math.hypot(ox-x,oy-y)<.2:return []
    angle = math.atan2(oy-y,ox-x)
    stand_off = radius+padding+margin+row['position_uncertainty_m']+.02
    # Only the observed side; a point does not establish the far side of an object.
    return [[x+stand_off*math.cos(angle+d),y+stand_off*math.sin(angle+d),
             math.atan2(-math.sin(angle+d),-math.cos(angle+d))] for d in (0.,-.25,.25)]
