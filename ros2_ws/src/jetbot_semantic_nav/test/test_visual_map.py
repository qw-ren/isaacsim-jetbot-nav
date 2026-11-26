import copy,unittest,math
from jetbot_semantic_nav.planar_objects import VisualMap,point_approaches,validate_point
from jetbot_semantic_nav.core import distance_to_object


def packet(sequence,stamp,rows=None,session='test'):
    if rows is None:rows=[dict(track_id=1,class_name='pallet',confidence=.9,position_xy=[3.,0.],
        position_reference='visible_surface',range_m=3.,localization_method='depth')]
    rows=copy.deepcopy(rows)
    for r in rows:r['object_id']='vision:%s:%d'%(session,r['track_id'])
    return dict(schema='atongmu.planar_observations',version=1,source='vision',frame_id='map',
                session_id=session,sequence=sequence,stamp=stamp,objects=rows)


class VisualMapTests(unittest.TestCase):
    def confirmed(self):
        s=VisualMap()
        for n in range(3):r=s.update(packet(n,1.+n*.2),1.+n*.2,10.+n*.2,[0.,0.])
        return s,r

    def test_confirmation_surface_geometry_and_reassociation(self):
        s,r=self.confirmed();self.assertEqual(len(r['objects']),1);o=r['objects'][0]
        self.assertNotIn('dimensions',o);self.assertNotIn('position',o);validate_point(o)
        p=packet(3,1.6);p['objects'][0].update(track_id=9,object_id='vision:test:9')
        r=s.update(p,1.6,10.6,[0.,0.]);self.assertEqual(r['objects'][0]['object_id'],o['object_id'])
        self.assertFalse(r['geometry_complete'])

    def test_empty_frame_memory_then_both_clock_expirations(self):
        s,r=self.confirmed();self.assertEqual(len(s.update(packet(3,2.,[]),2.,11.,[0.,0.])['objects']),1)
        self.assertEqual(s.snapshot(8.,17.)['objects'][0]['status'],'memory')
        self.assertFalse(s.snapshot(47.,56.)['objects'])
        s,r=self.confirmed();self.assertFalse(s.snapshot(1.5,101.)['objects'])

    def test_history_retained_but_never_used_as_current_target(self):
        s,r=self.confirmed();key=r['objects'][0]['object_id']
        self.assertFalse(s.snapshot(47.,56.)['objects'])
        history=s.historical_map(47.,56.);self.assertFalse(history['navigation_input'])
        self.assertEqual(history['objects'][0]['object_id'],key)
        self.assertEqual(history['objects'][0]['status'],'stale')
        self.assertFalse(history['objects'][0]['current_observation_valid'])

    def test_session_reset_and_clock_rollback(self):
        s,r=self.confirmed();self.assertFalse(s.update(packet(0,2.,[],session='new'),2.,11.,[0.,0.])['objects'])
        with self.assertRaises(ValueError):s.update(packet(4,2.1),2.1,11.1,[0.,0.])
        with self.assertRaises(ValueError):s.snapshot(1.,12.)
        with self.assertRaises(ValueError):s.snapshot(3.,13.)

    def test_atomic_bad_batch_and_wrong_sources(self):
        s,r=self.confirmed();before=copy.deepcopy(s.rows)
        p=packet(3,1.6);p['objects'].append(dict(p['objects'][0],object_id='bad'))
        with self.assertRaises(ValueError):s.update(p,1.6,10.6,[0.,0.])
        self.assertEqual(before,s.rows)
        p=packet(3,1.6);p['frame_id']='chassis'
        with self.assertRaises(ValueError):s.update(p,1.6,10.6,[0.,0.])
        p=packet(3,1.6);p['source']='ground_truth'
        with self.assertRaises(ValueError):s.update(p,1.6,10.6,[0.,0.])

    def test_dynamic_ground_and_low_confidence_not_static(self):
        s=VisualMap();base=packet(0,1.)['objects'][0]
        rows=[dict(base,track_id=1,class_name='person'),dict(base,track_id=2,class_name='cart'),
              dict(base,track_id=3,confidence=.4),dict(base,track_id=4,localization_method='ground',position_reference='ground_contact_estimate')]
        for n in range(3):r=s.update(packet(n,1.+n*.2,rows),1.+n*.2,10.+n*.2,[0.,0.])
        self.assertFalse(r['objects'])

    def test_two_close_tracks_not_collapsed_in_same_frame(self):
        s=VisualMap();base=packet(0,1.)['objects'][0]
        rows=[base,dict(base,track_id=2,position_xy=[3.1,0.])]
        for n in range(3):r=s.update(packet(n,1.+n*.2,rows),1.+n*.2,10.+n*.2,[0.,0.])
        self.assertEqual(len(r['objects']),2)

    def test_point_approaches_keep_observed_side_and_uncertainty(self):
        _,r=self.confirmed();o=r['objects'][0]
        for goal in point_approaches(o,.4,.01,.8):
            self.assertLess(goal[0],3.)
            self.assertGreaterEqual(distance_to_object(goal,o),1.21)
