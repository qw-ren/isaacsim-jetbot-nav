import copy
import math
from types import SimpleNamespace as NS
import unittest
from jetbot_semantic_nav.core import ObjectStore,grid_cost,approaches,distance_to_object
from test_core import obj


class ProtocolGeometryTest(unittest.TestCase):
    def test_atomic_duplicate_id_and_stale_batches(self):
        store=ObjectStore(5.)
        batch=dict(version=1,frame_id='map',stamp=10.,objects=[obj(),obj()])
        with self.assertRaises(ValueError):store.update(batch,10.,100.)
        self.assertFalse(store.objects)
        batch['objects']=[obj()];store.update(batch,10.,100.)
        newer=copy.deepcopy(batch);newer['stamp']=11.;newer['objects'][0]['source']='vision'
        with self.assertRaises(ValueError):store.update(newer,11.,101.)
        self.assertEqual(store.objects[obj()['object_id']]['source'],'ground_truth')

    def test_rotated_costmap_and_unknown_rejection(self):
        origin=NS(position=NS(x=10.,y=20.),orientation=NS(x=0.,y=0.,z=math.sin(math.pi/4),w=math.cos(math.pi/4)))
        grid=NS(metadata=NS(origin=origin,resolution=1.,size_x=2,size_y=2),data=[0,253,254,255])
        self.assertEqual(grid_cost(grid,[9.5,20.5]),0)
        self.assertEqual(grid_cost(grid,[9.5,21.5]),253)
        self.assertEqual(grid_cost(grid,[8.5,20.5]),254)
        self.assertEqual(grid_cost(grid,[10.5,20.5]),255)

    def test_two_close_objects_reject_approach_inside_neighbor(self):
        first=obj();second=obj();second['object_id']='other';second['position'][0]+=1.3
        points=approaches(first,.4,.01,.25,8)
        self.assertTrue(any(distance_to_object(p,second)<.41 for p in points))
        self.assertTrue(any(distance_to_object(p,second)>.66 for p in points))

    def test_expiration_when_sim_pauses_but_wall_advances(self):
        store=ObjectStore(5.);store.update(dict(version=1,frame_id='map',stamp=10.,objects=[obj()]),10.,100.)
        self.assertEqual(store.active(10.,106.),[])

    def test_tilted_tall_box_projects_height_into_ground_plane(self):
        row=obj();row['position']=[0.,0.,0.];row['dimensions']=[.2,.2,2.]
        row['orientation_xyzw']=[0.,math.sin(math.pi/4),0.,math.cos(math.pi/4)]
        self.assertAlmostEqual(distance_to_object([.9,0.],row),0.)
        for p in approaches(row,.4,.01,.6,8):
            self.assertGreaterEqual(distance_to_object(p,row),1.01-1e-8)

    def test_clock_rounding_or_dds_order_does_not_delete_fresh_object(self):
        store=ObjectStore(5.)
        store.update(dict(version=1,frame_id='map',stamp=10.0000000003,objects=[obj()]),10.,100.)
        self.assertEqual(len(store.active(10.,100.)),1)
        self.assertEqual(len(store.active(10.05,100.05)),1)
        with self.assertRaises(ValueError):
            store.update(dict(version=1,frame_id='map',stamp=10.2,objects=[obj()]),10.,100.)


if __name__=='__main__':unittest.main()
