import copy,unittest
from jetbot_semantic_nav.goal_bindings import validate_bindings,bound_goal
class BindingTests(unittest.TestCase):
    def setUp(self):
        self.obj=dict(object_id='box_A',position=[1.,2.,.5],dimensions=[.7,.5,1.],orientation_xyzw=[0.,0.,0.,1.])
        self.row=dict(object_id='box_A',goal=[1.,.5,1.57],object_position=self.obj['position'],object_dimensions=self.obj['dimensions'],object_orientation_xyzw=self.obj['orientation_xyzw'])
        self.doc=dict(version=1,frame_id='map',scene_id='scene_A',objects=[self.row])
    def test_returns_bound_free_space_goal_not_object_center(self):
        self.assertEqual(bound_goal(validate_bindings(self.doc,'scene_A'),self.obj),[1.,.5,1.57])
    def test_wrong_scene_and_duplicate_id_rejected(self):
        with self.assertRaises(ValueError):validate_bindings(self.doc,'scene_B')
        self.doc['objects'].append(copy.deepcopy(self.row))
        with self.assertRaises(ValueError):validate_bindings(self.doc,'scene_A')
    def test_moved_rotated_and_resized_object_invalidates_goal(self):
        b=validate_bindings(self.doc,'scene_A')
        for field,value in [('position',[1.2,2.,.5]),('dimensions',[1.,.5,1.]),('orientation_xyzw',[0.,0.,1.,0.])]:
            o=copy.deepcopy(self.obj);o[field]=value
            with self.assertRaises(ValueError):bound_goal(b,o)
    def test_unbound_and_nonfinite_rejected(self):
        with self.assertRaises(ValueError):bound_goal({},self.obj)
        self.row['goal'][0]=float('nan')
        with self.assertRaises(ValueError):validate_bindings(self.doc,'scene_A')
if __name__=='__main__':unittest.main()
