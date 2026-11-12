import math
import unittest
from jetbot_semantic_nav.core import (parse_command, ObjectStore, transform_object,
    approaches,distance_to_object,sample_path,validate_observation)


def obj():
    return dict(object_id='/World/cart_1',class_name='cart',source='ground_truth',confidence=1.,
        position=[2.,3.,.5],orientation_xyzw=[0.,0.,0.,1.],dimensions=[1.,.6,1.])


class SemanticCoreTest(unittest.TestCase):
    def test_chinese_and_ambiguous(self):
        for text,kind in [('到最近的推车那里','cart'),('到最近的纸箱那里','box'),('到最近的托盘那里','pallet'),
                          ('去推车旁边','cart'),('去纸箱旁边','box'),('去货架附近','shelf'),('到标牌那里','sign')]:
            self.assertEqual(parse_command(text)['object_class'],kind)
        for text in ['不要去纸箱旁边','去推车和纸箱旁边','去人那里','到纸箱旁边然后回家','推车']:
            with self.assertRaises(ValueError):parse_command(text)

    def test_store_update_and_expiry(self):
        store=ObjectStore(3.);packet=dict(version=1,frame_id='map',stamp=10.,objects=[obj()])
        store.update(packet,10.1,100.)
        row=obj();row['position'][0]=4.;packet.update(stamp=11.,objects=[row]);store.update(packet,11.,101.)
        rows=store.active(11.1,101.1);self.assertEqual(len(rows),1);self.assertEqual(rows[0]['position'][0],4.)
        self.assertEqual(store.active(15.,105.),[])
        packet['stamp']=10.
        with self.assertRaises(ValueError):store.update(packet,15.,105.)

    def test_numerical_hygiene(self):
        for field,value in [('position',[float('nan'),0,0]),('orientation_xyzw',[0,0,0,0]),('dimensions',[1,-1,1])]:
            row=obj();row[field]=value
            with self.assertRaises(ValueError):validate_observation(row)

    def test_nonidentity_registration(self):
        row=transform_object(obj(),[10.,-2.,0.],[0.,0.,math.sin(math.pi/4),math.cos(math.pi/4)])
        self.assertAlmostEqual(row['position'][0],7.);self.assertAlmostEqual(row['position'][1],0.)

    def test_approach_rotated_box_and_full_margin(self):
        row=obj();row['orientation_xyzw']=[0.,0.,math.sin(.4),math.cos(.4)]
        candidates=approaches(row,.4,.01,.2,12)
        self.assertGreaterEqual(len(candidates),4)
        for x,y,yaw in candidates:
            self.assertGreaterEqual(distance_to_object([x,y],row),.61-1e-8)
            self.assertAlmostEqual(yaw,math.atan2(3-y,2-x))

    def test_path_sampling_detects_between_waypoint_crossing(self):
        row=obj();points=list(sample_path([[0.,3.],[4.,3.]]))
        self.assertTrue(any(distance_to_object(p,row)<.4 for p in points))
        self.assertLessEqual(max(math.dist(a,b)for a,b in zip(points,points[1:])),.025001)


if __name__=='__main__':unittest.main()
