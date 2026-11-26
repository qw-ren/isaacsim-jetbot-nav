"""Coordinate checks independent of ROS/Isaac installations."""
import math
from types import SimpleNamespace as NS
import unittest
from capture_route import transform_xy


class ScanProjectionTests(unittest.TestCase):
    def test_scan_rotation_translation(self):
        tf=NS(translation=NS(x=3.,y=4.),rotation=NS(x=0.,y=0.,z=math.sqrt(.5),w=math.sqrt(.5)))
        x,y=transform_xy(2.,0.,tf)
        self.assertAlmostEqual(x,3.)
        self.assertAlmostEqual(y,6.)

    def test_tilted_sensor_does_not_use_yaw_only(self):
        tf=NS(translation=NS(x=0.,y=0.),rotation=NS(x=0.,y=math.sqrt(.5),z=0.,w=math.sqrt(.5)))
        x,y=transform_xy(2.,1.,tf)
        self.assertAlmostEqual(x,0.)
        self.assertAlmostEqual(y,1.)


if __name__=='__main__':
    unittest.main()
