"""Frame/velocity and fault regression cases for the navigation boundary."""
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from lio_nav_math import LioNavConverter, align_up, pose_in_map, clearance_in_map
from imu_noise_model import ImuNoiseModel


class NavigationBoundaryTests(unittest.TestCase):
    def test_map_guard_does_not_assume_odom_is_map(self):
        pose = [0.,0.,.4,0.,0.]
        transform = [1.,2.,np.pi/2]
        np.testing.assert_allclose(pose_in_map(pose,transform),[1.,2.,.4,np.pi/2,0.])
        boxes = np.array([[[.8,3.5],[1.2,4.]]])
        self.assertAlmostEqual(clearance_in_map(pose,boxes,transform,.8,0.),.3)
    def test_sensor_only_gravity_levels_initial_lio_frame(self):
        gravity = np.array([-.15, .02, 9.81])
        rotation = align_up(gravity)
        np.testing.assert_allclose(rotation.apply(gravity), [0.,0.,np.linalg.norm(gravity)], atol=1e-12)
        c = LioNavConverter([2.,-1.,.4], .2, gravity)
        np.testing.assert_allclose(c.rotation.apply(gravity), [0.,0.,np.linalg.norm(gravity)], atol=1e-12)
    def test_rotated_anchor_keeps_body_forward_velocity(self):
        c = LioNavConverter([2., -1., .4], np.pi/2)
        self.assertIsNone(c.update(1., [0., 0., 0.], [0., 0., 0., 1.]))
        row = c.update(1.1, [.08, 0., 0.], [0., 0., 0., 1.])
        np.testing.assert_allclose(row['xyz'], [2., -.92, .4], atol=1e-12)
        np.testing.assert_allclose(row['linear_body'], [.8, 0., 0.], atol=1e-12)

    def test_rotation_velocity_and_quaternion_sign(self):
        c = LioNavConverter([0., 0., .4])
        c.update(1., [0., 0., 0.], [0., 0., 0., 1.])
        q = -Rotation.from_euler('z', .06).as_quat()
        row = c.update(1.1, [0., 0., 0.], q)
        np.testing.assert_allclose(row['angular_body'], [0., 0., .6], atol=1e-12)

    def test_reset_gap_and_jump_rejected(self):
        for stamp, position in [(1., [0., 0., 0.]), (.9, [0., 0., 0.]),
                                (1.5, [0., 0., 0.]), (1.1, [5., 0., 0.])]:
            c = LioNavConverter([0., 0., .4])
            c.update(1., [0., 0., 0.], [0., 0., 0., 1.])
            with self.assertRaises(ValueError):
                c.update(stamp, position, [0., 0., 0., 1.])

    def test_noise_is_seeded_and_disabled_profile_is_identity(self):
        a, b = ImuNoiseModel(5), ImuNoiseModel(5)
        ideal = ImuNoiseModel(5, enabled=False)
        for i in range(100):
            stamp = i*.005
            x = a.update(stamp, [0., 0., 9.81], [0., 0., .1])
            y = b.update(stamp, [0., 0., 9.81], [0., 0., .1])
            for p, q in zip(x, y):
                np.testing.assert_array_equal(p, q)
            p, q, _, _ = ideal.update(stamp, [0., 0., 9.81], [0., 0., .1])
            np.testing.assert_array_equal(p, [0., 0., 9.81])
            np.testing.assert_array_equal(q, [0., 0., .1])


if __name__ == '__main__':
    unittest.main()
