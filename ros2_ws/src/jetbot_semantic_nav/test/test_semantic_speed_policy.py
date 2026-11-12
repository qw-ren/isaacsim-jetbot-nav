import json
import tempfile
import unittest
from pathlib import Path
from jetbot_semantic_nav.core import proximity_speed_limit,native_speed_limit_value
from jetbot_semantic_nav.config import load


class SpeedPolicyTest(unittest.TestCase):
    def test_native_full_speed_request_clears_previous_limit(self):
        self.assertEqual(native_speed_limit_value(.8,.8),0.)
        self.assertEqual(native_speed_limit_value(.4,.8),.4)

    def test_hysteresis_missing_data_and_fixed_shelves(self):
        policy=dict(topic='/speed_limit',near_speed_m_s=.4,near_distance_m=2.,release_distance_m=2.5)
        box=dict(object_id='box',class_name='box',position=[0.,0.,.5],dimensions=[1.,1.,1.],orientation_xyzw=[0.,0.,0.,1.])
        self.assertEqual(proximity_speed_limit([box],[2.,0.],policy,.8)[0],.4)
        self.assertEqual(proximity_speed_limit([box],[2.8,0.],policy,.8,.4)[0],.4)
        self.assertEqual(proximity_speed_limit([box],[3.,0.],policy,.8,.4)[0],.8)
        self.assertEqual(proximity_speed_limit([box],[2.8,0.],policy,.8,.8)[0],.8)
        self.assertEqual(proximity_speed_limit([], [0.,0.],policy,.8)[0],.4)
        shelf=dict(box,class_name='shelf')
        self.assertEqual(proximity_speed_limit([shelf,dict(box,position=[8.,0.,.5])],[0.,0.],policy,.8)[0],.8)

    def test_config_rejects_invalid_cap_and_unsafe_transition_distance(self):
        semantic=dict(mode='ground_truth_oracle',planning_radius_m=.4,
            speed_policy=dict(topic='/speed_limit',near_speed_m_s=.4,near_distance_m=2.,release_distance_m=2.5))
        config=dict(simulation_only=True,motion_limits=dict(max_linear_m_s=.8,scan_clearance_m=.4),
            navigation=dict(semantic=semantic,parameter_overrides={s+'/ros__parameters/robot_radius':.4 for s in('global_costmap/global_costmap','local_costmap/local_costmap')}))
        with tempfile.TemporaryDirectory()as folder:
            path=Path(folder)/'CASE.json';path.write_text(json.dumps(config));self.assertEqual(load(folder)[1]['speed_policy']['near_speed_m_s'],.4)
            for key,value in [('near_speed_m_s',.9),('near_speed_m_s',0.),('near_distance_m',1.),('release_distance_m',1.9),('near_speed_m_s',float('nan'))]:
                bad=json.loads(json.dumps(config));bad['navigation']['semantic']['speed_policy'][key]=value;path.write_text(json.dumps(bad))
                with self.assertRaises(ValueError):load(folder)


if __name__=='__main__':unittest.main()
