import math,unittest
from jetbot_semantic_nav.grounding_dataset import generate,label
class DatasetTests(unittest.TestCase):
    def test_split_seed_and_text_families_do_not_overlap(self):
        parts=[generate(s,80,o)for s,o in [('train',300000),('dev',500000),('test',700000)]]
        for i in range(3):
            for j in range(i):
                self.assertFalse({r['scene_seed']for r in parts[i]} & {r['scene_seed']for r in parts[j]})
                self.assertFalse({r['text']for r in parts[i]} & {r['text']for r in parts[j]})
    def test_unique_ambiguity_and_extreme_distance_semantics(self):
        objects=[dict(object_id='a',class_name='box',position=[1.,0.,0.],dimensions=[.5]*3),dict(object_id='b',class_name='box',position=[4.,0.,0.],dimensions=[.5]*3)]
        self.assertEqual(label(objects,[0.,0.],[0,0,0])[0],'AMBIGUOUS')
        self.assertEqual(label(objects,[0.,0.],[0,0,1])[0],'a')
        self.assertEqual(label(objects,[0.,0.],[0,0,2])[0],'b')
        self.assertEqual(label(objects,[0.,0.],[0,2,1])[0],'NOT_FOUND')
    def test_no_navigation_intent_overrides_present_target_without_erasing_class(self):
        objects=[dict(object_id='a',class_name='box',position=[1.,0.,0.],dimensions=[.5]*3)]
        self.assertEqual(label(objects,[0.,0.],[0,0,1,1])[0],'a')
        target,eligible=label(objects,[0.,0.],[0,0,1,0])
        self.assertEqual(target,'NOT_FOUND');self.assertEqual(eligible,[False])

    def test_positive_questions_contain_negation_character_without_no_motion_label(self):
        rows=generate('train',400,300000)
        self.assertTrue(any('不' in r['text'] and r['slots'][3]==1 for r in rows))
        self.assertTrue(any('不' in r['text'] and r['slots'][3]==0 for r in rows))

    def test_generated_target_exists_and_geometry_finite(self):
        for r in generate('train',100,0):
            self.assertTrue(r['target']in ('NOT_FOUND','AMBIGUOUS') or r['target'] in {o['object_id']for o in r['objects']})
            self.assertEqual(len(r['eligible']),len(r['objects']))
            self.assertTrue(all(math.isfinite(v)for o in r['objects']for v in o['position']+o['dimensions']))
