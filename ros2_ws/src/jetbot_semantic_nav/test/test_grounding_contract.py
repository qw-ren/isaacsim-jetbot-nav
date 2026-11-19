import unittest
from jetbot_semantic_nav.grounding_contract import check_choice
class ContractTests(unittest.TestCase):
    def setUp(self):
        self.objects=[dict(object_id='near',class_name='box',position=[1.,0.,0.],dimensions=[.5]*3),dict(object_id='far',class_name='box',position=[4.,0.,0.],dimensions=[.5]*3)]
    def test_far_command_never_accepts_near_neural_choice(self):
        self.assertIsNotNone(check_choice(self.objects,[0.,0.],[0,0,2,1],'near'))
        self.assertIsNone(check_choice(self.objects,[0.,0.],[0,0,2,1],'far'))
    def test_no_navigation_and_ambiguous_unique_rejected(self):
        self.assertIsNotNone(check_choice(self.objects,[0.,0.],[0,0,1,0],'near'))
        self.assertIsNotNone(check_choice(self.objects,[0.,0.],[0,0,0,1],'near'))
    def test_missing_reference_and_wrong_category_rejected(self):
        self.assertIsNotNone(check_choice(self.objects,[0.,0.],[0,2,1,1],'near'))
        self.assertIsNotNone(check_choice(self.objects,[0.,0.],[1,0,1,1],'near'))
