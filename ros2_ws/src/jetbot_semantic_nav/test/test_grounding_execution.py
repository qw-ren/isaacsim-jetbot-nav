import unittest
from jetbot_semantic_nav.grounding_execution import permitted


class ExecutionTests(unittest.TestCase):
    def test_scoped_simulation_execution(self):
        runtime = dict(simulation_only=True, operator_enabled=True)
        config = dict(backend='pretrained_chinese_encoder', simulation_execution=dict(
            enabled=True, commands={'到第二排货架旁的托盘': 'target'}))
        self.assertTrue(permitted(runtime, config, '到第二排货架旁的托盘', 'target'))
        self.assertFalse(permitted(runtime, config, '到第二排货架旁的托盘', 'other'))
        self.assertFalse(permitted(runtime, config, '去别的地方', 'target'))
        self.assertFalse(permitted(dict(runtime, simulation_only=False), config, '到第二排货架旁的托盘'))
        self.assertFalse(permitted(dict(runtime, operator_enabled=False), config, '到第二排货架旁的托盘'))
        self.assertFalse(permitted(runtime, dict(backend='pretrained_chinese_encoder'), '到第二排货架旁的托盘'))
