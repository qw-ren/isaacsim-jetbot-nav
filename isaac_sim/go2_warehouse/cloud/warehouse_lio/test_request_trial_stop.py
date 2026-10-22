import unittest
from pathlib import Path
from request_trial_stop import ROOT, supervisor_case, foreground_targets

class StopSelection(unittest.TestCase):
    def setUp(self):self.case=ROOT/'moving_test'
    def test_accept_exact_supervisor_only(self):
        argv=['bash',str(self.case/'source/warehouse_lio/run_nav_trial.sh'),'__run',str(self.case)]
        self.assertEqual(supervisor_case(argv),self.case)
        self.assertIsNone(supervisor_case(['timeout',*argv]))
        self.assertIsNone(supervisor_case(['bash','-c',' '.join(argv)]))
    def test_reject_other_roots_and_scripts(self):
        self.assertIsNone(supervisor_case(['bash','/tmp/run_nav_trial.sh','__run',str(self.case)]))
        self.assertIsNone(supervisor_case(['bash','/tmp/x/source/warehouse_lio/run_nav_trial.sh','__run','/tmp/x']))
    def test_async_groups_and_unrelated_processes_untouched(self):
        worker=dict(pid=1,group=9,argv=['/kit/python/bin/python3',str(self.case/'source/check.py')])
        recorder=dict(pid=2,group=2,argv=['/kit/python/bin/python3',str(self.case/'source/record.py')])
        unrelated=dict(pid=3,group=9,argv=['python3','/tmp/unrelated.py'])
        self.assertEqual(foreground_targets([worker,recorder,unrelated],9,self.case),[worker])
    def test_ambiguity_is_visible_to_caller(self):
        records=[dict(pid=i,group=9,argv=['python3',str(self.case/'source/check.py')]) for i in [1,2]]
        self.assertEqual(len(foreground_targets(records,9,self.case)),2)

if __name__=='__main__':unittest.main()
