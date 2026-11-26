import json,subprocess,sys,tempfile,unittest
from pathlib import Path


class VisualConfigTests(unittest.TestCase):
    def test_exclusive_config_preserves_nav_and_removes_truth_bindings(self):
        with tempfile.TemporaryDirectory()as folder:
            root=Path(folder);(root/'perception/configs').mkdir(parents=True)
            (root/'perception/configs/perception.yaml').write_text('detector:\n  weights: old.pt\n  classes: [shelf, wall, box, pallet, barrier, cart, sign, person]\n')
            # Fake bytes are configuration fixtures; no model is imported or executed.
            (root/'weights.pt').write_bytes(b'configuration fixture only')
            (root/'grounding.pt').write_bytes(b'configuration fixture only')
            c=dict(semantic=dict(topics={},mode='ground_truth_oracle',fixtures=[{'id':'old'}],registered_groups=[{'id':'old'}],object_grounding={'bindings':{'old':'truth'}}),extensions={'probe':True},visualization={'record_video':True},parameter_overrides={'frozen':123})
            (root/'base.json').write_text(json.dumps(c))
            cmd=[sys.executable,str(Path(__file__).parents[1]/'tools/make_visual_map_config.py'),'--base',str(root/'base.json'),'--output',str(root/'new'),'--vision-root',str(root),'--vision-python',sys.executable,'--weights',str(root/'weights.pt'),'--language-model',str(root),'--language-deps',str(root),'--vision-deps',str(root)]
            subprocess.run(cmd,check=True,capture_output=True)
            result=json.loads((root/'new/NAVIGATION.json').read_text());s=result['semantic']
            self.assertEqual(result['parameter_overrides'],c['parameter_overrides'])
            self.assertEqual(s['mode'],'vision');self.assertEqual(s['object_geometry'],'measured_point')
            self.assertFalse(s['fixtures']);self.assertFalse(s['registered_groups']);self.assertFalse(s['registered_prims'])
            self.assertNotIn('bindings',s['object_grounding']);self.assertNotIn('simulation_execution',s['object_grounding'])
            self.assertTrue(result['extensions']['depth']);self.assertFalse(result['visualization']['record_video'])
            self.assertEqual(s['topics']['observations'],'/perception/objects_2d')
            (root/'weights.pt').unlink();cmd[cmd.index('--output')+1]=str(root/'rejected')
            self.assertNotEqual(subprocess.run(cmd,capture_output=True).returncode,0)
            self.assertFalse((root/'rejected').exists())
