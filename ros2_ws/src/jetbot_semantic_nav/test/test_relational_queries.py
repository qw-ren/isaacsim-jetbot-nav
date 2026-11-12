import math
import unittest
from jetbot_semantic_nav.core import parse_command,resolve_query,rank_reachable,object_gap,validate_observation


def row(name,kind,x,y,sizes=(1.,1.,1.),**extra):
    return dict(object_id=name,class_name=kind,source='ground_truth',confidence=1.,
                position=[x,y,.5],dimensions=list(sizes),orientation_xyzw=[0.,0.,0.,1.],**extra)


class RelationalQueryTest(unittest.TestCase):
    def test_grammar_and_reject_unsupported_composition(self):
        for text in ['找最远的纸箱','去离我最远的箱子旁边','前往最远的box']:
            self.assertEqual(parse_command(text)['selection'],'farthest_reachable')
        p=parse_command('走到第二排货架旁边的那个纸箱')
        self.assertEqual((p['object_class'],p['reference']['row_index'],p['selection']),('box',2,'unique'))
        self.assertEqual(parse_command('到第十二排货架附近最远的托盘')['reference']['row_index'],12)
        for text in ['不要找最远的纸箱','去第二排货架和推车旁边的纸箱','去第零排货架旁的纸箱',
                     '去第百排货架旁的纸箱','去纸箱然后去推车','到第二排货架左边的纸箱']:
            with self.assertRaises(ValueError):parse_command(text)

    def test_surface_relation_unique_ambiguous_and_missing_reference(self):
        shelf=row('shelf2','shelf',0.,0.,(1.,10.,3.),row_index=2)
        near=row('near','box',1.7,3.);far=row('far','box',5.,0.)
        p=parse_command('走到第二排货架旁边的那个纸箱')
        found,audit,error=resolve_query([shelf,near,far],p,1.5)
        self.assertIsNone(error);self.assertEqual([r['object_id']for r in found],['near'])
        self.assertEqual(audit['reference_objects'][0]['row_index'],2)
        self.assertEqual(resolve_query([near],p,1.5)[2],'REFERENCE_NOT_FOUND')
        other=row('also_near','box',-1.7,0.)
        self.assertEqual(resolve_query([shelf,near,other],p,1.5)[2],'AMBIGUOUS')
        self.assertEqual(resolve_query([shelf,dict(shelf,object_id='duplicate'),near],p,1.5)[2],'AMBIGUOUS_REFERENCE')
        p=parse_command('到第二排货架旁边最近的纸箱')
        self.assertIsNone(resolve_query([shelf,near,other],p,1.5)[2])
        with self.assertRaises(ValueError):validate_observation(dict(near,row_index=2))

    def test_farthest_uses_each_objects_shortest_approach(self):
        # A long alternate approach must not make the near object look farthest.
        paths=[(2.,'near',[0.,0.,0.],{},[]),(100.,'near',[1.,0.,0.],{},[]),
               (12.,'far',[0.,0.,0.],{},[]),(10.,'far',[1.,0.,0.],{},[])]
        result=rank_reachable(paths,'farthest_reachable')
        self.assertEqual([(r[1],r[0])for r in result],[('far',10.),('near',2.)])
        self.assertEqual(rank_reachable(paths,'nearest_reachable')[0][1],'near')

    def test_intersection_without_enclosed_corners(self):
        a=row('a','shelf',0.,0.,(10.,1.,2.));b=row('b','shelf',0.,0.,(1.,10.,2.))
        self.assertEqual(object_gap(a,b),0.)
        b['position'][0]=7.;self.assertAlmostEqual(object_gap(a,b),1.5)
        b['orientation_xyzw']=[0.,0.,math.sin(.3),math.cos(.3)]
        self.assertAlmostEqual(object_gap(a,b),object_gap(b,a))


if __name__=='__main__':unittest.main()
