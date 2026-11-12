"""Derive explicit shelf-row catalog from existing prior geometry; no scene generation."""
import argparse
import json
from pathlib import Path

p=argparse.ArgumentParser();p.add_argument('--base',required=True,type=Path)
p.add_argument('--collision-boxes',required=True,type=Path);p.add_argument('--output',required=True,type=Path);a=p.parse_args()
cfg=json.loads(a.base.read_text());boxes=json.loads(a.collision_boxes.read_text());groups=[]
# Site convention fixed in map coordinates, westernmost row first. Only this known aisle bank.
for index,x in enumerate([-15.413,-10.46,-5.5,-.55],1):
    paths=set()
    for row in boxes:
        cx=(row['min'][0]+row['max'][0])/2;cy=(row['min'][1]+row['max'][1])/2
        if 'SM_Rack' in row['path'] and abs(cx-x)<.15 and 8.5<cy<25.2:
            paths.add('/'.join(row['path'].split('/')[:4]))
    if len(paths)!=7:raise ValueError('Unexpected warehouse shelf geometry; review row registration')
    groups.append(dict(object_id='/SemanticCatalog/shelf_row_'+str(index),class_name='shelf',
        row_index=index,label='第'+str(index)+'排货架',prim_paths=sorted(paths)))
cfg['scope']='Independent relational semantic query validation; frozen navigation overrides preserved'
cfg['semantic'].update(registered_groups=groups,near_shelf_distance_m=1.5,
    row_numbering='Explicit site catalog: these four aisle rows numbered west-to-east, increasing map X; not robot-relative')
# Keep the test paths distinguishable: center-line near props otherwise occlude native paths to unseen far props.
cfg['semantic']['fixtures']=[r for r in cfg['semantic']['fixtures']if r['class_name']!='pallet' and r['id']!='cart_far']
next(r for r in cfg['semantic']['fixtures']if r['id']=='box_near')['center']=[-13.9,12.,0.]
cfg['semantic']['fixtures'].append(dict(id='box_relation',class_name='box',asset_kind='box',center=[-8.4,12.,0.]))
with a.output.open('x')as f:json.dump(cfg,f,ensure_ascii=False,indent=2);f.write('\n')
print('Registered 4 original shelf rows and one extra native demonstration box')
