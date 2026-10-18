"""Preserve original SLAM files; produce navigation YAML with faithful unknown cells."""
import argparse,json,hashlib
from pathlib import Path
import yaml,numpy as np
from PIL import Image
p=argparse.ArgumentParser();p.add_argument('--map-dir',type=Path,required=True);p.add_argument('--grid',type=Path,required=True);a=p.parse_args();source=a.map_dir/'map.yaml';target=a.map_dir/'map_nav.yaml'
if target.exists():raise SystemExit('Navigation metadata exists; refusing overwrite')
cfg=yaml.safe_load(source.read_text());image=a.map_dir/cfg['image'];gray=np.asarray(Image.open(image).convert('L'));saved=np.load(a.grid);expected=saved['grid'];prob=(255-gray[::-1].astype(float))/255
old=np.full(gray.shape,-1,dtype=np.int8);old[prob>cfg['occupied_thresh']]=100;old[prob<cfg['free_thresh']]=0
original_origin=list(cfg['origin'])
cfg['free_thresh']=.196
cfg['origin']=[float(saved['origin'][0]),float(saved['origin'][1]),float(cfg['origin'][2])]
actual=np.full(gray.shape,-1,dtype=np.int8);actual[prob>cfg['occupied_thresh']]=100;actual[prob<cfg['free_thresh']]=0
assert actual.shape==expected.shape and np.array_equal(actual,expected),'Saved map does not match captured SLAM grid'
assert abs(float(saved['resolution'])-cfg['resolution'])<1e-6
assert np.max(np.abs(saved['origin'][:2]-cfg['origin'][:2]))<1e-5
with target.open('x') as f:yaml.safe_dump(cfg,f,sort_keys=False)
r={'original_yaml':str(source),'nav_yaml':str(target),'pgm_sha256':hashlib.sha256(image.read_bytes()).hexdigest(),'original_origin':original_origin,'nav_origin':cfg['origin'],'original_mismatched_cells':int((old!=expected).sum()),'corrected_mismatched_cells':int((actual!=expected).sum()),'unknown_cells':int((actual==-1).sum()),'map_pixels_unchanged':True};(a.map_dir/'map_encoding_check.json').write_text(json.dumps(r,indent=2)+'\n');(a.map_dir/'navigation.sha256').write_text(''.join(hashlib.sha256(f.read_bytes()).hexdigest()+'  '+f.name+'\n' for f in [target,image]));print(json.dumps(r,indent=2))
