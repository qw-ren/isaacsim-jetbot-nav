"""Readonly inference preflight. No automatic weights download or sim launch."""
import argparse,hashlib,importlib,json
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--weights',required=True,type=Path);p.add_argument('--output',required=True,type=Path);a=p.parse_args()
if a.output.exists():raise FileExistsError(a.output)
r=dict(modules={},errors=[],weights=str(a.weights),inference_performed=False)
for name in ('rclpy','cv_bridge','message_filters','tf2_ros','torch','torchvision','cv2','scipy','ultralytics','atongmu_perception'):
 try:m=importlib.import_module(name);r['modules'][name]=getattr(m,'__version__','import_ok')
 except Exception as e:r['errors'].append(name+': '+repr(e))
if 'ultralytics'in r['modules']:
 try:
  from ultralytics import YOLO
  r['yolo_class_import']=True
 except Exception as e:r['errors'].append('YOLO class import: '+repr(e))
if not a.weights.is_file():r['errors'].append('Missing eight-class detector weights; no substitute model loaded')
elif 'ultralytics'in r['modules']:
 try:
  from ultralytics import YOLO
  model=YOLO(str(a.weights));r['weights_sha256']=hashlib.sha256(a.weights.read_bytes()).hexdigest();r['classes']=model.names
  if set(model.names.values())!=set(('shelf','wall','box','pallet','barrier','cart','sign','person')):r['errors'].append('Eight-class ontology mismatch')
 except Exception as e:r['errors'].append('Model load: '+repr(e))
r['ready_for_camera_inference']=not r['errors'];a.output.write_text(json.dumps(r,ensure_ascii=False,indent=2));print(json.dumps(r,ensure_ascii=False,indent=2))
raise SystemExit(0 if r['ready_for_camera_inference']else 1)
