"""Offline neural query + explicit goal binding. Never sends ROS or motion."""
import argparse,json
from pathlib import Path
from jetbot_semantic_nav.goal_bindings import validate_bindings,bound_goal
p=argparse.ArgumentParser();p.add_argument('--min-confidence',type=float,default=.9);p.add_argument('--model',required=True);p.add_argument('--backend',choices=['prototype_gru','pretrained_chinese_tiny','pretrained_chinese_encoder'],default='prototype_gru');p.add_argument('--objects',required=True,type=Path);p.add_argument('--bindings',required=True,type=Path);p.add_argument('--scene-id',required=True);p.add_argument('--robot-xy',nargs=2,type=float,required=True);p.add_argument('text');a=p.parse_args()
objects=json.loads(a.objects.read_text())['objects'];bindings=validate_bindings(json.loads(a.bindings.read_text()),a.scene_id)
if a.backend in ('pretrained_chinese_tiny','pretrained_chinese_encoder'):
    from jetbot_semantic_nav.pretrained_grounding import PretrainedGrounder
    grounder=PretrainedGrounder(a.model,a.min_confidence)
else:
    from jetbot_semantic_nav.object_grounding import Grounder
    grounder=Grounder(a.model,a.min_confidence)
r=grounder.predict(a.text,objects,a.robot_xy)
if r['state']=='SELECTED':
    obj=next(o for o in objects if o['object_id']==r['object_id'])
    try:r.update(goal=bound_goal(bindings,obj),frame_id='map',requires_native_path_check=True)
    except ValueError as e:r.update(state=str(e),object_id=None)
print(json.dumps(r,ensure_ascii=False,indent=2))
