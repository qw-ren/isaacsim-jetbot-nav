"""Verify the neural choice against its own predicted numeric semantic contract.

This never chooses an alternative object or parses text. Bounds are axis-aligned,
matching the oracle and training dataset; oriented visual boxes need normalization.
"""
import math
from .grounding_dataset import CLASSES,surface_gap

def check_choice(objects,robot_xy,slots,object_id):
    kind,row,selection=slots[:3]
    if len(slots)>3 and slots[3]==0:return 'intent_does_not_request_navigation'
    if not 0<=kind<len(CLASSES):return 'unsupported_class'
    candidates=[o for o in objects if o['class_name']==CLASSES[kind]]
    if row:
        references=[o for o in objects if o.get('row_index')==row and o['class_name']=='shelf']
        if len(references)!=1:return 'reference_missing_or_ambiguous'
        candidates=[o for o in candidates if surface_gap(o,references[0])<=1.5]
    selected=next((o for o in candidates if o['object_id']==object_id),None)
    if selected is None:return 'selected_object_violates_category_or_relation'
    if selection==0:return None if len(candidates)==1 else 'unique_reference_is_ambiguous'
    if selection not in (1,2):return 'unsupported_selection'
    distances=[math.dist(o['position'][:2],robot_xy) for o in candidates]
    expected=(min if selection==1 else max)(distances)
    if abs(math.dist(selected['position'][:2],robot_xy)-expected)>1e-6:return 'selected_object_violates_distance_order'
    return None
