"""Derive a separate semantic demo without changing any frozen navigation value."""
import argparse
import json
from pathlib import Path

p=argparse.ArgumentParser();p.add_argument('--baseline',required=True,type=Path);p.add_argument('--output',required=True,type=Path)
a=p.parse_args();cfg=json.loads(a.baseline.read_text())
cfg.update(goals=[[0.,7.,1.5707963267948966]],obstacles=[],timeout_wall_s=1750.,
           scope='Independent semantic oracle demo; frozen navigation parameter overrides preserved')
cfg['visualization']['record_video']=True
cfg['extensions']['controller_probe']=False
cfg['semantic']=dict(version=1,mode='ground_truth_oracle',world_frame='usd_world',
    world_to_map=dict(method='same_usd_world_as_map_raster',translation=[0.,0.,0.],rotation_xyzw=[0.,0.,0.,1.],
        source_scene_sha256='3d2fbe4238e2652572c5a5ee68749caf165adf47648c2b140b65cc9dfd20eec0',
        planning_map_sha256='d42be37e98acac1017e473188ad1f69bd8f202818d985ed9976450a9e8aed4d0'),
    declared_spawn_map=[0.,6.,0.],observation_hz=2.,object_ttl_s=5.,
    planning_radius_m=.4,footprint_padding_m=.01,approach_margin_m=.6,approach_samples=8,
    target_move_m=.15,target_rotate_rad=.2,plan_timeout_wall_s=10.,query_timeout_wall_s=180.,
    navigation_timeout_wall_s=600.,max_goal_cost=20,
    ready_after_sim_s=10.,healthy_ready_wall_s=2.,
    topics=dict(observations='/semantic/objects/observations',objects='/semantic/objects/map',
        command='/semantic/command',cancel='/semantic/cancel',status='/semantic/status',
        markers='/semantic/objects/markers',selected_goal='/semantic/selected_goal'),
    actions=dict(plan='/compute_path_to_pose',navigate='/navigate_to_pose'),
    services=dict(arm='/jetbot/arm',global_costmap='/global_costmap/get_costmap'),
    registered_prims=[],
    fixtures=[dict(id='cart_near',class_name='cart',asset_kind='pushcart',center=[-3.,7.,0.]),
        dict(id='cart_far',class_name='cart',asset_kind='pushcart',center=[-13.,20.,0.]),
        dict(id='pallet_near',class_name='pallet',asset_kind='pallet',center=[-8.,12.,0.]),
        dict(id='pallet_far',class_name='pallet',asset_kind='pallet',center=[-8.,20.,0.]),
        dict(id='box_near',class_name='box',asset_kind='box',center=[-13.,12.,0.]),
        dict(id='box_far',class_name='box',asset_kind='box',center=[-13.,24.,0.])])
with a.output.open('x')as f:json.dump(cfg,f,ensure_ascii=False,indent=2);f.write('\n')
