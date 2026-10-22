"""Explicit actual-camera parameters and sensor units for isolated LIVO."""
import argparse
import json
from pathlib import Path
import yaml

parser=argparse.ArgumentParser()
parser.add_argument('--contract',type=Path,required=True)
parser.add_argument('--output',type=Path,required=True)
parser.add_argument('--image-enabled',type=int,choices=[0,1],default=1)
parser.add_argument('--imu-propagation',action='store_true')
args=parser.parse_args()
c=json.loads(args.contract.read_text());k=c['k']
p=dict(use_sim_time=True,
 camera=dict(width=c.get('width',640),height=c.get('height',480),fx=k[0][0],fy=k[1][1],cx=k[0][2],cy=k[1][2]),
 common=dict(img_topic='/livo/image_raw',lid_topic='/lio/points_frame20ms',imu_topic='/lio/imu',img_en=args.image_enabled,lidar_en=1,ros_driver_bug_fix=False),
 extrin_calib=dict(extrinsic_T=[.2,0.,.2],extrinsic_R=[1.,0.,0.,0.,1.,0.,0.,0.,1.],Rcl=sum(c['Rcl'],[]),Pcl=c['Pcl']),
 time_offset=dict(imu_time_offset=0.,img_time_offset=0.,lidar_time_offset=0.,exposure_time_init=0.),
 preprocess=dict(point_filter_num=4,filter_size_surf=.3,lidar_type=2,scan_line=32,blind=.3),
 vio=dict(max_iterations=5,outlier_threshold=1000,img_point_cov=100,patch_size=8,patch_pyrimid_level=4,normal_en=True,raycast_en=False,inverse_composition_en=False,exposure_estimate_en=True,inv_expo_cov=.1),
 imu=dict(imu_en=True,imu_int_frame=30,acc_cov=.1,gyr_cov=.1,b_acc_cov=.0001,b_gyr_cov=.0001),
 lio=dict(max_iterations=5,dept_err=.02,beam_err=.05,min_eigen_value=.0025,voxel_size=.5,max_layer=2,max_points_num=50,layer_init_num=[5,5,5,5,5]),
 local_map=dict(map_sliding_en=False,half_map_size=100,sliding_thresh=8.),
 uav=dict(imu_rate_odom=args.imu_propagation,gravity_align_en=False),
 publish=dict(dense_map_en=False,pub_effect_point_en=False,pub_plane_en=False,pub_scan_num=1,blind_rgb_points=0.),
 evo=dict(seq_name='go2_warehouse_isolated',pose_output_en=False),
 pcd_save=dict(pcd_save_en=False,type=0,colmap_output_en=False,filter_size_pcd=.15,interval=-1),
 image_save=dict(img_save_en=False,interval=1000))
if args.output.exists():raise SystemExit('Refuse replacing replay parameters')
args.output.write_text(yaml.safe_dump({'/**':{'ros__parameters':p}},sort_keys=False))
print(args.output)
