"""Use the pinned Go2 FAST-LIVO backend with JetBot's measured native mounts."""
import argparse,hashlib,json
from pathlib import Path
import yaml
p=argparse.ArgumentParser();p.add_argument('--case',type=Path,required=True);a=p.parse_args();root=Path('/home/admin/go2-warehouse');binary=root/'nav_stage2/deps/fastlivo_trial_20261022/bin/fastlivo_mapping'
expected='8a8e61dd543befa5f9da26bdf2674af804f7d2d5a81542d9f1fb7a9cf11e958f'
if hashlib.sha256(binary.read_bytes()).hexdigest()!=expected:raise RuntimeError('FAST-LIVO binary pin changed')
c=json.loads((a.case/'camera_contract.json').read_text());data=yaml.safe_load(Path(__file__).with_name('livo_base.yaml').read_text());v=data['/**']['ros__parameters'];k=c['k']
v['camera']=dict(width=c['width'],height=c['height'],fx=k[0][0],fy=k[1][1],cx=k[0][2],cy=k[1][2]);v['common'].update(img_topic='/jetbot/image_raw',lid_topic='/jetbot/livo/points',imu_topic='/jetbot/imu',img_en=1,lidar_en=1)
v['extrin_calib']=dict(extrinsic_T=c['extrinsic_T'],extrinsic_R=c['extrinsic_R'],Rcl=sum(c['Rcl'],[]),Pcl=c['Pcl']);v['preprocess']['blind']=.15
v['evo']['seq_name']='jetbot_migration';v['uav']['imu_rate_odom']=True
output=a.case/'livo_params.yaml'
if output.exists():raise RuntimeError('Refuse overwriting frozen params')
output.write_text(yaml.safe_dump(data,sort_keys=False));(a.case/'ESTIMATOR_PROVENANCE.json').write_text(json.dumps(dict(binary=str(binary),sha256=expected,base_params='livo_base.yaml copied byte-for-byte from Go2 B_native_long_09',base_params_sha256=hashlib.sha256(Path(__file__).with_name('livo_base.yaml').read_bytes()).hexdigest(),sensor_params='native JetBot camera_contract.json',truth_input=False,added_imu_noise=False,community_ros2_port=True),indent=2)+'\n')
