"""Derive a stopped single-use sensor runtime, preserving accepted navigation copies."""
from pathlib import Path
import shutil,json,hashlib,ast
r=Path('/home/admin/go2-warehouse');src=r/'motion_lidar_world_20261015';dst=r/'motion_lio_sensors_20261018';s=Path(__file__).resolve().parent
if dst.exists():raise SystemExit('LIO sensor runtime exists; inspect rather than overwrite')
shutil.copytree(src,dst,ignore=shutil.ignore_patterns('__pycache__','outputs','.git'))
shutil.copy2(s/'native_imu_adapter.py',dst/'native_imu_adapter.py')
p=dst/'go2/go2_sensors.py';v=p.read_text();old='lidar_annotators = []';assert v.count(old)==1;v=v.replace(old,old+'\n        self.lio_raw_annotators = []');old='annotator.attach(hydra_texture.path)';assert v.count(old)==1;v=v.replace(old,old+'''\n            raw = rep.AnnotatorRegistry.get_annotator("RtxSensorCpuIsaacReadRTXLidarData")
            raw.initialize(keepOnlyPositiveDistance=False)
            raw.attach(hydra_texture.path)
            self.lio_raw_annotators.append(raw)''');ast.parse(v);p.write_text(v)
p=dst/'isaac_go2_ros2.py';v=p.read_text();anchor='go2_env_cfg.scene.num_envs = cfg.num_envs';assert v.count(anchor)==1;v=v.replace(anchor,anchor+'\n    from native_imu_adapter import spawn_go2_with_native_imu\n    go2_env_cfg.scene.unitree_go2.spawn.func = spawn_go2_with_native_imu');old='obs, _ = env.reset()';assert v.count(old)==1;v=v.replace(old,'''from native_imu_adapter import NativeImuAdapter
    lio_sensors = NativeImuAdapter(env, dm, sm.lio_raw_annotators, args_cli.artifact_dir)
    obs, _ = env.reset()
    lio_sensors.attach()''');old='dm.pub_ros2_data()';assert v.count(old)==1;v=v.replace(old,old+'\n            lio_sensors.audit_lidar()');old='go2_ctrl.base_vel_cmd_input.zero_()';assert v.count(old)==1;v=v.replace(old,'lio_sensors.close()\n    '+old);ast.parse(v);p.write_text(v)
meta={'source':str(src),'target':str(dst),'changes':'native IMU sampling after physical steps and separate raw RTX diagnostic; no policy, existing topics, odom or guard changes','files_sha256':{str(p.relative_to(dst)):hashlib.sha256(p.read_bytes()).hexdigest() for p in [dst/'isaac_go2_ros2.py',dst/'go2/go2_sensors.py',dst/'native_imu_adapter.py']}}
(dst/'LIO_SENSOR_COPY.json').write_text(json.dumps(meta,indent=2)+'\n');print(json.dumps(meta,indent=2))
