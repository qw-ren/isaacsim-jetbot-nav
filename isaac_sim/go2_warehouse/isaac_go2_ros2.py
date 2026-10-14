import os
import sys
import hydra
import rclpy
import torch
import time
import math
import argparse
from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Tutorial on running the cartpole RL environment.")
parser.add_argument("--run-seconds", type=float, default=0.0)
parser.add_argument("--artifact-dir", default="")
parser.add_argument("--spawn-x", type=float, default=0.0)
parser.add_argument("--spawn-y", type=float, default=0.0)
parser.add_argument("--spawn-yaw", type=float, default=0.0)

# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
#parse AppLauncher args, keep hydra overrides
args_cli, hydra_args = parser.parse_known_args()

#pass remaining args to hydra
sys.argv = [sys.argv[0]] + hydra_args



# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import torch

from go2.go2_env import Go2RSLEnvCfg, camera_follow
import env.sim_env as sim_env
import go2.go2_sensors as go2_sensors
import omni
import carb
import go2.go2_ctrl as go2_ctrl
import ros2.go2_ros2_bridge as go2_ros2_bridge

FILE_PATH = os.path.join(os.path.dirname(__file__), "cfg")
@hydra.main(config_path=FILE_PATH, config_name="sim", version_base=None)
def run_simulator(cfg):
    # Go2 Environment setup
    go2_env_cfg = Go2RSLEnvCfg()
    go2_env_cfg.scene.num_envs = cfg.num_envs
    go2_env_cfg.scene.unitree_go2.init_state.pos = (
        args_cli.spawn_x, args_cli.spawn_y, go2_env_cfg.scene.unitree_go2.init_state.pos[2])
    go2_env_cfg.scene.unitree_go2.init_state.rot = (
        math.cos(args_cli.spawn_yaw / 2), 0.0, 0.0, math.sin(args_cli.spawn_yaw / 2))
    go2_env_cfg.decimation = math.ceil(1./go2_env_cfg.sim.dt/cfg.freq)
    go2_env_cfg.sim.render_interval = go2_env_cfg.decimation
    go2_ctrl.init_base_vel_cmd(cfg.num_envs)
    #env, policy = go2_ctrl.get_rsl_flat_policy(go2_env_cfg)
    env, policy = go2_ctrl.get_rsl_rough_policy(go2_env_cfg)

    # Simulation environment
    if (cfg.env_name == "obstacle-dense"):
        sim_env.create_obstacle_dense_env() # obstacles dense
    elif (cfg.env_name == "obstacle-medium"):
        sim_env.create_obstacle_medium_env() # obstacles medium
    elif (cfg.env_name == "obstacle-sparse"):
        sim_env.create_obstacle_sparse_env() # obstacles sparse
    elif (cfg.env_name == "warehouse"):
        sim_env.create_warehouse_env() # warehouse
    elif (cfg.env_name == "warehouse-forklifts"):
        sim_env.create_warehouse_forklifts_env() # warehouse forklifts
    elif (cfg.env_name == "warehouse-shelves"):
        sim_env.create_warehouse_shelves_env() # warehouse shelves
    elif (cfg.env_name == "full-warehouse"):
        sim_env.create_full_warehouse_env() # full warehouse

    # Sensor setup
    sm = go2_sensors.SensorManager(cfg.num_envs)
    lidar_annotators = sm.add_rtx_lidar() if cfg.sensor.enable_lidar else []
    cameras = sm.add_camera(cfg.freq) if cfg.sensor.enable_camera else []

    # Keyboard control
    system_input = carb.input.acquire_input_interface()
    app_window = omni.appwindow.get_default_app_window()
    if app_window is not None and not args_cli.headless:
        system_input.subscribe_to_keyboard_events(app_window.get_keyboard(), go2_ctrl.sub_keyboard_event)
    
    # ROS2 Bridge
    rclpy.init()
    dm = go2_ros2_bridge.RobotDataManager(env, lidar_annotators, cameras, cfg)
   
    # Run simulation
    sim_step_dt = float(go2_env_cfg.sim.dt * go2_env_cfg.decimation)
    obs, _ = env.reset()
    started = time.monotonic()
    import signal
    shutdown_requested = [False]
    def request_shutdown(signum, frame):
        shutdown_requested[0] = True
    signal.signal(signal.SIGINT, request_shutdown)
    signal.signal(signal.SIGTERM, request_shutdown)
    next_status = started
    step_count = 0
    if args_cli.artifact_dir:
        import json
        from pathlib import Path
        from pxr import UsdGeom, UsdPhysics
        stage = omni.usd.get_context().get_stage()
        warehouse = stage.GetPrimAtPath("/World/Warehouse")
        cache = UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
        bounds = cache.ComputeWorldBound(warehouse).ComputeAlignedRange()
        metadata = {
            "env_name": str(cfg.env_name),
            "spawn_xy": [args_cli.spawn_x, args_cli.spawn_y],
            "spawn_yaw_rad": args_cli.spawn_yaw,
            "warehouse_prim_valid": warehouse.IsValid(),
            "warehouse_prims": sum(1 for p in stage.Traverse() if str(p.GetPath()).startswith("/World/Warehouse/")),
            "warehouse_collision_prims": sum(1 for p in stage.Traverse() if str(p.GetPath()).startswith("/World/Warehouse/") and p.HasAPI(UsdPhysics.CollisionAPI)),
            "meters_per_unit": UsdGeom.GetStageMetersPerUnit(stage),
            "up_axis": str(UsdGeom.GetStageUpAxis(stage)),
            "bounds_min": list(bounds.GetMin()),
            "bounds_max": list(bounds.GetMax()),
        }
        artifact_dir = Path(args_cli.artifact_dir)
        artifact_dir.mkdir(parents=True, exist_ok=True)
        (artifact_dir / "scene.json").write_text(json.dumps(metadata, indent=2) + "\n")
        print("WAREHOUSE_SCENE " + json.dumps(metadata), flush=True)
    while simulation_app.is_running() and not shutdown_requested[0]:
        if args_cli.run_seconds > 0 and time.monotonic() - started >= args_cli.run_seconds:
            break
        start_time = time.time()
        with torch.inference_mode():            
            # control joints
            actions = policy(obs)

            # step the environment
            obs, _, _, _ = env.step(actions)

            dm.pub_ros2_data()
            step_count += 1

            # Camera follow
            if (cfg.camera_follow):
                camera_follow(env)
            if args_cli.artifact_dir and step_count == 80:
                from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport
                capture_viewport_to_file(get_active_viewport(), str(artifact_dir / "warehouse_viewport.png"))

            # limit loop time
            elapsed_time = time.time() - start_time
            if elapsed_time < sim_step_dt:
                sleep_duration = sim_step_dt - elapsed_time
                time.sleep(sleep_duration)
        actual_loop_time = time.time() - start_time
        rtf = min(1.0, sim_step_dt/elapsed_time)
        if time.monotonic() >= next_status:
            print(f"Step time: {actual_loop_time*1000:.2f}ms, Real Time Factor: {rtf:.2f}", flush=True)
            next_status = time.monotonic() + 2.0
    
    go2_ctrl.base_vel_cmd_input.zero_()
    dm._executor.shutdown()
    dm._spin_thread.join(timeout=5)
    dm.destroy_node()
    rclpy.shutdown()
    env.close()
    simulation_app.close()

if __name__ == "__main__":
    run_simulator()
    
