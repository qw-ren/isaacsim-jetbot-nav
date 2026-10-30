"""Bounded native-sensor JetBot runtime; no estimator truth input or training."""
import argparse,json,signal,time,sys
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--case',type=Path,required=True);p.add_argument('--duration',type=float,default=35.);a=p.parse_args()
config=json.loads((a.case/'CASE.json').read_text());view=config.get('visualization',dict(headless=False,follow=True));headless=bool(view['headless'])
from isaacsim import SimulationApp
app=SimulationApp({'headless':headless,'width':1280 if not headless else 960,'height':720 if not headless else 540})
import numpy as np
import math
import omni.usd
from isaacsim.core.utils.extensions import enable_extension
enable_extension('isaacsim.ros2.bridge');enable_extension('isaacsim.sensors.physics');enable_extension('isaacsim.sensors.rtx')
for _ in range(10):app.update()
from isaacsim.core.utils.stage import open_stage,is_stage_loading
open_stage(str(a.case/'jetbot_resolved.usda'))
while is_stage_loading():app.update()
from isaacsim.core.api import World
from isaacsim.robot.wheeled_robots.robots import WheeledRobot
from isaacsim.robot.wheeled_robots.controllers.differential_controller import DifferentialController
world=World(stage_units_in_meters=1.,physics_dt=.005,rendering_dt=.02,physics_prim_path='/World/PhysicsScene',device='cpu')
robot=world.scene.add(WheeledRobot(prim_path='/World/jetbot',name='migration_jetbot',wheel_dof_names=['left_wheel_joint','right_wheel_joint'],create_robot=False))
world.reset()
from sensor_bridge import SensorBridge
bridge=SensorBridge(world,robot,a.case)
from dynamic_fixture import DynamicFixture
fixture=DynamicFixture(a.case,world)
limits=config.get('motion_limits',{});wheel_max=float(limits.get('max_wheel_rad_s',1e20))
controller=DifferentialController(name='migration_controller',wheel_radius=float(limits.get('wheel_radius_m',.03)),wheel_base=float(limits.get('wheel_base_m',.1125)),max_wheel_speed=wheel_max)
(a.case/'RUNTIME_JOINT_LIMITS.json').write_text(json.dumps(dict(dof_max_velocity_rad_s=robot.dof_properties['maxVelocity'].tolist(),controller_max_wheel_rad_s=wheel_max,joint_limits_modified=False),indent=2)+'\n')
from observer_follow import ObserverFollow
observer_follow=ObserverFollow();observer_boxes=json.loads((a.case/'map/collision_boxes.json').read_text())if(a.case/'map/collision_boxes.json').exists()else [];observer_log=(a.case/'observer_pose.jsonl').open('x')
video=None;viewport=None;captures=[];capture_times=[5.,15.];observer=view.get('observer_camera','/World/JetbotObserverCamera')
if not headless:
 from omni.kit.viewport.utility import get_active_viewport,capture_viewport_to_file
 from isaacsim.core.utils.viewports import set_camera_view
 from pxr import UsdGeom,Gf
 viewport=get_active_viewport();assert viewport is not None, 'GUI requested but no active viewport'
 stage=omni.usd.get_context().get_stage();camera=UsdGeom.Camera.Define(stage,observer);camera.CreateClippingRangeAttr(Gf.Vec2f(.03,1000.));camera.CreateFocalLengthAttr(24.);camera.CreateHorizontalApertureAttr(36.);camera.CreateVerticalApertureAttr(20.25);viewport.camera_path=observer
if viewport is not None and view.get('record_video',False):
 from viewport_video import ViewportVideo
 video=ViewportVideo(viewport,a.case)
(a.case/'VIEWPORT_PROVENANCE.json').write_text(json.dumps(dict(headless=headless,observer_camera=observer,follow=bool(view.get('follow',True)),observer_offset_behind_m=2.8,observer_height_m=2.6,lookahead_m=1.8,observer_smoothing=dict(position_tau_sim_s=.35,yaw_tau_sim_s=.65,yaw_deadband_deg=3.,max_yaw_rad_s=.4,level_height_fixed=True,occlusion_check="display-only static CAD and authored props; shorten distance, expand smoothly"),look_target_height_m=.35,focal_length_mm=24.,horizontal_aperture_mm=36.,algorithm_camera='/World/jetbot/chassis/rgb_camera/jetbot_camera',truth_pose_used_only_for_viewer_follow=True,navigation_truth_input=False),indent=2)+'\n')
stop=False
def request_stop(*_):
 global stop;stop=True
signal.signal(signal.SIGINT,request_stop);signal.signal(signal.SIGTERM,request_stop)
runtime_wall_s=float(json.loads((a.case/'CASE.json').read_text()).get('runtime_wall_s',600));assert 0<runtime_wall_s<=1900
started=time.monotonic();error=None
try:
 while app.is_running() and not stop and world.current_time<a.duration and time.monotonic()-started<runtime_wall_s:
  bridge.spin();robot.apply_wheel_actions(controller.forward(command=bridge.command()))
  world.step(render=True);bridge.tick();fixture.tick()
  if viewport is not None and view.get('follow',True):
   position,q=robot.get_world_pose();yaw=math.atan2(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]*q[2]+q[3]*q[3]));eye,target=observer_follow.update(position,yaw,float(world.current_time),observer_boxes+fixture.display_boxes);set_camera_view(np.asarray(eye),np.asarray(target),camera_prim_path=observer,viewport_api=viewport);observer_log.write(json.dumps(dict(sim=float(world.current_time),eye=eye,target=target,observer_yaw=observer_follow.yaw,observer_distance=observer_follow.distance))+'\n')
   if video is None and capture_times and world.current_time>=capture_times[0]:
    requested=capture_times.pop(0);captures.append(capture_viewport_to_file(viewport,str(a.case/('observer_'+str(int(requested))+'.png'))))
  if video is not None:video.tick(float(world.current_time))
except Exception as exc:
 error=repr(exc);raise
finally:
 try:robot.apply_wheel_actions(controller.forward(command=[0.,0.]));world.step(render=True)
 except Exception:pass
 final_sim=float(world.current_time)
 if video is not None:
  for _ in range(5):app.update()
  video.close(final_sim)
 observer_log.close();bridge.close();world.stop()
 (a.case/'SIM_RESULT.json').write_text(json.dumps(dict(error=error,sim_s=final_sim,wall_s=time.monotonic()-started,requested_stop=stop,power_operation=False),indent=2)+'\n')
 app.close()
