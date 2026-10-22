"""Qualify a bounded 20ms capture-time approximation for isolated trials only."""
import argparse,json
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--plane',type=Path,required=True);p.add_argument('--clock',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--accept-frame-quantization-ms',type=int,required=True);a=p.parse_args()
if a.output.exists():raise SystemExit('Gate exists; never overwrite evidence')
if a.accept_frame_quantization_ms!=20:raise SystemExit('Only explicit 20ms evaluation mode supported')
plane=json.loads(a.plane.read_text());clock=json.loads(a.clock.read_text());required=['target_front_plane_is_expected','motion_coverage','stationary_coverage','end_pose_absolute_range_accuracy']
checks={k:plane['checks'][k] for k in required};checks['all_clock_and_pose_checks']=all(clock['checks'].values());checks['clock_motion_coverage']=clock['cruise_frames_over_0_6_mps']>=50
allowed=all(checks.values())
result={'frame20ms_trial_allowed':allowed,'mode':'isolated approximate capture-time evaluation, NOT validated continuous per-ray timing','topic':'/lio/points_frame20ms','capture_packet_interval_s':.02,'assumed_intra_packet_time_uncertainty_s':.02,'strict_uncertainty_bound_validated':False,'point_time_assignment':'all rays from each render packet use its measured end capture time; concatenate five consecutive packets','native_deltaTimes_used_as_measurement_time':False,'strict_per_ray_calibration_passed':plane['end_frame_sampling_supported'],'checks':checks,'plane_report':str(a.plane),'clock_report':str(a.clock),'production_odom_switch_allowed':False,'time_sync_en':False}
a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
if not allowed:raise SystemExit(1)
