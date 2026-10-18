# 2026-10-15 Go2 warehouse straight-corridor navigation

Two actual NavigateToPose runs including complete simulator/scan/planner/controller/guard restart. Cruise means 0.767783 and 0.768677 m/s; final stopped goal errors 0.157702 and 0.118826 m. All 9 checks per final run and all 7 runtime audit checks passed.

Motion commands 0.6–0.8 m/s; startup/stopping and physical gait variation are excluded from cruise-average requirement. Fixed straight corridor only, simulator truth localization and conservative USD geometry map. No SLAM/AMCL, arbitrary turns, global live obstacle updates or JetBot migration yet.

Failed navigation_v2 (RotateToGoal conflict) and navigation_v6 (front-scan stop, source unresolved, stop stability failed) are retained here; all other failed reports and raw telemetry/logs remain in the private backup. Two successful runs do not establish the unresolved scan event is fixed.

See ../../docs/GO2_WAREHOUSE_MOTION_M1.md for reproducible commands, watchdog boundaries and limitations. A single small Boost runtime was extracted locally; no Isaac/CUDA/PyTorch/system ROS replacement. No credentials, model weights or package binaries included.

The raw RViz window capture includes overlap from the Isaac desktop window; use navigation_comparison.png and machine-readable reports for measured motion evidence.
