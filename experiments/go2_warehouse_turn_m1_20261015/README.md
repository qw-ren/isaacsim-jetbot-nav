# 2026-10-15 Go2 turning and stop diagnostics

Turning navigation is NOT accepted. One early 10-check trial passed; later stricter cases exposed orientation, stop stability, position drift, and low-command response failures. All original JSON results remain unchanged, including action SUCCEEDED cases that fail physical checks. The flat policy comparison also failed its height limit.

See ../../docs/GO2_WAREHOUSE_TURN_M1.md. Final short-horizon trial: actual cruise mean 0.7753755 m/s, final stopped position error 0.2645016 m against a 0.25 m requirement. After zero-command confirmation, body displacement over 5.02 simulation seconds was 0.2076284 m. Do not promote this experimental profile or infer old scan-stop root cause was resolved.

response_diagnostics.json and plots are derived from private raw telemetry. Source snapshots, full telemetry and logs are preserved in the full backup; screenshots, credentials, model weights and package installers are not part of this selected export. All trial processes were stopped; cloud power was not operated. Prior straight-corridor results remain separate.
