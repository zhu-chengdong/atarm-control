# Short-window progress policy, 2026-10-27

Explicitly enable with window_progress_config=WindowProgressConfig(...). This isolated copy integrates fixed-goal, source-timestamp, noise-aware minimum progress windows into deployment and three-actuator bending. Existing total time, travel, iteration, camera validity and completion gates remain active. Unresolvable motion is not labelled stalled from noise alone.

The policy scales with requested speed and bend step size; it does not learn a lower limit from observed stalls or reset on Jacobian updates. The complete-cycle deployment handoff now checks the downstream 3D precision budget and a calibrated prediction of pending actuator travel, preserving transverse residual allowance for repeated cycles. See the Chinese report and raw paired tests under outputs/短窗进展与慢速误判验证_2026-10-27/full_run04.

319 acceptance cases pass, including 19 new logical cases; one pre-existing explicit motor-only protocol limitation diagnostic remains separate. All original and twelve preceding copies are preserved. Native sources/binaries and kinematic formula AST remain unchanged. All parameters are simulation examples, not real-arm calibration. No physical device or ROS middleware was used.
