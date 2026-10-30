# Complete simulated camera arm cycle, 2026-10-27

New isolated copy of atarm_controller_coupled_bend_20261027. Original and seven preceding copies are SHA-256 preserved. No hardware or ROS middleware was connected.

Public API: run_camera_cycle(CyclePlan(neutral_goal, tuple(bend_goals))), cancel_camera_cycle(), cycle_status(). Explicit calibrated deploy/retract goals, bend configuration, camera provider and a mechanically confirmed retracted reference are required. There is no automatic homing or recovery. The neutral goal must equal the configured deployed camera reference. Neutralization has a tighter pose budget before retraction; inconsistent noise/tolerance budgets reject the plan before I/O.

Sequence: VERIFY_START -> DEPLOY -> BEND_1..N -> NEUTRALIZE -> RETRACT -> VERIFY_END. Each stage retains an independent deep-copied receipt and pose/motor evidence. Only all verified stages may yield complete=True. Each proof must have the expected goal, root state, fresh source frame and a newer source ID than the previous stage. Final verification is independent of the retract return flag.

Cancellation and overall deadline are checked in both existing motion loops and verification. Run admission is nonblocking and exclusive. Cancel acceptance and final success are serialized. Public manual motion, reference reset and legacy step() cannot take ownership during the cycle. A failed stage latches FAULT, attempts one bounded zero-effort request per motor, records individual receipts, and never automatically retracts or continues the next stage. Retrying cannot clear the failure.

The repeated scalar endpoint path now first holds an already confirmed unchanged motor endpoint while camera error stays within the task tolerance plus the configured online noise bound. It must still obtain fresh camera stability confirmation. This removes an inherited noise-driven repeat correction; it does not relax the camera gate.

ROS wrapper exposes run_camera_cycle and cancel_camera_cycle Trigger callbacks with an injected cycle_plan, and camera_cycle JSON status. Callback function tests use ROS stubs. No ROS executor, real subscriber, live service scheduling, physical stop, tension, collision geometry or real neutral posture calibration is validated. A single-threaded executor cannot process a cancellation service while a blocking run service occupies its thread; a dedicated coordinator/executor strategy still needs real integration and testing.

Final tests: 205 acceptance cases pass including 32 new cases; one explicit motor-only protocol limitation diagnostic is separately retained. The full-cycle noise example uses bounded ±20 micrometers per coordinate, not the earlier bend-only ±250 micrometer setting. An incompatible larger uncertainty budget is rejected. Synthetic final errors and accelerated timing are not real-arm specifications.

Evidence: outputs/完整流程模拟验证_2026-10-27/full_run02, case_summary.csv, execution_scope.json, delivery_manifest.json and changes_from_coupled_bend.patch. Native codec sources and binaries are unchanged. Only local WSL and process-created PTYs were used, without compilation, installation, network, physical UART, real camera or robot.
