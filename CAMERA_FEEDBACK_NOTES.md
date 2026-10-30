# Camera completion evidence addition, 2026-10-27

This is an isolated copy of atarm_controller_motion_checks_20261027. All older copies remain unchanged.

camera_required defaults to True. Inject a nonblocking camera_feedback callback returning a frozen CameraPose and camera_goals keyed by deploy Boolean. Capture times must already map into the controller monotonic clock. Targets must share frame, body, map epoch and metric scale with measurements. Default missing inputs fail closed at motion preflight. The ROS constructor exposes injection only; no live camera subscriber is installed here.

Deploy/retract success now requires motor settling AND independent position/orientation settling across new camera frames. Cached frames do not count as additional evidence. Source identity, capture age/order, tracking, geometry, pose continuity and translation progress are checked. Explicit reference confirmation is still an operator assertion, not automatic homing. Faults reject subsequent controller requests; this is not a tested physical emergency stop.

The only camera producer included is a test fixture with a fixed base and pan/arc mechanism, synthetic endpoint poses and a 1:1 simulated displacement map. It reads simulated physical state, never reported motor q or command q. No visual servo, online Jacobian, SLAM, hardware calibration or live ROS camera integration is claimed.

Local WSL final evidence: outputs/相机独立反馈验证_2026-10-27/final_run02. 71 acceptance cases passed, including 27 camera cases; one explicit motor-only diagnostic retains the protocol limitation. Native codecs and PTY library were reused unchanged. No real device was connected.
