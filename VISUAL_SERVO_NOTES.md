# Bounded deployment visual correction, 2026-10-27

Independent copy of atarm_controller_camera_feedback_20261027. All original sources and four earlier copies are preserved and checked by SHA-256.

Enable only by injecting VisualServoConfig, a metric independent camera source and both endpoint CameraGoals. This configuration requires an explicit nonzero local Jacobian column, stroke limits, offset and cumulative correction limits; no hardware values are guessed. Default visual_servo_config=None retains camera-only completion checking.

The current implementation corrects ONE deployment actuator. The three bending actuators and pan/arc mechanics are unchanged. The injected 3x1 column maps deployment actuator travel to camera translation in the goal frame. No online Jacobian update, six-DOF solver, hardware calibration or actual SLAM implementation is included.

The nominal deploy/retract command ramps with the existing speed limit. Once the motor settles, the controller collects an odd window of new source frames captured after that settling barrier, takes a componentwise median error, applies a damped projected correction, and clips the step magnitude. It then waits for motion and new camera evidence again. Hysteresis and a deadband avoid pursuing small noise. Completion still uses the original raw-pose quality and settling checks, and motor tracking is checked against the UPDATED target. Successful corrected endpoint actuator targets are remembered for repeated requests, until explicit reference recovery clears them.

No-progress, iteration, elapsed-time, stroke, offset and total-correction limits bound failure. A proposed target outside a bound is rejected, without clipping it to a boundary for another attempt. Orthogonal translation error is not controllable by this one actuator and is rejected. Failure latches FAULT, rejects later normal commands, and makes one bounded zero-effort exchange per motor, including when ordinary motion validation is faulted. These exchanges only set kp=kd=dq=tau=0 and do not alter trusted feedback history. A valid reply proves communication, not a physical stop or safe load support. No real arm may be enabled on this evidence.

Only local WSL, existing unchanged native A1 codec/PTY libraries, simulated physical motor state and an independent synthetic camera were used. No ROS middleware, SDK SerialPort, device UART, real camera, arm, install or network transfer was used in testing.

Final evidence: outputs/视觉小步纠偏验证_2026-10-27/final_run03. 99 acceptance cases passed, including 28 new servo cases. One explicit motor-only protocol-limitation diagnostic is kept outside acceptance counts.
