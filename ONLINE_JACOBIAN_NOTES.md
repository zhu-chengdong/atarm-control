# Online deployment Jacobian addition, 2026-10-27

Independent copy of atarm_controller_visual_servo_20261027. See ONLINE_JACOBIAN_NOTES.md for current behavior; earlier repair notes describe retained baselines.

VisualServoConfig.online_update defaults to None. Explicit JacobianUpdateConfig enables a regularized damped Broyden update for the existing 3x1 deployment-translation column. It requires a configured metric camera noise bound and model norm limits. No extra probing/calibration moves, online initial-zero guess, or six-DOF identification is added.

Each requested corrective step stores a settled source-pose window and actual measured actuator length. Only a later disjoint, fresh, settled window may update the model. Uses actual encoder motion, not target increments. Other actuator motion/spread, motor tracking, reference identity, pair duration, pose MAD, minimum excitation, signal relative to camera noise, slope bounds and direction consistency are checked. Prediction errors compatible with the two-window noise bound are ignored. The last accepted column is retained on rejection. Each update has a fractional change and angle limit.

Learned columns persist across endpoint requests in the same reference. Pending pairs are cleared between requests. Explicit mechanical reference recovery resets the estimator to its configured initial value; changed reference frames/maps require recovery. The original motor, camera completion, travel, progress, timeout and bounded stop protections remain enabled.

Only one stretch actuator and camera translation are identified. The initial Jacobian, model bounds, noise bound and actuator limits are synthetic test configurations, not hardware calibration. The estimator never reads simulated physical truth. Noise can still bias accepted estimates; the tests do not establish universal stability or absolute camera accuracy.

Final local WSL evidence: outputs/雅可比在线更新验证_2026-10-27/final_run03. 129 acceptance cases passed, including 30 new adaptive/guard cases. One explicit motor-only limitation diagnostic is not an acceptance case. No ROS runtime, physical arm, camera or UART was used; native binaries remain unchanged.
