# Three-cable coupled bend camera control, 2026-10-27

Independent copy of atarm_controller_online_jacobian_20261027; prior notes describe the retained deployment stages. All originals and six prior copies are preserved.

Explicit BendServoConfig enables the new mode. The caller must provide a finite rank-two, zero-common-mode 3x2 cable mixing matrix, an initial 3x2 camera-translation Jacobian, per-cable travel limits and a camera noise bound. Optional JacobianUpdateConfig enables matrix Broyden learning. No physical calibration is inferred. Default bend_servo_config=None retains legacy behavior.

move_bend_camera(CameraGoal) coordinates all three bending motors through two bend coordinates. The deployment actuator is held and monitored. Small vector steps use damped least squares; all cable commands are ramped with one interpolation factor. Learning uses actual cable-motion vectors and disjoint, settled, independent camera windows; redundant common-mode motion is rejected. Candidate singular values, conditioning, noise, tracking, reference and update magnitude are guarded. Status reports excitation coverage and never claims full-matrix calibration from a single trajectory.

Camera confirmation uses raw-pose validity/settling and a filtered error plus the configured noise margin. Once in the inner tolerance, hold until stable confirmation; resume only beyond the outer task tolerance. Repeated confirmed goals retain this hold state, reducing boundary noise chasing. No extra calibration oscillations are commanded.

In this mode legacy angle commands and raw micro-adjusts are rejected. step() cannot chase legacy targets or take ownership during a bend. Deployment/reference requests are refused while bending. Deployment and explicit mechanical reference recovery reset the bend estimator. move_bend_camera requires confirmed DEPLOYED state.

ROS wrapper adds bend/set_camera_position_m (Float64MultiArray[3]) using an explicitly injected CameraGoal template for frame/body/map/orientation, and bend_visual status. The actual callback and JSON publication were exercised with ROS stubs; no ROS middleware or real camera subscriber was started. Live ROS scheduling and pose acquisition remain unverified.

Failure latches FAULT, makes one bounded zero-effort request per actuator and rejects later normal commands. A failed transport can prevent stop confirmation; receipts carry actuator indices and validity. Zero effort is not a verified brake or tension-support strategy. No load/tension sensing is included.

Only a synthetic local two-coordinate bending model is validated. The 120-degree mixing constraint and all numerical limits are assumptions for simulation, not measured properties of the real arm. No arbitrary 3D/6D pose or full 3x3 cable Jacobian is claimed.

Evidence: outputs/三电机弯曲闭环验证_2026-10-27/full_run04. 173 acceptance cases passed including 44 new bend cases; one explicit motor-only limitation diagnostic is excluded from acceptance counts. All testing used local WSL and unchanged native codecs with process-created PTYs; no hardware, installation or remote transfer.
