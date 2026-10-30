# Late formal-motion jam after a passing recovery probe, 2026-10-27

New isolated copy of atarm_controller_recovery_probe_20261027. All production controller modules and native sources/binaries are unchanged. Only test harness injection and new regression cases are added. The original and ten prior copies are hash-preserved.

Every new scenario first passes passive recovery, all six small-motion stages and a separate fresh passive home check. The fault injector is attached only afterwards. It runs inside the synthetic board, keyed to the active formal cycle phase and independent physical rotor state. Deployment freezes at 12.5/25/37.5 mm on the 50 mm task. Bend freezes at 2.5/5/7.5 mm projected camera progress toward the 10 mm XY target. Bend progress means translation toward the camera goal, not a measured physical bend angle.

Actual-feedback cases report the frozen physical value; coherent-false cases keep reporting the requested target with valid CRC while physical state remains frozen. The production controller never reads the injector event or truth. Single-cable cases freeze one rotor once its next effective command crosses the threshold; their actual onset may be later than exactly 50 percent and is recorded.

Existing motor progress/consistency checks or independent camera no-progress checks latch FAULT. No later neutralization, retraction or final-verification stage starts. Four zero-effort receipts are matched independently to native TX/RX timestamps and raw request bytes. Later ordinary commands and timer steps send no new movement. External fixture repair, a new recovery/probe and a full retry are exercised separately.

Evidence: outputs/试运动通过后中途卡滞验证_2026-10-27/full_run02. All 283 acceptance cases pass including 20 new scenarios; one existing motor-only protocol limitation diagnostic remains separate. Reported times are accelerated synthetic fault-to-command/return timing, not a physical braking or tension-safety measurement. No hardware, ROS runtime, network for tests, installation or recompilation is used.
