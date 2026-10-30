# Confirmation budget, 2026-10-28

Only progress_window.py changes among production files. Confirmation retains its fixed 200 ms source-time deadline and three-fresh-frame gate. Its displacement requirement deducts one 80 ms pause plus the existing 20 ms shared sampling slack. Small progress cannot reset the clock. A bad primary window may arm confirmation when its uncertainty interval reaches severe insufficiency; only independent fresh confirmation with an optimistic progress bound may stop. Default severe_fraction=0 remains disabled.

All 27 recovery pairs now complete; 10 genuine-creep pairs stop within the stated targets, including repeated noisy bending seeds. Final 67 focused/old-pause checks plus 31 early-policy checks pass. The initial red tests and the first implementation with two remaining false stops are retained. No hardware, real serial/camera, ROS, compilation, installation or remote execution. See outputs/恢复确认暂停预算协调验证_2026-10-28/实现与验证报告.md.
