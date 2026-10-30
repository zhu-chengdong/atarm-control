# Matched 200/400 ms window validation, 2026-10-27

Production controller/defaults and native code/binaries are identical to atarm_controller_repeated_pause_20261027. Only test helpers, a new matched suite and notes change. Explicitly use WindowProgressConfig(window_s=0.4) with the unchanged 80 ms PauseBudgetConfig to test the candidate.

Twenty-seven matched checks cover fifty-four physical runs. Six 400 ms dense-pause controls complete all eight 80 ms pauses with only 10 ms motion between pauses, including camera noise. All genuine fault and deadline profiles stop without false completion. Intermittent micro-progress detection becomes slower: approximately 0.400 to 0.795 s deployment, 0.510 to 1.122 s bending. Common jam/creep faults remain around 0.115-0.416 s due to other independent guards. Cumulative action/cycle deadlines remain unchanged.

One hundred ninety-two raw zero-effort stop receipts are verified. All original and sixteen preceding copies are preserved. No hardware, ROS middleware, installation, compilation or remote execution was used. Other historical suites were not rerun because production code is unchanged. See outputs/400毫秒短窗综合验证_2026-10-27/实现与验证报告.md.
