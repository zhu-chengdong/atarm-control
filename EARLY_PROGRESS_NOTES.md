# Early recovery confirmation, 2026-10-27

Explicitly enable WindowProgressConfig(window_s=0.4,severe_fraction=0.6,severe_confirm_s=0.2,severe_confirm_frames=3) with the existing PauseBudgetConfig. Default severe_fraction=0 preserves the original policy. Only progress_window.py changes among production modules.

After a full 400 ms window shows severe insufficiency even with optimistic noise allowance, collect at least three fresh frames over another 200 ms and test renewed minimum average progress. Recovering evidence cancels the suspicion; unresolved evidence cannot trigger this early stop. Mild deficits retain the original two-window policy. Overall deadlines, goal accuracy, travel and correction limits remain independent and unchanged.

31 focused checks and 19 legacy window checks pass. 29 matched pairs yield 58 physical runs: 12 full completions, 46 bounded failures. Four quiet/noisy intermittent-motion comparisons shorten deployment from about 0.79-0.80 to 0.62-0.63 s and bending from 1.11-1.12 to 0.94-0.95 s. 144 source-time replays show no false stops, including far goals and varied phases/noise. 184 raw stop receipts are verified. All original and seventeen preceding copies are preserved. No hardware, ROS runtime, installation, compilation or remote execution was used. See outputs/400毫秒窗口与快速恢复确认验证_2026-10-27/实现与验证报告.md.
