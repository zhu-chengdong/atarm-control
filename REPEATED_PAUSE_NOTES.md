# Repeated 80 ms pauses, 2026-10-27

Only tests and notes differ from atarm_controller_pause_budget_20261027. All production controller code, defaults and native sources/binaries are byte-identical. Original and fifteen preceding copies are preserved.

Nine regular full cycles complete after three or six 80 ms pauses with 60 ms motion between pauses, including camera noise. Dense pauses with only 10 ms motion between them stop early under the existing 200 ms window in all three phases; 400 ms diagnostic configurations complete all eight pauses within the original action limits. These diagnostics are not a deployed fix or a claim of universal pause tolerance.

Four isolated cumulative-time cases stop at the unchanged simulated 1.4 s deployment/retraction and 2 s bend limits, or an explicitly configured 1 s whole-cycle limit, despite repeated real resumption. Twenty new behavior checks pass; three are early-stop boundary characterizations. Nineteen physical runs yield twelve completions and seven stops; twenty-eight stop receipts are verified. Eighteen separate exact-source-time replays explain window frequency sensitivity. Historical 374 cases were not rerun because production code and their bodies are unchanged. No hardware, ROS middleware, installation, compilation or remote execution was used.
