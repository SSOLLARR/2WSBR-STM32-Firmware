# MATLAB analysis scripts and session telemetry

This folder contains the analysis pipeline referenced in the dissertation.

## Identification scripts (Chapter 3 / Appendix G)
- ch3_lqr_synthesis.m         — LQR synthesis from the identified plant (Appendix G.4-G.5)
- fit_motor_step.m            — Stage 2 actuator identification
- fit_motor_steptwo.m         — Stage 2 refinement (timing reconstruction)
- fit_motor_stepthree.m       — Stage 2 refinement (sign handling)
- ch3_loaded_torque_check.m   — Loaded torque sweep diagnostic
- ch3_torque_sweep_analysis.m — Torque sweep analysis

## Results scripts (Chapter 4)
- ch4_balance_analysis.m         — Balancing run analysis (Section 4.7)
- ch4_bandwidth_lco_analysis.m   — Bandwidth-ceiling experiment (Section 4.6)
- ch4_lqr_sweep_analysis.m       — LQR gain-sweep analysis

## Live telemetry
- main_live_dashboard.m       — Live capture entry point
- live_dashboard_cascade.m    — Cascade-mode dashboard
- live_dashboard_cascadetwo.m — Updated cascade dashboard
- initDashboard.m, updateDashboard.m, parsePacket.m, makeSessionFolder.m, analyze_session.m — Helpers

## Session files (named in dissertation)
- session_lqrd_trial1_99s.mat ... session_lqrd_t6_94s.mat — Six-trial LQR-direct cohort (Section 4.7)
- session_headline_balance_35s.mat        — First-deployment LQR validation (Section 4.5)
- session_bandwidth_lco_0p26hz.mat        — Bandwidth-ceiling experiment (Section 4.6)
- session_motor_step_for_id.mat           — Stage 2 actuator-step data
- session_torque_speed_sweep.mat          — Open-loop pull-out characterisation (Section 3.6)
- session_fall_with_saturation.mat        — Empirical-controller drift/runaway diagnostic (Section 4.4)
- cascade_params.mat, sweep_synthesis.mat — Synthesis outputs
