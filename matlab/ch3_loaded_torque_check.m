%% Stage 3 — Loaded torque check (saturation diagnosis)
% *Two-wheeled self-balancing robot — dissertation reproduction script*
%
% This Live Script reproduces the loaded-torque calculation reported in
% section 4.5 of the dissertation. It analyses a closed-loop balance
% recording where the stepper saturated, and compares the observed angular
% acceleration during saturation against the angular acceleration predicted
% by the linearised plant model fitted in stages 1 and 2.
%
% A discrepancy between observed and predicted dq/dt during saturation is
% diagnostic of motor torque collapse under load. The stage-3 sweep
% (ch3_torque_sweep_analysis.m) ruled out step loss as the mechanism by
% verifying 100.6%% step tracking up to 4250 PPS under no load. Therefore
% the only remaining explanation is back-EMF-limited torque delivery at
% high step rates with a 12 V supply.
%
% *Expected output for session_fall_with_saturation.mat:*
%
% * Saturation samples (|motor_applied| >= 3400) detected within the run
% * Mean predicted |dq/dt| during saturation ~ 2320 deg/s^2
% * Mean observed |dq/dt|  during saturation ~ 343 deg/s^2
% * Torque delivery ratio ~ 15%, justifying MOTOR_CMD_LIMIT_PPS reduction
%
% Re-run with sessionFile = 'session_20260421_012900.mat' (limit at 2500)
% to verify the post-fix improvement: ratio rises from ~15% to ~33%.

%% Settings
clear; clc; close all;

% Session to analyse
sessionFile = 'session_fall_with_saturation.mat';   % MOTOR_CMD_LIMIT = 3500 (failure)
% Other interesting:
%   session_20260421_012900.mat   MOTOR_CMD_LIMIT = 2500 (post-fix improvement)

% Plant parameters from stages 1 and 2 (quote your fits here)
g      = 9.81;
l_eff  = 0.28;          % m, from passive-fall fit (stage 1)
d_pass = 2.03;          % 1/s, from passive-fall fit (stage 1)
k_u    = 0.270;         % (deg/s^2)/PPS, magnitude from motor-step fit (stage 2)
                        % NB: sign reconciled by MOTOR_SIGN = -1 in firmware,
                        % so the controller-side effective gain is +0.270.

a_pass = g / l_eff;     % gravitational stiffness, deg-units consistent

% Saturation detection threshold (PPS).
saturation_threshold_pps = 3400;

% Optional: restrict analysis to time after the controller is engaged
analysis_start_s = 0.0;     % set above 0 to skip startup transient

%% Load
S = load(sessionFile);
data = S.data;
N    = size(data, 1);

% Reconstruct uniform timing
Ts = 0.005;
t  = (0:N-1)' * Ts;

ep = data(:, 4);   % estimated_pitch_deg
gr = data(:, 3);   % gyro_rate_dps
ma = data(:, 8);   % motor_applied_pps

fprintf('\nLoaded %s\n', sessionFile);
fprintf('  Samples: %d, %.2f s\n', N, t(end));
fprintf('  Pitch range:    %+6.2f to %+6.2f deg\n',     min(ep), max(ep));
fprintf('  Motor applied:  %+6.0f to %+6.0f PPS\n',     min(ma), max(ma));

%% Predicted vs observed angular acceleration
% Continuous-time plant (controller-side, sign-corrected):
%   q_dot_predicted = a_pass*theta - d_pass*q + k_u*u
%
% Where u is motor_applied_pps as the actual plant input.
% We use the passive parameters and the fitted actuator gain.

% Note on units: a_pass has units of 1/s^2 if theta is in rad. We are
% expressing everything in degrees and deg/s, so the formula is
% dimensionally consistent if we interpret g/l_eff as (deg/s^2)/deg, which
% it is when both sides are in degrees.

q_dot_predicted = a_pass*ep - d_pass*gr - k_u*ma;
% (negative sign on k_u*ma here because the firmware applies MOTOR_SIGN = -1,
%  so the physical actuator effect on theta_dot_dot is -k_u*motor_applied)

% Observed angular acceleration from finite difference of gyro
q_dot_observed = gradient(gr, t);

%% Saturation analysis
sat_mask = (abs(ma) >= saturation_threshold_pps) & (t >= analysis_start_s);
n_sat = sum(sat_mask);

if n_sat < 5
    warning('Only %d saturation samples found. Check threshold or analysis_start_s.', n_sat);
    return;
end

mean_pred = mean(abs(q_dot_predicted(sat_mask)));
mean_obs  = mean(abs(q_dot_observed(sat_mask)));
ratio     = mean_obs / mean_pred;

fprintf('\n=== Loaded torque diagnostic ===\n');
fprintf('  Saturation samples (|motor_applied| >= %.0f): %d (%.1f%% of run)\n', ...
        saturation_threshold_pps, n_sat, 100*n_sat/N);
fprintf('  Mean |dq/dt| predicted by model : %.0f deg/s^2\n', mean_pred);
fprintf('  Mean |dq/dt| observed in gyro   : %.0f deg/s^2\n', mean_obs);
fprintf('  Torque delivery ratio           : %.2f  (%.0f%%)\n', ratio, ratio*100);
fprintf('\n');
if ratio < 0.5
    fprintf('  Conclusion: motor delivers only %.0f%% of modelled torque under load.\n', ratio*100);
    fprintf('  Combined with the no-load 100.6%% step-tracking from\n');
    fprintf('  ch3_torque_sweep_analysis.m, this rules out step loss as the\n');
    fprintf('  mechanism. The remaining explanation is back-EMF-limited torque\n');
    fprintf('  collapse at high step rates with a 12 V supply.\n');
else
    fprintf('  Conclusion: motor torque tracking is acceptable for this regime.\n');
end

%% Plot
figure('Name','Stage 3 — loaded torque check','Color','w','Position',[80 80 980 720]);

subplot(3,1,1);
plot(t, ep, 'b-', 'LineWidth', 1.0); hold on;
yline(0, 'k:');
ylabel('Pitch \theta (deg)');
title(sprintf('%s — pitch trajectory', sessionFile), 'Interpreter', 'none');
grid on;

subplot(3,1,2);
plot(t, ma, 'k-', 'LineWidth', 1.0); hold on;
yline( saturation_threshold_pps, 'r:', 'LineWidth', 0.5);
yline(-saturation_threshold_pps, 'r:', 'LineWidth', 0.5);
% Highlight saturation regions
sat_idx_runs = find(diff([0; sat_mask; 0]));
for k = 1:2:length(sat_idx_runs)-1
    s = sat_idx_runs(k); e = sat_idx_runs(k+1) - 1;
    if e <= N
        x_fill = [t(s), t(e), t(e), t(s)];
        y_fill = [-4000, -4000, 4000, 4000];
        patch(x_fill, y_fill, 'r', 'FaceAlpha', 0.1, 'EdgeColor', 'none');
    end
end
ylabel('motor\_applied (PPS)');
title('Motor command — red shading marks saturation samples');
grid on;

subplot(3,1,3);
plot(t, q_dot_predicted, 'b-', 'LineWidth', 0.8); hold on;
plot(t, q_dot_observed,  'r-', 'LineWidth', 0.8);
yline(0, 'k:');
xlabel('Time (s)');
ylabel('dq/dt (deg/s^2)');
title(sprintf('Angular acceleration: predicted (blue) vs observed (red)  |  ratio %.0f%% during sat', ratio*100));
legend({'Model prediction', 'Observed (numerical derivative)'}, 'Location', 'best', 'FontSize', 8);
grid on;
ylim([-3000 3000]);

sgtitle(sprintf('Stage 3: loaded torque diagnostic  (k_u = %.3f, ratio = %.0f%%)', k_u, ratio*100));

try, exportgraphics(gcf, 'ch3_loaded_torque.png', 'Resolution', 200); end %#ok<TRYNC>
