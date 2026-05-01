%% Balance-record analysis and failure-envelope verification
% *Two-wheeled self-balancing robot — dissertation reproduction script, chapter 4*
% 
% This Live Script reproduces the balance-record analysis reported in 
% sections 4.7–4.10 of the dissertation. It loads any balancing session, 
% extracts the quiescent window statistics against the four performance 
% objectives, computes the failure envelope, and verifies the failure 
% mechanism prediction from chapter 3.
% 
% *Expected output for session 20260421_150702 (retuned headline):*
%% 
% * Continuous balance duration $\approx$ 40 s
% * Quiescent pitch $\sigma \approx 0.056°$ over $[2, 30]$ s
% * Net wheel displacement $\approx 1.69$ mm over 30 s
% * 0 inner-loop saturation events
% * Failure-envelope exponential growth rate $\lambda_{\mathrm{closed}} \approx 0.31$ s$^{-1}$ 
% over the interval $[30, 38]$ s
%% Settings
clear; clc; close all;

% Session to analyse
sessionFile = 'session_20260421_150702.mat';   % retuned headline run
% Other interesting sessions:
%   session_20260421_022009.mat   LQR-deployed 35.71 s
%   session_20260421_022943.mat   bandwidth-widening, 0.26 Hz limit cycle
%   session_20260422_115639.mat   reproducibility clean run, 55.8 s
%   session_20260422_115951.mat   reproducibility clean run, 78.1 s

% Quiescent window for the objective evaluation
quiescent_start_s = 2.0;
quiescent_end_s   = 30.0;

% Actuator constants (from the locked firmware configuration)
u_sat_pps  = 3800;          % command saturation
wheel_diam_m = 0.065;
steps_per_rev = 200 * 16;   % 1/16 microstepping, 200 full-steps/rev

% Failure-envelope fit window (dissertation uses [30, 38] s on the 150702 run)
failure_fit_start_s = 30;
failure_fit_end_s   = 38;

% Load cascade parameters (from chapter 3)
if isfile('cascade_params.mat')
    P = load('cascade_params.mat');
    fprintf('Loaded cascade_params.mat from chapter 3 script.\n');
else
    warning('cascade_params.mat not found; using dissertation-reported values.');
    P = struct('l_eff', 0.28, 'd_eff', 2.03, 'k_u', -0.270, 'g', 9.81, ...
               'Kp_theta_retune', 10, 'Kp_q_retune', 18, 'a_theta', 5);
end
%% Load session
T = load_session(sessionFile);

% Authoritative time axis starts at zero of the post-last-reset segment.
t = T.t_mcu_s;
theta = T.estimated_pitch_deg;
q = T.gyro_rate_dps;
u_cmd = T.motor_cmd_pps;
u_applied = T.motor_applied_pps;

fprintf('\nLoaded %s\n', sessionFile);
fprintf('  Duration: %.2f s  |  Samples: %d\n', t(end) - t(1), height(T));

% Balancing (MODE_BALANCING = 0) samples only
bal = T.mode == 0;
if nnz(bal) == 0
    error('No MODE_BALANCING samples in %s.', sessionFile);
end
fprintf('  Balancing samples: %d (%.2f s)\n', nnz(bal), nnz(bal)*0.005);
%% Headline pitch trace
figure('Name','Pitch record','Color','w');
plot(t, theta, 'LineWidth', 1.0); hold on;
yline(0, '-', 'upright', 'Color', [0.5 0.5 0.5]);
yline(8, '--r', '\theta_{safe}');
yline(-8, '--r');
xline(quiescent_start_s, ':k');
xline(quiescent_end_s, ':k', sprintf('quiescent window [%g, %g] s', ...
    quiescent_start_s, quiescent_end_s));
xlabel('t [s]  (MCU-side, sample\_idx \times 0.005)');
ylabel('\theta [deg]');
grid on;
title(sprintf('Pitch estimate — %s', sessionFile), 'Interpreter', 'none');
%% Quiescent-window statistics
qmask = bal & t >= quiescent_start_s & t <= quiescent_end_s;
if nnz(qmask) < 10
    warning('Quiescent window [%g, %g] s contains only %d samples.', ...
        quiescent_start_s, quiescent_end_s, nnz(qmask));
end

theta_q   = theta(qmask);
u_q       = u_applied(qmask);
t_q       = t(qmask);

sigma_theta = std(theta_q);
mean_theta  = mean(theta_q);
max_abs_theta = max(abs(theta_q));
max_abs_u   = max(abs(u_q));
sat_events  = sum(abs(u_q) >= u_sat_pps);

% Net wheel displacement: integrate applied step rate over the window,
% divide by steps_per_rev to get revolutions, then multiply by wheel
% circumference.
dt_q = [0; diff(t_q)];
steps_cumulative = cumtrapz(t_q, u_q);        % pps integrated over time
rev_cumulative   = steps_cumulative / steps_per_rev;
displacement_m   = rev_cumulative * pi * wheel_diam_m;
net_disp_m       = displacement_m(end) - displacement_m(1);

fprintf('\n---- Quiescent window statistics [%g, %g] s ----\n', ...
    quiescent_start_s, quiescent_end_s);
fprintf('  sigma(theta)      = %.4f deg\n', sigma_theta);
fprintf('  mean(theta)       = %.4f deg\n', mean_theta);
fprintf('  max |theta|       = %.4f deg\n', max_abs_theta);
fprintf('  max |u|           = %.2f pps   (saturation is +/-%d pps)\n', ...
    max_abs_u, u_sat_pps);
fprintf('  saturation events = %d\n', sat_events);
fprintf('  net wheel disp.   = %.2f mm  (over %.2f s)\n', ...
    net_disp_m*1000, t_q(end) - t_q(1));

% Check against the four performance objectives reported in the dissertation
fprintf('\n---- Objective check ----\n');
fprintf('  P1 (>=30 s continuous):       %s\n', tf((t(end)-t(1)) >= 30));
fprintf('  P2 (sigma < 0.5 deg):         %s   (margin: %.2fx inside threshold)\n', ...
    tf(sigma_theta < 0.5), 0.5/sigma_theta);
fprintf('  P3 (|disp| < 5 cm in 30 s):   %s   (margin: %.1fx inside threshold)\n', ...
    tf(abs(net_disp_m) < 0.05), 0.05/max(abs(net_disp_m), 1e-9));
fprintf('  P4 (no saturation):           %s\n', tf(sat_events == 0));
%% Failure envelope analysis
% Maximum absolute pitch within successive half-second windows, then a 
% log-linear fit over the specified growth window.
win_s = 0.5;
if t(end) - t(1) > win_s
    edges = t(1):win_s:(t(end) + win_s);
    envelope = zeros(numel(edges)-1, 1);
    env_t    = zeros(numel(edges)-1, 1);
    for k = 1:numel(envelope)
        m = t >= edges(k) & t < edges(k+1) & bal;
        if nnz(m) > 0
            envelope(k) = max(abs(theta(m)));
            env_t(k) = (edges(k) + edges(k+1)) / 2;
        end
    end
    keep = envelope > 0;
    envelope = envelope(keep);
    env_t    = env_t(keep);

    % Log-linear fit over the growth window
    fit_mask = env_t >= failure_fit_start_s & env_t <= failure_fit_end_s;
    lambda_closed = NaN;
    if nnz(fit_mask) >= 3
        logEnv = log(envelope(fit_mask));
        coeff = polyfit(env_t(fit_mask), logEnv, 1);
        lambda_closed = coeff(1);
        A_fit = exp(coeff(2));
    end

    % Plot
    figure('Name','Failure envelope','Color','w');
    plot(env_t, envelope, 'o-', 'LineWidth', 1.2); hold on;
    if isfinite(lambda_closed)
        plot(env_t(fit_mask), A_fit*exp(lambda_closed*env_t(fit_mask)), ...
             '--r', 'LineWidth', 1.5);
    end
    xlabel('t [s]'); ylabel('max |\theta| over 0.5 s window [deg]');
    title('Failure envelope of balancing record');
    grid on;
    legend('envelope', sprintf('exp fit \\lambda = %.3f s^{-1}', lambda_closed), ...
           'Location', 'northwest');

    fprintf('\n---- Failure-envelope fit ----\n');
    fprintf('  Fit interval: [%.1f, %.1f] s\n', failure_fit_start_s, ...
        failure_fit_end_s);
    if isfinite(lambda_closed)
        fprintf('  lambda_closed = %.4f s^-1\n', lambda_closed);
        fprintf('  doubling time = %.3f s\n', log(2)/lambda_closed);
    else
        fprintf('  Fit window contained insufficient points.\n');
    end
end
%% Failure-mechanism verification: $\omega_n(e_\theta)$ prediction
% Walk along the balancing record, compute the effective closed-loop natural 
% frequency under the retuned gains as a function of the instantaneous 
% tracking error magnitude, and mark the point at which the envelope crosses 
% the complementary filter's 0.98 Hz corner.
Kp_eff = @(e) P.Kp_theta_retune + P.a_theta * abs(e);
omega_n_of_e = @(e) sqrt(max(abs(P.k_u) * P.Kp_q_retune * Kp_eff(e) ...
                              - P.g/P.l_eff, 0));
f_n_of_e = @(e) omega_n_of_e(e) / (2*pi);

% At quiescent
fprintf('\n---- Gain-scheduler bandwidth check ----\n');
fprintf('  At |e_theta| = 0:     f_n = %.3f Hz\n', f_n_of_e(0));
fprintf('  At |e_theta| = 1.00:  f_n = %.3f Hz (should be ~ CF corner)\n', ...
    f_n_of_e(1));
fprintf('  At |e_theta| = 1.78:  f_n = %.3f Hz (should be ~ 1.20 Hz)\n', ...
    f_n_of_e(1.78));

% Overlay: effective bandwidth trace during the actual balancing run
f_n_trace = arrayfun(f_n_of_e, theta);

figure('Name','Effective closed-loop bandwidth during run','Color','w');
yyaxis left
plot(t, theta, 'LineWidth', 0.9);
ylabel('\theta [deg]');
yyaxis right
plot(t, f_n_trace, 'LineWidth', 1.2); hold on;
yline(0.98, '--r');
ylabel('f_n(|e_\theta|) [Hz]');
xlabel('t [s]');
title('Gain-scheduler-driven effective bandwidth during the run');
grid on;
%% Helpers
function s = tf(b)
    if b, s = 'PASS'; else, s = 'FAIL'; end
end
