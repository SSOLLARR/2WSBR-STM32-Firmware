function fit_motor_step()
% fit_motor_step.m
%
% Fits actuated plant k_u and tau_u from motor step data.
%
% TIMING NOTE:
%   The MATLAB dashboard timestamps arrive-time, not sample-time, because
%   the Windows serial buffer batches USART2 packets. This script ignores
%   those timestamps and reconstructs timing from sample index * Ts, where
%   Ts = 0.005 s (the firmware's enforced loop period).
%
% Model (state: theta, q, z):
%   x_dot = [ q ;
%             a_pass*theta - d_pass*q + k_u*z ;
%             (1/tau_u)*(u - z) ]
% Passive params fixed from fall-test fit.

%% ── Load ────────────────────────────────────────────────────────────────
[fname, fpath] = uigetfile('*.mat','Select motor step session .mat');
if isequal(fname,0), error('No file selected.'); end
S = load(fullfile(fpath, fname));
data = S.data;

ep = data(:,4);    % estimated_pitch_deg
gr = data(:,3);    % gyro_rate_dps
ma = data(:,8);    % motor_applied_pps

N = size(data,1);

%% ── Fixed passive params ───────────────────────────────────────────────
Ts     = 0.005;
g      = 9.81;
h_eff  = 0.28;
d_pass = 2.03;
a_pass = g / h_eff;
u_step = 450.0;

fprintf('\nPassive params (fixed): a_pass=%.3f  d_pass=%.3f\n', a_pass, d_pass);
fprintf('Sample period (assumed uniform): %.0f ms\n', Ts*1000);

%% ── Detect step edges by sample INDEX ──────────────────────────────────
edges = find( (ma(1:end-1) < 50) & (ma(2:end) > 50) );
fprintf('Step edges detected: %d\n', numel(edges));

%% ── Extract trials ─────────────────────────────────────────────────────
FIT_WINDOW_SAMPLES = round(0.45 / Ts);    % 90 samples = 450 ms
MAX_THETA0_DEG     = 2.0;

trials = struct('t',{},'theta',{},'q',{},'theta0',{},'q0',{});

for k = 1:numel(edges)
    idx0 = edges(k);
    theta0 = ep(idx0);
    if abs(theta0) > MAX_THETA0_DEG
        fprintf('  Trial %d skipped: theta0=%.2f deg\n', k, theta0);
        continue
    end
    idx_end = min(N, idx0 + FIT_WINDOW_SAMPLES);
    idxs    = idx0:idx_end;
    if numel(idxs) < 20, continue; end

    seg.t      = (0:numel(idxs)-1)' * Ts;   % uniform-5ms reconstruction
    seg.theta  = ep(idxs);
    seg.q      = gr(idxs);
    seg.theta0 = theta0;
    seg.q0     = gr(idx0);
    trials(end+1) = seg; %#ok<AGROW>
    fprintf('  Trial %d accepted: theta0=%.2f deg, n=%d\n', k, theta0, numel(idxs));
end

if isempty(trials), error('No trials passed theta0 filter.'); end

%% ── Fit via fminsearch on log-params ───────────────────────────────────
p0 = log([0.05; 0.05]);
cost = @(p) total_sse(exp(p(1)), exp(p(2)), trials, a_pass, d_pass, u_step);
opts = optimset('Display','iter','MaxFunEvals',10000,'MaxIter',10000,...
                'TolX',1e-10,'TolFun',1e-10);
p_hat = fminsearch(cost, p0, opts);

k_u   = exp(p_hat(1));
tau_u = exp(p_hat(2));

fprintf('\n════════════════════════════════════════\n');
fprintf('  k_u   = %.5f  (deg/s^2 per PPS)\n', k_u);
fprintf('  tau_u = %.4f  s\n', tau_u);
fprintf('════════════════════════════════════════\n\n');

%% ── RMSE per trial ─────────────────────────────────────────────────────
fprintf('Per-trial RMSE:\n');
for k = 1:numel(trials)
    th_sim = simulate_trial(trials(k).t, trials(k).theta0, trials(k).q0, ...
                            k_u, tau_u, a_pass, d_pass, u_step);
    rmse = sqrt(mean((th_sim - trials(k).theta).^2));
    fprintf('  Trial %d: RMSE = %.3f deg\n', k, rmse);
end

%% ── Transfer function ──────────────────────────────────────────────────
fprintf('\nTheta(s)/U(s) = %.5f / [(%.4f*s + 1)(s^2 + %.3f*s - %.3f)]\n\n', ...
        k_u, tau_u, d_pass, a_pass);

if exist('tf','file') == 2
    G = tf(k_u, conv([tau_u 1], [1 d_pass -a_pass]));
    disp(G);
    fprintf('Poles: '); fprintf('%.4f  ', pole(G)); fprintf('\n');
end

%% ── Plots ───────────────────────────────────────────────────────────────
n = numel(trials);
figure('Name','Motor step fit','Color','w','Position',[80 80 320*n 560]);

for k = 1:n
    th_sim = simulate_trial(trials(k).t, trials(k).theta0, trials(k).q0, ...
                            k_u, tau_u, a_pass, d_pass, u_step);
    rmse = sqrt(mean((th_sim - trials(k).theta).^2));

    subplot(2, n, k); hold on; grid on;
    plot(trials(k).t, trials(k).theta, 'b-',  'LineWidth', 1.8);
    plot(trials(k).t, th_sim,          'r--', 'LineWidth', 1.5);
    yline(0, 'k:');
    title(sprintf('Trial %d  |  RMSE %.2f°', k, rmse), 'FontSize', 9);
    ylabel('\theta (°)'); xlabel('t (s) [reconstructed]');
    if k == 1, legend('Measured','Model','Location','southwest','FontSize',7); end
    ylim([-20 4]);

    subplot(2, n, n+k); hold on; grid on;
    plot(trials(k).t, trials(k).q, 'r-', 'LineWidth', 1.8);
    yline(0, 'k:');
    ylabel('q (°/s)'); xlabel('t (s) [reconstructed]');
    title(sprintf('Gyro rate — trial %d', k), 'FontSize', 9);
    ylim([-80 30]);
end

sgtitle(sprintf('Actuated plant fit  |  k_u=%.5f  tau_u=%.4fs  |  timing: Ts=%.0fms uniform', ...
                k_u, tau_u, Ts*1000));

try
    exportgraphics(gcf,'motor_step_fit.png','Resolution',200);
    fprintf('Saved motor_step_fit.png\n');
catch
end

%% ── Local functions ────────────────────────────────────────────────────

function sse = total_sse(k_u, tau_u, trials, a_pass, d_pass, u_step)
    if ~isfinite(k_u) || ~isfinite(tau_u) || k_u <= 0 || tau_u <= 0 || tau_u > 5
        sse = 1e12; return;
    end
    sse = 0;
    for j = 1:numel(trials)
        th_sim = simulate_trial(trials(j).t, trials(j).theta0, trials(j).q0, ...
                                k_u, tau_u, a_pass, d_pass, u_step);
        sse = sse + sum((th_sim - trials(j).theta).^2);
    end
end

function theta_out = simulate_trial(t_seg, theta0, q0, k_u, tau_u, a_pass, d_pass, u)
    ode_fun = @(~, x) [ x(2);
                         a_pass*x(1) - d_pass*x(2) + k_u*x(3);
                        (1/tau_u)*(u - x(3)) ];
    x0 = [theta0; q0; 0.0];
    opts = odeset('RelTol',1e-8,'AbsTol',1e-10);
    [~, X] = ode45(ode_fun, t_seg, x0, opts);
    theta_out = X(:,1);
end

end
