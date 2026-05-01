function fit_motor_step()
% fit_motor_step.m
%
% Fits k_u (sign-free) and tau_u (positive) from motor step data.
% Timing reconstructed from sample index * Ts to bypass dashboard batching.

%% ── Load ────────────────────────────────────────────────────────────────
[fname, fpath] = uigetfile('*.mat','Select motor step session .mat');
if isequal(fname,0), error('No file selected.'); end
S = load(fullfile(fpath, fname));
data = S.data;

ep = data(:,4);
gr = data(:,3);
ma = data(:,8);
N  = size(data,1);

%% ── Fixed passive params ───────────────────────────────────────────────
Ts     = 0.005;
g      = 9.81;
h_eff  = 0.28;
d_pass = 2.03;
a_pass = g / h_eff;
u_step = 450.0;

fprintf('\nPassive fixed: a_pass=%.3f  d_pass=%.3f   |  u_step=%.0f PPS\n\n', ...
        a_pass, d_pass, u_step);

%% ── Detect step edges + extract trials ─────────────────────────────────
edges = find( (ma(1:end-1) < 50) & (ma(2:end) > 50) );
fprintf('Step edges detected: %d\n', numel(edges));

FIT_WINDOW_SAMPLES = round(0.45 / Ts);
MAX_THETA0_DEG     = 2.0;

trials = struct('t',{},'theta',{},'q',{},'theta0',{},'q0',{});
for k = 1:numel(edges)
    idx0   = edges(k);
    theta0 = ep(idx0);
    if abs(theta0) > MAX_THETA0_DEG
        fprintf('  Trial %d skipped: theta0=%.2f deg\n', k, theta0); continue
    end
    idx_end = min(N, idx0 + FIT_WINDOW_SAMPLES);
    idxs    = idx0:idx_end;
    if numel(idxs) < 20, continue; end

    seg.t      = (0:numel(idxs)-1)' * Ts;
    seg.theta  = ep(idxs);
    seg.q      = gr(idxs);
    seg.theta0 = theta0;
    seg.q0     = gr(idx0);
    trials(end+1) = seg; %#ok<AGROW>
    fprintf('  Trial %d accepted: theta0=%.2f deg, n=%d\n', k, theta0, numel(idxs));
end
if isempty(trials), error('No trials passed theta0 filter.'); end

%% ── Initial guess from data ────────────────────────────────────────────
% Rough estimate: observed theta ~ -6 deg at 400ms with u=450
% Solve analytically assuming tau_u=0 (no actuator lag):
%   theta(t) ~ (k_u*u/lambda^2)*(cosh(lambda*t) - 1) for small theta
% where lambda = sqrt(a_pass) = 5.92
% -6 = (k_u*450/35)*(cosh(5.92*0.4) - 1)
% cosh(2.37) ~ 5.35, so: -6 ~ k_u * 12.86 * 4.35 ~ 56 * k_u
% k_u ~ -0.107
k_u_init   = -0.10;
tau_u_init = 0.03;

% Parameterise: k_u linear (sign-free), log(tau_u) positive
p0 = [k_u_init; log(tau_u_init)];

cost = @(p) total_sse(p(1), exp(p(2)), trials, a_pass, d_pass, u_step);

opts = optimset('Display','iter','MaxFunEvals',15000,'MaxIter',15000,...
                'TolX',1e-10,'TolFun',1e-10);
p_hat = fminsearch(cost, p0, opts);

k_u   = p_hat(1);
tau_u = exp(p_hat(2));

fprintf('\n════════════════════════════════════════\n');
fprintf('  k_u   = %+.5f  (deg/s^2 per PPS)\n', k_u);
fprintf('  tau_u = %.4f   s\n', tau_u);
fprintf('════════════════════════════════════════\n\n');

fprintf('Physical interpretation:\n');
fprintf('  Sign of k_u: %s\n', ternary(k_u<0,'NEGATIVE (wheels drive forward → body tips backward)','POSITIVE'));
fprintf('  Actuator lag: %.1f ms (driver + stepper electrical/mechanical delay)\n\n', tau_u*1000);

%% ── RMSE ───────────────────────────────────────────────────────────────
fprintf('Per-trial RMSE:\n');
for k = 1:numel(trials)
    th_sim = simulate_trial(trials(k).t, trials(k).theta0, trials(k).q0, ...
                            k_u, tau_u, a_pass, d_pass, u_step);
    rmse = sqrt(mean((th_sim - trials(k).theta).^2));
    fprintf('  Trial %d: RMSE = %.3f deg\n', k, rmse);
end

%% ── TF ─────────────────────────────────────────────────────────────────
fprintf('\nTheta(s)/U(s) = %+.5f / [(%.4f*s + 1)(s^2 + %.3f*s - %.3f)]\n\n', ...
        k_u, tau_u, d_pass, a_pass);
if exist('tf','file') == 2
    G = tf(k_u, conv([tau_u 1], [1 d_pass -a_pass]));
    disp(G);
    fprintf('Poles: '); fprintf('%+.4f  ', pole(G)); fprintf('\n\n');
end

%% ── Plots ──────────────────────────────────────────────────────────────
n = numel(trials);
figure('Name','Motor step fit','Color','w','Position',[80 80 320*n 560]);
for k = 1:n
    th_sim = simulate_trial(trials(k).t, trials(k).theta0, trials(k).q0, ...
                            k_u, tau_u, a_pass, d_pass, u_step);
    rmse = sqrt(mean((th_sim - trials(k).theta).^2));

    subplot(2,n,k); hold on; grid on;
    plot(trials(k).t, trials(k).theta, 'b-',  'LineWidth', 1.8);
    plot(trials(k).t, th_sim,          'r--', 'LineWidth', 1.5);
    yline(0,'k:');
    title(sprintf('Trial %d  |  RMSE %.2f°', k, rmse), 'FontSize', 9);
    ylabel('\theta (°)'); xlabel('t (s)');
    if k == 1, legend('Measured','Model','Location','southwest','FontSize',7); end
    ylim([-20 4]);

    subplot(2,n,n+k); hold on; grid on;
    plot(trials(k).t, trials(k).q, 'r-', 'LineWidth', 1.8);
    yline(0,'k:');
    ylabel('q (°/s)'); xlabel('t (s)');
    title(sprintf('Gyro — trial %d', k), 'FontSize', 9);
    ylim([-80 30]);
end
sgtitle(sprintf('Actuated plant fit   k_u=%+.5f   tau_u=%.4fs', k_u, tau_u));

try, exportgraphics(gcf,'motor_step_fit.png','Resolution',200); end %#ok<TRYNC>

%% ── Locals ─────────────────────────────────────────────────────────────
function sse = total_sse(k_u, tau_u, trials, a_pass, d_pass, u_step)
    if ~isfinite(k_u) || ~isfinite(tau_u) || tau_u <= 1e-4 || tau_u > 2
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

function out = ternary(cond, a, b), if cond, out=a; else, out=b; end, end

end
