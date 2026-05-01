function fit_motor_step()
% fit_motor_step.m
%
% Fits the actuated plant model to motor step-response data.
%
% Model structure (grey-box, passive params fixed from fall test):
%
%   State:   x = [theta (deg); q (deg/s); z (deg/s)]
%
%   x_dot = [ q
%              a_pass*theta - d_pass*q + k_u*z
%             (1/tau_u)*(u - z)              ]
%
%   where:
%     theta   = pitch angle (deg)
%     q       = pitch rate  (deg/s)
%     z       = actuator lag state (deg/s) — first-order motor lag
%     u       = motor command (PPS, constant step = 450)
%     a_pass  = g/h_eff = 35.04 s^-2   (fixed from fall test)
%     d_pass  = 2.03    s^-1            (fixed from fall test)
%     k_u     = actuator-to-pitch gain  (deg/s^2 per PPS) — FREE
%     tau_u   = actuator lag time const (s)                — FREE
%
% Outputs:
%   Fitted k_u, tau_u
%   Overlay plots of measured vs simulated theta for each trial
%   Transfer function G(s) = Theta(s)/U(s) printed to console

%% ── 1. Load data ────────────────────────────────────────────────────────
[fname, fpath] = uigetfile('*.mat','Select motor step session .mat file');
if isequal(fname,0), error('No file selected.'); end
S = load(fullfile(fpath, fname));
data     = S.data;
colnames = S.colnames;

t_abs = data(:,1);
ep    = data(:,4);   % estimated_pitch_deg
gr    = data(:,3);   % gyro_rate_dps
ma    = data(:,8);   % motor_applied_pps

t = t_abs - t_abs(1);

%% ── 2. Fixed passive-plant parameters ──────────────────────────────────
g      = 9.81;
h_eff  = 0.28;
d_pass = 2.03;
a_pass = g / h_eff;   % 35.04 s^-2
u_step = 450.0;       % PPS

fprintf('\nFixed passive parameters:\n');
fprintf('  h_eff  = %.3f m\n',  h_eff);
fprintf('  d_pass = %.3f 1/s\n', d_pass);
fprintf('  a_pass = %.3f 1/s^2\n', a_pass);
fprintf('  u_step = %.1f PPS\n\n', u_step);

%% ── 3. Detect step edges ────────────────────────────────────────────────
edges = find( (ma(1:end-1) < 50) & (ma(2:end) > 50) );
fprintf('Step edges detected: %d\n', numel(edges));

%% ── 4. Extract clean trials (start within ±2 deg of upright) ───────────
FIT_WINDOW_S = 0.45;   % seconds of response to fit
MAX_THETA0   = 2.0;    % deg — discard trials starting too far from upright

trials = struct('t',{},'theta',{},'q',{},'theta0',{},'q0',{});

for k = 1:numel(edges)
    idx0  = edges(k);
    theta0 = ep(idx0);
    if abs(theta0) > MAX_THETA0
        fprintf('  Trial %d skipped: theta0=%.2f deg (outside linear region)\n', k, theta0);
        continue
    end
    t_end = t(idx0) + FIT_WINDOW_S;
    win   = (t >= t(idx0)) & (t <= t_end);
    if sum(win) < 20, continue; end

    seg.t      = t(win) - t(idx0);
    seg.theta  = ep(win);
    seg.q      = gr(win);
    seg.theta0 = theta0;
    seg.q0     = gr(idx0);
    trials(end+1) = seg;
    fprintf('  Trial %d accepted: theta0=%.2f deg, %d samples\n', k, theta0, sum(win));
end

if isempty(trials)
    error('No trials passed the theta0 filter. Check MAX_THETA0 or data.');
end

%% ── 5. Fit k_u and tau_u via fminsearch ─────────────────────────────────
% log-parameterise so both stay positive
p0 = log([0.05; 0.05]);   % initial guess: k_u=0.05, tau_u=0.05

cost = @(p) total_sse(exp(p(1)), exp(p(2)), trials, a_pass, d_pass, u_step);

opts = optimset('Display','iter','MaxFunEvals',8000,'MaxIter',8000,...
                'TolX',1e-10,'TolFun',1e-10);
p_hat = fminsearch(cost, p0, opts);

k_u   = exp(p_hat(1));
tau_u = exp(p_hat(2));

fprintf('\n════════════════════════════════════════\n');
fprintf('  Fitted actuator parameters\n');
fprintf('  k_u   = %.5f  (deg/s^2 per PPS)\n', k_u);
fprintf('  tau_u = %.4f  s\n', tau_u);
fprintf('════════════════════════════════════════\n\n');

%% ── 6. Per-trial RMSE ───────────────────────────────────────────────────
fprintf('Per-trial RMSE:\n');
for k = 1:numel(trials)
    th_sim = simulate_trial(trials(k).t, trials(k).theta0, trials(k).q0, ...
                            k_u, tau_u, a_pass, d_pass, u_step);
    rmse = sqrt(mean((th_sim - trials(k).theta).^2));
    fprintf('  Trial %d: RMSE = %.3f deg\n', k, rmse);
end

%% ── 7. Transfer function ────────────────────────────────────────────────
% Theta(s)/U(s) = k_u / ( (tau_u*s + 1)*(s^2 + d_pass*s - a_pass) )
fprintf('\nTransfer function  Theta(s)/U(s):\n');
fprintf('         %.5f\n', k_u);
fprintf('  ─────────────────────────────────────────────────────\n');
fprintf('  (%.4f*s + 1) * (s^2 + %.3f*s - %.3f)\n\n', tau_u, d_pass, a_pass);

if exist('tf','file') == 2
    num = k_u;
    den = conv([tau_u 1], [1 d_pass -a_pass]);
    G = tf(num, den);
    fprintf('  Transfer function object G created.\n');
    disp(G);
    poles_G = pole(G);
    fprintf('  Poles: '); fprintf('%.4f  ', poles_G); fprintf('\n');
end

%% ── 8. Plots ────────────────────────────────────────────────────────────
n = numel(trials);
figure('Name','Motor step fit','Color','w','Position',[100 100 300*n 520]);

for k = 1:n
    th_sim = simulate_trial(trials(k).t, trials(k).theta0, trials(k).q0, ...
                            k_u, tau_u, a_pass, d_pass, u_step);

    subplot(2, n, k);
    plot(trials(k).t, trials(k).theta, 'b-',  'LineWidth', 1.8); hold on;
    plot(trials(k).t, th_sim,          'r--', 'LineWidth', 1.5);
    yline(0, 'k:', 'LineWidth', 0.5);
    rmse = sqrt(mean((th_sim - trials(k).theta).^2));
    title(sprintf('Trial %d  |  RMSE %.2f°', k, rmse), 'FontSize', 9);
    ylabel('\theta (°)'); xlabel('Time (s)'); grid on;
    if k == 1, legend('Measured','Model','Location','southwest','FontSize',7); end
    ylim([-18 4]);

    subplot(2, n, n+k);
    plot(trials(k).t, trials(k).q, 'r-', 'LineWidth', 1.8);
    yline(0, 'k:', 'LineWidth', 0.5);
    ylabel('q (°/s)'); xlabel('Time (s)'); grid on;
    title(sprintf('Gyro rate — trial %d', k), 'FontSize', 9);
    ylim([-80 30]);
end

sgtitle(sprintf('Actuated plant fit  |  k_u=%.5f  tau_u=%.4fs', k_u, tau_u));

try
    exportgraphics(gcf,'motor_step_fit.png','Resolution',200);
    fprintf('Saved motor_step_fit.png\n');
catch
end

%% ── Local functions ──────────────────────────────────────────────────────

function sse = total_sse(k_u, tau_u, trials, a_pass, d_pass, u_step)
    if ~isfinite(k_u) || ~isfinite(tau_u) || k_u <= 0 || tau_u <= 0
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
    % State: [theta; q; z]  (all in degrees / deg/s)
    ode_fun = @(~, x) [ x(2);
                         a_pass*x(1) - d_pass*x(2) + k_u*x(3);
                        (1/tau_u)*(u - x(3)) ];
    x0   = [theta0; q0; 0.0];   % z starts at 0 (motor at rest before step)
    opts = odeset('RelTol',1e-8,'AbsTol',1e-10);
    [~, X] = ode45(ode_fun, t_seg, x0, opts);
    theta_out = X(:,1);
end

end % function fit_motor_step
