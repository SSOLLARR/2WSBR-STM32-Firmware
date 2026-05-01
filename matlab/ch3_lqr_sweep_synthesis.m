%% LQR cost-weight sensitivity sweep — synthesis stage
% *Two-wheeled self-balancing robot — dissertation reproduction script, chapter 3*
%
% This Live Script extends |ch3_lqr_synthesis.m| with a structured sensitivity 
% sweep over the LQR cost weights. The deployed weights $Q = \mathrm{diag}(100,1)$, 
% $R = 1$ are one point in a seven-point grid; the remainder probe the 
% surrounding region so the chosen weighting can be characterised, not merely 
% asserted.
%
% Scope and conventions:
%
% * The identified plant $(A, B)$ from Stage 2 is held fixed across the sweep.
% * $R = 1$ throughout. Only the diagonal of $Q$ is varied; varying $R$ is 
%   redundant up to scaling.
% * The deployed integrator gains $K_{i,\theta} = 2$, $K_{i,q} = 15$ are held 
%   constant during the hardware sweep and are not part of the synthesis here.
% * Cascade decomposition is only valid when $\mathrm{sign}(K_\theta) = 
%   \mathrm{sign}(K_q)$; non-decomposable points are flagged and skipped on 
%   hardware.
%
% *Expected output:*
%
% * A table of seven $(Q, K, K_{p,\theta}, K_{p,q})$ rows ready to be 
%   transcribed into BT |OPx.xx| / |IPx.xx| commands.
% * Grid point #2 must reproduce $K = [259.94, 37.01]$ and cascade 
%   $(K_{p,\theta}, K_{p,q}) = (7.02, 37.01)$. If it does not, the plant or 
%   sign convention has drifted and the sweep is invalid.
% * |sweep_synthesis.mat| saved for the analysis script to consume.
%% Load identified plant
clear; clc; close all;

if isfile('stage2_params.mat')
    S = load('stage2_params.mat');
    A = S.A;  B = S.B;  g = S.g;  l_eff = S.l_eff;  d_eff = S.d_eff;  k_u = S.k_u;
    fprintf('Loaded identified plant from stage2_params.mat\n');
elseif isfile('cascade_params.mat')
    P0 = load('cascade_params.mat');
    A = P0.A;  B = P0.B;  k_u = P0.k_u;  l_eff = P0.l_eff;  d_eff = P0.d_eff;  g = P0.g;
    fprintf('Loaded identified plant from cascade_params.mat\n');
else
    warning('Identified-plant .mat not found; using dissertation-reported values.');
    g = 9.81;  l_eff = 0.28;  d_eff = 2.03;  k_u = -0.270;
    A = [0, 1; g/l_eff, -d_eff];
    B = [0; k_u];
end

fprintf('\nPlant:\n');
fprintf('  A = [%+7.3f %+7.3f; %+7.3f %+7.3f]\n', A(1,1), A(1,2), A(2,1), A(2,2));
fprintf('  B = [%+7.3f; %+7.3f]\n', B(1), B(2));
fprintf('  Open-loop eigenvalues: ');
disp(eig(A)');
%% Define the sweep grid
% Seven points, ordered as committed before any hardware run. The numbering 
% is preserved in the analysis script to allow per-grid-point session 
% identification.
%
% Grid #2 is the deployed-appendix point and serves as the synthesis sanity 
% check. Grids #6 and #7 are scaled versions of #2; #1 and #3 vary the 
% pitch-error weight by a decade in each direction; #4 and #5 vary the 
% pitch-rate weight relative to a fixed pitch weight.
sweep_grid(1) = struct('id', 1, 'Q', diag([10,    1]),   'note', 'low pitch weight');
sweep_grid(2) = struct('id', 2, 'Q', diag([100,   1]),   'note', 'deployed appendix point (sanity check)');
sweep_grid(3) = struct('id', 3, 'Q', diag([1000,  1]),   'note', 'high pitch weight');
sweep_grid(4) = struct('id', 4, 'Q', diag([100,   0.1]), 'note', 'aggressive pitch / weak rate damping');
sweep_grid(5) = struct('id', 5, 'Q', diag([100,   10]),  'note', 'bias toward (10,18) refinement direction');
sweep_grid(6) = struct('id', 6, 'Q', diag([500,   5]),   'note', 'scaled-up #2 (5x)');
sweep_grid(7) = struct('id', 7, 'Q', diag([50,    5]),   'note', 'scaled-down #2 (0.5x pitch, 5x rate)');

R = 1;
n_grid = numel(sweep_grid);
%% LQR synthesis across the grid
% For each grid point: solve the CARE, recover $K = [K_\theta, K_q]$, attempt 
% the cascade decomposition $K_{p,q} = K_q$, $K_{p,\theta} = K_\theta / K_q$, 
% and compute the closed-loop eigenvalues for diagnostic context.
results = repmat(struct('id',[], 'Q',[], 'K',[], 'Kp_theta',[], 'Kp_q',[], ...
                         'cl_poles',[], 'decomposable',[], 'note',''), n_grid, 1);

fprintf('\n%-3s %-22s %-20s %-22s %-22s\n', ...
    '#', 'Q', 'K=[K_theta K_q]', 'Cascade (OP, IP)', 'Closed-loop poles');
fprintf('%s\n', repmat('-', 1, 100));

for i = 1:n_grid
    Q = sweep_grid(i).Q;
    try
        [K_state, ~, cl_poles] = lqr(A, B, Q, R);
    catch
        warning('lqr() unavailable for grid #%d; using manual CARE.', i);
        [K_state, ~] = manualCARE(A, B, Q, R);
        cl_poles = eig(A - B*K_state);
    end
    K_theta = K_state(1);
    K_q     = K_state(2);

    % Cascade decomposition: only meaningful if both gains share a sign.
    same_sign = (sign(K_theta) == sign(K_q)) && (K_q ~= 0);
   if same_sign
        Kp_theta = K_theta / K_q;       % ratio: sign-invariant
        Kp_q     = abs(K_q);            % firmware deploys magnitude; MOTOR_SIGN handles polarity
        decomp_str = sprintf('OP=%7.3f IP=%7.3f', Kp_theta, Kp_q);
    else
        Kp_theta = NaN;  Kp_q = NaN;
        decomp_str = 'NON-DECOMPOSABLE';
    end

    results(i).id           = sweep_grid(i).id;
    results(i).Q            = Q;
    results(i).K            = K_state;
    results(i).Kp_theta     = Kp_theta;
    results(i).Kp_q         = Kp_q;
    results(i).cl_poles     = cl_poles;
    results(i).decomposable = same_sign;
    results(i).note         = sweep_grid(i).note;

    pole_str = sprintf('%+6.2f, %+6.2f', cl_poles(1), cl_poles(2));
    if any(imag(cl_poles) ~= 0)
        pole_str = sprintf('%+6.2f%+6.2fj (pair)', real(cl_poles(1)), abs(imag(cl_poles(1))));
    end

    fprintf('#%-2d diag(%4g, %5g)   [%+7.2f %+6.2f]   %-22s %s\n', ...
        sweep_grid(i).id, Q(1,1), Q(2,2), K_theta, K_q, decomp_str, pole_str);
end
%% Sanity check: grid #2 must reproduce the appendix point
% The deployed weights $Q = \mathrm{diag}(100, 1)$, $R = 1$ produce 
% $K = [259.94, 37.01]$ and cascade $(K_{p,\theta}, K_{p,q}) = (7.02, 37.01)$ 
% in the appendix. Recovering these here confirms the plant has not drifted 
% between scripts.
ref = struct('K_theta', 259.94, 'K_q', 37.01, 'Kp_theta', 7.02, 'Kp_q', 37.01);
tol = struct('K', 1.0, 'Kp_theta', 0.1, 'Kp_q', 1.0);

g2 = results(2);
errs = struct( ...
    'K_theta',  abs(abs(g2.K(1)) - ref.K_theta), ...
    'K_q',      abs(abs(g2.K(2)) - ref.K_q), ...
    'Kp_theta', abs(g2.Kp_theta  - ref.Kp_theta), ...
    'Kp_q',     abs(g2.Kp_q      - ref.Kp_q));

fprintf('\n---- Sanity check vs appendix G.4 (grid #2) ----\n');
fprintf('  |K_theta|  : %.3f  (ref %.3f, |err| %.3f, tol %.1f) %s\n', ...
    abs(g2.K(1)), ref.K_theta, errs.K_theta, tol.K,    pf(errs.K_theta  < tol.K));
fprintf('  |K_q|      : %.3f  (ref %.3f, |err| %.3f, tol %.1f) %s\n', ...
    abs(g2.K(2)), ref.K_q,     errs.K_q,     tol.K,    pf(errs.K_q      < tol.K));
fprintf('  K_p,theta  : %.3f  (ref %.3f, |err| %.3f, tol %.2f) %s\n', ...
    g2.Kp_theta,  ref.Kp_theta, errs.Kp_theta, tol.Kp_theta, pf(errs.Kp_theta < tol.Kp_theta));
fprintf('  K_p,q      : %.3f  (ref %.3f, |err| %.3f, tol %.1f) %s\n', ...
    g2.Kp_q,      ref.Kp_q,     errs.Kp_q,     tol.Kp_q,     pf(errs.Kp_q     < tol.Kp_q));

if (errs.K_theta < tol.K) && (errs.K_q < tol.K) && ...
   (errs.Kp_theta < tol.Kp_theta) && (errs.Kp_q < tol.Kp_q)
    fprintf('\n  Sanity PASS — synthesis chain matches appendix. Proceed to hardware.\n');
else
    error(['Sanity FAIL — grid #2 does not match appendix. Plant or sign ' ...
           'convention has drifted; resolve before any hardware run.']);
end
%% Bluetooth command table (transcribe to robot)
% For each decomposable grid point, the BT command pair below sets the 
% proportional gains. Integrator gains |OI=2|, |II=15| are held constant 
% across the sweep and are not part of these commands.
fprintf('\n---- BT command table ----\n');
fprintf('%-3s %-26s %-10s %-10s\n', '#', 'note', 'OP cmd', 'IP cmd');
fprintf('%s\n', repmat('-', 1, 60));
for i = 1:n_grid
    r = results(i);
    if r.decomposable
        fprintf('#%-2d %-26s OP%-8.3f IP%-8.3f\n', ...
            r.id, r.note, r.Kp_theta, r.Kp_q);
    else
        fprintf('#%-2d %-26s SKIP (non-decomposable)\n', r.id, r.note);
    end
end
fprintf('\nProtocol per grid point (also send before each):\n');
fprintf('  1. OP<value>     (set angle_Kp)\n');
fprintf('  2. IP<value>     (set rate_Kp)\n');
fprintf('  3. ZE            (reset controllers and integrators)\n');
fprintf('  4. MO0           (enter MODE_BALANCING)\n');
fprintf('  5. lift to upright, hold ~2 s, release\n');
fprintf('  Stop conditions: fall, drift > 0.5 m, 10 min elapsed.\n');
%% Save sweep synthesis for the analysis script
% The analysis script consumes |sweep_synthesis.mat| to map grid point id to 
% the synthesised gains, so the post-hoc table is fully reproducible from 
% the .mat alone.
save('sweep_synthesis.mat', 'A', 'B', 'R', 'g', 'l_eff', 'd_eff', 'k_u', ...
     'sweep_grid', 'results');
fprintf('\nSaved sweep_synthesis.mat for ch4_lqr_sweep_analysis.m\n');
%% Helpers
function s = pf(b)
    if b, s = '[PASS]'; else, s = '[FAIL]'; end
end

function [K, P] = manualCARE(A, B, Q, R)
% Manual CARE solver via the Hamiltonian, replicated from ch3_lqr_synthesis.m.
    n = size(A, 1);
    H = [A, -B*(R\B'); -Q, -A'];
    [V, D] = eig(H);
    d = diag(D);
    stableIdx = find(real(d) < 0);
    if numel(stableIdx) ~= n
        error('CARE: wrong number of stable eigenvalues (%d, expected %d).', ...
            numel(stableIdx), n);
    end
    Vs = V(:, stableIdx);
    X1 = Vs(1:n, :);
    X2 = Vs(n+1:end, :);
    P = real(X2 / X1);
    K = R \ (B' * P);
end
