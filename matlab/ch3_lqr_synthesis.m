%% LQR synthesis from the identified plant
% *Two-wheeled self-balancing robot — dissertation reproduction script, chapter 3*
% 
% This Live Script reproduces the LQR synthesis reported in section 3.5.6 
% and validated in section 4.6. It loads the identified state-space plant 
% from Stage 2, solves the continuous-time algebraic Riccati equation with 
% weights $Q = \mathrm{diag}(100, 1)$ and $R = 1$, maps the resulting 
% state-feedback gain onto the cascade's proportional gains using the 
% algebraic identity $K_\theta = K_{p,q} K_{p,\theta}$, $K_q = K_{p,q}$, 
% and evaluates the closed-loop eigenvalues against three gain combinations:
%% 
% * the LQR design point $(7.0, 37.0)$ — dominant pole at 0.78 Hz (real, overdamped);
% * the bandwidth-widening attempt $(?, ?)$ at 1.20 Hz — which produced 
% the limit cycle in session 022943;
% * the retuned deployed gains $(10, 18)$ — complex pair at 0.59 Hz.
% 
% *Expected output:*
%% 
% * $K = [K_\theta, K_q] \approx [259.94, 37.01]$
% * Cascade proportional gains $K_{p,q} \approx 37.0$, $K_{p,\theta} \approx 7.02$
% * LQR closed-loop poles at approximately $-4.99, -7.01$ s$^{-1}$
% * Retuned closed-loop poles at approximately $-3.445 \pm 1.316j$ s$^{-1}$
%% Load identified plant
clear; clc; close all;

if isfile('stage2_params.mat')
    S = load('stage2_params.mat');
    A = S.A;  B = S.B;  g = S.g;  l_eff = S.l_eff;  d_eff = S.d_eff;  k_u = S.k_u;
    fprintf('Loaded identified plant from stage2_params.mat\n');
else
    warning('stage2_params.mat not found; using dissertation-reported values.');
    g = 9.81;  l_eff = 0.28;  d_eff = 2.03;  k_u = -0.270;
    A = [0, 1; g/l_eff, -d_eff];
    B = [0; k_u];
end

fprintf('\nPlant:\n');
fprintf('  A = [%+7.3f %+7.3f; %+7.3f %+7.3f]\n', A(1,1), A(1,2), A(2,1), A(2,2));
fprintf('  B = [%+7.3f; %+7.3f]\n', B(1), B(2));

fprintf('\nOpen-loop eigenvalues (unstable upright): ');
disp(eig(A)');
%% LQR synthesis
% The weights express the engineering judgement that pitch error is the 
% primary state to regulate (100:1 against pitch rate) and that $R = 1$ fixes 
% the absolute scale of the cost.
Q = diag([100, 1]);
R = 1;

fprintf('\nLQR weights:\n');
disp(Q)
fprintf('  R = %.1f\n\n', R);

% Requires Control System Toolbox (lqr + care).  If not available, a manual 
% CARE solver via matrix Hamiltonian is provided below as a fallback.
try
    [K_state, P, cl_poles] = lqr(A, B, Q, R);
    fprintf('Used lqr() from Control System Toolbox.\n');
catch
    warning('lqr() unavailable; solving CARE manually via the Hamiltonian.');
    [K_state, P] = manualCARE(A, B, Q, R);
    cl_poles = eig(A - B*K_state);
end

K_theta = K_state(1);
K_q     = K_state(2);

fprintf('\n---- LQR state-feedback gain ----\n');
fprintf('  K = [K_theta, K_q] = [%.3f, %.3f]\n', K_theta, K_q);
fprintf('  LQR closed-loop poles = ');
disp(cl_poles')
%% Cascade mapping
% The cascade architecture does not implement full-state feedback as a single 
% operation; the outer loop produces a rate setpoint from pitch, and the inner 
% loop produces the motor command from rate error. The equivalent proportional 
% gains satisfy
% 
% $$K_\theta = K_{p,q} K_{p,\theta}, \quad K_q = K_{p,q}.$$
Kp_q_LQR     = K_q;
Kp_theta_LQR = K_theta / K_q;

fprintf('\n---- LQR-derived cascade gains ----\n');
fprintf('  K_{p,theta} = K_theta / K_q = %.4f\n', Kp_theta_LQR);
fprintf('  K_{p,q}     = K_q           = %.4f\n', Kp_q_LQR);
fprintf('(Reported in dissertation as K_{p,theta}=7.0, K_{p,q}=37.0.)\n');

%% Verification: closed-loop under cascade mapping
% Evaluate the closed-loop system $\dot x = (A + BK_{\mathrm{cascade}})x$, where 
% $K_{\mathrm{cascade}} = [K_{p,q} K_{p,\theta}, K_{p,q}]$. Because we just 
% computed the equivalence, the closed-loop poles should match the LQR poles.
K_cascade_LQR = [Kp_q_LQR * Kp_theta_LQR, Kp_q_LQR];
A_cl_LQR = A + B * (-K_cascade_LQR);      % A - B*K_state
cl_poles_cascade = eig(A_cl_LQR);
fprintf('\nClosed-loop poles via cascade mapping: ');
disp(cl_poles_cascade')
fprintf('(These should match the LQR poles above; discrepancy indicates a ');
fprintf('mapping error.)\n');
%% Retuned gains and closed-loop analysis
% The deployed gains under which the headline results are obtained are 
% $K_{p,\theta} = 10$ and $K_{p,q} = 18$. The cascade's total angle gain drops 
% from $K_\theta = 259$ under LQR to $K_\theta = 180$ under the retune — a 
% reduction to approximately 70% of the LQR value.
Kp_theta_retune = 10;
Kp_q_retune     = 18;

K_cascade_retune = [Kp_q_retune * Kp_theta_retune, Kp_q_retune];
A_cl_retune = A + B * (-K_cascade_retune);
cl_poles_retune = eig(A_cl_retune);

fprintf('\n---- Retuned closed-loop (deployed) ----\n');
fprintf('  K_{p,theta} = %d, K_{p,q} = %d\n', Kp_theta_retune, Kp_q_retune);
fprintf('  K_theta     = K_{p,q} * K_{p,theta} = %d\n', Kp_q_retune*Kp_theta_retune);
fprintf('  Retuned poles: ');
disp(cl_poles_retune')

% Natural frequency and damping ratio
if any(imag(cl_poles_retune) ~= 0)
    p = cl_poles_retune(1);
    wn = abs(p);
    zeta = -real(p) / wn;
    f_n = wn / (2*pi);
    fprintf('  Natural frequency   = %.3f rad/s (%.3f Hz)\n', wn, f_n);
    fprintf('  Damping ratio zeta  = %.3f\n', zeta);
    fprintf('(Dissertation reports f_n ~ 0.59 Hz and zeta ~ 0.93.)\n');
end
%% Gain-scheduler bandwidth growth (failure-mechanism check)
% The outer-loop gain schedule is $K_{p,\theta}^{\mathrm{eff}} = K_{p,\theta} + a_\theta|e_\theta|$ 
% with $a_\theta = 5$. The closed-loop natural frequency grows as $|e_\theta|$ grows:
% 
% $$\omega_n(e_\theta) = \sqrt{|k_u|\,K_{p,q}(K_{p,\theta} + a_\theta|e_\theta|) - g/l_{\mathrm{eff}}}.$$
% 
% The dissertation shows that this crosses the complementary filter's 0.98 Hz 
% corner at $|e_\theta| \approx 1.0°$, and reaches 1.20 Hz (the observed 
% limit-cycle bandwidth) at $|e_\theta| \approx 1.78°$.
a_theta = 5;
e_theta_grid = 0:0.01:3;
wn_grid = zeros(size(e_theta_grid));
for i = 1:numel(e_theta_grid)
    Kp_eff = Kp_theta_retune + a_theta * e_theta_grid(i);
    arg = abs(k_u) * Kp_q_retune * Kp_eff - g/l_eff;
    wn_grid(i) = sqrt(max(arg, 0));
end
fn_grid = wn_grid / (2*pi);

figure('Name','Gain-scheduler bandwidth growth','Color','w');
plot(e_theta_grid, fn_grid, 'LineWidth', 2); hold on;
yline(0.98, '--r', 'CF corner 0.98 Hz');
yline(1.20, '--k', 'Limit-cycle bandwidth 1.20 Hz');
xlabel('|e_\theta| [deg]'); ylabel('Closed-loop natural frequency [Hz]');
title('Gain-scheduler-driven bandwidth growth with tracking error');
grid on;

% Find the error magnitudes at which f_n crosses the two thresholds
e_cross_cf = interp1(fn_grid, e_theta_grid, 0.98, 'linear');
e_cross_lc = interp1(fn_grid, e_theta_grid, 1.20, 'linear');
fprintf('\n---- Failure-mechanism arithmetic ----\n');
fprintf('  omega_n(0) = %.3f Hz   (quiescent)\n', fn_grid(1));
fprintf('  f_n crosses 0.98 Hz at |e_theta| = %.3f deg\n', e_cross_cf);
fprintf('  f_n crosses 1.20 Hz at |e_theta| = %.3f deg\n', e_cross_lc);
fprintf('(Dissertation reports crossings at ~1.00 and ~1.78 deg.)\n');
%% Save for the next script
save('cascade_params.mat', 'A', 'B', 'Kp_theta_LQR', 'Kp_q_LQR', ...
     'Kp_theta_retune', 'Kp_q_retune', 'a_theta', 'k_u', 'l_eff', 'd_eff', 'g');
fprintf('\nSaved cascade_params.mat for the balance-analysis script.\n');
%% Local fallback: manual CARE via the Hamiltonian matrix
% Used only if the Control System Toolbox is not available. Solves the CARE
%   A'P + PA - PBR^-1B'P + Q = 0
% by diagonalising the Hamiltonian $H = [A,-BR^{-1}B';-Q,-A']$ and selecting 
% the stable invariant subspace.
function [K, P] = manualCARE(A, B, Q, R)
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
