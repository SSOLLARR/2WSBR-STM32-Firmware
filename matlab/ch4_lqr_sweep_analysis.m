%% LQR cost-weight sensitivity sweep — telemetry analysis stage
% *Two-wheeled self-balancing robot — dissertation reproduction script, chapter 4*
%
% Companion to |ch3_lqr_sweep_synthesis.m|. This Live Script consumes the 
% session .mat files produced by the seven hardware runs of the structured 
% Q-weight sweep, computes per-run quiescent-window statistics against the 
% same four performance objectives used in |ch4_balance_analysis.m|, produces 
% a small-multiples figure, and tabulates the result for the appendix G.4.2 
% rewrite.
%
% Conventions:
%
% * The authoritative time axis is $t = \mathrm{sample\_idx} \times 0.005$ s. 
%   PC-side wall clock is bursty and is diagnostic only.
% * |session.colnames| confirms the 10-column schema; |sample_idx| is the 
%   last column.
% * Quiescent window matches |ch4_balance_analysis.m| at $[2, 30]$ s.
% * Saturation events count samples with $|u_{\mathrm{cmd}}| \geq 3799$ pps 
%   (one count below the locked $\pm 3800$ limit, to absorb numerical 
%   roundoff at the rail).
% * If a run lasts shorter than the full quiescent window the σ_θ statistic 
%   is reported over $[2, t_{\mathrm{fall}} - 2]$ s with a flag.
%
% *Configure |runs| below* with the grid-point → session-file mapping after 
% the hardware sweep is complete.
%% Configure: grid-point to session-file mapping
% Edit this block after each hardware run. The grid id matches the id field 
% of |sweep_grid| in |sweep_synthesis.mat|. Multiple trials per grid point 
% are supported by repeating the id with a different file.
clear; clc; close all;

logs_dir = 'logs';   % session files live in logs/<session>/telemetry_data.mat
                     % or as flat session_*.mat in the working directory

runs = struct( ...
    'id',   {1, 2, 3, 4, 5, 6, 7}, ...
    'file', { ...
        'session_YYYYMMDD_HHMMSS.mat', ...   % grid #1: diag(10, 1)
        'session_YYYYMMDD_HHMMSS.mat', ...   % grid #2: diag(100, 1)   <- sanity baseline
        'session_YYYYMMDD_HHMMSS.mat', ...   % grid #3: diag(1000, 1)
        'session_YYYYMMDD_HHMMSS.mat', ...   % grid #4: diag(100, 0.1)
        'session_YYYYMMDD_HHMMSS.mat', ...   % grid #5: diag(100, 10)
        'session_YYYYMMDD_HHMMSS.mat', ...   % grid #6: diag(500, 5)
        'session_YYYYMMDD_HHMMSS.mat'  ...   % grid #7: diag(50, 5)
    });

% Quiescent window for the objective evaluation (same as ch4_balance_analysis)
quiescent_start_s = 2.0;
quiescent_end_s   = 30.0;

% Actuator constants (locked firmware configuration)
u_sat_pps     = 3800;
sat_thresh_pps = 3799;
wheel_diam_m  = 0.065;
steps_per_rev = 200 * 16;        % 1/16 microstepping, 200 full-steps/rev
%% Load synthesis side-cargo
% The synthesis script saves the grid definition and the resulting K, K_p,θ, 
% K_p,q values keyed by grid id. The analysis carries the grid metadata 
% across so the final table is self-contained.
if ~isfile('sweep_synthesis.mat')
    error(['sweep_synthesis.mat not found. Run ch3_lqr_sweep_synthesis.m ' ...
           'first.']);
end
S = load('sweep_synthesis.mat');
fprintf('Loaded sweep_synthesis.mat (%d grid points)\n', numel(S.results));
%% Per-run analysis loop
% For each configured run: locate the .mat (logs/<id>/telemetry_data.mat 
% takes precedence over a flat session_*.mat); load via the local helper; 
% compute statistics; collect into a results table.
n_runs = numel(runs);
A = repmat(struct('id',[], 'file','', 'found',false, 'duration_s',NaN, ...
                  't_fall_s',NaN, 'sigma_theta_deg',NaN, 'mean_theta_deg',NaN, ...
                  'max_abs_theta_deg',NaN, 'max_abs_u_pps',NaN, ...
                  'sat_events',NaN, 'net_disp_mm',NaN, 'q_window_s',[NaN NaN], ...
                  'short_run',false, 't',[],'theta',[],'q',[],'u_cmd',[],'u_app',[],'mode',[]), ...
            n_runs, 1);

for k = 1:n_runs
    A(k).id   = runs(k).id;
    A(k).file = runs(k).file;
    full = locate_session(A(k).file, logs_dir);

    if isempty(full) || ~isfile(full)
        warning('Run %d (grid #%d): file %s not found, skipping.', ...
            k, A(k).id, A(k).file);
        continue;
    end
    A(k).found = true;
    fprintf('\n--- Run %d : grid #%d : %s ---\n', k, A(k).id, full);

    T = load_session_local(full);
    t       = T.t_mcu_s;
    theta   = T.estimated_pitch_deg;
    q       = T.gyro_rate_dps;
    u_cmd   = T.motor_cmd_pps;
    u_app   = T.motor_applied_pps;
    mode    = T.mode;

    % Restrict to MODE_BALANCING samples only.
    bal = (mode == 0);
    if nnz(bal) < 10
        warning('  No usable balancing samples; skipping.');
        continue;
    end
    t_bal = t(bal);  theta_bal = theta(bal);  q_bal = q(bal);
    u_cmd_bal = u_cmd(bal);  u_app_bal = u_app(bal);

    % Re-zero balancing time so each run starts at t = 0.
    t_bal = t_bal - t_bal(1);

    duration_s = t_bal(end) - t_bal(1);
    A(k).duration_s = duration_s;
    A(k).t_fall_s   = duration_s;     % for short runs, fall = end
    fprintf('  Balancing duration : %.2f s (%d samples)\n', duration_s, nnz(bal));

    % Quiescent window: clamp to available duration.
    if duration_s < quiescent_start_s + 1.0
        warning('  Run too short (%.1f s) for any quiescent window; ', duration_s);
        warning('  reporting fall stats only.');
        A(k).short_run = true;
        A(k).t = t_bal;  A(k).theta = theta_bal;  A(k).q = q_bal;
        A(k).u_cmd = u_cmd_bal;  A(k).u_app = u_app_bal;  A(k).mode = mode(bal);
        continue;
    end

    q_end = min(duration_s - 1.0, quiescent_end_s);
    qmask = (t_bal >= quiescent_start_s) & (t_bal <= q_end);
    A(k).q_window_s = [quiescent_start_s, q_end];
    A(k).short_run  = (q_end < quiescent_end_s);

    theta_q = theta_bal(qmask);
    u_q     = u_app_bal(qmask);
    t_q     = t_bal(qmask);

    A(k).sigma_theta_deg   = std(theta_q);
    A(k).mean_theta_deg    = mean(theta_q);
    A(k).max_abs_theta_deg = max(abs(theta_q));
    A(k).max_abs_u_pps     = max(abs(u_q));
    A(k).sat_events        = sum(abs(u_q) >= sat_thresh_pps);

    % Net wheel displacement: integrate applied step rate over the quiescent
    % window, divide by steps_per_rev to get revolutions, multiply by wheel
    % circumference.
    steps_cumulative = cumtrapz(t_q, u_q);
    rev_cumulative   = steps_cumulative / steps_per_rev;
    displacement_m   = rev_cumulative * pi * wheel_diam_m;
    A(k).net_disp_mm = (displacement_m(end) - displacement_m(1)) * 1000;

    fprintf('  Quiescent [%.1f, %.1f] s%s\n', A(k).q_window_s(1), ...
        A(k).q_window_s(2), tern(A(k).short_run, ' (truncated)', ''));
    fprintf('    sigma(theta)      = %.4f deg\n', A(k).sigma_theta_deg);
    fprintf('    mean(theta)       = %.4f deg\n', A(k).mean_theta_deg);
    fprintf('    max|theta|        = %.4f deg\n', A(k).max_abs_theta_deg);
    fprintf('    max|u|            = %.1f pps  (sat = +/-%d)\n', ...
        A(k).max_abs_u_pps, u_sat_pps);
    fprintf('    saturation events = %d\n', A(k).sat_events);
    fprintf('    net displacement  = %+.2f mm  (over %.1f s)\n', ...
        A(k).net_disp_mm, t_q(end) - t_q(1));

    % Stash trace for the small-multiples figure.
    A(k).t = t_bal;  A(k).theta = theta_bal;  A(k).q = q_bal;
    A(k).u_cmd = u_cmd_bal;  A(k).u_app = u_app_bal;  A(k).mode = mode(bal);
end
%% Comparison table
% One row per grid point, ordered by id. The K and (K_p,θ, K_p,q) columns 
% come from the synthesis side-cargo; the remaining columns come from the 
% per-run statistics above.
fprintf('\n\n==== Sweep comparison table ====\n');
fprintf('%-3s %-22s %-18s %-14s %-9s %-12s %-9s %-7s %-13s\n', ...
    '#', 'Q', 'K=[K_th K_q]', 'Cascade', 't_fall[s]', 'sigma[deg]', ...
    '|disp|[mm]', '#sat', 'note');
fprintf('%s\n', repmat('-', 1, 130));
for k = 1:n_runs
    id = runs(k).id;
    % Find the matching synthesis result.
    sidx = find([S.results.id] == id, 1, 'first');
    if isempty(sidx), continue; end
    r = S.results(sidx);
    a = A(k);

    qstr = sprintf('diag(%g, %g)', r.Q(1,1), r.Q(2,2));
    kstr = sprintf('[%+6.2f %+5.2f]', r.K(1), r.K(2));
    if r.decomposable
        cstr = sprintf('(%5.2f, %5.2f)', r.Kp_theta, r.Kp_q);
    else
        cstr = '(NON-DECOMP)';
    end

    if a.found
        tfs   = sprintf('%6.2f', a.t_fall_s);
        sigs  = sprintf('%8.4f', a.sigma_theta_deg);
        dsps  = sprintf('%7.2f',  abs(a.net_disp_mm));
        sats  = sprintf('%5d',    a.sat_events);
        notes = '';
        if a.short_run, notes = 'short run'; end
    else
        tfs = '   N/A'; sigs = '     N/A'; dsps = '    N/A'; sats = '  N/A';
        notes = 'NO TELEMETRY';
    end

    fprintf('#%-2d %-22s %-18s %-14s %-9s %-12s %-9s %-7s %-13s\n', ...
        id, qstr, kstr, cstr, tfs, sigs, dsps, sats, notes);
end
%% Identify the best-measured point
% "Best" is defined as longest balancing duration. Quiescent σ_θ is reported 
% alongside as the secondary metric, but ranking is by t_fall, consistent 
% with the headline objective ("match or beat the 9-min empirical run").
valid = arrayfun(@(a) a.found && ~isnan(a.t_fall_s), A);
if any(valid)
    [~, best_local] = max(arrayfun(@(a) a.t_fall_s, A(valid)));
    valid_idx = find(valid);
    best_k = valid_idx(best_local);
    best_id = runs(best_k).id;
    sidx = find([S.results.id] == best_id, 1, 'first');

    fprintf('\n---- Best-measured grid point ----\n');
    fprintf('  Grid #%d : Q = diag(%g, %g)\n', ...
        best_id, S.results(sidx).Q(1,1), S.results(sidx).Q(2,2));
    fprintf('  Cascade gains (OP, IP) = (%.2f, %.2f)\n', ...
        S.results(sidx).Kp_theta, S.results(sidx).Kp_q);
    fprintf('  Duration  = %.2f s\n', A(best_k).t_fall_s);
    if ~isnan(A(best_k).sigma_theta_deg)
        fprintf('  sigma     = %.4f deg over [%.1f, %.1f] s\n', ...
            A(best_k).sigma_theta_deg, A(best_k).q_window_s(1), ...
            A(best_k).q_window_s(2));
    end
    fprintf('  Saturation events: %d\n', A(best_k).sat_events);
else
    warning('No valid runs found; configure runs(*).file before re-running.');
end
%% Small-multiples figure
% One panel per grid point, same y-axis limits across panels for visual 
% comparison. Quiescent-window markers and the safety threshold are shown 
% on each panel.
plot_runs = arrayfun(@(a) a.found && ~isempty(a.theta), A);
if any(plot_runs)
    pidx = find(plot_runs);
    n_panels = numel(pidx);
    n_cols = min(3, n_panels);
    n_rows = ceil(n_panels / n_cols);

    figure('Name', 'Sweep small-multiples', 'Color', 'w', ...
        'Position', [100, 100, 360*n_cols, 220*n_rows]);

    % Common y-limits sized by the worst-behaved run (clipped to fall threshold)
    y_max = max(arrayfun(@(a) max(abs(a.theta)), A(pidx)));
    y_max = min(max(y_max, 1.0), 8.5);

    for kk = 1:n_panels
        k = pidx(kk);
        id = runs(k).id;
        sidx = find([S.results.id] == id, 1, 'first');

        subplot(n_rows, n_cols, kk);
        plot(A(k).t, A(k).theta, 'LineWidth', 0.9); hold on;
        yline(0, '-', 'Color', [0.6 0.6 0.6]);
        yline(8, '--r'); yline(-8, '--r');
        if ~A(k).short_run && all(isfinite(A(k).q_window_s))
            xline(A(k).q_window_s(1), ':k');
            xline(A(k).q_window_s(2), ':k');
        end
        ylim([-y_max, y_max]);
        xlabel('t [s]'); ylabel('\theta [deg]');
        if S.results(sidx).decomposable
            ttl = sprintf('#%d  Q=diag(%g,%g)  (OP,IP)=(%.1f,%.1f)  t=%.1fs', ...
                id, S.results(sidx).Q(1,1), S.results(sidx).Q(2,2), ...
                S.results(sidx).Kp_theta, S.results(sidx).Kp_q, A(k).t_fall_s);
        else
            ttl = sprintf('#%d  Q=diag(%g,%g)  NON-DECOMP', ...
                id, S.results(sidx).Q(1,1), S.results(sidx).Q(2,2));
        end
        title(ttl, 'FontSize', 9);
        grid on;
    end
    sgtitle('LQR cost-weight sweep — pitch records', 'FontWeight', 'bold');
end
%% Save analysis output
% Saved table is consumed by the appendix G.4.2 rewrite.
sweep_table = struct('runs', runs, 'analysis', A, 'synthesis', S.results, ...
                     'quiescent_window_s', [quiescent_start_s, quiescent_end_s]);
save('sweep_analysis.mat', 'sweep_table');
fprintf('\nSaved sweep_analysis.mat\n');
%% Helpers
function s = tern(b, ifTrue, ifFalse)
    if b, s = ifTrue; else, s = ifFalse; end
end

function full = locate_session(file, logs_dir)
% Search precedence: logs/<basename>/telemetry_data.mat, then logs/<file>, 
% then flat <file> in the working directory. Returns '' if nothing found.
    [~, base, ~] = fileparts(file);
    candidates = { ...
        fullfile(logs_dir, base, 'telemetry_data.mat'), ...
        fullfile(logs_dir, file), ...
        file };
    full = '';
    for i = 1:numel(candidates)
        if isfile(candidates{i})
            full = candidates{i};
            return;
        end
    end
end

function T = load_session_local(matfile)
% Load a session .mat in the project's standard schema:
%   colnames : cellstr or string array of length 10
%   data     : (N, 10) double matrix
%   meta     : struct with control_loop_dt_s etc.
% Returns a table with named columns and a derived t_mcu_s = sample_idx*dt.
    raw = load(matfile);
    if ~isfield(raw, 'data') || ~isfield(raw, 'colnames')
        error('Unexpected schema in %s (no data/colnames).', matfile);
    end
    data = raw.data;
    cols = cellstr(string(raw.colnames));
    if size(data, 2) ~= numel(cols)
        error('Column count mismatch in %s (%d data cols vs %d names).', ...
            matfile, size(data, 2), numel(cols));
    end

    T = array2table(data, 'VariableNames', cols);

    % Authoritative MCU-side time axis. Handle MCU resets (sample_idx jumps
    % backwards) by taking the post-last-reset segment.
    if ismember('sample_idx', cols)
        si = T.sample_idx;
        d  = diff(si);
        reset_pts = find(d < 0);
        if ~isempty(reset_pts)
            start_idx = reset_pts(end) + 1;
            T = T(start_idx:end, :);
            si = T.sample_idx;
        end
        dt = 0.005;
        if isfield(raw, 'meta') && isstruct(raw.meta) && ...
                isfield(raw.meta, 'control_loop_dt_s')
            dt = double(raw.meta.control_loop_dt_s);
        end
        T.t_mcu_s = (si - si(1)) * dt;
    else
        warning('No sample_idx column; falling back to PC time_s.');
        T.t_mcu_s = T.time_s - T.time_s(1);
    end
end
