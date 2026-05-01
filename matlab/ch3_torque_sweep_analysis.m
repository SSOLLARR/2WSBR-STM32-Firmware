%% Stage 3 — Stepper torque-speed envelope characterisation
% *Two-wheeled self-balancing robot — dissertation reproduction script*
%
% This Live Script reproduces the torque-speed sweep result reported in
% section 4.4 of the dissertation. It analyses an open-loop PPS staircase
% recording (wheels off the ground, MODE_MOTOR_STEP firmware mode), reports
% the pulses commanded per level, and compares against the rotation count
% measured by slow-motion video to test for step loss.
%
% *Expected output for session_torque_speed_sweep.mat:*
%
% * Nine PPS plateaus held 1.5 s each, ranging 1000 to 4250 PPS
% * Total pulses commanded = 40,287 across the sweep
% * Total revolutions expected (3200 pulses/rev at 1/16 microstep) = 12.589
% * Total revolutions observed (slow-motion video count) = 12.667
% * Step-tracking ratio = 100.6%, demonstrating no step loss to 4250 PPS
%
% This conclusion underpins the back-EMF interpretation of the loaded
% saturation observations in section 4.5 (see ch3_loaded_torque_check.m):
% the motor follows commanded PPS perfectly under no load, so the reduced
% delivered torque under load cannot be a synchronism failure.

%% Settings
clear; clc; close all;

% Session to analyse
sessionFile = 'session_torque_speed_sweep.mat';

% Stepper configuration (locked firmware constants)
microstep_factor   = 16;
full_steps_per_rev = 200;
pulses_per_rev     = full_steps_per_rev * microstep_factor;   % 3200

% Plateau detection
plateau_change_threshold_pps = 50;     % min PPS jump to register as new plateau
min_plateau_duration_s       = 0.8;    % filter out transient segments

% Visual rotation count from the slow-motion video (interactive if empty)
% For the headline session, total measured was 12 + 2/3 = 12.667 revs
measured_total_revs = 12.667;          % set to NaN to be prompted at runtime

%% Load
S = load(sessionFile);
data = S.data;
N    = size(data, 1);

% Reconstruct uniform timing (PC timestamps unreliable due to USB batching)
Ts = 0.005;
t  = (0:N-1)' * Ts;

% Extract motor command applied (post-slew, post-clamp; the actual plant input)
motor_applied_pps = data(:, 8);

fprintf('\nLoaded %s\n', sessionFile);
fprintf('  Samples: %d, reconstructed duration %.2f s (Ts = %.0f ms)\n', ...
        N, t(end), Ts*1000);

%% Segment plateaus
% Detect transitions between commanded PPS levels by absolute change.
diff_ma = abs(diff(motor_applied_pps));
transitions = find(diff_ma > plateau_change_threshold_pps);

% Build segment list as [start_idx, end_idx, mean_pps]
segment_starts = [1; transitions + 1];
segment_ends   = [transitions; N];
segments = struct('start_idx', {}, 'end_idx', {}, 'pps', {}, ...
                  'duration_s', {}, 'pulses', {}, 'revs_expected', {});

for k = 1:numel(segment_starts)
    s = segment_starts(k);
    e = segment_ends(k);
    pps = mean(motor_applied_pps(s:e));
    dur = (e - s) * Ts;
    if dur < min_plateau_duration_s, continue; end
    if abs(pps) < plateau_change_threshold_pps, continue; end   % skip the zero plateaus
    seg.start_idx     = s;
    seg.end_idx       = e;
    seg.pps           = pps;
    seg.duration_s    = dur;
    seg.pulses        = abs(pps) * dur;
    seg.revs_expected = seg.pulses / pulses_per_rev;
    segments(end+1) = seg; %#ok<SAGROW>
end

if isempty(segments)
    error('No PPS plateaus detected. Check sessionFile and threshold.');
end

%% Print plateau table
fprintf('\nDetected PPS plateaus:\n');
fprintf('  %-3s | %-10s | %-10s | %-10s | %-10s | %-13s\n', ...
        '#', 'Start (s)', 'End (s)', 'PPS', 'Duration', 'Revs expected');
fprintf('  %s\n', repmat('-', 1, 75));
total_pulses = 0;
total_revs_expected = 0;
for k = 1:numel(segments)
    seg = segments(k);
    fprintf('  %-3d | %-10.2f | %-10.2f | %+10.0f | %-9.2fs | %-13.3f\n', ...
            k, t(seg.start_idx), t(seg.end_idx), seg.pps, seg.duration_s, seg.revs_expected);
    total_pulses = total_pulses + seg.pulses;
    total_revs_expected = total_revs_expected + seg.revs_expected;
end
fprintf('  %s\n', repmat('-', 1, 75));
fprintf('  Total commanded pulses     : %.0f\n', total_pulses);
fprintf('  Total revolutions expected : %.3f\n', total_revs_expected);

%% Acquire visual rotation count
if isnan(measured_total_revs)
    prompt = sprintf('Enter measured rotations from slow-motion video (e.g. 12.667): ');
    measured_total_revs = input(prompt);
end

ratio = measured_total_revs / total_revs_expected;
fprintf('\n=== Step-tracking result ===\n');
fprintf('  Revolutions expected (no slip) : %.3f\n', total_revs_expected);
fprintf('  Revolutions observed (video)   : %.3f\n', measured_total_revs);
fprintf('  Tracking ratio                 : %.4f  (%.1f %%)\n', ratio, ratio*100);
if abs(1 - ratio) < 0.05
    fprintf('  Conclusion: no step loss to %.0f PPS under no load.\n', max([segments.pps]));
else
    fprintf('  Conclusion: step loss observed (ratio departs from unity).\n');
end

%% Plot
figure('Name','Stage 3 — torque-speed sweep','Color','w','Position',[80 80 980 520]);

subplot(2,1,1);
plot(t, motor_applied_pps, 'k-', 'LineWidth', 1.0); hold on;
for k = 1:numel(segments)
    seg = segments(k);
    plot([t(seg.start_idx), t(seg.end_idx)], [seg.pps, seg.pps], 'r-', 'LineWidth', 2.5);
    text(t(seg.start_idx) + seg.duration_s/2, seg.pps + 150, ...
         sprintf('%.0f PPS', seg.pps), 'HorizontalAlignment', 'center', 'FontSize', 8);
end
yline(0, 'k:');
ylabel('motor\_applied (PPS)');
xlabel('Time (s) [reconstructed]');
title(sprintf('Commanded PPS plateaus  -  %d levels detected', numel(segments)));
grid on;
ylim([-200 max([segments.pps]) + 600]);

subplot(2,1,2);
ax2pps   = [segments.pps];
ax2dur   = [segments.duration_s];
ax2revs  = [segments.revs_expected];
bar(ax2pps, ax2revs, 0.6, 'FaceColor', [0.3 0.5 0.8]);
ylabel('Revolutions expected');
xlabel('Commanded PPS');
title(sprintf('Per-level revolutions  -  total expected %.3f vs measured %.3f  (%.1f%% tracking)', ...
              total_revs_expected, measured_total_revs, ratio*100));
grid on;

sgtitle(sprintf('Stage 3: stepper torque-speed envelope characterisation (%s)', sessionFile));

try, exportgraphics(gcf, 'ch3_torque_sweep.png', 'Resolution', 200); end %#ok<TRYNC>

fprintf('\nThis result feeds into ch3_loaded_torque_check.m, which compares the\n');
fprintf('no-load tracking observed here against the loaded behaviour during the\n');
fprintf('saturated balance attempt to isolate back-EMF torque collapse from step loss.\n');
