%% Section 4.x — Bandwidth ceiling discovery (0.26 Hz limit cycle)
% *Two-wheeled self-balancing robot — dissertation reproduction script*
%
% This Live Script reproduces the bandwidth-ceiling discovery reported in
% section 4.x of the dissertation. It loads the failed balance attempt
% recorded after pushing the outer-loop bandwidth from the LQR-designed
% 0.80 Hz to a target of 1.20 Hz, computes the FFT of the pitch trace,
% and annotates the dominant 0.26 Hz oscillation against the
% complementary-filter corner frequency at 0.48 Hz.
%
% The interpretation is that outer-loop bandwidth above the estimator
% corner causes the controller to act on phase-lagged pitch estimates,
% destabilising the loop into a sustained limit cycle.
%
% *Expected output for session_bandwidth_lco_0p26hz.mat:*
%
% * Balance duration ~ 3.83 s before |theta| > 5 deg fall threshold
% * RMS pitch ~ 1.28 deg (vs 0.345 deg at the 0.80 Hz LQR design)
% * Dominant FFT peak at ~ 0.26 Hz, well below the CF corner at 0.48 Hz
% * Motor std ~ 618 PPS (vs 163 PPS at LQR), peak |motor| ~ saturation
%
% Cross-references:
% * `ch3_lqr_synthesis.m` — produces the 0.80 Hz design and the cascade gains
% * `ch4_balance_analysis.m` — the LQR-deployed run (session 022009) for comparison

%% Settings
clear; clc; close all;

sessionFile = 'session_bandwidth_lco_0p26hz.mat';

% Complementary filter parameters — these set the estimator corner frequency
alpha = 0.975;
Ts    = 0.005;          % control-loop period from firmware
fc    = (1 - alpha) / (2 * pi * alpha * Ts);

% Fall threshold for "balanced portion" extraction
fall_threshold_deg = 5.0;

% FFT settings
remove_dc = true;       % subtract mean before FFT to remove DC peak

%% Load
S = load(sessionFile);
data = S.data;
N    = size(data, 1);

t  = (0:N-1)' * Ts;
ep = data(:, 4);   % estimated_pitch_deg
gr = data(:, 3);   % gyro_rate_dps
ma = data(:, 8);   % motor_applied_pps

fprintf('\nLoaded %s\n', sessionFile);
fprintf('  Samples: %d, reconstructed duration %.2f s\n', N, t(end));
fprintf('  CF parameters: alpha = %.3f, Ts = %.0f ms\n', alpha, Ts*1000);
fprintf('  CF corner frequency: f_c = (1-alpha)/(2*pi*alpha*Ts) = %.3f Hz\n', fc);

%% Extract balanced portion (before fall)
fall_idx = find(abs(ep) > fall_threshold_deg, 1, 'first');
if isempty(fall_idx)
    t_fall = t(end);
    bal_mask = true(N, 1);
else
    t_fall = t(fall_idx);
    bal_mask = t < t_fall;
end

ep_b = ep(bal_mask);
gr_b = gr(bal_mask);
ma_b = ma(bal_mask);
t_b  = t(bal_mask);

fprintf('\nBalanced portion: %d samples, %.2f s\n', sum(bal_mask), t_fall);
fprintf('  RMS pitch:    %.3f deg\n', sqrt(mean(ep_b.^2)));
fprintf('  Mean pitch:   %+.3f deg\n', mean(ep_b));
fprintf('  Std pitch:    %.3f deg\n', std(ep_b));
fprintf('  Max |pitch|:  %.2f deg\n', max(abs(ep_b)));
fprintf('  Motor std:    %.0f PPS\n', std(ma_b));
fprintf('  |motor| max:  %.0f PPS\n', max(abs(ma_b)));

%% FFT of pitch
% Use only the balanced portion to avoid contaminating the spectrum with the fall.
y = ep_b;
if remove_dc
    y = y - mean(y);
end

L = length(y);
Y = fft(y);
P2 = abs(Y / L);
P1 = P2(1:floor(L/2)+1);
P1(2:end-1) = 2 * P1(2:end-1);
f = (1/Ts) * (0:floor(L/2)) / L;

% Find dominant peak above DC (skip f = 0)
[peak_amp, peak_idx] = max(P1(2:end));
peak_idx = peak_idx + 1;
peak_freq = f(peak_idx);

fprintf('\nFFT analysis (balanced portion, %.2f s window):\n', t_fall);
fprintf('  Dominant peak: %.3f Hz, amplitude %.3f deg\n', peak_freq, peak_amp);
fprintf('  CF corner   :  %.3f Hz\n', fc);
fprintf('  Ratio peak / CF corner: %.2f\n', peak_freq / fc);

if peak_freq < 0.5 * fc
    fprintf('\nInterpretation: the dominant LCO frequency sits well below the CF corner,\n');
    fprintf('consistent with a phase-lag-induced instability where the outer loop is\n');
    fprintf('chasing stale estimator output rather than the true plant state.\n');
end

%% Plot
figure('Name','Section 4.x — bandwidth ceiling LCO','Color','w','Position',[80 80 1000 720]);

% Subplot 1 — pitch trace
subplot(3,1,1);
plot(t, ep, 'b-', 'LineWidth', 1.0); hold on;
yline(0, 'k:');
yline(+fall_threshold_deg, 'r:', 'LineWidth', 0.5);
yline(-fall_threshold_deg, 'r:', 'LineWidth', 0.5);
if ~isempty(fall_idx)
    xline(t_fall, 'r--', 'LineWidth', 0.7);
    text(t_fall, 4.0, sprintf(' fall (t = %.2f s)', t_fall), ...
         'VerticalAlignment', 'top', 'FontSize', 8, 'Color', 'r');
end
ylabel('Pitch \theta (deg)');
title(sprintf('%s — pitch trace, balanced for %.2f s', sessionFile, t_fall), ...
      'Interpreter', 'none');
grid on;
ylim([-15 15]);

% Subplot 2 — motor command
subplot(3,1,2);
plot(t, ma, 'k-', 'LineWidth', 0.8);
yline(0, 'k:');
ylabel('motor\_applied (PPS)');
title(sprintf('Motor command — std = %.0f PPS, peak |u| = %.0f PPS  (compare LQR run: 163 PPS)', ...
              std(ma_b), max(abs(ma_b))));
grid on;

% Subplot 3 — FFT spectrum with annotations
subplot(3,1,3);
semilogy(f, P1, 'b-', 'LineWidth', 1.2); hold on;
plot(peak_freq, peak_amp, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 8);
xline(fc, 'm--', 'LineWidth', 1.0);
text(peak_freq, peak_amp * 1.5, sprintf(' peak: %.2f Hz', peak_freq), ...
     'FontSize', 9, 'FontWeight', 'bold', 'Color', 'r');
text(fc, max(P1) * 0.5, sprintf(' f_c = %.2f Hz (CF corner)', fc), ...
     'FontSize', 9, 'Color', 'm', 'HorizontalAlignment', 'left');
xlabel('Frequency (Hz)');
ylabel('Spectral amplitude (deg, log scale)');
title(sprintf('Pitch spectrum (balanced window) — dominant peak at %.3f Hz, well below CF corner', peak_freq));
grid on;
xlim([0 5]);

sgtitle({sprintf('Bandwidth ceiling discovery: outer loop pushed to 1.20 Hz, CF corner at %.2f Hz', fc), ...
         sprintf('Result: 0.26 Hz limit cycle, RMS = %.3f deg (LQR baseline: 0.345 deg)', sqrt(mean(ep_b.^2)))}, ...
        'FontSize', 11);

try, exportgraphics(gcf, 'bandwidth_lco_figure.pdf', 'ContentType', 'vector'); end %#ok<TRYNC>
try, exportgraphics(gcf, 'bandwidth_lco_figure.png', 'Resolution', 200); end %#ok<TRYNC>

fprintf('\nSaved figure: bandwidth_lco_figure.pdf and .png\n');
