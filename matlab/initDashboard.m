function [fig, hEst, hAcc, hGyro, hPID] = initDashboard()
    % Create the main figure window
    fig = figure('Name', '2WSBR Live Telemetry - Sensor Fusion', 'NumberTitle', 'off', 'Color', 'w', 'Position', [100, 100, 900, 700]);

    % --- Top Subplot: Pitch estimate vs raw accelerometer signal ---
    % The raw accelerometer trace is atan2(a_y, a_z) computed every loop
    % with no filtering. During closed-loop balance the dominant content is
    % drivetrain-induced lateral-acceleration impulses, not chassis pitch.
    % The complementary filter rejects those impulses to recover the true
    % gravitational reference. See dissertation section 4.3.
    ax1 = subplot(3, 1, 1);
    hold(ax1, 'on');
    hAcc = plot(ax1, NaN, NaN, 'Color', [1, 0.6, 0.6], 'LineWidth', 1.0, ...
        'DisplayName', 'Raw atan2(a_y, a_z) (drivetrain-contaminated)');
    hEst = plot(ax1, NaN, NaN, 'b-', 'LineWidth', 1.5, ...
        'DisplayName', 'Complementary-filter pitch estimate');
    title(ax1, 'Pitch estimate vs raw accelerometer signal');
    ylabel(ax1, 'atan2(a_y, a_z) [deg]');
    legend(ax1, 'Location', 'northeast');
    grid(ax1, 'on');
    ylim(ax1, [-45, 45]);

    % --- Middle Subplot: Raw gyroscope rate ---
    % Direct measurement of body angular rate from the MPU6050 gyroscope,
    % calibrated and bias-corrected. Inner-loop feedback signal of the
    % deployed cascade controller.
    ax2 = subplot(3, 1, 2);
    hGyro = plot(ax2, NaN, NaN, 'k-', 'LineWidth', 1.0);
    title(ax2, 'Gyroscope rate (inner-loop feedback)');
    ylabel(ax2, 'q_g [deg/s]');
    grid(ax2, 'on');
    ylim(ax2, [-250, 250]);

    % --- Bottom Subplot: Motor command ---
    % Inner-loop output u_k after saturation and slew limiting.
    % Saturation envelope is +/-3800 pps (ylim retains margin for
    % open-loop modes that exercise up to 4250 pps).
    ax3 = subplot(3, 1, 3);
    hPID = plot(ax3, NaN, NaN, 'r-', 'LineWidth', 1.5);
    title(ax3, 'Motor command u_k (post saturation and slew)');
    xlabel(ax3, 'Time (s)');
    ylabel(ax3, 'Step rate u_k [pps]');
    grid(ax3, 'on');
    ylim(ax3, [-3500, 3500]);
end