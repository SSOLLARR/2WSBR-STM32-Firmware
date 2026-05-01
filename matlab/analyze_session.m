% analyze_session.m
% Expects csvFile to already point to packet_log.csv

T = readtable(csvFile);

% Packet A is the one that contains:
% A,acc_pitch,gyro_rate,estimated_pitch,pid_output
TA = T(T.type=="A", :);

if isempty(TA)
    error('No A packets found in %s', csvFile);
end

% Time and raw PID output from your existing logger
t = TA.pc_time_s;
pid_output = TA.cmd;

% Clean any bad values just in case
pid_output(~isfinite(pid_output)) = 0;

% =========================================================
% Reconstruct processed u_k[k] from firmware logic
% Based on set_motor_speed() in main.c:
% 1) saturation to +/-3000
% 2) slew limiting with max change 2000 per loop
% 3) deadzone: |u| < 20 -> 0
% =========================================================

% 1) Saturation
u_sat = min(max(pid_output, -3000), 3000);

% 2) Slew limiting
u_proc = zeros(size(u_sat));
prev = 0;

max_change_per_loop = 2000;
instant_kick_speed  = 2000;

for k = 1:numel(u_sat)
    target = u_sat(k);

    if prev == 0 && target > instant_kick_speed
        curr = instant_kick_speed;
    elseif prev == 0 && target < -instant_kick_speed
        curr = -instant_kick_speed;
    else
        if target > prev + max_change_per_loop
            curr = prev + max_change_per_loop;
        elseif target < prev - max_change_per_loop
            curr = prev - max_change_per_loop;
        else
            curr = target;
        end
    end

    % 3) Deadzone
    if abs(curr) < 20
        curr = 0;
    end

    u_proc(k) = curr;
    prev = curr;
end

% =========================================================
% Overlay plot: raw pid_output vs processed plant input
% =========================================================
figure('Name','PID Output vs Processed Plant Input');
plot(t, pid_output, 'DisplayName', 'Raw pid\_output'); hold on;
plot(t, u_proc,     'DisplayName', 'Processed u\_k[k]');
grid on;
xlabel('PC Time (s)');
ylabel('Command');
title('Raw pid\_output overlaid with processed plant input');
legend('Location','best');

% Optional second figure with estimated pitch too
figure('Name','Pitch with Raw and Processed Command');
yyaxis left
plot(t, TA.est, 'DisplayName', 'Estimated pitch');
ylabel('Pitch (deg)');

yyaxis right
hold on
plot(t, pid_output, 'DisplayName', 'Raw pid\_output');
plot(t, u_proc,     'DisplayName', 'Processed u\_k[k]');
ylabel('Command');

grid on;
xlabel('PC Time (s)');
title('Estimated pitch with raw and processed command');
legend('Location','best');