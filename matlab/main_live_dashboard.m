clear; clc; close all;

% --- 1. SETUP SERIAL PORT ---
s = serialport("COM6", 460800); % <-- 
configureTerminator(s, "LF");
flush(s);

% --- 2. PRE-ALLOCATE ARRAYS FOR HIGH-SPEED LOGGING ---
% By filling the array with zeros up front, MATLAB doesn't have to pause 
% and ask Windows for more RAM 200 times a second.
MAX_SAMPLES = 120000; % 120,000 samples = 10 minutes of continuous recording at 200Hz
log_time  = zeros(1, MAX_SAMPLES);
log_pitch = zeros(1, MAX_SAMPLES);
log_acc   = zeros(1, MAX_SAMPLES); % NEW: Raw Accel
log_gyro  = zeros(1, MAX_SAMPLES); % NEW: Raw Gyro
log_pid   = zeros(1, MAX_SAMPLES);

% --- 3. INIT DASHBOARD ---
% Call your init script and get the handles!
[fig, hEst, hAcc, hGyro, hPID] = initDashboard();

% Setup Decoupling Variables
sample_count = 0;
PLOT_DECIMATION = 10; % 200 Hz / 10 = Update screen at 20 Hz
WINDOW_SIZE = 1000;   % Only show the last 5 seconds (1000 points) on screen

disp('Starting Live Telemetry... Close the figure window to stop and save.');
t_start = tic;

% --- 4. MAIN REAL-TIME LOOP ---
while isvalid(fig) % Runs continuously until you click the 'X' on the graph window
    
    if s.NumBytesAvailable > 0
        try
            raw_str = readline(s); 
            
            % Assume parsePacket returns: [isValid, pkt_type, [acc_pitch, gyro, est_pitch, pid]]
            [isValid, pkt_type, vals] = parsePacket(raw_str);
            
            if isValid && pkt_type == 'A'
                
                % =========================================================
                % STEP A: LOGGING (Runs at 200 Hz)
                % Always append to the arrays. Zero data loss for your thesis.
                % =========================================================
                sample_count = sample_count + 1;
                
                if sample_count > MAX_SAMPLES
                    disp('Maximum allocated memory reached. Stopping...');
                    break;
                end
                
                log_time(sample_count)  = toc(t_start);
                log_acc(sample_count)   = vals(1); % acc_pitch
                log_gyro(sample_count)  = vals(2); % gyro_rate
                log_pitch(sample_count) = vals(3); % estimated_pitch
                log_pid(sample_count)   = vals(4); % pid_output
                
                % =========================================================
                % STEP B: PLOTTING (Decoupled, Runs at 20 Hz)
                % =========================================================
                % Only update the screen every 10th packet
                if mod(sample_count, PLOT_DECIMATION) == 0
                    
                    % Create a rolling window (Plotting 100,000 points crashes MATLAB; 
                    % plotting the last 1000 points is instant.)
                    if sample_count > WINDOW_SIZE
                        idx_start = sample_count - WINDOW_SIZE + 1;
                    else
                        idx_start = 1;
                    end
                    
                    % Extract just the recent 5 seconds for the live plot
                   t_show = log_time(idx_start : sample_count);
                    p_show = log_pitch(idx_start : sample_count);
                    a_show = log_acc(idx_start : sample_count);
                    g_show = log_gyro(idx_start : sample_count);
                    u_show = log_pid(idx_start : sample_count);

% Call the high-speed update function
updateDashboard(hEst, hAcc, hGyro, hPID, t_show, p_show, a_show, g_show, u_show);
                    
                end
            end
        catch
            % If a Bluetooth packet is corrupted, gracefully ignore it
            continue;
        end
    end
end

% --- 5. CLEANUP & SAVING ---
disp('Telemetry stopped. Saving pristine session data...');

% 1. Truncate the arrays to remove empty zeros
final_time  = log_time(1:sample_count);
final_pitch = log_pitch(1:sample_count);
final_acc   = log_acc(1:sample_count);   % <-- Creates final_acc
final_gyro  = log_gyro(1:sample_count);  % <-- Creates final_gyro
final_pid   = log_pid(1:sample_count);

% 2. Call your session folder script
session_dir = makeSessionFolder(); 

% 3. Save ALL the arrays to the .mat file
save(fullfile(session_dir, 'telemetry_data.mat'), ...
    'final_time', 'final_pitch', 'final_acc', 'final_gyro', 'final_pid');

disp(['Data saved successfully to: ', session_dir]);
clear s;