function live_dashboard_cascade()
% live_dashboard_cascade.m
% Live telemetry dashboard for main_cascade_hybrid.c
% COM6 | 460800 baud
%
% A packet (parsed here), updated format:
%   A,sample_idx,acc_pitch_deg,gyro_rate_dps,estimated_pitch_deg,
%     theta_command_deg,q_ref_dps,motor_cmd_pps,motor_applied_pps,mode
%
% Notes on the time axis:
%   time_s   = PC-side wall-clock time (toc from start of script). Known to
%              be bursty because the USB side receives frames in chunks.
%              Kept as column 1 for continuity with older recordings and
%              because it is the useful axis for the live plot.
%   sample_idx = monotonic loop counter from the MCU. For any quantitative
%              analysis, use  t = sample_idx * 5e-3  instead of time_s.
%
% Controls:
%   REC / STOP  - record to memory
%   Save .mat   - write session file with column names
%   Close fig   - clean shutdown

COM_PORT  = 'COM6';
BAUD      = 460800;
WIN_S     = 10;
TIMER_MS  = 50;
BUF_SIZE  = 20000;
DT_S      = 5.0e-3;       % nominal control-loop period, used only in status readout

% --- serial ---
try
    s = serialport(COM_PORT, BAUD);
    configureTerminator(s, "LF");
    flush(s);
    fprintf('Connected: %s @ %d baud\n', COM_PORT, BAUD);
catch e
    error('Cannot open %s: %s', COM_PORT, e.message);
end

% --- circular buffers ---
B.t     = nan(1,BUF_SIZE);  B.si  = nan(1,BUF_SIZE);   % NEW: sample_idx
B.ap    = nan(1,BUF_SIZE);  B.gr  = nan(1,BUF_SIZE);
B.ep    = nan(1,BUF_SIZE);  B.tc  = nan(1,BUF_SIZE);
B.qr    = nan(1,BUF_SIZE);  B.mc  = nan(1,BUF_SIZE);
B.ma    = nan(1,BUF_SIZE);  B.md  = nan(1,BUF_SIZE);
B.idx   = 1;
B.cnt   = 0;
B.t0    = tic;

% --- recording ---
% Columns: [time_s, acc_pitch, gyro_rate, estimated_pitch, theta_command,
%           q_ref, motor_cmd, motor_applied, mode, sample_idx]  -> 10 columns
%
% sample_idx is deliberately placed LAST rather than inserted after time_s
% so that any existing analysis script that uses hard-coded column indices
% (e.g. data(:,4) for estimated_pitch) continues to reference the same
% field as before. Only scripts that assume exactly 9 columns need touching.
R.on   = false;
R.data = zeros(0,10);

% --- partial serial line ---
pbuf = '';

% --- figure ---
fig = figure('Name','Cascade Telemetry | main_cascade_hybrid',...
    'NumberTitle','off','Position',[80 60 1120 720],'Color','w',...
    'CloseRequestFcn', @onClose);

ax1 = subplot(3,1,1); hold on; grid on;
la_ep  = plot(nan,nan,'b-',  'LineWidth',1.8,'DisplayName','est\_pitch (°)');
la_ap  = plot(nan,nan,'c-',  'LineWidth',0.8,'DisplayName','acc\_pitch (°)');
la_tc  = plot(nan,nan,'r--', 'LineWidth',1.0,'DisplayName','\theta\_cmd (°)');
yline(0,'k:','LineWidth',0.5);
ylabel('Pitch (°)'); title('Pitch angle'); ylim([-20 20]);
legend('Location','northwest','FontSize',8);

ax2 = subplot(3,1,2); hold on; grid on;
la_gr  = plot(nan,nan,'r-',  'LineWidth',1.5,'DisplayName','gyro\_rate (°/s)');
la_qr  = plot(nan,nan,'m--', 'LineWidth',1.0,'DisplayName','q\_ref (°/s)');
yline(0,'k:','LineWidth',0.5);
ylabel('Rate (°/s)'); title('Pitch rate'); ylim([-200 200]);
legend('Location','northwest','FontSize',8);

ax3 = subplot(3,1,3); hold on; grid on;
la_mc  = plot(nan,nan,'Color',[0.5 0.5 0.5],'LineWidth',0.8,'DisplayName','motor\_cmd (PPS)');
la_ma  = plot(nan,nan,'k-',  'LineWidth',1.8,'DisplayName','motor\_applied (PPS)');
yline(0,'k:','LineWidth',0.5);
ylabel('Motor (PPS)'); xlabel('Time (s)');
title('Motor commands  —  motor\_applied is your plant input u');
ylim([-4000 4000]);
legend('Location','northwest','FontSize',8);

for ax = [ax1 ax2 ax3], xlim(ax,[0 WIN_S]); end

% --- status bar ---
st_txt = uicontrol(fig,'Style','text','Units','normalized',...
    'Position',[0 0.001 0.55 0.025],'BackgroundColor','w',...
    'HorizontalAlignment','left','FontSize',8,'String','waiting...');
md_txt = uicontrol(fig,'Style','text','Units','normalized',...
    'Position',[0.55 0.001 0.18 0.025],'BackgroundColor','w',...
    'HorizontalAlignment','center','FontSize',9,'FontWeight','bold',...
    'String','MODE: --');
rb = uicontrol(fig,'Style','togglebutton','Units','normalized',...
    'Position',[0.74 0.003 0.10 0.022],'String','● REC',...
    'BackgroundColor',[0.88 0.88 0.88],'FontSize',8,...
    'Callback',@toggleRec);
uicontrol(fig,'Style','pushbutton','Units','normalized',...
    'Position',[0.85 0.003 0.07 0.022],'String','Save .mat',...
    'FontSize',8,'Callback',@saveMat);
uicontrol(fig,'Style','pushbutton','Units','normalized',...
    'Position',[0.93 0.003 0.06 0.022],'String','Clear',...
    'FontSize',8,'Callback',@clearBufs);

MODE_NAMES  = {'BALANCING','RATE_STEP','MOTOR_STEP','FALL_TEST'};
MODE_COLORS = {[0.1 0.7 0.1],[0.9 0.55 0],[0.15 0.45 0.9],[0.85 0.15 0.15]};

% --- timer ---
tmr = timer('ExecutionMode','fixedRate','Period',TIMER_MS/1000,...
            'TimerFcn',@tick,'ErrorFcn',@timerErr);
start(tmr);
fprintf('Dashboard running. Close window to stop.\n');
waitfor(fig);

% ================================================================
    function tick(~,~)
        if ~isvalid(fig), return; end
        nav = s.NumBytesAvailable;
        if nav > 0
            raw   = read(s, nav, 'char');
            chunk = [pbuf, raw];
            parts = strsplit(chunk, newline);
            pbuf  = parts{end};
            for k = 1:numel(parts)-1
                parseLine(strtrim(parts{k}));
            end
        end
        if B.cnt == 0, return; end
        drawPlots();
    end

    function parseLine(ln)
        % Expected A packet has 10 comma-separated fields:
        %   A, sample_idx, acc_pitch, gyro_rate, est_pitch,
        %      theta_cmd, q_ref, motor_cmd, motor_applied, mode
        %
        % Backwards-compatible fallback: if a 9-field (legacy) packet arrives,
        % sample_idx is left as NaN so it is visible that the log came from
        % pre-update firmware. Downstream code should skip those rows or use
        % time_s only.
        if numel(ln) < 2 || ln(1) ~= 'A', return; end
        tok = strsplit(ln, ',');
        n_tok = numel(tok);

        if n_tok >= 10
            v_all = str2double(tok(2:10));
            if any(isnan(v_all)), return; end
            si_val = v_all(1);
            v = v_all(2:9);
        elseif n_tok >= 9
            % Legacy 9-field packet (no sample_idx).
            v = str2double(tok(2:9));
            if any(isnan(v)), return; end
            si_val = NaN;
        else
            return;
        end

        i = B.idx;
        B.t(i)  = toc(B.t0);
        B.si(i) = si_val;
        B.ap(i) = v(1); B.gr(i) = v(2);
        B.ep(i) = v(3); B.tc(i) = v(4);
        B.qr(i) = v(5); B.mc(i) = v(6);
        B.ma(i) = v(7); B.md(i) = v(8);
        B.idx = mod(B.idx, BUF_SIZE) + 1;
        B.cnt = B.cnt + 1;
        if R.on
            % 10 columns: [time_s, v(1..8), sample_idx]
            R.data(end+1,:) = [B.t(i), v(:)', si_val];
        end
    end

    function drawPlots()
        n  = min(B.cnt, BUF_SIZE);
        if B.cnt <= BUF_SIZE
            ir = 1:B.cnt;
        else
            ir = [B.idx:BUF_SIZE, 1:B.idx-1];
        end
        t_all = B.t(ir);
        tnow  = t_all(end);
        mask  = t_all >= tnow - WIN_S;
        tw    = t_all(mask);

        set(la_ep,'XData',tw,'YData',B.ep(ir(mask)));
        set(la_ap,'XData',tw,'YData',B.ap(ir(mask)));
        set(la_tc,'XData',tw,'YData',B.tc(ir(mask)));
        set(la_gr,'XData',tw,'YData',B.gr(ir(mask)));
        set(la_qr,'XData',tw,'YData',B.qr(ir(mask)));
        set(la_mc,'XData',tw,'YData',B.mc(ir(mask)));
        set(la_ma,'XData',tw,'YData',B.ma(ir(mask)));

        for ax = [ax1 ax2 ax3], xlim(ax,[tnow-WIN_S, tnow]); end

        md = B.md(ir(end));
        if ~isnan(md) && md>=0 && md<4
            set(md_txt,'String',['MODE: ' MODE_NAMES{md+1}],...
                'ForegroundColor',MODE_COLORS{md+1});
        end

        % Status readout: show MCU-derived time alongside wall-clock when
        % sample_idx is available. For the current firmware this will
        % usually match wall-clock to within the USB buffer depth.
        si_now = B.si(ir(end));
        if isnan(si_now)
            t_mcu_str = '--';
        else
            t_mcu_str = sprintf('%.2fs', si_now * DT_S);
        end

        set(st_txt,'String',sprintf(...
            't_pc=%.1fs  t_mcu=%s  n=%d  θ̂=%.2f°  q=%.1f°/s  u=%.0f PPS  q_ref=%.1f°/s',...
            tnow, t_mcu_str, B.cnt, B.ep(ir(end)), B.gr(ir(end)),...
            B.ma(ir(end)), B.qr(ir(end))));
        drawnow limitrate;
    end

    function toggleRec(src,~)
        if src.Value
            R.on = true;
            R.data = zeros(0,10);   % 10 columns including sample_idx
            set(src,'String','■ STOP','BackgroundColor',[1 0.3 0.3]);
            fprintf('Recording started\n');
        else
            R.on = false;
            set(src,'String','● REC','BackgroundColor',[0.88 0.88 0.88]);
            fprintf('Recording stopped — %d samples\n', size(R.data,1));
        end
    end

    function saveMat(~,~)
        if isempty(R.data)
            msgbox('No data — press REC first, then STOP, then Save.','Save'); return;
        end
        fn = sprintf('session_%s.mat', datestr(now,'yyyymmdd_HHMMSS'));
        colnames = {'time_s','acc_pitch_deg','gyro_rate_dps',...
                    'estimated_pitch_deg','theta_command_deg',...
                    'q_ref_dps','motor_cmd_pps','motor_applied_pps','mode',...
                    'sample_idx'};
        data = R.data; %#ok<NASGU>

        % Metadata sidecar inside the .mat for defensibility.
        meta = struct();
        meta.firmware_packet = ...
            'A,sample_idx,acc_pitch,gyro_rate,theta_hat,theta_cmd,q_ref,motor_cmd,motor_applied,mode';
        meta.control_loop_dt_s = DT_S;
        meta.time_axis_notice  = ...
            'Use sample_idx*dt for analysis. time_s is PC-side wall clock and is bursty.';
        meta.saved_at = string(datetime('now'));
        meta.com_port = COM_PORT;
        meta.baud     = BAUD;
        meta.n_rows   = size(R.data,1); %#ok<STRNU>

        save(fn,'data','colnames','meta');
        fprintf('Saved: %s  (%d rows)\n', fn, size(R.data,1));
        msgbox(['Saved: ' fn],'Save complete');
    end

    function clearBufs(~,~)
        B.t(:)=nan; B.si(:)=nan;
        B.ap(:)=nan; B.gr(:)=nan; B.ep(:)=nan;
        B.tc(:)=nan; B.qr(:)=nan; B.mc(:)=nan; B.ma(:)=nan;
        B.md(:)=nan; B.idx=1; B.cnt=0; B.t0=tic;
        R.data=zeros(0,10);
        fprintf('Buffers cleared\n');
    end

    function timerErr(~,evt)
        fprintf('Timer error: %s\n', evt.Data.message);
    end

    function onClose(~,~)
        try, stop(tmr); delete(tmr); end %#ok<TRYNC>
        try, delete(s); end %#ok<TRYNC>
        delete(fig);
        fprintf('Shutdown complete.\n');
    end

end
