function updateDashboard(hEst, hAcc, hGyro, hPID, t_win, p_win, a_win, g_win, u_win)
    % Instantly overwrite the X and Y data inside the existing lines
    set(hEst,  'XData', t_win, 'YData', p_win);
    set(hAcc,  'XData', t_win, 'YData', a_win);
    set(hGyro, 'XData', t_win, 'YData', g_win);
    set(hPID,  'XData', t_win, 'YData', u_win);

    % Shift the X-axis limits so the graph scrolls smoothly
    if ~isempty(t_win) && (t_win(end) > t_win(1))
        set(hEst.Parent,  'XLim', [t_win(1), t_win(end)]);
        set(hGyro.Parent, 'XLim', [t_win(1), t_win(end)]);
        set(hPID.Parent,  'XLim', [t_win(1), t_win(end)]);
    end

    % Drop frames if graphics card falls behind
    drawnow limitrate; 
end