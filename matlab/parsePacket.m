function [isValid, pkt_type, vals] = parsePacket(line)
    % Initialize defaults
    isValid = false;
    pkt_type = '';
    vals = [];

    if strlength(string(line)) == 0
        return;
    end

    parts = split(string(strtrim(line)), ",");

    if isempty(parts)
        return;
    end

    pkt_type = char(parts(1));

    try
        if pkt_type == 'A' && numel(parts) == 5
            % Convert strings to numbers: [acc_pitch, gyro, est, pid]
            vals = [str2double(parts(2)), str2double(parts(3)), ...
                    str2double(parts(4)), str2double(parts(5))];
            
            if ~any(isnan(vals))
                isValid = true;
            end
        end
    catch
        isValid = false;
    end
end