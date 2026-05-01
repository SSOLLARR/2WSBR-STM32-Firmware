function sessionDir = makeSessionFolder()
    rootDir = fullfile(pwd, "logs");
    if ~exist(rootDir, "dir")
        mkdir(rootDir);
    end

    stamp = datestr(now, 'yyyymmdd_HHMMSS');
    sessionDir = fullfile(rootDir, "session_" + string(stamp));
    mkdir(sessionDir);
end