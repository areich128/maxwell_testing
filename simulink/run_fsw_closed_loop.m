function R = run_fsw_closed_loop(useFSW, antenna)
% Closed-loop run of ADCS_Simulink_FSW (8000 to 8700 s DCOMM scenario).
% useFSW = 1 (default): plant driven by flight control_fdbk on q_BN_FSW.
% useFSW = 0: plant driven by Pranith's PD (baseline). Model is not saved.
% antenna = 0 (default, as saved): X-band, flight points -X at the ground station.
% antenna = 1: T-DAHGR, flight points +X (same axis as Pranith's desired attitude).
% Results go to fsw_loop_<useFSW>_ant<antenna>.mat next to this file.

    if nargin < 1, useFSW = 1; end
    if nargin < 2, antenna = 0; end
    mdl = 'ADCS_Simulink_FSW';
    here = fileparts(mfilename('fullpath'));
    old = cd(here);
    restoreDir = onCleanup(@() cd(old));
    load_system(mdl);
    closeMdl = onCleanup(@() close_system(mdl, 0));

    set_param([mdl '/useFSWtorque'], 'Value', num2str(useFSW));
    pc = get_param([mdl '/createDes_RN'], 'PortConnectivity');
    antBlk = getfullname(pc(3).SrcBlock);            % createDes_RN in 3 = antenna
    set_param(antBlk, 'Value', num2str(antenna));
    logPort([mdl '/Estimate Attitude/Quaternion Normalize'], 1, 'q_true');
    logPort([mdl '/Estimate Attitude'], 5, 'q_fsw');
    logPort([mdl '/PD Controller'], 2, 'err_sim');      % pointing error from truth
    logPort([mdl '/control_fdbk'], 2, 'err_fsw');       % pointing error flight sees
    logPort([mdl '/Simulate Omega Rates'], 1, 'W');
    logPort([mdl '/Switch'], 1, 'tau_plant');
    set_param(mdl, 'SignalLogging', 'on', 'SignalLoggingName', 'logsout', ...
        'ReturnWorkspaceOutputs', 'on');

    fprintf('Running %s, useFSWtorque = %d, antenna = %d (%s) ...\n', mdl, useFSW, antenna, antBlk);
    txt = evalc('out = sim(mdl);');       % capture mexPrintf output from flight code
    L = out.logsout;

    [t, qt]  = getV(L, 'q_true');
    [tf, qf] = getV(L, 'q_fsw');     qf = interp1(tf, qf, t, 'previous', 'extrap');
    [ts, es] = getV(L, 'err_sim');   es = interp1(ts, es, t, 'previous', 'extrap');
    [te, ef] = getV(L, 'err_fsw');   ef = interp1(te, ef, t, 'previous', 'extrap');
    [tw, W]  = getV(L, 'W');         W  = interp1(tw, W,  t, 'previous', 'extrap');
    [tt, tau] = getV(L, 'tau_plant'); tau = interp1(tt, tau, t, 'previous', 'extrap');

    R.t = t; R.q_true = qt; R.q_fsw = qf; R.useFSW = useFSW; R.antenna = antenna;
    R.estErr  = 2 * acosd(min(1, abs(sum(qt .* qf, 2))));   % FSW QUEST vs truth
    R.pointSim = 2 * acosd(min(1, abs(es(:,1))));           % true pointing error
    R.pointFsw = 2 * acosd(min(1, abs(ef(:,1))));           % pointing error flight sees
    R.rate = vecnorm(W, 2, 2);
    R.tau = vecnorm(tau, 2, 2);
    R.nErrQuest = numel(strfind(txt, 'ERR->QUEST'));
    R.nSingular = numel(strfind(txt, 'SINGULAR'));
    R.nLines = numel(strfind(txt, newline));

    [q0min, k] = min(abs(qt(:,1)));
    fprintf('\nuseFSWtorque = %d, antenna = %d, %d samples (%.1f to %.1f s)\n', useFSW, antenna, numel(t), t(1), t(end));
    fprintf('closest to 180 deg from ECI: %.4f deg at t = %.1f s\n', 2*acosd(q0min), t(k));
    fprintf('printed lines from sim: %d (ERR->QUEST: %d, SINGULAR: %d)\n', R.nLines, R.nErrQuest, R.nSingular);
    fprintf('FSW QUEST vs truth:  max %.4f deg, 95th pct %.4f deg\n', max(R.estErr), prctile(R.estErr, 95));
    w = t >= t(end) - 100;
    fprintf('true pointing error: max %.3f deg, last 100 s mean %.4f deg\n', max(R.pointSim), mean(R.pointSim(w)));
    fprintf('flight-seen error:   max %.3f deg, last 100 s mean %.4f deg\n', max(R.pointFsw), mean(R.pointFsw(w)));
    fprintf('body rate |W|:       max %.4g, last 100 s mean %.4g (model units)\n', max(R.rate), mean(R.rate(w)));
    fprintf('plant torque |tau|:  max %.4g N m\n', max(R.tau));
    if ~isempty(strtrim(txt))
        fprintf('\nfirst sim output lines:\n%s\n', txt(1:min(end, 600)));
    end

    save(fullfile(here, sprintf('fsw_loop_%d_ant%d.mat', useFSW, antenna)), 'R');

    figure('Name', sprintf('Closed loop, useFSWtorque = %d, antenna = %d', useFSW, antenna));
    subplot(4,1,1); plot(t, qt(:,1)); grid on; ylabel('q_0 true')
    subplot(4,1,2); plot(t, R.pointSim, t, R.pointFsw, '--'); grid on
    ylabel('pointing err (deg)'); legend('true', 'flight sees')
    subplot(4,1,3); plot(t, R.estErr); grid on; ylabel('FSW QUEST err (deg)')
    subplot(4,1,4); plot(t, R.tau); grid on; ylabel('|\tau| (N m)'); xlabel('t (s)')
end

function logPort(blk, port, name)
    ph = get_param(blk, 'PortHandles');
    set_param(ph.Outport(port), 'DataLogging', 'on', ...
        'DataLoggingNameMode', 'Custom', 'DataLoggingName', name);
end

function [t, v] = getV(L, name)
    s = L.getElement(name).Values;
    t = s.Time;
    d = squeeze(double(s.Data));
    if size(d, 1) ~= numel(t), d = d.'; end
    v = d;
end