function check_truth_kinematics()
% Diagnostic for ADCS_Simulink_FSW (does not save the model).
% Question: the truth kinematics in "Estimate Attitude" integrate
%   dq/dt = 1/2 * Omega(W) * q_BN
% where q_BN is Pranith's QUEST output (forced to q0 >= 0 by Finalize Quest Q),
% not the integrator's own state. If the true state has q0 < 0, the estimate
% is -q_true and the truth would integrate backwards. This logs both and checks.

    mdl = 'ADCS_Simulink_FSW';
    here = fileparts(mfilename('fullpath'));
    old = cd(here);                       % PreLoadFcn uses paths relative to this folder
    restoreDir = onCleanup(@() cd(old));

    load_system(mdl);
    closeMdl = onCleanup(@() close_system(mdl, 0));   % discard logging changes

    logPort([mdl '/Estimate Attitude/Quaternion Normalize'], 1, 'q_true');
    logPort([mdl '/Estimate Attitude'], 1, 'q_est');
    logPort([mdl '/Estimate Attitude'], 5, 'q_fsw');
    set_param(mdl, 'SignalLogging', 'on', 'SignalLoggingName', 'logsout', ...
        'ReturnWorkspaceOutputs', 'on');

    fprintf('Running %s from %s to %s s ...\n', mdl, get_param(mdl,'StartTime'), get_param(mdl,'StopTime'));
    out = sim(mdl);
    L = out.logsout;

    [t,  qt] = getQ(L, 'q_true');
    [te, qe] = getQ(L, 'q_est');
    [tf, qf] = getQ(L, 'q_fsw');
    qe = interp1(te, qe, t, 'previous', 'extrap');
    qf = interp1(tf, qf, t, 'previous', 'extrap');

    s      = sum(qt .* qe, 2);                          % sign of q_true . q_est
    angEst = 2 * acosd(min(1, abs(s)));                 % Pranith QUEST vs truth (deg)
    angFsw = 2 * acosd(min(1, abs(sum(qt .* qf, 2))));  % FSW QUEST vs truth (deg)

    neg = s < 0;
    fprintf('\nsamples: %d (%.1f to %.1f s)\n', numel(t), t(1), t(end));
    fprintf('q0_true range: %.4f to %.4f\n', min(qt(:,1)), max(qt(:,1)));
    fprintf('samples with q_true . q_est < 0: %d (%.1f%%)\n', sum(neg), 100*mean(neg));
    if any(neg)
        k = find(neg, 1);
        w = t >= t(k) & t <= t(k) + 50;
        fprintf('first sign mismatch at t = %.1f s; q0_true in next 50 s: %.4f to %.4f\n', ...
            t(k), min(qt(w,1)), max(qt(w,1)));
    end
    fprintf('Pranith QUEST vs truth: max %.4f deg\n', max(angEst));
    fprintf('FSW QUEST vs truth:     max %.4f deg, 95th pct %.4f deg\n', max(angFsw), prctile(angFsw, 95));

    figure('Name', 'Truth kinematics check');
    subplot(3,1,1); plot(t, qt(:,1), t, qe(:,1), '--'); grid on
    ylabel('q_0'); legend('q\_true (integrator)', 'q\_est (Pranith QUEST)');
    subplot(3,1,2); plot(t, sign(s)); grid on; ylim([-1.5 1.5])
    ylabel('sign(q_{true}\cdotq_{est})');
    subplot(3,1,3); plot(t, angEst, t, angFsw); grid on
    ylabel('error vs truth (deg)'); xlabel('t (s)'); legend('Pranith QUEST', 'FSW QUEST');
end

function logPort(blk, port, name)
    ph = get_param(blk, 'PortHandles');
    set_param(ph.Outport(port), 'DataLogging', 'on', ...
        'DataLoggingNameMode', 'Custom', 'DataLoggingName', name);
end

function [t, q] = getQ(L, name)
    v = L.getElement(name).Values;
    t = v.Time;
    d = squeeze(double(v.Data));
    if size(d, 1) ~= numel(t), d = d.'; end
    q = d;
end