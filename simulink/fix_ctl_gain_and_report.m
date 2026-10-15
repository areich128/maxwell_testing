function fix_ctl_gain_and_report(mdl)
% control_fdbk expects ctl_gain = [Kp Kd Ki Kbdot] (4x1). The model supplies [Kp Kd Ki].
% Append Kbdot = 0 (B-dot gain, used only in BDOT), then report compiled sample times.
    if nargin < 1, mdl = 'ADCS_Simulink_FSW'; end
    load_system(mdl);
    ctl = [mdl '/control_fdbk'];
    pc = get_param(ctl, 'PortHandles');
    l = get_param(pc.Inport(6), 'Line');
    src = get_param(l, 'SrcPortHandle');
    if contains(get_param(get_param(src, 'Parent'), 'Name'), 'ctl_gain_4_FSW')
        fprintf('ctl_gain already 4 elements, skipping\n');
    else
        delete_line(l);
        p = get_param(pc.Inport(6), 'Position');
        cat = add_block('simulink/Math Operations/Vector Concatenate', [mdl '/ctl_gain_4_FSW'], 'MakeNameUnique', 'on', ...
            'NumInputs', '2', 'Position', [p(1)-60 p(2)-15 p(1)-50 p(2)+15]);
        kb = add_block('simulink/Sources/Constant', [mdl '/Kbdot_FSW'], 'MakeNameUnique', 'on', 'Value', '0', ...
            'OutDataTypeStr', 'single', 'SampleTime', 'dT_Control', 'Position', [p(1)-150 p(2)+20 p(1)-120 p(2)+36]);
        cp = get_param(cat, 'PortHandles'); kp = get_param(kb, 'PortHandles');
        add_line(mdl, src, cp.Inport(1), 'autorouting', 'on');
        add_line(mdl, kp.Outport(1), cp.Inport(2), 'autorouting', 'on');
        add_line(mdl, cp.Outport(1), pc.Inport(6), 'autorouting', 'on');
        fprintf('+ ctl_gain = [NtomN gains; Kbdot = 0] -> control_fdbk in 6\n');
        save_system(mdl);
    end

    blks = {[ctl '/control_fdbk'], [mdl '/createDes_RN/create_desRN'], [mdl '/Estimate Attitude/est_quest_rp/est_quest_rp']};
    try
        feval(mdl, [], [], [], 'compile');
    catch err
        fprintf(2, '\nCompile failed:\n'); disp(getReport(err, 'basic')); return;
    end
    c = onCleanup(@() feval(mdl, [], [], [], 'term'));
    fprintf('\ncompiled sample time [period offset] (want [0.1 0]):\n');
    for i = 1:numel(blks)
        st = get_param(blks{i}, 'CompiledSampleTime');
        if iscell(st), st = cell2mat(st(:)); end
        fprintf('  %-60s %s\n', blks{i}, mat2str(st));
    end
end