function fix_bfield_and_report(mdl)
% createDes_RN in 4 (magECI) is fed by B_ECI: double and continuous. Flight samples
% mag_eci_vec once per loop, so insert ZOH(dT_Control) + single on that branch only.
% Then compile and report each flight block's sample time and its inputs' sources.
    if nargin < 1, mdl = 'ADCS_Simulink_FSW'; end
    load_system(mdl);
    crn = [mdl '/createDes_RN'];
    pr = get_param(crn, 'PortHandles');
    l = get_param(pr.Inport(4), 'Line');
    src = get_param(l, 'SrcPortHandle');
    if contains(get_param(get_param(src, 'Parent'), 'Name'), 'B_ECI_single_FSW')
        fprintf('B_ECI branch already sampled, skipping\n');
    else
        delete_line(l);                                  % only this branch
        p = get_param(pr.Inport(4), 'Position');
        z = add_block('simulink/Discrete/Zero-Order Hold', [mdl '/B_ECI_sample_FSW'], 'MakeNameUnique', 'on', ...
            'SampleTime', 'dT_Control', 'Position', [p(1)-150 p(2)-12 p(1)-120 p(2)+12]);
        d = add_block('simulink/Signal Attributes/Data Type Conversion', [mdl '/B_ECI_single_FSW'], 'MakeNameUnique', 'on', ...
            'OutDataTypeStr', 'single', 'Position', [p(1)-90 p(2)-12 p(1)-50 p(2)+12]);
        zp = get_param(z, 'PortHandles'); dp = get_param(d, 'PortHandles');
        add_line(mdl, src, zp.Inport(1), 'autorouting', 'on');
        add_line(mdl, zp.Outport(1), dp.Inport(1), 'autorouting', 'on');
        add_line(mdl, dp.Outport(1), pr.Inport(4), 'autorouting', 'on');
        fprintf('+ B_ECI -> ZOH(dT_Control) -> single -> createDes_RN in 4\n');
        save_system(mdl);
    end

    subs = {[mdl '/control_fdbk'], 'control_fdbk';
            crn, 'create_desRN';
            [mdl '/Estimate Attitude/est_quest_rp'], 'est_quest_rp'};
    try
        feval(mdl, [], [], [], 'compile');
    catch err
        fprintf(2, '\nCompile failed:\n'); disp(getReport(err, 'basic')); return;
    end
    c = onCleanup(@() feval(mdl, [], [], [], 'term'));
    for i = 1:size(subs, 1)
        st = get_param([subs{i,1} '/' subs{i,2}], 'CompiledSampleTime');
        fprintf('\n%s  ->  %s   (want [0.1 0])\n', subs{i,1}, fmtST(st));
        ph = get_param(subs{i,1}, 'PortHandles');
        for k = 1:numel(ph.Inport)
            l = get_param(ph.Inport(k), 'Line');
            if l == -1, fprintf('   in %2d  UNCONNECTED\n', k); continue; end
            sb = get_param(get_param(l, 'SrcPortHandle'), 'Parent');
            sst = get_param(sb, 'CompiledSampleTime');
            flag = ''; s = sst; if iscell(s), s = cell2mat(s(:)); end
            if any(s(:,1) == 0), flag = '   <-- continuous'; end
            fprintf('   in %2d  %-45s %s%s\n', k, get_param(sb, 'Name'), fmtST(sst), flag);
        end
    end
end

function s = fmtST(st)
    if iscell(st), st = cell2mat(st(:)); end
    s = mat2str(st, 4);
end