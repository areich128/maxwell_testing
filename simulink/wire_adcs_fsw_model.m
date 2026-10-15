function wire_adcs_fsw_model(mdl)
% Feed the new flight-block ports in the FSW copy of ADCS_Simulink, then report
% the compiled sample time of each flight block (should be the 0.1 s control rate).
    if nargin < 1, mdl = 'ADCS_Simulink_FSW'; end
    load_system(mdl);
    Ts  = get_param([mdl '/Zero-Order Hold4'], 'SampleTime');      % control-rate hold already feeding control_fdbk
    ctl = [mdl '/control_fdbk'];
    crn = [mdl '/createDes_RN'];
    eqb = [mdl '/Estimate Attitude/est_quest_rp'];
    pc = get_param(ctl, 'PortHandles'); pr = get_param(crn, 'PortHandles'); pe = get_param(eqb, 'PortHandles');
    assert(numel(pc.Inport) == 11 && numel(pc.Outport) == 3, 'control_fdbk link not updated (%d in)', numel(pc.Inport));
    assert(numel(pe.Inport) == 3, 'est_quest_rp link not updated'); assert(numel(pr.Outport) == 2, 'createDes_RN link not updated');
    fprintf('control period Ts = %s s\n', Ts);

    % des_rates (in 9) <- same signal as Pranith's PD "Desired rates", converted to single
    if free(pc.Inport(9))
        pd = get_param([mdl '/PD Controller'], 'PortHandles');
        src = get_param(get_param(pd.Inport(1), 'Line'), 'SrcPortHandle');
        p = get_param(pc.Inport(9), 'Position');
        b = add_block('simulink/Signal Attributes/Data Type Conversion', [mdl '/des_rates_to_FSW'], ...
            'MakeNameUnique', 'on', 'OutDataTypeStr', 'single', 'Position', [p(1)-120 p(2)-10 p(1)-60 p(2)+10]);
        bp = get_param(b, 'PortHandles');
        add_line(mdl, src, bp.Inport(1), 'autorouting', 'on');
        add_line(mdl, bp.Outport(1), pc.Inport(9), 'autorouting', 'on');
        fprintf('+ control_fdbk in 9  des_rates        <- PD Controller desired rates (single)\n');
    end
    % validity flags (in 10, 11) <- constant 1, int32, control rate
    names = {'gyro_rates_valid_FSW', 'mag_vec_valid_FSW'};
    for k = 1:2
        if free(pc.Inport(9 + k))
            p = get_param(pc.Inport(9 + k), 'Position');
            b = add_block('simulink/Sources/Constant', [mdl '/' names{k}], 'MakeNameUnique', 'on', 'Value', '1', ...
                'OutDataTypeStr', 'int32', 'SampleTime', Ts, 'Position', [p(1)-90 p(2)-8 p(1)-60 p(2)+8]);
            bp = get_param(b, 'PortHandles'); add_line(mdl, bp.Outport(1), pc.Inport(9 + k), 'autorouting', 'on');
            fprintf('+ control_fdbk in %d %-17s <- 1 (int32)\n', 9 + k, names{k});
        end
    end
    % status outputs -> Terminator
    term(mdl, pc.Outport(3), 'control_fdbk_status_term');
    term(mdl, pr.Outport(2), 'createDes_RN_dcm_term');
    % QUEST weights (in 3) <- flight compute_weights for 6 lit CSS: [mag; sun]
    if free(pe.Inport(3))
        sys = [mdl '/Estimate Attitude']; p = get_param(pe.Inport(3), 'Position');
        b = add_block('simulink/Sources/Constant', [sys '/quest_weights_FSW'], 'MakeNameUnique', 'on', ...
            'Value', '[1/0.0087^2; 6/0.0349^2] / (1/0.0087^2 + 6/0.0349^2)', 'VectorParams1D', 'off', ...
            'OutDataTypeStr', 'single', 'SampleTime', Ts, 'Position', [p(1)-160 p(2)-10 p(1)-60 p(2)+10]);
        bp = get_param(b, 'PortHandles'); add_line(sys, bp.Outport(1), pe.Inport(3), 'autorouting', 'on');
        fprintf('+ est_quest_rp in 3  weights          <- compute_weights(6 CSS) = [0.73; 0.27]\n');
    end
    save_system(mdl);

    % Report compiled sample times of the inner flight S-functions
    blks = {[ctl '/control_fdbk'], [crn '/create_desRN'], [eqb '/est_quest_rp']};
    feval(mdl, [], [], [], 'compile');
    c = onCleanup(@() feval(mdl, [], [], [], 'term'));
    fprintf('\ncompiled sample time [period offset] (want [%s 0]):\n', Ts);
    for i = 1:numel(blks)
        st = get_param(blks{i}, 'CompiledSampleTime');
        if iscell(st), st = cell2mat(st(:)); end
        fprintf('  %-55s %s\n', blks{i}, mat2str(st));
    end
end

function tf = free(ph), tf = get_param(ph, 'Line') == -1; end

function term(mdl, ph, name)
    if ~free(ph), return; end
    sys = get_param(get_param(ph, 'Parent'), 'Parent');
    p = get_param(ph, 'Position');
    b = add_block('simulink/Sinks/Terminator', [sys '/' name], 'MakeNameUnique', 'on', 'Position', [p(1)+40 p(2)-8 p(1)+60 p(2)+8]);
    bp = get_param(b, 'PortHandles'); add_line(sys, ph, bp.Inport(1), 'autorouting', 'on');
    fprintf('+ %s\n', name);
end