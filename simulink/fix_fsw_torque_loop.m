function fix_fsw_torque_loop()
% ADCS_Simulink_FSW: close the loop on the flight controller.
% control_fdbk already uses q_BN_FSW (Estimate Attitude out 5 -> control_fdbk in 2).
% The useFSWtorque switch sends its torque to the plant, but straight from
% Selector1 in mN m (the gains are scaled by NtomN = 1000), while the plant and
% the PD Controller path are in N m. This adds the 1/1000 and turns the switch on.

    mdl = 'ADCS_Simulink_FSW';
    here = fileparts(mfilename('fullpath'));
    old = cd(here);
    restoreDir = onCleanup(@() cd(old));
    load_system(mdl);

    sw  = [mdl '/Switch'];
    sel = [mdl '/Selector1'];
    gName = 'FSW_torque_mNm_to_Nm';
    g   = [mdl '/' gName];

    pc = get_param(sw, 'PortConnectivity');
    src = getfullname(pc(1).SrcBlock);
    if isempty(find_system(mdl, 'SearchDepth', 1, 'Name', gName))
        if ~strcmp(src, sel)
            error('Switch input 1 is fed by %s, not Selector1. Nothing changed.', src);
        end
        delete_line(mdl, 'Selector1/1', 'Switch/1');
        p = get_param(sel, 'Position');
        add_block('simulink/Math Operations/Gain', g, 'Gain', '1/1000', ...
            'OutDataTypeStr', 'double', 'BlockMirror', 'on', 'Position', p + [-90 0 -90 0]);
        add_line(mdl, 'Selector1/1', [gName '/1'], 'autorouting', 'on');
        add_line(mdl, [gName '/1'], 'Switch/1', 'autorouting', 'on');
        fprintf('Inserted %s (1/1000) between Selector1 and Switch input 1\n', gName);
    else
        fprintf('%s already present, wiring unchanged\n', gName);
    end

    set_param([mdl '/useFSWtorque'], 'Value', '1');

    set_param(mdl, 'SimulationCommand', 'update');   % compile check
    pc = get_param(sw, 'PortConnectivity');
    fprintf('Switch criteria: %s, threshold %s\n', get_param(sw, 'Criteria'), get_param(sw, 'Threshold'));
    fprintf('useFSWtorque = %s\n', get_param([mdl '/useFSWtorque'], 'Value'));
    fprintf('Switch in1 <- %s\n', getfullname(pc(1).SrcBlock));
    fprintf('Switch in3 <- %s\n', getfullname(pc(3).SrcBlock));
    fprintf('Switch out -> %s\n', strjoin(cellstr(getfullname(pc(4).DstBlock)), ', '));
    save_system(mdl);
    fprintf('Saved %s\n', mdl);
end