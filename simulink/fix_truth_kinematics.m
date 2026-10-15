function fix_truth_kinematics()
% ADCS_Simulink_FSW, Estimate Attitude: the truth integrator computes
%   dq/dt = 1/2 * Omega(W) * q
% with q taken from the "q_BN initial" input, which is Pranith's QUEST output
% fed back from the top level. That output is forced to q0 >= 0, so once the
% true q0 goes negative the derivative uses -q_true and the truth runs
% backwards (q0 pinned near 0). This feeds the integrator its own normalized
% state instead. Original ADCS_Simulink is not touched.

    mdl = 'ADCS_Simulink_FSW';
    here = fileparts(mfilename('fullpath'));
    old = cd(here);
    restoreDir = onCleanup(@() cd(old));
    load_system(mdl);

    ea = [mdl '/Estimate Attitude'];
    mm = [ea '/Matrix Multiply1'];
    qn = [ea '/Quaternion Normalize'];
    qi = [ea '/q_BN initial'];
    term = 'q_BN_initial_unused';

    pc = get_param(mm, 'PortConnectivity');
    src = getfullname(pc(2).SrcBlock);
    if strcmp(src, qi)
        delete_line(ea, 'q_BN initial/1', 'Matrix Multiply1/2');
        add_line(ea, 'Quaternion Normalize/1', 'Matrix Multiply1/2', 'autorouting', 'on');
        p = get_param(qi, 'Position');
        add_block('simulink/Sinks/Terminator', [ea '/' term], 'Position', p + [60 0 60 0]);
        add_line(ea, 'q_BN initial/1', [term '/1']);
        fprintf('Matrix Multiply1 in 2 now fed by Quaternion Normalize (was q_BN initial)\n');
    elseif strcmp(src, qn)
        fprintf('Already fed by Quaternion Normalize, wiring unchanged\n');
    else
        error('Matrix Multiply1 in 2 is fed by %s, expected q_BN initial. Nothing changed.', src);
    end

    integ = find_system(ea, 'SearchDepth', 1, 'BlockType', 'DiscreteIntegrator');
    fprintf('integrator: %s, method %s, sample time %s\n', get_param(integ{1}, 'Name'), ...
        get_param(integ{1}, 'IntegratorMethod'), get_param(integ{1}, 'SampleTime'));

    set_param(mdl, 'SimulationCommand', 'update');   % compile check (catches algebraic loops)
    pc = get_param(mm, 'PortConnectivity');
    fprintf('Matrix Multiply1 in 2 <- %s\n', getfullname(pc(2).SrcBlock));
    save_system(mdl);
    fprintf('Saved %s\n', mdl);
end