function testResults = quat_dcm_conventions_Test(show)
% Test 3.1: quaternion and DCM conventions of the q_2_dcm and dcm_2_q blocks.
% quat_dcm_conventions_Test(true) leaves the harness model open after the run.
% Runs the flight C code through the S-function blocks and compares it
% against an independent MATLAB reference. Quaternions are scalar first.

    libDir = fileparts(fileparts(mfilename('fullpath')));   % ...\sFunctionLibrary
    addpath(libDir);
    addpath(fullfile(libDir, 'mexFiles'));
    load_system('sFunctionLibrary');
    tol = 1e-5;   % single precision

    %% Build the harness model (not saved)
    mdl = 'harness_quat_dcm_conventions';
    if bdIsLoaded(mdl), close_system(mdl, 0); end
    new_system(mdl);
    set_param(mdl, 'SolverType', 'Fixed-step', 'Solver', 'FixedStepDiscrete', ...
        'FixedStep', '1', 'StopTime', '0', 'ReturnWorkspaceOutputs', 'on');
    set_param(mdl, 'ParameterPrecisionLossMsg', 'none');

    add_block('simulink/Sources/Constant', [mdl '/q_in'], 'Value', '[1;0;0;0]', ...
        'OutDataTypeStr', 'single', 'VectorParams1D', 'off', 'SampleTime', '1');
    add_block('simulink/Sources/Constant', [mdl '/dcm_in'], 'Value', 'eye(3)', ...
        'OutDataTypeStr', 'single', 'SampleTime', '1');
    add_block('sFunctionLibrary/q_2_dcm', [mdl '/q_2_dcm']);
    add_block('sFunctionLibrary/dcm_2_q', [mdl '/dcm_2_q']);
    add_block('sFunctionLibrary/dcm_2_q', [mdl '/dcm_2_q_rt']);
    add_block('simulink/Sinks/To Workspace', [mdl '/dcm_out'], 'VariableName', 'dcm_out', 'SaveFormat', 'Timeseries');
    add_block('simulink/Sinks/To Workspace', [mdl '/q_out'],   'VariableName', 'q_out',   'SaveFormat', 'Timeseries');
    add_block('simulink/Sinks/To Workspace', [mdl '/q_rt'],    'VariableName', 'q_rt',    'SaveFormat', 'Timeseries');

    add_line(mdl, 'q_in/1',       'q_2_dcm/1');
    add_line(mdl, 'q_2_dcm/1',    'dcm_out/1');
    add_line(mdl, 'q_2_dcm/1',    'dcm_2_q_rt/1');
    add_line(mdl, 'dcm_2_q_rt/1', 'q_rt/1');
    add_line(mdl, 'dcm_in/1',     'dcm_2_q/1');
    add_line(mdl, 'dcm_2_q/1',    'q_out/1');

    %% Test cases
    qa = @(axis, deg) [cosd(deg/2); sind(deg/2) * axis(:) / norm(axis)];
    cases = {
        'identity',               [1; 0; 0; 0]
        '+90 deg about x',        qa([1 0 0],  90)
        '+90 deg about y',        qa([0 1 0],  90)
        '+90 deg about z',        qa([0 0 1],  90)
        '-90 deg about z',        qa([0 0 1], -90)
        '180 deg about x',        qa([1 0 0], 180)
        '180 deg about y',        qa([0 1 0], 180)
        '180 deg about z',        qa([0 0 1], 180)
        '180 deg about [1 1 0]',  qa([1 1 0], 180)
        '179.9 deg, random axis', qa([0.3 -0.8 0.5], 179.9)
        '0.001 deg about z',      qa([0 0 1], 0.001)
    };
    rng(1);
    for k = 1:10
        r = randn(4, 1); r = r / norm(r);
        cases(end+1, :) = {sprintf('random %d', k), r}; %#ok<AGROW>
    end

    %% Run
    nCases = size(cases, 1);
    testResults = struct();
    testResults.passed = false(nCases, 1);
    fprintf('%-24s %11s %11s %11s %11s %6s  %s\n', 'case', 'q2dcm vs A', 'vs A''', ...
        'dcm2q', 'round trip', 'q0>=0', 'result');
    for k = 1:nCases
        q = cases{k, 2};
        A = refA(q);
        set_param([mdl '/q_in'],   'Value', mat2str(q, 17));
        set_param([mdl '/dcm_in'], 'Value', mat2str(A, 17));
        out = sim(mdl);

        D  = double(lastSample(out.dcm_out, [3 3]));
        qb = double(lastSample(out.q_out,   [4 1]));
        qr = double(lastSample(out.q_rt,    [4 1]));

        eA  = max(abs(D(:) - A(:)));
        eAt = max(abs(D(:) - reshape(A', [], 1)));
        eq  = qdist(qb, q);
        er  = qdist(qr, q);
        pos = qb(1) >= -tol && qr(1) >= -tol;

        ok = eA < tol && eq < tol && er < tol && pos;
        testResults.passed(k) = ok;
        if ok, res = 'PASS'; else, res = 'FAIL'; end
        fprintf('%-24s %11.2e %11.2e %11.2e %11.2e %6d  %s\n', cases{k, 1}, eA, eAt, eq, er, pos, res);
    end

    %% Which way does the DCM map?
    q = qa([0 0 1], 90);
    set_param([mdl '/q_in'], 'Value', mat2str(q, 17));
    out = sim(mdl);
    D = double(lastSample(out.dcm_out, [3 3]));
    fprintf('\n+90 deg about z: dcm * [1;0;0] = [%.4f %.4f %.4f]\n', D * [1; 0; 0]);
    fprintf('  [0 -1 0]: dcm maps inertial components into body components (dcm = [BN])\n');
    fprintf('  [0  1 0]: dcm maps body components into inertial components (dcm = [NB])\n');
    fprintf('\n%d passed, %d failed\n', sum(testResults.passed), sum(~testResults.passed));

    if nargin >= 1 && show
        Simulink.BlockDiagram.arrangeSystem(mdl);   % blocks were added without positions
        open_system(mdl);   % leave the harness open (Constants show the last case)
    else
        close_system(mdl, 0);
    end
end

%% Helpers
function A = refA(q)
    q0 = q(1); v = q(2:4); v = v(:);
    A = (q0^2 - v' * v) * eye(3) + 2 * (v * v') - 2 * q0 * skew(v);
end

function S = skew(v)
    S = [0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];
end

function d = qdist(a, b)
    d = min(norm(a(:) - b(:)), norm(a(:) + b(:)));
end

function v = lastSample(ts, sz)
    d = ts.Data;
    if numel(d) == prod(sz)
        v = reshape(d, sz);
    elseif ndims(d) == 3
        v = reshape(d(:, :, end), sz);
    else
        v = reshape(d(end, :), sz);
    end
end