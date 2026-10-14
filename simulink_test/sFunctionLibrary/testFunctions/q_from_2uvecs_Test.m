function testResults = q_from_2uvecs_Test()
% Test 3.2: q_from_2uvecs block against an independent reference.
% Convention from test 3.1: dcm = A(q) = [BN], scalar-first q.
% Expected relation: A(q)' * v1hat = v2hat.

    libDir = fileparts(fileparts(mfilename('fullpath')));
    addpath(libDir);
    addpath(fullfile(libDir, 'mexFiles'));
    load_system('sFunctionLibrary');
    tol = 1e-5;   % rad

    %% Harness (not saved)
    mdl = 'harness_q_from_2uvecs';
    if bdIsLoaded(mdl), close_system(mdl, 0); end
    new_system(mdl);
    set_param(mdl, 'SolverType', 'Fixed-step', 'Solver', 'ode1', 'FixedStep', '1', ...
        'StopTime', '0', 'ReturnWorkspaceOutputs', 'on', 'ParameterPrecisionLossMsg', 'none');
    add_block('simulink/Sources/Constant', [mdl '/v1'], 'Value', '[1;0;0]', ...
        'OutDataTypeStr', 'single', 'VectorParams1D', 'off', 'SampleTime', '1');
    add_block('simulink/Sources/Constant', [mdl '/v2'], 'Value', '[0;1;0]', ...
        'OutDataTypeStr', 'single', 'VectorParams1D', 'off', 'SampleTime', '1');
    add_block('sFunctionLibrary/q_from_2uvecs', [mdl '/q_from_2uvecs']);
    add_block('simulink/Sinks/To Workspace', [mdl '/q_out'], 'VariableName', 'q_out', 'SaveFormat', 'Timeseries');
    add_line(mdl, 'v1/1', 'q_from_2uvecs/1');
    add_line(mdl, 'v2/1', 'q_from_2uvecs/2');
    add_line(mdl, 'q_from_2uvecs/1', 'q_out/1');

    %% Cases: {name, v1, v2, expectIdentity}
    rng(2);
    cases = {};
    for k = 1:15
        a = randn(3, 1) * 10^(randi([-1 3]));
        b = randn(3, 1) * 10^(randi([-1 3]));
        cases(end+1, :) = {sprintf('random %d', k), a, b, false}; %#ok<AGROW>
    end
    v1n = [0.48; -0.6; 0.64];
    for deg = [89 89.9 89.99 89.999 90 90.001 90.01 90.1 91]
        cases(end+1, :) = {sprintf('%.3f deg', deg), v1n, atAngle(v1n, deg), false}; %#ok<AGROW>
    end
    cases(end+1, :) = {'parallel, scaled',     v1n, 7 * v1n,               false};
    cases(end+1, :) = {'1e-4 deg',             v1n, atAngle(v1n, 1e-4),    false};
    cases(end+1, :) = {'1e-7 rad (< EPS)',     v1n, atAngle(v1n, rad2deg(1e-7)), false};
    cases(end+1, :) = {'opposite, v1 = x',     [1; 0; 0], [-2; 0; 0],      false};
    cases(end+1, :) = {'opposite, general',    v1n, -3 * v1n,              false};
    cases(end+1, :) = {'179.99 deg',           v1n, atAngle(v1n, 179.99),  false};
    cases(end+1, :) = {'179.9999 deg',         v1n, atAngle(v1n, 179.9999), false};
    cases(end+1, :) = {'zero v1',              [0; 0; 0], v1n,             true};
    cases(end+1, :) = {'zero v2',              v1n, [0; 0; 0],             true};

    %% Run
    n = size(cases, 1);
    testResults = struct();
    testResults.passed = false(n, 1);
    fprintf('%-20s %12s %12s %10s  %s\n', 'case', 'A''v1 vs v2', 'A v1 vs v2', '|q|-1', 'result');
    for k = 1:n
        a = double(single(cases{k, 2}));
        b = double(single(cases{k, 3}));
        set_param([mdl '/v1'], 'Value', mat2str(a, 17));
        set_param([mdl '/v2'], 'Value', mat2str(b, 17));
        out = sim(mdl);
        q = double(lastSample(out.q_out, [4 1]));
        nErr = abs(norm(q) - 1);

        if cases{k, 4}
            e1 = norm(q - [1; 0; 0; 0]); e2 = NaN;
            ok = e1 < tol;
        else
            A  = refA(q);
            ah = a / norm(a); bh = b / norm(b);
            e1 = vecAngle(A' * ah, bh);   % expected relation
            e2 = vecAngle(A  * ah, bh);   % opposite convention, for diagnosis
            ok = e1 < tol && nErr < tol;
        end
        testResults.passed(k) = ok;
        if ok, res = 'PASS'; else, res = 'FAIL'; end
        fprintf('%-20s %12.2e %12.2e %10.2e  %s\n', cases{k, 1}, e1, e2, nErr, res);
    end
    fprintf('\nangles in rad, tolerance %.0e rad (%.4f deg)\n', tol, rad2deg(tol));
    fprintf('%d passed, %d failed\n', sum(testResults.passed), sum(~testResults.passed));
    close_system(mdl, 0);
end

%% Helpers
function v2 = atAngle(v1, deg)
    v1 = v1 / norm(v1);
    u = cross(v1, [0; 0; 1]); u = u / norm(u);
    v2 = cosd(deg) * v1 + sind(deg) * u;
end

function t = vecAngle(a, b)
    t = atan2(norm(cross(a, b)), dot(a, b));
end

function A = refA(q)
    q0 = q(1); v = q(2:4); v = v(:);
    A = (q0^2 - v' * v) * eye(3) + 2 * (v * v') - 2 * q0 * [0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];
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