function testResults = create_desRN_value_Test()
% Test 3.3: create_desRN (flight C) against frame properties.
% Convention from test 3.1: A(q) = [BN]-style, scalar-first. des_RN = q_RN, so A(des_RN) = dcm_NR'.
% Modes from global.h: CTRL_SUN 0x1, DCOMM 0x2, CSAC 0x3, BDOT 0x5.

    libDir = fileparts(fileparts(mfilename('fullpath')));
    addpath(libDir);
    addpath(fullfile(libDir, 'mexFiles'));
    load_system('sFunctionLibrary');
    tol = 1e-5;

    %% Harness (not saved). Inner S-function, so dcm_NR_out is available.
    mdl = 'harness_create_desRN';
    if bdIsLoaded(mdl), close_system(mdl, 0); end
    new_system(mdl);
    set_param(mdl, 'SolverType', 'Fixed-step', 'Solver', 'ode1', 'FixedStep', '1', ...
        'StopTime', '0', 'ReturnWorkspaceOutputs', 'on', 'ParameterPrecisionLossMsg', 'none');
    C = @(name, val, dt) add_block('simulink/Sources/Constant', [mdl '/' name], 'Value', val, ...
        'OutDataTypeStr', dt, 'VectorParams1D', 'off', 'SampleTime', '1');
    C('opmode',  '1',        'uint8');
    C('des_vec', '[1;0;0]',  'single');
    C('antenna', '0',        'uint8');
    C('b_eci',   '[0;0;1]',  'single');
    add_block('sFunctionLibrary/createDes_RN/create_desRN', [mdl '/create_desRN']);
    add_block('simulink/Sinks/To Workspace', [mdl '/q_out'],   'VariableName', 'q_out',   'SaveFormat', 'Timeseries');
    add_block('simulink/Sinks/To Workspace', [mdl '/dcm_out'], 'VariableName', 'dcm_out', 'SaveFormat', 'Timeseries');
    add_line(mdl, 'opmode/1',  'create_desRN/1');
    add_line(mdl, 'des_vec/1', 'create_desRN/2');
    add_line(mdl, 'antenna/1', 'create_desRN/3');
    add_line(mdl, 'b_eci/1',   'create_desRN/4');
    add_line(mdl, 'create_desRN/1', 'q_out/1');
    add_line(mdl, 'create_desRN/2', 'dcm_out/1');

    %% Cases: {name, opmode, antenna, des_vec, b_eci, kind}
    % kind: 'b' = roll from b_eci, 'x' = fallback roll from [1 0 0], 'id' = identity expected
    rng(4);
    cases = {};
    for k = 1:6
        d = randn(3,1); b = 0.45 * randn(3,1);
        cases(end+1,:) = {sprintf('CTRL_SUN random %d', k), 1, 0, d, b, 'b'}; %#ok<AGROW>
    end
    d0 = [0.3; -0.8; 0.52];
    b60 = 0.45 * rotTo(d0, 60);
    cases(end+1,:) = {'CSAC',                 3, 0, d0, b60, 'b'};
    cases(end+1,:) = {'DCOMM antenna 0',      2, 0, d0, b60, 'b'};
    cases(end+1,:) = {'DCOMM antenna 1',      2, 1, d0, b60, 'b'};
    cases(end+1,:) = {'CTRL_SUN antenna 1',   1, 1, d0, b60, 'b'};
    cases(end+1,:) = {'opmode 0 (no flip)',   0, 0, d0, b60, 'b'};
    cases(end+1,:) = {'des scaled x1000',     1, 0, 1000*d0, b60, 'b'};
    cases(end+1,:) = {'units: b in Tesla',    1, 0, d0, 1e-4 * b60, 'b'};
    cases(end+1,:) = {'units: b in Gauss',    1, 0, d0, b60, 'b'};
    cases(end+1,:) = {'units: b in nT',       1, 0, d0, 1e5 * b60, 'b'};
    cases(end+1,:) = {'des parallel to b',    1, 0, d0, 0.45 * d0 / norm(d0), 'x'};
    cases(end+1,:) = {'des, b, x all parallel', 1, 0, [-1;0;0], [-0.45;0;0], 'id'};
    cases(end+1,:) = {'zero des_vec',         1, 0, [0;0;0], b60, 'id'};

    %% Run
    n = size(cases,1);
    testResults = struct();
    testResults.passed = false(n,1);
    fprintf('%-24s %9s %9s %9s %9s %9s  %s\n', 'case', 'orthonorm', 'x axis', 'y perp', 'q vs dcm', 'q0>=0', 'result');
    for k = 1:n
        [name, op, ant, d, b, kind] = cases{k,:};
        d = double(single(d)); b = double(single(b));
        set_param([mdl '/opmode'],  'Value', num2str(op));
        set_param([mdl '/antenna'], 'Value', num2str(ant));
        set_param([mdl '/des_vec'], 'Value', mat2str(d, 17));
        set_param([mdl '/b_eci'],   'Value', mat2str(b, 17));
        out = sim(mdl);
        q = double(lastSample(out.q_out,   [4 1]));
        D = double(lastSample(out.dcm_out, [3 3]));

        if strcmp(kind, 'id')
            e1 = norm(D - eye(3)); e2 = norm(q - [1;0;0;0]);
            ok = e1 < tol && e2 < tol;
            fprintf('%-24s %9s %9s %9s %9s %9s  %s  (identity: dcm %.1e, q %.1e)\n', name, '-', '-', '-', '-', '-', passStr(ok), e1, e2);
        else
            if any(op == [1 2 3])
                s = -1; if ant ~= 0 && op == 2, s = 1; end
            else
                s = 1;
            end
            xExp = s * d / norm(d);
            if strcmp(kind, 'b'), rollRef = b / norm(b); else, rollRef = [1;0;0]; end

            eOrth = max(norm(D'*D - eye(3)), abs(det(D) - 1));
            eX    = vecAngle(D(:,1), xExp);
            eY    = abs(dot(D(:,2), rollRef));
            eQ    = norm(refA(q) - D');
            pos   = q(1) >= -tol;
            ok = eOrth < tol && eX < tol && eY < tol && eQ < tol && pos;
            fprintf('%-24s %9.1e %9.1e %9.1e %9.1e %9d  %s\n', name, eOrth, eX, eY, eQ, pos, passStr(ok));
        end
        testResults.passed(k) = ok;
    end
    fprintf('\nx axis: angle (rad) between dcm_NR(:,1) and the expected pointing axis\n');
    fprintf('y perp: |dcm_NR(:,2) . roll reference|, roll reference = b_eci (or [1 0 0] in the fallback)\n');
    fprintf('%d passed, %d failed\n', sum(testResults.passed), sum(~testResults.passed));
    close_system(mdl, 0);
end

%% Helpers
function v2 = rotTo(v1, deg)
    v1 = v1 / norm(v1);
    u = cross(v1, [0;0;1]); u = u / norm(u);
    v2 = cosd(deg) * v1 + sind(deg) * u;
end

function t = vecAngle(a, b)
    t = atan2(norm(cross(a, b)), dot(a, b));
end

function A = refA(q)
    q0 = q(1); v = q(2:4); v = v(:);
    A = (q0^2 - v'*v) * eye(3) + 2*(v*v') - 2*q0*[0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];
end

function s = passStr(ok)
    if ok, s = 'PASS'; else, s = 'FAIL'; end
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