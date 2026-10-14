function testResults = control_fdbk_gps_on_Test()
% Does GPS-on control_fdbk use the attitude error from q_BN and q_des_RN,
% or the stale q_BR passed in? The q_BR_noGPS input is set to a sentinel that
% disagrees with the true error. Kp = 1, Kd = Ki = 0, CTRL_SUN (0x1), GPS on.

    libDir = fileparts(fileparts(mfilename('fullpath')));
    addpath(libDir);
    addpath(fullfile(libDir, 'mexFiles'));
    load_system('sFunctionLibrary');

    mdl = 'harness_ctl_gps_on';
    if bdIsLoaded(mdl), close_system(mdl, 0); end
    new_system(mdl);
    set_param(mdl, 'SolverType', 'Fixed-step', 'Solver', 'ode1', 'FixedStep', '1', ...
        'StopTime', '0', 'ReturnWorkspaceOutputs', 'on', 'ParameterPrecisionLossMsg', 'none');
    C = @(name, val, dt) add_block('simulink/Sources/Constant', [mdl '/' name], 'Value', val, ...
        'OutDataTypeStr', dt, 'VectorParams1D', 'off', 'SampleTime', '1');
    C('op_mode',  '1',              'uint8');
    C('q_BN',     '[1;0;0;0]',      'single');
    C('q_des_RN', '[1;0;0;0]',      'single');
    C('gyro',     '[0;0;0]',        'single');
    C('mag_bf',   '[0.2;0.1;-0.3]', 'single');
    C('ctl_gain', '[1;0;0;0]',      'single');
    C('gps_on',   'true',           'boolean');
    C('q_BR_in',  '[1;0;0;0]',      'single');
    C('des_rates','[0;0;0]',        'single');
    C('valid1',   '1',              'int32');
    C('valid2',   '1',              'int32');
    add_block('sFunctionLibrary/control_fdbk/control_fdbk', [mdl '/control_fdbk']);
    add_block('simulink/Sinks/To Workspace', [mdl '/u_out'],   'VariableName', 'u_out',   'SaveFormat', 'Timeseries');
    add_block('simulink/Sinks/To Workspace', [mdl '/qBR_out'], 'VariableName', 'qBR_out', 'SaveFormat', 'Timeseries');
    src = {'op_mode','q_BN','q_des_RN','gyro','mag_bf','ctl_gain','gps_on','q_BR_in','des_rates','valid1','valid2'};
    for i = 1:numel(src), add_line(mdl, [src{i} '/1'], sprintf('control_fdbk/%d', i)); end
    add_line(mdl, 'control_fdbk/1', 'u_out/1');
    add_line(mdl, 'control_fdbk/2', 'qBR_out/1');

    qa = @(axis, deg) [cosd(deg/2); sind(deg/2) * axis(:) / norm(axis)];
    rng(6);
    cases = {
        'B 30 deg about z of R, sentinel = identity',   qa([0 0 1], 30), [1;0;0;0], [1;0;0;0]
        'B 30 deg about z of R, sentinel = [.5 .5 .5 .5]', qa([0 0 1], 30), [1;0;0;0], [0.5;0.5;0.5;0.5]
        'random q_BN and q_RN, sentinel = identity',    unit(randn(4,1)), unit(randn(4,1)), [1;0;0;0]
        'random q_BN and q_RN, sentinel = [0 1 0 0]',   unit(randn(4,1)), unit(randn(4,1)), [0;1;0;0]
    };

    n = size(cases, 1);
    testResults = struct();
    testResults.passed = false(n, 1);
    fprintf('%-46s %22s %22s %22s  %s\n', 'case', 'torque (C code)', 'from true error', 'from sentinel', 'result');
    for k = 1:n
        [name, qBN, qRN, qS] = cases{k, :};
        set_param([mdl '/q_BN'],     'Value', mat2str(qBN, 17));
        set_param([mdl '/q_des_RN'], 'Value', mat2str(qRN, 17));
        set_param([mdl '/q_BR_in'],  'Value', mat2str(qS, 17));
        out = sim(mdl);
        u = double(lastSample(out.u_out, [6 1]));
        tq = u(1:3);

        % True error: dcm_BR = A(q_BN) * A(q_RN)', q0 >= 0 (as in flight)
        qBR = dcm2q(refA(qBN) * refA(qRN)');
        tTrue = -qBR(2:4);
        qSn = qS / norm(qS); if qSn(1) < 0, qSn = -qSn; end
        tSent = -qSn(2:4);

        ok = norm(tq - tTrue) < 1e-4;
        testResults.passed(k) = ok;
        if ok, r = 'PASS'; else, r = 'FAIL'; end
        fprintf('%-46s %22s %22s %22s  %s\n', name, vstr(tq), vstr(tTrue), vstr(tSent), r);
    end
    fprintf('\nPASS: torque follows the true attitude error. FAIL with torque = "from sentinel": GPS-on uses the stale q_BR input.\n');
    fprintf('%d passed, %d failed\n', sum(testResults.passed), sum(~testResults.passed));
    close_system(mdl, 0);
end

function s = vstr(v), s = sprintf('[%6.3f %6.3f %6.3f]', v); end
function v = unit(v), v = v / norm(v); end

function q = dcm2q(A)
    % inverse of refA, scalar first, q0 >= 0
    q0 = 0.5 * sqrt(max(0, 1 + trace(A)));
    if q0 > 1e-3
        q = [q0; (A(2,3)-A(3,2))/(4*q0); (A(3,1)-A(1,3))/(4*q0); (A(1,2)-A(2,1))/(4*q0)];
    else
        [~, i] = max(diag(A));
        v = zeros(3,1); v(i) = 0.5 * sqrt(1 + 2*A(i,i) - trace(A));
        j = mod(i,3)+1; k = mod(i+1,3)+1;
        v(j) = (A(i,j)+A(j,i))/(4*v(i)); v(k) = (A(i,k)+A(k,i))/(4*v(i));
        q0 = (A(j,k)-A(k,j))/(4*v(i)); q = [q0; v];
    end
    q = q / norm(q); if q(1) < 0, q = -q; end
end

function A = refA(q)
    q0 = q(1); v = q(2:4); v = v(:);
    A = (q0^2 - v'*v) * eye(3) + 2*(v*v') - 2*q0*[0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];
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