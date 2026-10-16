function testResults = sunpoint_no_gps_sign_Test(show)
% GPS-off sun pointing: create_sun_des_q -> control_fdbk, flight C code.
% sunpoint_no_gps_sign_Test(true) leaves the harness model open after the run.
% Correct behaviour: torque along des x sun_B (turns the -X panels toward the sun)
% and A(q_BR)*des = sun_B (q_BR is [BR]). Convention from the 3.1 test.

    libDir = fileparts(fileparts(mfilename('fullpath')));
    addpath(libDir);
    addpath(fullfile(libDir, 'mexFiles'));
    load_system('sFunctionLibrary');
    des = [-1; 0; 0];            % flight des_bf_vec (-X panels)
    CTRL_SUN = 1;

    %% Harness (not saved)
    mdl = 'harness_sunpoint_sign';
    if bdIsLoaded(mdl), close_system(mdl, 0); end
    new_system(mdl);
    set_param(mdl, 'SolverType', 'Fixed-step', 'Solver', 'ode1', 'FixedStep', '1', ...
        'StopTime', '0', 'ReturnWorkspaceOutputs', 'on', 'ParameterPrecisionLossMsg', 'none');

    C = @(name, val, dt) add_block('simulink/Sources/Constant', [mdl '/' name], 'Value', val, ...
        'OutDataTypeStr', dt, 'VectorParams1D', 'off', 'SampleTime', '1');
    C('op_mode',  num2str(CTRL_SUN),  'uint8');
    C('sun_B',    '[0;-1;0]',         'single');
    C('b_eci',    '[0;0;1]',          'single');
    C('gps_off',  'false',            'boolean');
    C('q_BN',     '[1;0;0;0]',        'single');
    C('q_des_RN', '[1;0;0;0]',        'single');
    C('gyro',     '[0;0;0]',          'single');
    C('mag_bf',   '[0.2;0.1;-0.3]',   'single');
    C('ctl_gain', '[1;0;0;0]',        'single');
    C('des_rates','[0;0;0]',          'single');
    C('valid1',   '1',                'int32');
    C('valid2',   '1',                'int32');

    add_block('sFunctionLibrary/create_sun_des_q', [mdl '/create_sun_des_q']);
    add_block('sFunctionLibrary/control_fdbk/control_fdbk', [mdl '/control_fdbk']);
    add_block('simulink/Sinks/To Workspace', [mdl '/u_out'], 'VariableName', 'u_out', 'SaveFormat', 'Timeseries');
    add_block('simulink/Sinks/To Workspace', [mdl '/qBR_out'], 'VariableName', 'qBR_out', 'SaveFormat', 'Timeseries');

    % create_sun_des_q: op_mode, sun_vec, b_eci, gps_result_override
    add_line(mdl, 'op_mode/1', 'create_sun_des_q/1');
    add_line(mdl, 'sun_B/1',   'create_sun_des_q/2');
    add_line(mdl, 'b_eci/1',   'create_sun_des_q/3');
    add_line(mdl, 'gps_off/1', 'create_sun_des_q/4');
    % control_fdbk: op_mode, q_BN, q_des_RN, gyro, mag_bf, ctl_gain, gps, q_BR_noGPS,
    %               des_rates, gyro_valid, mag_valid
    add_line(mdl, 'op_mode/1',            'control_fdbk/1');
    add_line(mdl, 'q_BN/1',               'control_fdbk/2');
    add_line(mdl, 'q_des_RN/1',           'control_fdbk/3');
    add_line(mdl, 'gyro/1',               'control_fdbk/4');
    add_line(mdl, 'mag_bf/1',             'control_fdbk/5');
    add_line(mdl, 'ctl_gain/1',           'control_fdbk/6');
    add_line(mdl, 'gps_off/1',            'control_fdbk/7');
    add_line(mdl, 'create_sun_des_q/2',   'control_fdbk/8');
    add_line(mdl, 'des_rates/1',          'control_fdbk/9');
    add_line(mdl, 'valid1/1',             'control_fdbk/10');
    add_line(mdl, 'valid2/1',             'control_fdbk/11');
    add_line(mdl, 'control_fdbk/1',       'u_out/1');
    add_line(mdl, 'create_sun_des_q/2',   'qBR_out/1');

    %% Sun directions in the body frame (all 5 to 175 deg away from des)
    cases = {
        'sun at -Y (worked example)', [0; -1; 0]
        'sun at +Y',                  [0;  1; 0]
        'sun at +Z',                  [0;  0; 1]
        'sun at -Z',                  [0;  0; -1]
        'sun 45 deg, in xy',          [-1; -1; 0]
        'sun 135 deg, in xz',         [1; 0; 1]
        'sun 10 deg off -X',          [-cosd(10); sind(10); 0]
        'sun 170 deg off -X',         [cosd(10); 0; sind(10)]
    };
    rng(3);
    while size(cases, 1) < 16
        s = randn(3, 1); s = s / norm(s);
        ang = acosd(dot(s, des));
        if ang > 5 && ang < 175
            cases(end+1, :) = {sprintf('random, %.0f deg', ang), s}; %#ok<AGROW>
        end
    end

    %% Run
    n = size(cases, 1);
    testResults = struct();
    testResults.passed = false(n, 1);
    fprintf('%-28s %16s %16s  %s\n', 'case', 'torque vs ideal', 'A(qBR)*des', 'result');
    for k = 1:n
        s = cases{k, 2}; s = s / norm(s);
        set_param([mdl '/sun_B'], 'Value', mat2str(s, 17));
        out = sim(mdl);
        u   = double(lastSample(out.u_out,   [6 1]));
        qBR = double(lastSample(out.qBR_out, [4 1]));

        ideal = cross(des, s);
        tq = u(1:3);
        tAng = atan2d(norm(cross(tq, ideal)), dot(tq, ideal));   % 0 = correct, 180 = reversed
        cErr = atan2d(norm(cross(refA(qBR) * des, s)), dot(refA(qBR) * des, s));

        ok = tAng < 1 && cErr < 0.01;
        testResults.passed(k) = ok;
        if ok, res = 'PASS'; else, res = 'FAIL'; end
        fprintf('%-28s %13.2f deg %13.4f deg  %s\n', cases{k, 1}, tAng, cErr, res);
    end
    fprintf('\ntorque vs ideal: 0 deg = turns panels toward the sun, 180 deg = away\n');
    fprintf('A(qBR)*des: 0 deg = q_BR is [BR] (correct sign for -Kp*q_vec)\n');
    fprintf('%d passed, %d failed\n', sum(testResults.passed), sum(~testResults.passed));
    if nargin >= 1 && show
        Simulink.BlockDiagram.arrangeSystem(mdl);   % blocks were added without positions
        open_system(mdl);   % leave the harness open (Constants show the last case)
    else
        close_system(mdl, 0);
    end
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