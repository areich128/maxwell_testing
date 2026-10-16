function testResults = control_fdbk_full_Test(show)
% control_fdbk (flight C): mode routing, PD law, validity flags, shortest path,
% control_fdbk_full_Test(true) leaves the harness model open after the run.
% DCOMM integral, B-dot. Modes (global.h): CTRL_SUN 1, DCOMM 2, CSAC 3, BDOT 5.
% ctl_gain = [Kp Kd Ki Kbdot]. gyro in deg/s, des_rates in rad/s.

    libDir = fileparts(fileparts(mfilename('fullpath')));
    addpath(libDir);
    addpath(fullfile(libDir, 'mexFiles'));
    load_system('sFunctionLibrary');
    tol = 1e-5;

    % SENS_UPDATE_RATE from the flight headers
    incDir = fullfile(libDir, '..', '..', '..', 'maxwell_adcs', 'flight', 'include');
    sur = NaN; hs = dir(fullfile(incDir, '*.h'));
    for i = 1:numel(hs)
        tok = regexp(fileread(fullfile(incDir, hs(i).name)), ...
            '#define\s+SENS_UPDATE_RATE\s+\(?\s*([-\d.eE+]+)', 'tokens', 'once');
        if ~isempty(tok), sur = str2double(tok{1}); fprintf('SENS_UPDATE_RATE = %g s (%s)\n\n', sur, hs(i).name); break; end
    end
    assert(~isnan(sur), 'SENS_UPDATE_RATE not found in %s', incDir);

    %% Harness (not saved). mag_bf = B0 + slope * t, so B-dot sees a real change.
    mdl = 'harness_ctl_full';
    if bdIsLoaded(mdl), close_system(mdl, 0); end
    new_system(mdl);
    set_param(mdl, 'SolverType', 'Fixed-step', 'Solver', 'ode1', 'FixedStep', '1', ...
        'StopTime', '0', 'ReturnWorkspaceOutputs', 'on', 'ParameterPrecisionLossMsg', 'none');
    C = @(name, val, dt) add_block('simulink/Sources/Constant', [mdl '/' name], 'Value', val, ...
        'OutDataTypeStr', dt, 'VectorParams1D', 'off', 'SampleTime', '1');
    C('op_mode',  '1',          'uint8');
    C('q_BN',     '[1;0;0;0]',  'single');
    C('q_des_RN', '[1;0;0;0]',  'single');
    C('gyro',     '[0;0;0]',    'single');
    C('ctl_gain', '[1;0;0;1]',  'single');
    C('gps',      'true',       'boolean');
    C('q_BR_in',  '[1;0;0;0]',  'single');
    C('des_rates','[0;0;0]',    'single');
    C('gyro_ok',  '1',          'int32');
    C('mag_ok',   '1',          'int32');
    C('B0',       '[0.2;0.1;-0.3]', 'double');
    C('Bslope',   '[0;0;0]',        'double');
    add_block('simulink/Sources/Clock', [mdl '/clk']);
    add_block('simulink/Math Operations/Product', [mdl '/ramp']);
    add_block('simulink/Math Operations/Sum', [mdl '/Bsum'], 'Inputs', '++');
    add_block('simulink/Signal Attributes/Data Type Conversion', [mdl '/Bsingle'], 'OutDataTypeStr', 'single');
    add_line(mdl, 'clk/1', 'ramp/1');  add_line(mdl, 'Bslope/1', 'ramp/2');
    add_line(mdl, 'B0/1', 'Bsum/1');   add_line(mdl, 'ramp/1', 'Bsum/2');
    add_line(mdl, 'Bsum/1', 'Bsingle/1');
    add_block('sFunctionLibrary/control_fdbk/control_fdbk', [mdl '/ctl']);
    src = {'op_mode','q_BN','q_des_RN','gyro','Bsingle','ctl_gain','gps','q_BR_in','des_rates','gyro_ok','mag_ok'};
    for i = 1:numel(src), add_line(mdl, [src{i} '/1'], sprintf('ctl/%d', i)); end
    snk = {'u_out','qBR_out','st_out'};
    for i = 1:3
        add_block('simulink/Sinks/To Workspace', [mdl '/' snk{i}], 'VariableName', snk{i}, 'SaveFormat', 'Timeseries');
        add_line(mdl, sprintf('ctl/%d', i), [snk{i} '/1']);
    end

    qa = @(ax, deg) [cosd(deg/2); sind(deg/2) * ax(:) / norm(ax)];
    K  = [0.8; 2.0; 0.01; 1.0];
    rng(9);
    names = {}; ok = [];
    fprintf('%-40s %10s %10s %7s  %s\n', 'case', 'wheel err', 'rods', 'status', 'result');

    %% 1. PD law, GPS on, CTRL_SUN / CSAC
    for k = 1:4
        c = randCase(); clear mex;
        [u, st] = runc(1, c.qBN, c.qRN, c.gyro, c.des, K, true, [1;0;0;0], 1, 1, 1);
        e = norm(u(1:3) - pdExpect(c, K, 1, 0));
        rep(sprintf('CTRL_SUN PD #%d', k), e, norm(u(4:6)), st, e < tol && norm(u(4:6)) == 0 && st == 0);
    end
    c = randCase(); clear mex;
    [u, st] = runc(3, c.qBN, c.qRN, c.gyro, c.des, K, true, [1;0;0;0], 1, 1, 1);
    e = norm(u(1:3) - pdExpect(c, K, 1, 0));
    rep('CSAC PD (integral off)', e, norm(u(4:6)), st, e < tol && st == 0);

    %% 2. Validity flags
    c = randCase(); clear mex;
    [u, st] = runc(1, c.qBN, c.qRN, c.gyro, c.des, K, true, [1;0;0;0], 0, 1, 1);
    e = norm(u(1:3) - pdExpect(c, K, 0, 0));
    rep('gyro_rates_valid = 0 (no damping)', e, norm(u(4:6)), st, e < tol && st == 0);
    [u, st] = runc(1, c.qBN, c.qRN, c.gyro, c.des, K, true, [1;0;0;0], 1, 0, 1);
    e = norm(u(1:3) - pdExpect(c, K, 1, 0));
    rep('mag_vec_valid = 0 in CTRL_SUN (no effect)', e, norm(u(4:6)), st, e < tol && st == 0);

    %% 3. Shortest path: 200 deg error must be taken as 160 the other way
    c = struct('qBN', qa([0 0 1], 200), 'qRN', [1;0;0;0], 'gyro', [0;0;0], 'des', [0;0;0]); clear mex;
    [u, st] = runc(1, c.qBN, c.qRN, c.gyro, c.des, K, true, [1;0;0;0], 1, 1, 1);
    e = norm(u(1:3) - pdExpect(c, K, 1, 0));
    rep('200 deg error -> short way', e, norm(u(4:6)), st, e < tol && st == 0);

    %% 4. GPS off: torque from the q_BR input
    qin = qa(randn(3,1), 30); g = 2*randn(3,1); clear mex;
    [u, st] = runc(1, [1;0;0;0], [1;0;0;0], g, [0;0;0], K, false, qin, 1, 1, 1);
    tExp = -K(1) * qin(2:4) - K(2) * (g*pi/180);
    rep('GPS off, q_BR input', norm(u(1:3) - tExp), norm(u(4:6)), st, norm(u(1:3) - tExp) < tol && st == 0);

    %% 5. DCOMM integral (static int_err): 1 deg error for 5 steps, then 5 deg
    q1 = qa(randn(3,1), 1); c = struct('qBN', q1, 'qRN', [1;0;0;0], 'gyro', [0;0;0], 'des', [0;0;0]);
    clear mex;
    [u, st] = runc(2, c.qBN, c.qRN, c.gyro, c.des, K, true, [1;0;0;0], 1, 1, 5);
    qe = dcm2q(refA(c.qBN) * refA(c.qRN)');
    e = norm(u(1:3) - pdExpect(c, K, 1, 5 * 0.3 * qe(2:4)));
    rep('DCOMM 1 deg, 5 steps: int = 5*0.3*q', e, norm(u(4:6)), st, e < tol && st == 0);
    c.qBN = qa(randn(3,1), 5); clear mex;
    [u, st] = runc(2, c.qBN, c.qRN, c.gyro, c.des, K, true, [1;0;0;0], 1, 1, 5);
    e = norm(u(1:3) - pdExpect(c, K, 1, 0));
    rep('DCOMM 5 deg: integral held at 0', e, norm(u(4:6)), st, e < tol && st == 0);

    %% 6. B-dot: B ramps, rods = -Kb * dB / SENS_UPDATE_RATE, wheels zero
    B0 = [0.2; 0.1; -0.3]; dB = 1e-3 * randn(3,1); clear mex;
    [U, S] = runc(5, [1;0;0;0], [1;0;0;0], [0;0;0], [0;0;0], K, true, [1;0;0;0], 1, 1, 3, B0, dB, true);
    rodsExp = -K(4) * dB / sur;
    e = norm(U(4:6, end) - rodsExp);
    dtEff = -K(4) * (dB' * U(4:6, end)) / (U(4:6, end)' * U(4:6, end));
    rep(sprintf('B-dot law (effective dt %.3g s)', dtEff), norm(U(1:3, end)), norm(U(4:6, end)), S(end), ...
        e < 1e-4 * norm(rodsExp) + 1e-9 && norm(U(1:3, end)) == 0 && S(end) == 0);
    rep('B-dot first call outputs zero', norm(U(:, 1)), norm(U(4:6, 1)), S(1), norm(U(:, 1)) == 0);

    clear mex;
    [u, st] = runc(5, [1;0;0;0], [1;0;0;0], [0;0;0], [0;0;0], K, true, [1;0;0;0], 1, 0, 3, B0, dB);
    rep('B-dot mag_vec_valid = 0', norm(u(1:3)), norm(u(4:6)), st, norm(u) == 0 && st == -1);

    %% 7. Other modes: all outputs zero
    for op = [0 4]
        c = randCase(); clear mex;
        [u, st] = runc(op, c.qBN, c.qRN, c.gyro, c.des, K, true, [1;0;0;0], 1, 1, 1);
        rep(sprintf('op_mode %d -> zero', op), norm(u(1:3)), norm(u(4:6)), st, norm(u) == 0);
    end

    %% Info: B-dot gain flip after 1800 calls (gain_multiplier)
    clear mex;
    [U, ~] = runc(5, [1;0;0;0], [1;0;0;0], [0;0;0], [0;0;0], K, true, [1;0;0;0], 1, 1, 1805, B0, dB, true);
    flipped = sign(U(4, end)) ~= sign(U(4, 2));
    fprintf('\n[info] B-dot output sign after 1800 calls: %s\n', ...
        ternary(flipped, 'FLIPPED (gain_multiplier is applied)', 'unchanged (gain_multiplier never used)'));

    testResults = struct();
    testResults.passed = logical(ok(:));
    fprintf('\nwheel err: |out_u(1:3) - expected|; rods: |out_u(4:6)|\n');
    fprintf('%d passed, %d failed\n', sum(ok), sum(~ok));
    if nargin >= 1 && show
        Simulink.BlockDiagram.arrangeSystem(mdl);   % blocks were added without positions
        open_system(mdl);   % leave the harness open (Constants show the last case)
    else
        close_system(mdl, 0);
    end

    %% ---- nested helpers ----
    function c = randCase()
        c.qBN = unit(randn(4,1)); c.qRN = unit(randn(4,1));
        c.gyro = 3 * randn(3,1);            % deg/s
        c.des  = 0.02 * randn(3,1);         % rad/s
    end

    function [u, st] = runc(op, qBN, qRN, gyro, des, Kg, gps, qBRin, gv, mv, n, B0v, dBv, allSamples)
        if nargin < 12, B0v = [0.2;0.1;-0.3]; dBv = [0;0;0]; end
        if nargin < 14, allSamples = false; end
        set_param([mdl '/op_mode'],  'Value', num2str(op));
        set_param([mdl '/q_BN'],     'Value', mat2str(qBN, 17));
        set_param([mdl '/q_des_RN'], 'Value', mat2str(qRN, 17));
        set_param([mdl '/gyro'],     'Value', mat2str(gyro, 17));
        set_param([mdl '/des_rates'],'Value', mat2str(des, 17));
        set_param([mdl '/ctl_gain'], 'Value', mat2str(Kg, 17));
        set_param([mdl '/gps'],      'Value', ternary(gps, 'true', 'false'));
        set_param([mdl '/q_BR_in'],  'Value', mat2str(qBRin, 17));
        set_param([mdl '/gyro_ok'],  'Value', num2str(gv));
        set_param([mdl '/mag_ok'],   'Value', num2str(mv));
        set_param([mdl '/B0'],       'Value', mat2str(B0v, 17));
        set_param([mdl '/Bslope'],   'Value', mat2str(dBv, 17));
        set_param(mdl, 'StopTime', num2str(n - 1));
        out = sim(mdl);
        N = numel(out.u_out.Time);
        if allSamples
            u = zeros(6, N); st = zeros(1, N);
            for j = 1:N, u(:, j) = double(sampleAt(out.u_out, j, 6)); st(j) = double(sampleAt(out.st_out, j, 1)); end
        else
            u = double(sampleAt(out.u_out, N, 6)); st = double(sampleAt(out.st_out, N, 1));
        end
    end

    function rep(name, e, rods, st, pass)
        names{end+1} = name; ok(end+1) = pass;
        fprintf('%-40s %10.2e %10.2e %7d  %s\n', name, e, rods, st, ternary(pass, 'PASS', 'FAIL'));
    end
end

%% ---- local helpers ----
function t = pdExpect(c, K, gyroValid, intErr)
    qe = dcm2q(refA(c.qBN) * refA(c.qRN)');     % [BR], q0 >= 0 (shortest path)
    t = -K(1) * qe(2:4);
    if gyroValid, t = t - K(2) * (c.gyro * pi/180 - c.des); end
    t = t - K(3) * intErr;
end

function v = sampleAt(ts, k, n)
    d = ts.Data; N = numel(ts.Time);
    if ndims(d) == 3, v = reshape(d(:, :, k), n, 1);
    elseif size(d, 1) == N && N > 1, v = reshape(d(k, :), n, 1);
    else, v = reshape(d(:, min(k, size(d, 2))), n, 1);
    end
end

function q = dcm2q(A)
    q0 = 0.5 * sqrt(max(0, 1 + trace(A)));
    if q0 > 1e-3
        q = [q0; (A(2,3)-A(3,2))/(4*q0); (A(3,1)-A(1,3))/(4*q0); (A(1,2)-A(2,1))/(4*q0)];
    else
        [~, i] = max(diag(A)); v = zeros(3,1);
        v(i) = 0.5 * sqrt(1 + 2*A(i,i) - trace(A));
        j = mod(i,3)+1; k = mod(i+1,3)+1;
        v(j) = (A(i,j)+A(j,i))/(4*v(i)); v(k) = (A(i,k)+A(k,i))/(4*v(i));
        q = [(A(j,k)-A(k,j))/(4*v(i)); v];
    end
    q = q / norm(q); if q(1) < 0, q = -q; end
end

function A = refA(q)
    q0 = q(1); v = q(2:4); v = v(:);
    A = (q0^2 - v'*v) * eye(3) + 2*(v*v') - 2*q0*[0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];
end

function v = unit(v), v = v / norm(v); end
function s = ternary(c, a, b), if c, s = a; else, s = b; end, end