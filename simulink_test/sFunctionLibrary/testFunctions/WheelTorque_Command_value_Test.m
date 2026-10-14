function testResults = WheelTorque_Command_value_Test()
% WheelTorque_Command (flight C) against physical properties of the MMCL.
% Body torque from wheels = -Gs*us, rod torque = mu x B. Units: wheel speeds rad/s,
% mag_bf Gauss, out_u N m, rw_torque N m, pwm_dipole mA m^2.

    libDir = fileparts(fileparts(mfilename('fullpath')));
    addpath(libDir);
    addpath(fullfile(libDir, 'mexFiles'));
    load_system('sFunctionLibrary');

    % Rod dipole limit, read from flight global.h (mA m^2), as used in rw_set.c
    gh = fullfile(libDir, '..', '..', '..', 'maxwell_adcs', 'flight', 'include', 'global.h');
    tok = regexp(fileread(gh), '#define\s+TORQ_ROD_DIPOLE_MAX\s+\(?\s*\(?\s*(?:float\))?\s*([-\d.eE+]+)', 'tokens', 'once');
    assert(~isempty(tok), 'TORQ_ROD_DIPOLE_MAX not found in %s', gh);
    muMax = str2double(tok{1}) * 1e-3 * 0.99;          % A m^2
    fprintf('TORQ_ROD_DIPOLE_MAX = %s mA m^2, limit used = %.4f A m^2\n\n', tok{1}, muMax);

    Iws = 0.000029382450521;
    OmD = 250 * 2*pi/60;                                % bias, rad/s
    c   = 0.05;                                         % global.c c_desat
    Gs  = [0.390731  0.390731  0.390731  0.390731;
           0        -0.920505  0.920505  0;
          -0.920505  0         0         0.920505];

    %% Harness (not saved), inner S-function
    mdl = 'harness_wheeltorque';
    if bdIsLoaded(mdl), close_system(mdl, 0); end
    new_system(mdl);
    set_param(mdl, 'SolverType', 'Fixed-step', 'Solver', 'ode1', 'FixedStep', '1', ...
        'StopTime', '0', 'ReturnWorkspaceOutputs', 'on', 'ParameterPrecisionLossMsg', 'none');
    C = @(name, val, dt) add_block('simulink/Sources/Constant', [mdl '/' name], 'Value', val, ...
        'OutDataTypeStr', dt, 'VectorParams1D', 'off', 'SampleTime', '1');
    C('wheel_speeds',   mat2str(OmD*ones(4,1)), 'single');
    C('mag_bf',         '[0.2;0.1;-0.2]',       'single');
    C('GsValues',       mat2str(Gs, 17),        'single');
    C('disabled_wheel', '0',                    'int32');
    C('out_u',          '[0;0;0]',              'single');
    C('c_desat',        num2str(c),             'single');
    add_block('sFunctionLibrary/WheelTorque_Command/S-Function Builder', [mdl '/WTC']);
    src = {'wheel_speeds','mag_bf','GsValues','disabled_wheel','out_u','c_desat'};
    for i = 1:6, add_line(mdl, [src{i} '/1'], sprintf('WTC/%d', i)); end
    snk = {'pwm','us','st'};
    for i = 1:3
        add_block('simulink/Sinks/To Workspace', [mdl '/' snk{i}], 'VariableName', snk{i}, 'SaveFormat', 'Timeseries');
        add_line(mdl, sprintf('WTC/%d', i), [snk{i} '/1']);
    end

    rng(8);
    names = {}; ok = []; statuses = [];
    fprintf('%-34s %10s %9s %9s %9s %7s  %s\n', 'case', 'body err', 'dump ang', 'dump mag', '|mu|/max', 'status', 'result');
    Bdir = @() unit(randn(3,1));

    %% 1. Attitude torque only (wheels at bias, no dump)
    for k = 1:4
        L = 1e-4 * randn(3,1);
        r = runCase(OmD*ones(4,1), 0.3*Bdir(), 0, L);
        report(sprintf('attitude only %d', k), r, r.eBody < 1e-4 && r.muRatio < 1e-6);
    end

    %% 2. Dump only (out_u = 0), small to large wheel speed errors
    for dOm = [0.5 2 20 200]
        for k = 1:2
            Om = OmD + dOm * randn(4,1);
            r = runCase(Om, 0.3*Bdir(), 0, zeros(3,1));
            pass = r.eBody < 1e-4 && r.dumpAng < 0.1 && r.muRatio <= 1 + 1e-4 && (r.sat || abs(r.dumpMag - 1) < 1e-3);
            report(sprintf('dump only, dOmega %g rad/s #%d', dOm, k), r, pass);
        end
    end

    %% 3. Combined attitude + dump
    for k = 1:4
        r = runCase(OmD + 20*randn(4,1), 0.3*Bdir(), 0, 1e-4*randn(3,1));
        report(sprintf('attitude + dump %d', k), r, r.eBody < 1e-4 && r.muRatio <= 1 + 1e-4);
    end

    %% 4. Weak field (30 nT = 3e-4 G): rods off, attitude torque still exact
    r = runCase(OmD + 20*randn(4,1), 3e-4*Bdir(), 0, 1e-4*randn(3,1));
    report('weak field 30 nT', r, r.eBody < 1e-4 && r.muRatio == 0);

    %% 5. Disabled wheels 1..4 (Gs column zeroed inside the law)
    for dw = 1:4
        Om = OmD + 20*randn(4,1);
        r = runCase(Om, 0.3*Bdir(), dw, 1e-4*randn(3,1));
        report(sprintf('disabled wheel %d (us_%d = %.2e)', dw, dw, r.us(dw)), r, r.eBody < 1e-4 && r.muRatio <= 1 + 1e-4);
    end

    %% 6. All wheels disabled (7): everything zero
    r = runCase(OmD + 20*randn(4,1), 0.3*Bdir(), 7, 1e-4*randn(3,1));
    report('disabled_wheel = 7', r, all(r.us == 0) && all(r.pwm == 0));

    %% 7. Status convention (0 = success) over all cases above
    pass = all(statuses == 0);
    names{end+1} = 'status is 0 on success'; ok(end+1) = pass; %#ok<SETNU>
    fprintf('%-34s %s  %s\n', 'status is 0 on success', mat2str(unique(statuses)), passStr(pass));

    %% Info only (not counted): field fed in Tesla instead of Gauss
    r = runCase(OmD + 20*randn(4,1), 0.3e-4*Bdir(), 0, zeros(3,1));
    fprintf('\n[info] field given in Tesla (3e-5): max|pwm_dipole| = %.3g mA m^2 (rods %s)\n', ...
        max(abs(r.pwm)), ternary(max(abs(r.pwm)) == 0, 'OFF', 'on'));

    testResults = struct();
    testResults.passed = logical(ok(:));
    fprintf(['\nbody err: |(-Gs*us + mu x B) - out_u| / scale (property 1)\n' ...
             'dump ang: angle (deg) between rod torque and -(dh)perp B (property 2, out_u = 0 only)\n' ...
             'dump mag: |rod torque| / |c (dh)perp B| (= 1 when not saturated)\n']);
    fprintf('%d passed, %d failed\n', sum(ok), sum(~ok));
    close_system(mdl, 0);

    function r = runCase(Om, magG, dw, L)
        set_param([mdl '/wheel_speeds'],   'Value', mat2str(double(single(Om)), 17));
        set_param([mdl '/mag_bf'],         'Value', mat2str(double(single(magG)), 17));
        set_param([mdl '/disabled_wheel'], 'Value', num2str(dw));
        set_param([mdl '/out_u'],          'Value', mat2str(double(single(L)), 17));
        out = sim(mdl);
        r.pwm = double(lastSample(out.pwm, [3 1]));
        r.us  = double(lastSample(out.us,  [4 1]));
        r.st  = double(lastSample(out.st,  [1 1]));
        statuses(end+1) = r.st;
        GsE = Gs; OmDv = OmD*ones(4,1);
        if dw >= 1 && dw <= 4, GsE(:,dw) = 0; OmDv(dw) = 0; end
        mu = r.pwm / 1000;  bT = double(single(magG)) * 1e-4;
        tauR = cross(mu, bT);
        scale = norm(L) + norm(GsE*r.us) + norm(tauR) + 1e-12;
        r.eBody = norm(-GsE*r.us + tauR - double(single(L))) / scale;
        r.muRatio = max(abs(mu)) / muMax;
        r.sat = r.muRatio > 1 - 1e-3;
        v = Iws * GsE * (double(single(Om)) - OmDv);
        bh = bT / norm(bT);
        tauExp = -c * (v - bh * (bh' * v));
        if norm(L) == 0 && norm(tauExp) > 0 && norm(tauR) > 0
            r.dumpAng = atan2d(norm(cross(tauR, tauExp)), dot(tauR, tauExp));
            r.dumpMag = norm(tauR) / norm(tauExp);
        else
            r.dumpAng = NaN; r.dumpMag = NaN;
        end
    end

    function report(name, r, pass)
        names{end+1} = name; ok(end+1) = pass;
        fprintf('%-34s %10.1e %9.3f %9.4f %9.4f %7d  %s\n', name, r.eBody, r.dumpAng, r.dumpMag, r.muRatio, r.st, passStr(pass));
    end
end

function s = passStr(p), if p, s = 'PASS'; else, s = 'FAIL'; end, end
function v = unit(v), v = v / norm(v); end
function s = ternary(c, a, b), if c, s = a; else, s = b; end, end

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