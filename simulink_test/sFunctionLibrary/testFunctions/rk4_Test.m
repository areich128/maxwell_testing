function testResults = rk4_Test()
% Task 5: flight rk4 (propagator.c) vs an independent reference.
% Reference: two-body + J2..J4 with the acceleration taken as the numerical
% gradient of the zonal potential (no formulas shared with the flight code),
% integrated with ode113 at RelTol 1e-12. Same constants as param_table_default.
% Drag is left out of the reference: at 500 km it moves the orbit by about 6 m
% in 90 min, far below the pass threshold.
% Flight loop: rk4 with dT = 0.1 s, 54000 steps = 90 min (the GPS-off period).

    libDir = fileparts(fileparts(mfilename('fullpath')));
    addpath(libDir);
    addpath(fullfile(libDir, 'mexFiles'));
    load_system('sFunctionLibrary');
    blk = find_system('sFunctionLibrary', 'LookUnderMasks', 'all', 'FollowLinks', 'on', ...
        'BlockType', 'S-Function', 'FunctionName', 'rk4_sfun');
    if numel(blk) ~= 1
        error('Expected one rk4_sfun block in sFunctionLibrary, found %d', numel(blk));
    end

    c.mu = 3.986004415e5; c.Re = 6378.1363;
    c.J = [0 0.001082626683553 -2.532656485e-6 -1.619621591e-6];

    %% Harness: x(k+1) = rk4(0.1, x(k)) through a Unit Delay (not saved)
    mdl = 'harness_rk4';
    if bdIsLoaded(mdl), close_system(mdl, 0); end
    new_system(mdl);
    nSteps = 54000; dT = 0.1;
    set_param(mdl, 'SolverType', 'Fixed-step', 'Solver', 'FixedStepDiscrete', 'FixedStep', '1', ...
        'StopTime', num2str(nSteps - 1), 'ReturnWorkspaceOutputs', 'on', 'ParameterPrecisionLossMsg', 'none');
    add_block('simulink/Sources/Constant', [mdl '/dT'], 'Value', num2str(dT), ...
        'OutDataTypeStr', 'single', 'SampleTime', '1');
    add_block('simulink/Discrete/Unit Delay', [mdl '/state'], 'InitialCondition', 'single(zeros(6,1))', ...
        'SampleTime', '1');
    add_block(blk{1}, [mdl '/rk4']);
    add_block('simulink/Sinks/To Workspace', [mdl '/x_out'], 'VariableName', 'x_out', 'SaveFormat', 'Array');
    add_block('simulink/Sinks/Terminator', [mdl '/st_term']);
    add_line(mdl, 'dT/1', 'rk4/1');
    add_line(mdl, 'state/1', 'rk4/2');
    add_line(mdl, 'rk4/1', 'state/1');
    add_line(mdl, 'rk4/1', 'x_out/1');
    add_line(mdl, 'rk4/2', 'st_term/1');

    %% Cases: 500 km circular, three inclinations; 400 km sun-synchronous
    cases = {'500 km, i = 97.4 (SSO)', 500, 97.4, 0;
             '500 km, i = 51.6',       500, 51.6, 40;
             '500 km, i = 0',          500, 0.0,  0;
             '400 km, i = 97.4',       400, 97.4, 120};
    checkMin = [10 30 60 90];
    names = {}; ok = [];
    fprintf('%-26s %10s %10s %10s %10s  %s\n', 'case (pos err, km)', '10 min', '30 min', '60 min', '90 min', 'result');
    for k = 1:size(cases, 1)
        h = cases{k, 2}; inc = cases{k, 3}; raan = cases{k, 4};
        r = c.Re + h; v = sqrt(c.mu / r);
        x0 = [r*cosd(raan); r*sind(raan); 0; ...
              -v*cosd(inc)*sind(raan); v*cosd(inc)*cosd(raan); v*sind(inc)];
        x0 = double(single(x0));                  % start both from the same single values
        set_param([mdl '/state'], 'InitialCondition', ['single(' mat2str(x0, 17) ')']);
        out = sim(mdl);
        X = double(out.x_out);                    % row j = state after j steps
        if ndims(X) == 3
            X = reshape(X, 6, []).';              % 6x1xN -> N x 6
        elseif size(X, 2) ~= 6
            X = X.';
        end
        tq = checkMin * 60;
        opts = odeset('RelTol', 1e-12, 'AbsTol', 1e-12);
        [~, Y] = ode113(@(t, y) eomRef(y, c), [0 tq], x0, opts);
        Y = Y(2:end, :);
        idx = round(tq / dT);
        err = vecnorm(X(idx, 1:3) - Y(:, 1:3), 2, 2).';
        pass = err(end) < 0.5;
        names{end+1} = cases{k, 1}; ok(end+1) = pass; %#ok<AGROW>
        fprintf('%-26s %10.3f %10.3f %10.3f %10.3f  %s\n', cases{k, 1}, err, pf(pass));
    end

    testResults = struct();
    testResults.passed = logical(ok(:));
    fprintf('\nPass: position error after 90 min < 0.5 km (float rounding alone gives about 0.1 km).\n');
    fprintf('%d passed, %d failed\n', sum(ok), sum(~ok));
    close_system(mdl, 0);
end

%% Reference dynamics: acceleration = gradient of the zonal potential
function dy = eomRef(y, c)
    x = y(1:3);
    a = zeros(3, 1); hstep = 1e-3;                % km
    for i = 1:3
        e = zeros(3, 1); e(i) = hstep;
        a(i) = (potential(x + e, c) - potential(x - e, c)) / (2 * hstep);
    end
    dy = [y(4:6); a];
end

function U = potential(x, c)
    r = norm(x); s = x(3) / r;
    P2 = (3*s^2 - 1) / 2; P3 = (5*s^3 - 3*s) / 2; P4 = (35*s^4 - 30*s^2 + 3) / 8;
    U = c.mu / r * (1 - c.J(2)*(c.Re/r)^2*P2 - c.J(3)*(c.Re/r)^3*P3 - c.J(4)*(c.Re/r)^4*P4);
end

function s = pf(p), if p, s = 'PASS'; else, s = 'FAIL'; end, end