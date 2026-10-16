function testResults = rk4_fidelity_Test(show)
% Task 5, part 2: how good is the flight orbit MODEL (two-body + J2..J4 + exponential
% rk4_fidelity_Test(true) leaves the harness model open after the run.
% drag, float RK4 at 0.1 s) compared with a high-fidelity gravity field?
% Reference: EGM2008 to degree/order 30 (gravitysphericalharmonic, Aerospace Toolbox),
% ECI -> ECEF as a rotation about z at Earth rate, ode113 RelTol 1e-10.
% No drag in the reference: at 400-500 km with the flight ballistic coefficient it
% moves the orbit by metres to tens of metres in 90 min, small next to gravity.
% This is a characterization: pass/fail uses a placeholder 2 km at 90 min, to be
% replaced by the requirement that comes from the pointing budget.

    libDir = fileparts(fileparts(mfilename('fullpath')));
    addpath(libDir);
    addpath(fullfile(libDir, 'mexFiles'));
    load_system('sFunctionLibrary');
    blk = find_system('sFunctionLibrary', 'LookUnderMasks', 'all', 'FollowLinks', 'on', ...
        'BlockType', 'S-Function', 'FunctionName', 'rk4_sfun');
    if numel(blk) ~= 1
        error('Expected one rk4_sfun block in sFunctionLibrary, found %d', numel(blk));
    end

    mu = 3.986004415e5; Re = 6378.1363; wE = 7.2921150e-5;   % km, s
    degree = 30;

    %% Harness: x(k+1) = rk4(0.1, x(k)) (not saved)
    mdl = 'harness_rk4_fid';
    if bdIsLoaded(mdl), close_system(mdl, 0); end
    new_system(mdl);
    nSteps = 54000; dT = 0.1;
    set_param(mdl, 'SolverType', 'Fixed-step', 'Solver', 'FixedStepDiscrete', 'FixedStep', '1', ...
        'StopTime', num2str(nSteps - 1), 'ReturnWorkspaceOutputs', 'on', 'ParameterPrecisionLossMsg', 'none');
    add_block('simulink/Sources/Constant', [mdl '/dT'], 'Value', num2str(dT), 'OutDataTypeStr', 'single', 'SampleTime', '1');
    add_block('simulink/Discrete/Unit Delay', [mdl '/state'], 'InitialCondition', 'single(zeros(6,1))', 'SampleTime', '1');
    add_block(blk{1}, [mdl '/rk4']);
    add_block('simulink/Sinks/To Workspace', [mdl '/x_out'], 'VariableName', 'x_out', 'SaveFormat', 'Array');
    add_block('simulink/Sinks/Terminator', [mdl '/st_term']);
    add_line(mdl, 'dT/1', 'rk4/1'); add_line(mdl, 'state/1', 'rk4/2');
    add_line(mdl, 'rk4/1', 'state/1'); add_line(mdl, 'rk4/1', 'x_out/1'); add_line(mdl, 'rk4/2', 'st_term/1');

    cases = {'500 km, i = 97.4 (SSO)', 500, 97.4, 0;
             '500 km, i = 51.6',       500, 51.6, 40;
             '400 km, i = 97.4',       400, 97.4, 120;
             '600 km, i = 97.4',       600, 97.4, 250};
    checkMin = [10 30 60 90];
    ok = [];
    fprintf('Flight model vs EGM2008 degree %d. Position error, km:\n', degree);
    fprintf('%-24s %8s %8s %8s %8s   %s\n', 'case', '10 min', '30 min', '60 min', '90 min', 'angle at 1000 km range');
    for k = 1:size(cases, 1)
        h = cases{k, 2}; inc = cases{k, 3}; raan = cases{k, 4};
        r = Re + h; v = sqrt(mu / r);
        x0 = [r*cosd(raan); r*sind(raan); 0; -v*cosd(inc)*sind(raan); v*cosd(inc)*cosd(raan); v*sind(inc)];
        x0 = double(single(x0));
        set_param([mdl '/state'], 'InitialCondition', ['single(' mat2str(x0, 17) ')']);
        out = sim(mdl);
        X = double(out.x_out);
        if ndims(X) == 3, X = reshape(X, 6, []).'; elseif size(X, 2) ~= 6, X = X.'; end
        tq = checkMin * 60;
        opts = odeset('RelTol', 1e-10, 'AbsTol', 1e-10);
        [~, Y] = ode113(@(t, y) eomEGM(t, y, wE, degree), [0 tq], x0, opts);
        Y = Y(2:end, :);
        err = vecnorm(X(round(tq / dT), 1:3) - Y(:, 1:3), 2, 2).';
        ang = rad2deg(err(end) / 1000);
        pass = err(end) < 2;
        ok(end+1) = pass; %#ok<AGROW>
        fprintf('%-24s %8.3f %8.3f %8.3f %8.3f   %.3f deg  %s\n', cases{k, 1}, err, ang, pf(pass));
    end

    testResults = struct();
    testResults.passed = logical(ok(:));
    fprintf('\nPlaceholder pass: < 2 km after 90 min. Set from the pointing budget.\n');
    if nargin >= 1 && show
        Simulink.BlockDiagram.arrangeSystem(mdl);   % blocks were added without positions
        open_system(mdl);   % leave the harness open (Constants show the last case)
    else
        close_system(mdl, 0);
    end
end

function dy = eomEGM(t, y, wE, degree)
    th = wE * t;
    R = [cos(th) sin(th) 0; -sin(th) cos(th) 0; 0 0 1];   % ECI -> ECEF
    rE = R * y(1:3) * 1000;                               % m, ECEF
    [gx, gy, gz] = gravitysphericalharmonic(rE.', 'EGM2008', degree, 'None');
    a = R.' * [gx; gy; gz] / 1000;                        % km/s^2, ECI
    dy = [y(4:6); a];
end

function s = pf(p), if p, s = 'PASS'; else, s = 'FAIL'; end, end
