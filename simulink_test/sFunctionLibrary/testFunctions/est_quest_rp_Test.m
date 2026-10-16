function testResults = est_quest_rp_Test(show)
% Test 3.4: est_quest_rp (flight C). Column 1 = magnetometer, column 2 = sun.
% est_quest_rp_Test(true) leaves the harness model open after the run.
% Expected: A(q) * eci_k(:,i) = b_k(:,i), with A(q) the [BN] matrix from test 3.1.

    libDir = fileparts(fileparts(mfilename('fullpath')));
    addpath(libDir);
    addpath(fullfile(libDir, 'mexFiles'));
    load_system('sFunctionLibrary');

    %% Harness (not saved)
    mdl = 'harness_est_quest_rp';
    if bdIsLoaded(mdl), close_system(mdl, 0); end
    new_system(mdl);
    set_param(mdl, 'SolverType', 'Fixed-step', 'Solver', 'ode1', 'FixedStep', '1', ...
        'StopTime', '0', 'ReturnWorkspaceOutputs', 'on', 'ParameterPrecisionLossMsg', 'none');
    C = @(name, val) add_block('simulink/Sources/Constant', [mdl '/' name], 'Value', val, ...
        'OutDataTypeStr', 'single', 'VectorParams1D', 'off', 'SampleTime', '1');
    C('b_k',     'eye(3,2)');
    C('eci_k',   'eye(3,2)');
    C('weights', '[0.5;0.5]');
    add_block('sFunctionLibrary/est_quest_rp/est_quest_rp', [mdl '/est_quest_rp']);
    add_block('simulink/Sinks/To Workspace', [mdl '/q_out'],   'VariableName', 'q_out',   'SaveFormat', 'Timeseries');
    add_block('simulink/Sinks/To Workspace', [mdl '/dcm_out'], 'VariableName', 'dcm_out', 'SaveFormat', 'Timeseries');
    add_block('simulink/Sinks/To Workspace', [mdl '/st_out'],  'VariableName', 'st_out',  'SaveFormat', 'Timeseries');
    add_line(mdl, 'b_k/1',     'est_quest_rp/1');
    add_line(mdl, 'eci_k/1',   'est_quest_rp/2');
    add_line(mdl, 'weights/1', 'est_quest_rp/3');
    add_line(mdl, 'est_quest_rp/1', 'q_out/1');
    add_line(mdl, 'est_quest_rp/2', 'dcm_out/1');
    add_line(mdl, 'est_quest_rp/3', 'st_out/1');

    run1 = @(B, R, w) runQuest(mdl, B, R, w);
    rng(5);
    names = {}; ok = [];
    fprintf('%-40s %12s %10s %8s  %s\n', 'case', 'att err deg', 'dcm err', 'status', 'result');

    %% 1. Noise-free random attitudes
    for k = 1:10
        qt = randn(4,1); qt = qt / norm(qt);
        R = [unit(randn(3,1)), unit(randn(3,1))];
        B = refA(qt) * R;
        [q, D, st] = run1(B, R, [0.5; 0.5]);
        e = attErr(q, qt); eD = norm(D - refA(q));
        pass = e < 0.01 && eD < 1e-5 && st == 0;
        report(sprintf('noise-free %d', k), e, eD, st, pass);
    end

    %% 2. Column order: perturb only the sun vector (column 2) by 5 deg.
    % The trusted column should end up with near-zero residual.
    qt = unit([0.8; 0.2; -0.4; 0.3]);
    R = [unit([0.2; 0.9; -0.3]), unit([0.7; -0.1; 0.7])];
    B = refA(qt) * R;
    B(:,2) = tilt(B(:,2), 5);
    [qm, ~, ~] = run1(B, R, [0.99; 0.01]);
    [qs, ~, ~] = run1(B, R, [0.01; 0.99]);
    rMag = vecAng(refA(qm) * R(:,1), B(:,1));   % residual on mag when trusting mag
    rSun = vecAng(refA(qs) * R(:,2), B(:,2));   % residual on sun when trusting sun
    report('trust mag: mag residual', rMag, NaN, 0, rMag < 0.2);
    report('trust sun: sun residual', rSun, NaN, 0, rSun < 0.2);
    %% 3. Realistic weights and noise (compute_weights with 6 CSS lit)
    ws = 6 / 0.0349^2; wm = 1 / 0.0087^2; w = [wm; ws] / (wm + ws);
    errs = zeros(20,1);
    for k = 1:20
        qt = unit(randn(4,1));
        R = [unit(randn(3,1)), unit(randn(3,1))];
        while vecAng(R(:,1), R(:,2)) < 30, R(:,2) = unit(randn(3,1)); end
        B = refA(qt) * R;
        B(:,1) = tilt(B(:,1), 0.5 * randn);
        B(:,2) = tilt(B(:,2), 2.0 * randn);
        [q, ~, ~] = run1(B, R, w);
        errs(k) = attErr(q, qt);
    end
    report(sprintf('noisy, weights [%.2f %.2f], 95th pct', w), prctile(errs, 95), NaN, 0, prctile(errs, 95) < 4);
    %% 4. Zero vector
    R = [unit([1;2;3]), unit([-2;1;0])];
    [~, ~, st] = run1([zeros(3,1), R(:,2)], R, [0.5;0.5]);
    report('zero mag vector', NaN, NaN, st, st == -1);

    %% 5. Near 180 deg (Shuster sequential rotations)
    R = [unit([0.3; 0.5; 0.8]), unit([0.9; -0.2; 0.1])];
    near180 = {{[0;0;1],179}, {[0;0;1],179.9}, {[0;0;1],179.99}, {[0;0;1],179.9999}, ...
               {[0;0;1],180}, {[1;0;0],180}, {[0;1;0],180}, {[0.3;-0.8;0.5],180}, {[0.3;-0.8;0.5],179.9999}};
    for c = near180
        ax = unit(c{1}{1}); deg = c{1}{2};
        qt = [cosd(deg/2); sind(deg/2) * ax];
        B = refA(qt) * R;
        [q, D, st] = run1(B, R, [0.5; 0.5]);
        e = attErr(q, qt); eD = norm(D - refA(q));
        report(sprintf('%.4f deg about [%.2f %.2f %.2f]', deg, ax), e, eD, st, e < 0.01 && eD < 1e-5 && st == 0);
    end
    for k = 1:10
        qt = [0; unit(randn(3,1))];
        Rr = [unit(randn(3,1)), unit(randn(3,1))];
        while vecAng(Rr(:,1), Rr(:,2)) < 30, Rr(:,2) = unit(randn(3,1)); end
        [q, D, st] = run1(refA(qt) * Rr, Rr, w);
        e = attErr(q, qt); eD = norm(D - refA(q));
        report(sprintf('180 deg random axis %d', k), e, eD, st, e < 0.01 && eD < 1e-5 && st == 0);
    end

    testResults = struct();
    testResults.passed = logical(ok(:));
    fprintf('\natt err: angle between estimated and true attitude (deg); status 0 = success, -1 = failure\n');
    fprintf('%d passed, %d failed\n', sum(ok), sum(~ok));
    if nargin >= 1 && show
        Simulink.BlockDiagram.arrangeSystem(mdl);   % blocks were added without positions
        open_system(mdl);   % leave the harness open (Constants show the last case)
    else
        close_system(mdl, 0);
    end

    function report(name, e, eD, st, pass)
        names{end+1} = name; ok(end+1) = pass; %#ok<SETNU>
        if pass, r = 'PASS'; else, r = 'FAIL'; end
        fprintf('%-40s %12.4f %10.1e %8d  %s\n', name, e, eD, st, r);
    end
end

%% Helpers
function [q, D, st] = runQuest(mdl, B, R, w)
    set_param([mdl '/b_k'],     'Value', mat2str(double(single(B)), 17));
    set_param([mdl '/eci_k'],   'Value', mat2str(double(single(R)), 17));
    set_param([mdl '/weights'], 'Value', mat2str(double(single(w)), 17));
    out = sim(mdl);
    q  = double(lastSample(out.q_out,   [4 1]));
    D  = double(lastSample(out.dcm_out, [3 3]));
    st = double(lastSample(out.st_out,  [1 1]));
end

function e = attErr(q, qt)
    e = 2 * acosd(min(1, abs(dot(q / norm(q), qt / norm(qt)))));
end

function v = unit(v), v = v / norm(v); end

function v2 = tilt(v, deg)
    p = cross(v, randn(3,1)); p = p / norm(p);
    v2 = cosd(deg) * v + sind(deg) * norm(v) * p;
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
function t = vecAng(a, b)
    t = atan2d(norm(cross(a, b)), dot(a, b));
end