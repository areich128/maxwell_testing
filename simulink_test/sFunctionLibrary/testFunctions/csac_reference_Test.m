function testResults = csac_reference_Test()
% create_phicone_des_q and create_des_rates_CSAC (flight C).
% phi = cos(phi_cone)*pos_hat + sin(phi_cone)*(sun orthogonalised to pos); CSAC points -X along phi.

    libDir = fileparts(fileparts(mfilename('fullpath')));
    addpath(libDir); addpath(fullfile(libDir, 'mexFiles'));
    load_system('sFunctionLibrary');

    mdl = 'harness_csac';
    if bdIsLoaded(mdl), close_system(mdl, 0); end
    new_system(mdl);
    set_param(mdl, 'SolverType', 'Fixed-step', 'Solver', 'ode1', 'FixedStep', '1', ...
        'StopTime', '0', 'ReturnWorkspaceOutputs', 'on', 'ParameterPrecisionLossMsg', 'none');
    C = @(name, val, dt) add_block('simulink/Sources/Constant', [mdl '/' name], 'Value', val, ...
        'OutDataTypeStr', dt, 'VectorParams1D', 'off', 'SampleTime', '1');
    C('op', '3', 'uint8'); C('pos', '[7e6;0;0]', 'single'); C('sun', '[0;1;0]', 'single');
    C('phi_ang', '45', 'single'); C('b_eci', '[0;0;3e4]', 'single');
    C('qBN', '[1;0;0;0]', 'single'); C('new_mode', '0', 'uint8');
    C('phi0', '[1;0;0]', 'double'); C('dphi', '[0;0;0]', 'double');
    add_block('simulink/Sources/Clock', [mdl '/clk']);
    add_block('simulink/Math Operations/Product', [mdl '/ramp']);
    add_block('simulink/Math Operations/Sum', [mdl '/psum'], 'Inputs', '++');
    add_block('simulink/Signal Attributes/Data Type Conversion', [mdl '/psingle'], 'OutDataTypeStr', 'single');
    add_line(mdl, 'clk/1', 'ramp/1'); add_line(mdl, 'dphi/1', 'ramp/2');
    add_line(mdl, 'phi0/1', 'psum/1'); add_line(mdl, 'ramp/1', 'psum/2'); add_line(mdl, 'psum/1', 'psingle/1');
    add_block('sFunctionLibrary/S-Function Builder5', [mdl '/phicone']);
    add_block('sFunctionLibrary/S-Function Builder3', [mdl '/rates']);
    in1 = {'op','pos','sun','phi_ang','b_eci'};
    for i = 1:5, add_line(mdl, [in1{i} '/1'], sprintf('phicone/%d', i)); end
    add_line(mdl, 'op/1', 'rates/1'); add_line(mdl, 'qBN/1', 'rates/2');
    add_line(mdl, 'psingle/1', 'rates/3'); add_line(mdl, 'new_mode/1', 'rates/4');
    W = {'phi_o','phicone',1; 'q_o','phicone',2; 'dcm_o','phicone',3; 'w_o','rates',1};
    for i = 1:4
        add_block('simulink/Sinks/To Workspace', [mdl '/' W{i,1}], 'VariableName', W{i,1}, 'SaveFormat', 'Timeseries');
        add_line(mdl, sprintf('%s/%d', W{i,2}, W{i,3}), [W{i,1} '/1']);
    end

    rng(11); names = {}; ok = [];
    fprintf('%-30s %9s %9s %9s %6s %9s %9s %9s  %s\n', 'case', '|phi|-1', 'cone err', 'coplanar', 'side', '-X err', 'orthon', 'q v dcm', 'result');

    %% create_phicone_des_q
    for phiDeg = [45 45 45 10 80 0 90]
        pos = 6.9e6 * unit(randn(3,1)); sun = unit(randn(3,1));
        while abs(dot(unit(pos), sun)) > 0.95, sun = unit(randn(3,1)); end
        o = runPhi(pos, sun, phiDeg, 3e4 * unit(randn(3,1)));
        [pos, sun] = sgl(pos, sun);
        ph = o.phi; ph_hat = unit(ph);
        sOrth = unit(sun - dot(sun, unit(pos)) * unit(pos));
        e1 = abs(norm(ph) - 1);
        e2 = abs(acosd(min(1, dot(ph_hat, unit(pos)))) - phiDeg);
        e3 = abs(dot(ph_hat, unit(cross(pos, sun))));
        side = phiDeg == 0 || dot(ph_hat, sOrth) > 0;
        e4 = atan2d(norm(cross(o.dcm(:,1), -ph_hat)), dot(o.dcm(:,1), -ph_hat));
        e5 = max(norm(o.dcm' * o.dcm - eye(3)), abs(det(o.dcm) - 1));
        e6 = norm(refA(o.q) - o.dcm');
        pass = e1 < 1e-5 && e2 < 1e-3 && e3 < 1e-5 && side && e4 < 1e-3 && e5 < 1e-5 && e6 < 1e-5;
        rep(sprintf('phi_cone %g deg', phiDeg), [e1 e2 e3], side, [e4 e5 e6], pass);
    end
    pos = 6.9e6 * unit([1;2;-1]);
    o = runPhi(pos, unit(pos), 45, [0;0;3e4]);
    e2 = abs(acosd(dot(unit(o.phi), unit(pos))) - 45);
    edgeRow('sun parallel to pos (fallback)', e2 < 1e-3, sprintf('cone err %.2e deg, phi [%.3f %.3f %.3f]', e2, o.phi));
    o = runPhi([0;0;0], [0;1;0], 45, [0;0;3e4]);
    edgeRow('zero pos vector', true, sprintf('[info] phi [%g %g %g], q [%.3f %.3f %.3f %.3f]', o.phi, o.q));

    %% create_des_rates_CSAC (static state): step 1 zero, step 2 = A(q)*axis*angle/0.1
    p0 = unit(randn(3,1)); dp = 0.01 * unit(cross(p0, randn(3,1)));
    qb = unit(randn(4,1)); clear mex;
    W2 = runRates(p0, dp, qb, 0, 2);
    pA = double(single(p0)); pB = double(single(p0 + dp)); qbs = double(single(qb));
    wExp = refA(qbs) * (unit(cross(pA, pB)) * atan2(norm(cross(pA, pB)), dot(pA, pB)) / 0.10);
    e = norm(W2(:,2) - wExp) / norm(wExp);
    edgeRow('CSAC rates: step 1 = 0', norm(W2(:,1)) == 0, sprintf('|w1| %g', norm(W2(:,1))));
    edgeRow('CSAC rates: step 2 = A(q) w_N', e < 1e-3, sprintf('rel err %.2e, |w| %.4f rad/s', e, norm(wExp)));
    clear mex;
    W3 = runRates(p0, dp, qb, 1, 3);
    edgeRow('CSAC rates: new_mode held 1', true, sprintf('[info] |w| per step [%g %g %g]', vecnorm(W3)));

    testResults.passed = logical(ok(:));
    fprintf('\n%d passed, %d failed\n', sum(ok), sum(~ok));
    close_system(mdl, 0);

    function o = runPhi(pos, sun, phiDeg, b)
        set_param([mdl '/pos'], 'Value', mat2str(double(single(pos)), 17));
        set_param([mdl '/sun'], 'Value', mat2str(double(single(sun)), 17));
        set_param([mdl '/phi_ang'], 'Value', num2str(phiDeg));
        set_param([mdl '/b_eci'], 'Value', mat2str(double(single(b)), 17));
        set_param(mdl, 'StopTime', '0');
        out = sim(mdl);
        o.phi = double(reshape(out.phi_o.Data(:,:,end), 3, 1));
        o.q   = double(reshape(out.q_o.Data(:,:,end), 4, 1));
        o.dcm = double(reshape(out.dcm_o.Data(:,:,end), 3, 3));
    end
    function W = runRates(p0, dp, qb, nm, n)
        set_param([mdl '/phi0'], 'Value', mat2str(p0, 17));
        set_param([mdl '/dphi'], 'Value', mat2str(dp, 17));
        set_param([mdl '/qBN'],  'Value', mat2str(double(single(qb)), 17));
        set_param([mdl '/new_mode'], 'Value', num2str(nm));
        set_param(mdl, 'StopTime', num2str(n - 1));
        out = sim(mdl);
        W = double(reshape(out.w_o.Data, 3, []));
    end
    function rep(name, e, side, e2, pass)
        names{end+1} = name; ok(end+1) = pass;
        fprintf('%-30s %9.1e %9.1e %9.1e %6d %9.1e %9.1e %9.1e  %s\n', name, e, side, e2, ternary(pass, 'PASS', 'FAIL'));
    end
    function edgeRow(name, pass, info)
        names{end+1} = name; ok(end+1) = pass;
        fprintf('%-32s %-60s %s\n', name, info, ternary(pass, 'PASS', 'FAIL'));
    end
end

function varargout = sgl(varargin), for i = 1:nargin, varargout{i} = double(single(varargin{i})); end, end
function A = refA(q)
    q0 = q(1); v = q(2:4); v = v(:);
    A = (q0^2 - v'*v) * eye(3) + 2*(v*v') - 2*q0*[0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];
end
function v = unit(v), v = v / norm(v); end
function s = ternary(c, a, b), if c, s = a; else, s = b; end, end