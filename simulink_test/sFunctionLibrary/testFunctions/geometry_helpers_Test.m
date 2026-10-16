function testResults = geometry_helpers_Test(show)
% sc_to_gs_vec, gs_to_sc_vec, sc_to_gs_unitvec, gs_angle_distance, calc_q_err,
% geometry_helpers_Test(true) leaves the harness model open after the run.
% calculateOmega (flight C) against exact vector geometry.

    libDir = fileparts(fileparts(mfilename('fullpath')));
    addpath(libDir); addpath(fullfile(libDir, 'mexFiles'));
    load_system('sFunctionLibrary');

    mdl = 'harness_geometry';
    if bdIsLoaded(mdl), close_system(mdl, 0); end
    new_system(mdl);
    set_param(mdl, 'SolverType', 'Fixed-step', 'Solver', 'ode1', 'FixedStep', '1', ...
        'StopTime', '0', 'ReturnWorkspaceOutputs', 'on', 'ParameterPrecisionLossMsg', 'none');
    C = @(name, val) add_block('simulink/Sources/Constant', [mdl '/' name], 'Value', val, ...
        'OutDataTypeStr', 'single', 'VectorParams1D', 'off', 'SampleTime', '1');
    C('sc', '[7e6;0;0]'); C('gnd', '[6.371e6;0;0]');
    C('q1', '[1;0;0;0]'); C('q2', '[1;0;0;0]');
    C('prev', '[1;0;0]'); C('cur', '[0;1;0]'); C('qBN', '[1;0;0;0]');
    blk = {'gsad','S-Function Builder'; 'gs2sc','S-Function Builder6'; 'sc2gs','S-Function Builder7';
           'sc2gsu','S-Function Builder12'; 'qerr','S-Function Builder4'; 'omega','S-Function Builder10'};
    for i = 1:size(blk,1), add_block(['sFunctionLibrary/' blk{i,2}], [mdl '/' blk{i,1}]); end
    for b = {'gsad','gs2sc','sc2gs','sc2gsu'}
        add_line(mdl, 'sc/1', [b{1} '/1']); add_line(mdl, 'gnd/1', [b{1} '/2']);
    end
    add_line(mdl, 'q1/1', 'qerr/1'); add_line(mdl, 'q2/1', 'qerr/2');
    add_line(mdl, 'prev/1', 'omega/1'); add_line(mdl, 'cur/1', 'omega/2'); add_line(mdl, 'qBN/1', 'omega/3');
    W = {'dist','gsad',1; 'ang','gsad',2; 'gs2sc_o','gs2sc',1; 'sc2gs_o','sc2gs',1;
         'unit_o','sc2gsu',1; 'qerr_o','qerr',1; 'om_o','omega',1};
    for i = 1:size(W,1)
        add_block('simulink/Sinks/To Workspace', [mdl '/' W{i,1}], 'VariableName', W{i,1}, 'SaveFormat', 'Timeseries');
        add_line(mdl, sprintf('%s/%d', W{i,2}, W{i,3}), [W{i,1} '/1']);
    end

    rng(10); names = {}; ok = []; conv = {};
    fprintf('%-26s %9s %9s %9s %9s %9s %9s %9s  %s\n', 'case', 'sc2gs', 'gs2sc', 'unit', 'dist', 'ang deg', 'q_err', 'omega', 'result');

    for k = 1:12
        sc  = 6.9e6 * unit(randn(3,1));
        gnd = 6.371e6 * unit(sc + 3e6 * randn(3,1));
        q1 = unit(randn(4,1)); q2 = unit(randn(4,1)); qb = unit(randn(4,1));
        p = unit(randn(3,1)); c = rotAbout(p, unit(randn(3,1)), 0.05 + 2*rand);   % 0.05 to 2 deg per step
        o = runAll(sc, gnd, q1, q2, p, c, qb);
        [sc, gnd, q1, q2, p, c, qb] = sgl(sc, gnd, q1, q2, p, c, qb);
        d = gnd - sc;
        e1 = norm(o.sc2gs - d) / norm(d);
        e2 = norm(o.gs2sc + d) / norm(d);
        e3 = norm(o.unit - d / norm(d));
        e4 = abs(o.dist - norm(d)) / norm(d);
        e5 = abs(o.ang - acosd(dot(gnd, -d) / (norm(gnd) * norm(d))));
        m12 = norm(refA(o.qerr) - refA(q1) * refA(q2)); m21 = norm(refA(o.qerr) - refA(q2) * refA(q1));
        if m12 < m21, conv{end+1} = 'A(q1)A(q2)'; else, conv{end+1} = 'A(q2)A(q1)'; end %#ok<AGROW>
        e6 = min(m12, m21);
        omExp = refA(qb) * (unit(cross(p, c)) * atan2(norm(cross(p, c)), dot(p, c)) / 0.10);
        e7 = norm(o.om - omExp) / norm(omExp);
        pass = max([e1 e2 e3 e4]) < 1e-5 && e5 < 1e-3 && e6 < 1e-5 && e7 < 1e-3;
        rep(sprintf('random %d', k), [e1 e2 e3 e4 e5 e6 e7], pass);
    end
    fprintf('calc_q_err matches %s in all cases: %d\n', conv{1}, all(strcmp(conv, conv{1})));

    % Edge cases
    sc = 6.9e6 * unit([1;2;3]);
    o = runAll(sc, sc, [1;0;0;0], [1;0;0;0], [1;0;0], [1;0;0], [1;0;0;0]);
    edge('sc == gnd: angle 180, dist 0', o.ang == 180 && o.dist == 0, sprintf('angle %g, dist %g, unit [%g %g %g]', o.ang, o.dist, o.unit));
    edge('omega, prev == cur -> 0', norm(o.om) == 0, sprintf('|omega| %g', norm(o.om)));
    p = unit([0.3;-0.5;0.8]);
    o = runAll(sc, 6.371e6*unit([1;0;0]), [1;0;0;0], [1;0;0;0], p, rotAbout(p, unit(randn(3,1)), rad2deg(1e-7)), unit(randn(4,1)));
    edge('omega, 1e-7 rad step (< EPS) -> 0', norm(o.om) == 0, sprintf('|omega| %g', norm(o.om)));
    o = runAll(sc, 6.371e6*unit([1;0;0]), [1;0;0;0], [1;0;0;0], p, -p, [1;0;0;0]);
    edge('omega, prev = -cur (180 deg)', true, sprintf('[info] omega = [%g %g %g]', o.om));

    testResults.passed = logical(ok(:));
    fprintf('\nerrors relative except ang (deg) and q_err (matrix norm)\n%d passed, %d failed\n', sum(ok), sum(~ok));
    if nargin >= 1 && show
        Simulink.BlockDiagram.arrangeSystem(mdl);   % blocks were added without positions
        open_system(mdl);   % leave the harness open (Constants show the last case)
    else
        close_system(mdl, 0);
    end

    function o = runAll(sc, gnd, q1, q2, p, c, qb)
        vals = {'sc',sc; 'gnd',gnd; 'q1',q1; 'q2',q2; 'prev',p; 'cur',c; 'qBN',qb};
        for j = 1:size(vals,1), set_param([mdl '/' vals{j,1}], 'Value', mat2str(double(single(vals{j,2})), 17)); end
        out = sim(mdl);
        g = @(n, s) double(reshape(out.(n).Data(:, :, end), s, 1));
        o.dist = lastScalar(out.dist); o.ang = lastScalar(out.ang);
        o.gs2sc = g('gs2sc_o', 3); o.sc2gs = g('sc2gs_o', 3); o.unit = g('unit_o', 3);
        o.qerr = g('qerr_o', 4); o.om = g('om_o', 3);
    end
    function rep(name, e, pass)
        names{end+1} = name; ok(end+1) = pass;
        fprintf('%-26s %9.1e %9.1e %9.1e %9.1e %9.1e %9.1e %9.1e  %s\n', name, e, ternary(pass, 'PASS', 'FAIL'));
    end
    function edge(name, pass, info)
        names{end+1} = name; ok(end+1) = pass;
        fprintf('%-38s %-40s %s\n', name, info, ternary(pass, 'PASS', 'FAIL'));
    end
end

function v = lastScalar(ts), d = ts.Data; v = double(d(end)); end
function varargout = sgl(varargin), for i = 1:nargin, varargout{i} = double(single(varargin{i})); end, end
function v2 = rotAbout(v, ax, deg)
    ax = unit(ax - dot(ax, v) * v);                     % rotate v toward ax by deg
    v2 = cosd(deg) * v + sind(deg) * ax;
end
function A = refA(q)
    q0 = q(1); v = q(2:4); v = v(:);
    A = (q0^2 - v'*v) * eye(3) + 2*(v*v') - 2*q0*[0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];
end
function v = unit(v), v = v / norm(v); end
function s = ternary(c, a, b), if c, s = a; else, s = b; end, end