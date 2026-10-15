function testResults = igrf_eci_Test()
% Task 4: flight igrf_eci (igrf_model.c) vs MATLAB igrfmagm (IGRF-14).
% Input: ECI position (km), J2000 time (s since 2000-01-01 12:00, treated as UTC here).
% Reference: ECI -> ECEF with dcmeci2ecef('IAU-2000/2006'), geodetic lat/lon/height,
% igrfmagm NED field, NED -> ECEF -> ECI.
% Reference limits: the flight time scale offset (leapsec handling, about 27 s)
% and no EOP in the MATLAB DCM limit agreement to about 0.1 deg, so the
% pass criteria are 0.25 deg in direction and 100 nT in magnitude.

    libDir = fileparts(fileparts(mfilename('fullpath')));
    addpath(libDir);
    addpath(fullfile(libDir, 'mexFiles'));
    load_system('sFunctionLibrary');
    blk = find_system('sFunctionLibrary', 'LookUnderMasks', 'all', 'FollowLinks', 'on', ...
        'BlockType', 'S-Function', 'FunctionName', 'igrf_eci_sfun');
    if numel(blk) ~= 1
        error('Expected one igrf_eci_sfun block in sFunctionLibrary, found %d', numel(blk));
    end

    %% Harness (not saved)
    mdl = 'harness_igrf_eci';
    if bdIsLoaded(mdl), close_system(mdl, 0); end
    new_system(mdl);
    set_param(mdl, 'SolverType', 'Fixed-step', 'Solver', 'ode1', 'FixedStep', '1', ...
        'StopTime', '0', 'ReturnWorkspaceOutputs', 'on', 'ParameterPrecisionLossMsg', 'none');
    C = @(name, val, dt) add_block('simulink/Sources/Constant', [mdl '/' name], 'Value', val, ...
        'OutDataTypeStr', dt, 'VectorParams1D', 'off', 'SampleTime', '1');
    C('eci_pos',    '[7000;0;0]', 'single');
    C('eci_vel',    '[0;7.5;0]',  'single');
    C('J2000_time', '0',          'uint32');
    C('J2000_frac', '0',          'single');
    C('leapsec',    '5',          'uint8');
    add_block(blk{1}, [mdl '/igrf']);
    add_block('simulink/Sinks/To Workspace', [mdl '/b_out'],  'VariableName', 'b_out',  'SaveFormat', 'Timeseries');
    add_block('simulink/Sinks/To Workspace', [mdl '/st_out'], 'VariableName', 'st_out', 'SaveFormat', 'Timeseries');
    ins = {'eci_pos', 'eci_vel', 'J2000_time', 'J2000_frac', 'leapsec'};
    for k = 1:5, add_line(mdl, [ins{k} '/1'], sprintf('igrf/%d', k)); end
    add_line(mdl, 'igrf/1', 'b_out/1');
    add_line(mdl, 'igrf/2', 'st_out/1');

    %% Cases: 500 km altitude, spread in ECI latitude/longitude, three dates
    Re = 6378.1363; rmag = Re + 500;
    dirs = [0 0; 45 30; -60 120; 80 200; -85 300; 10 250; 30 160; -20 45];   % [lat lon] deg in ECI
    dates = [2026 11 1 6 0 0; 2027 6 15 18 30 0; 2028 3 1 0 0 0];
    j2000 = datetime(2000, 1, 1, 12, 0, 0);

    names = {}; ok = [];
    fprintf('%-30s %9s %9s %8s %8s %8s %8s  %s\n', 'case', '|B| fsw', '|B| ref', 'dF nT', 'ang deg', 'dAz deg', 'dEl deg', 'result');
    for d = 1:size(dates, 1)
        utc = datetime(dates(d, :));
        tsec = seconds(utc - j2000);
        tInt = floor(tsec); tFrac = tsec - tInt;
        dcm = dcmeci2ecef('IAU-2000/2006', dates(d, :));
        for k = 1:size(dirs, 1)
            la = dirs(k, 1); lo = dirs(k, 2);
            r = rmag * [cosd(la)*cosd(lo); cosd(la)*sind(lo); sind(la)];
            v = 7.6 * unit(cross([0;0;1], r) + 0.1*[0;0;1]);
            [bF, st] = runIgrf(mdl, r, v, tInt, tFrac);
            bR = refField(r, dcm, utc);
            ang = vecAng(bF, bR);
            dF  = norm(bF) - norm(bR);
            [azF, elF] = azel(bF); [azR, elR] = azel(bR);
            dAz = wrapTo180(azF - azR); dEl = elF - elR;
            pass = ang < 0.25 && abs(dF) < 100 && st == 0;
            nm = sprintf('%s lat %+3d lon %3d', datestr(utc, 'yyyy-mm-dd'), la, lo);
            names{end+1} = nm; ok(end+1) = pass; %#ok<AGROW>
            fprintf('%-30s %9.0f %9.0f %8.0f %8.3f %8.2f %8.2f  %s\n', nm, norm(bF), norm(bR), dF, ang, dAz, dEl, pf(pass));
        end
    end

    testResults = struct();
    testResults.passed = logical(ok(:));
    fprintf('\ndAz/dEl: azimuth/elevation of the field direction in ECI, flight minus reference.\n');
    fprintf('A constant dAz with small dEl points to a rotation about the ECI z axis (GMST).\n');
    fprintf('%d passed, %d failed\n', sum(ok), sum(~ok));
    close_system(mdl, 0);
end

%% Helpers
function [b, st] = runIgrf(mdl, r, v, tInt, tFrac)
    set_param([mdl '/eci_pos'],    'Value', mat2str(double(single(r)), 17));
    set_param([mdl '/eci_vel'],    'Value', mat2str(double(single(v)), 17));
    set_param([mdl '/J2000_time'], 'Value', sprintf('%d', tInt));
    set_param([mdl '/J2000_frac'], 'Value', sprintf('%.9g', tFrac));
    out = sim(mdl);
    b  = double(lastSample(out.b_out,  [3 1]));
    st = double(lastSample(out.st_out, [1 1]));
end

function bECI = refField(rECI_km, dcm, utc)
    rECEF = dcm * (rECI_km * 1000);
    lla = ecef2lla(rECEF.');
    xyz = igrfmagm(lla(3), lla(1), lla(2), decyear(utc));   % IGRF-14, NED, nT
    ph = lla(1); la = lla(2);
    Rned2ecef = [-sind(ph)*cosd(la), -sind(la), -cosd(ph)*cosd(la);
                 -sind(ph)*sind(la),  cosd(la), -cosd(ph)*sind(la);
                  cosd(ph),           0,        -sind(ph)];
    bECI = dcm.' * (Rned2ecef * xyz(:));
end

function [az, el] = azel(b)
    az = atan2d(b(2), b(1)); el = asind(b(3) / norm(b));
end

function v = unit(v), v = v / norm(v); end

function t = vecAng(a, b), t = atan2d(norm(cross(a, b)), dot(a, b)); end

function s = pf(p), if p, s = 'PASS'; else, s = 'FAIL'; end, end

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