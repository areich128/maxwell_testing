function testResults = sun_eci_Test(show)
% sun_eci_Test  Flight sun_eci (pos.c, moved to igrf_model.c on -sfunc) vs JPL ephemeris.
%
% Runs the compiled flight C through the sun_eci_sfun block in an unsaved
% harness and compares the spacecraft-to-sun unit vector with MATLAB
% planetEphemeris (DE405, ICRF/J2000), which is independent of the flight
% formula (Astronomical Almanac low-precision sun).
%
% Two comparisons per case:
%   vs MOD   : reference rotated from J2000 to mean equator/equinox of date
%              (IAU-1976 precession). The Almanac formula works in the
%              of-date frame, so this checks the formula itself. PASS < 0.02 deg
%              (formula accuracy about 0.01 deg, plus aberration about 0.006 deg
%              and nutation up to 0.005 deg, which the formula omits or includes
%              differently from the geometric ephemeris).
%   vs J2000 : reference in J2000 directly. Reported, not pass/fail. This is
%              the error the rest of the flight code sees, because ECEF2ECI
%              (IAU-2006) and IGRF work in GCRF (about J2000).
%
% "old bug" column: angle between the reference at the true time and at the
% time the old code used (integer division by 86400 dropped the part of the
% day, so the sun only moved at each J2000 day boundary, 12:00 UTC).
% The time-of-day sweep shows the fix: flight stays under 0.02 deg while the
% old code would have drifted up to about 1 deg over the day.
%
% Inputs to the block: eci_pos [3] km single, J2000_time uint32 (s since
% 2000-01-01 12:00:00), J2000_frac single. Outputs: sun_vec [3] single, status int32.
% Time scale: test times are UTC; the flight formula treats the time as TT,
% about 69 s off, which is about 0.0008 deg of sun motion (negligible).
%
% sun_eci_Test(true) keeps the harness open.

if nargin < 1, show = false; end

%% Find the block in the library
lib = 'sFunctionLibrary';
load_system(lib);
blk = find_system(lib, 'LookUnderMasks', 'all', 'FollowLinks', 'on', ...
    'BlockType', 'S-Function', 'FunctionName', 'sun_eci_sfun');
assert(~isempty(blk), 'sun_eci_sfun block not found in %s', lib);
src = blk{1};
while ~strcmp(get_param(src, 'Parent'), lib)
    src = get_param(src, 'Parent');
end

%% Check the reference is available
try
    planetEphemeris(2451545.0, 'Earth', 'Sun');
catch err
    error(['planetEphemeris failed (%s). Install the Aerospace Toolbox ' ...
        'ephemeris data (aeroDataPackage) and rerun.'], err.message);
end

%% Build the harness
mdl = 'harness_sun_eci';
if bdIsLoaded(mdl), close_system(mdl, 0); end
new_system(mdl);
if show, open_system(mdl); end
set_param(mdl, 'SolverType', 'Fixed-step', 'Solver', 'FixedStepDiscrete', ...
    'FixedStep', '1', 'StopTime', '0', 'ReturnWorkspaceOutputs', 'on');

add_block(src, [mdl '/sun'], 'Position', [300 100 420 200]);
add_block('simulink/Sources/Constant', [mdl '/eci_pos'], 'Value', '[7000;0;0]', ...
    'OutDataTypeStr', 'single', 'VectorParams1D', 'off', 'SampleTime', '1', ...
    'Position', [100 90 180 120]);
add_block('simulink/Sources/Constant', [mdl '/J2000_time'], 'Value', '0', ...
    'OutDataTypeStr', 'uint32', 'SampleTime', '1', 'Position', [100 140 180 170]);
add_block('simulink/Sources/Constant', [mdl '/J2000_frac'], 'Value', '0', ...
    'OutDataTypeStr', 'single', 'SampleTime', '1', 'Position', [100 190 180 220]);
add_block('simulink/Sinks/To Workspace', [mdl '/sun_out'], 'VariableName', 'sun_out', ...
    'SaveFormat', 'Array', 'Position', [500 110 580 140]);
add_block('simulink/Sinks/To Workspace', [mdl '/status_out'], 'VariableName', 'status_out', ...
    'SaveFormat', 'Array', 'Position', [500 170 580 200]);
add_line(mdl, 'eci_pos/1', 'sun/1');
add_line(mdl, 'J2000_time/1', 'sun/2');
add_line(mdl, 'J2000_frac/1', 'sun/3');
add_line(mdl, 'sun/1', 'sun_out/1');
add_line(mdl, 'sun/2', 'status_out/1');

%% Cases
epoch = datetime(2000, 1, 1, 12, 0, 0);
dates = [ ...
    datetime(2026, 11,  1,  3, 17, 0); ...
    datetime(2026, 12, 31, 18,  0, 0); ...
    datetime(2027,  1, 15, 21, 42, 0); ...
    datetime(2027,  3, 20, 12,  0, 0); ...
    datetime(2027,  6, 21,  8,  5, 0); ...
    datetime(2027,  9, 23, 16, 30, 0); ...
    datetime(2027, 12, 22, 23, 10, 0); ...
    datetime(2028,  3,  1,  5, 55, 0)];
labels = cellstr(datestr(dates, 'yyyy-mm-dd HH:MM'));

% time-of-day sweep across one J2000 day (boundary at 12:00 UTC)
sweep = datetime(2026, 11, 15, 12, 0, 0) + hours([0; 6; 12; 18; 23.98]);
for k = 1:numel(sweep)
    labels{end+1, 1} = ['day sweep ' datestr(sweep(k), 'mm-dd HH:MM')]; %#ok<AGROW>
end
times = [dates; sweep];

pos = [ 6878     0     0; ...
           0  6878     0; ...
           0     0  6878; ...
       -4000  3500  4200; ...
        3000 -5000 -3800; ...
        6000  2500 -2000];

n = numel(times);
passed = false(n, 1);
angMod = zeros(n, 1); angJ2k = zeros(n, 1); angOld = zeros(n, 1);
nrmErr = zeros(n, 1); stat = zeros(n, 1);

for k = 1:n
    utc = times(k);
    rsc = pos(mod(k-1, size(pos, 1)) + 1, :)';

    secs  = seconds(utc - epoch);
    s_int = floor(secs);
    s_frac = secs - s_int;

    set_param([mdl '/eci_pos'], 'Value', mat2str(rsc, 10));
    set_param([mdl '/J2000_time'], 'Value', sprintf('%d', s_int));
    set_param([mdl '/J2000_frac'], 'Value', sprintf('%.9g', s_frac));

    out = sim(mdl);
    u = lastSample(out, 'sun_out', 3);
    stat(k) = lastSample(out, 'status_out', 1);

    % reference, J2000 and mean of date
    [rJ2k, jdtt] = sunRef(utc);
    rMod = precIAU76(jdtt) * rJ2k;
    uJ2k = unit(rJ2k - rsc);
    uMod = unit(rMod - rsc);

    % what the old code used: whole J2000 days only, plus the sub-second part
    utcOld = epoch + seconds(floor(s_int / 86400) * 86400 + s_frac);
    [rOld, jdOld] = sunRef(utcOld);
    uOld = unit(precIAU76(jdOld) * rOld - rsc);

    angMod(k) = angDeg(u, uMod);
    angJ2k(k) = angDeg(u, uJ2k);
    angOld(k) = angDeg(uOld, uMod);
    nrmErr(k) = abs(norm(u) - 1);

    passed(k) = angMod(k) < 0.02 && nrmErr(k) < 1e-5 && stat(k) == 0;
end

%% Report
fprintf('\n%-26s %9s %9s %9s %9s %7s  %s\n', 'case', 'vs MOD', 'vs J2000', 'old bug', '|u|-1', 'status', 'result');
for k = 1:n
    if passed(k), r = 'PASS'; else, r = 'FAIL'; end
    fprintf('%-26s %9.4f %9.4f %9.4f %9.1e %7d  %s\n', labels{k}, angMod(k), angJ2k(k), angOld(k), nrmErr(k), stat(k), r);
end
fprintf('\nAngles in deg. vs MOD: flight vs ephemeris in the of-date frame (pass < 0.02 deg).\n');
fprintf('vs J2000: flight vs ephemeris in J2000 (what GCRF users of the sun vector see), mean %.3f deg.\n', mean(angJ2k));
fprintf('old bug: error the integer-day code would have had at that time (reference vs reference).\n');
fprintf('%d passed, %d failed\n\n', sum(passed), sum(~passed));

testResults.passed = passed;

if ~show, close_system(mdl, 0); end
end

%% Helpers
function v = lastSample(out, name, m)
% Last sample of a logged signal of width m. To Workspace "Array" gives
% m x 1 x N for an [m,1] signal, but drops the trailing 1 when N = 1 (m x 1),
% or N x m for a 1-D signal.
x = double(out.get(name));
if ndims(x) == 3
    x = reshape(x, [], size(x, 3));
    v = x(:, end);
elseif m == 1
    v = x(end);
elseif size(x, 1) == m && size(x, 2) ~= m
    v = x(:, end);
else
    v = x(end, :).';
end
end

function [r, jdtt] = sunRef(utc)
% Earth to Sun, km, ICRF (J2000). TT = UTC + 37 s + 32.184 s (TDB ~ TT here).
jdtt = juliandate(utc) + 69.184 / 86400;
r = planetEphemeris(jdtt, 'Earth', 'Sun').';
end

function P = precIAU76(jdtt)
% J2000 to mean of date, IAU-1976 precession (Vallado): r_mod = P * r_j2000.
T = (jdtt - 2451545.0) / 36525;
as2r = pi / 180 / 3600;
zeta = (2306.2181*T + 0.30188*T^2 + 0.017998*T^3) * as2r;
z    = (2306.2181*T + 1.09468*T^2 + 0.018203*T^3) * as2r;
th   = (2004.3109*T - 0.42665*T^2 - 0.041833*T^3) * as2r;
P = rot3(-z) * rot2(th) * rot3(-zeta);
end

function R = rot3(a)
R = [cos(a) sin(a) 0; -sin(a) cos(a) 0; 0 0 1];
end

function R = rot2(a)
R = [cos(a) 0 -sin(a); 0 1 0; sin(a) 0 cos(a)];
end

function u = unit(v)
u = v / norm(v);
end

function d = angDeg(a, b)
d = atan2d(norm(cross(a, b)), dot(a, b));
end