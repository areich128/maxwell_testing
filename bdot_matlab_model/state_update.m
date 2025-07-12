% Computes the time derivative of the satellite state vector
function dstatedt = state_update(t, state)

global mag_bf BI invI I m nextMagUpdate lastMagUpdate pqr 
global detumble_gain out_u mag_bf_rate mag_bf_prev LMN_magtorquers

%% Extract position, velocity, quaternion, and angular velocity
x = state(1); y = state(2); z = state(3);
q0123 = state(7:10);
p = state(11); q = state(12); r = state(13); %these are our angular velocities
pqr = [p; q; r]; 
vel = state(4:6);

%% Quaternion derivative (kinematics) (I just looked this up I'm not good with quaternions)
PQRMAT = [0 -p -q -r; p 0 r -q; q -r 0 p; r q -p 0];
q0123dot = 0.5 * PQRMAT * q0123;

%% Gravity; gives us our Fgrav for orbital motion
earth_params
rvec = [x; y; z];
rho = norm(rvec);
rhat = rvec / rho;
Fgrav = -(G * M * m / rho^2) * rhat;

%% Magnetic field update (IGRF) 
if t >= lastMagUpdate
    %we need lat, long, and rho to call igrf
    lastMagUpdate = lastMagUpdate + nextMagUpdate;
    thetaE = acos(z / rho);
    psiE = atan2(y, x);
    latitude = 90 - thetaE * 180 / pi;
    longitude = psiE * 180 / pi;
    rhokm = rho / 1000;

    [BN, BE, BD] = igrf('01-Jan-2020', latitude, longitude, rhokm, 'geocentric');
    BNED = [BN; BE; BD]; %the igrf model gives us field in north east down format
    BI = TIB(0, thetaE + pi, psiE) * BNED;   % convert igrf in NED to ECI
    mag_bf = TIBquat(q0123)' * BI;    % Convert ECI to body frame in nT
    mag_bf = mag_bf * 1e-5;            % Convert nT to Gauss
end

%% Magnetic torque on satellite using cross product: T = m x B 
% to compute the torque produced on the satellite, we need mag field in
% Tesla and dipole in amps, so I convert here
if ~isempty(out_u)
    mag_bf_tesla = mag_bf * 1e-4;                % Convert Gauss to Tesla
    out_u_amps = out_u * 1e-3;                   % Convert mA to A
    LMN_magtorquers = cross(out_u_amps, mag_bf_tesla);  % Torque on satellite in N·m
end

%% Translational and rotational dynamics 
accel = Fgrav / m;
inertia
H = I * pqr;
pqrdot = invI * (LMN_magtorquers - cross(pqr, H)); % our change in angular rates 

%% output Derivative of state vector
dstatedt = [vel; accel; q0123dot; pqrdot];
