% Computes the time derivative of the satellite state vector
function dstatedt = state_update(t, state)

global mag_bf BI invI I m nextMagUpdate lastMagUpdate pqr 
global detumble_gain out_u mag_bf_rate mag_bf_prev LMN_magtorquers H_body

global H_rw LMN_rw reaction_wheel_active rw_ramptime_ms spool_time_ms omega omega_prev


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

    [BNED,H,D,A,F,DXDYDZ,DH,DD,DI,DF] = igrfmagm(rho-R, latitude, longitude, decyear(2026,3,4));
    %BNED=mag field in NED frame in nT
    BI = TIB(0, thetaE + pi, psiE) * BNED';   % convert igrf in NED to ECI
    mag_bf = TIBquat(q0123)' * BI;    % Convert ECI to body frame
    mag_bf = mag_bf * 1e-5;            % Convert nT to Gauss
end

%% Magnetic torque on satellite using cross product: T = m x B 
% to compute the torque produced on the satellite, we need mag field in
% Tesla and dipole in amps, so I convert here
if ~isempty(out_u)
    mag_bf_tesla = mag_bf * 1e-4;                % Convert Gauss to Tesla
    out_u_amps = out_u * 1e-3;                   % Convert mA to A
    LMN_magtorquers = 0;%cross(out_u_amps, mag_bf_tesla);  % Torque on satellite in N·m
end
%% Reaction wheels
total_torque = LMN_magtorquers;
omegadot=(omega-omega_prev)/(rw_ramptime_ms/1000);

if reaction_wheel_active
    reactionWheelParams;
    Gs = [ GsValues(1:4) ; GsValues(5:8) ; GsValues(9:12) ];   % 3x4

    conv_factor = (1/10) * (2*pi/60);   % = pi/300
    omegadot_rad_s2 = omegadot * conv_factor;  % 4x1

    % 3) Wheel torques (Nm): u_s = J * omegadot (elementwise)
    u_s = JsValues(:) .* omegadot_rad_s2(:);  % 4x1

    % 3) Body torque (Nm)
    LMN_rw = -Gs * u_s;
    total_torque = total_torque + LMN_rw;
    % Update reaction wheel momentum (H_rw) if desired here
    H_rw = H_rw + LMN_rw * (nextMagUpdate);  % integrate over time step (Δt ≈ 0.1)
end

%% Translational and rotational dynamics 
accel = Fgrav / m;
inertia
H_body = I * pqr;
H_total = H_body + H_rw;  % include RW momentum
pqrdot = invI * (total_torque - cross(pqr, H_total)); % our change in angular rates 


%% output Derivative of state vector
dstatedt = [vel; accel; q0123dot; pqrdot;omegadot];
