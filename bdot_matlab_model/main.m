%% Initialize environment
clear
clc
close all
tic

% global variables used across functions
global mag_bf BI lastMagUpdate nextMagUpdate out_u mag_bf_prev 
global BxBout ByBout BzBout LMN_magtorquers

%% begin simulation and initialize all variable
disp('Simulation Started')

% Add IGRF model path for geomagnetic field computations
addpath 'igrf/'

% load Earth and satellite parameters
earth_params   % defines R, M, G, mu, m
inertia        % defines satellite's inertia matrix I and I's inverse

% Define initial satellite orbital parameters
altitude = 450*1000;  % Altitude in meters
x0 = R + altitude;    % initial x-position
y0 = 0; z0 = 0;        % Initial y and z
xdot0 = 0;             %initial x-velocity

inclination = 55*pi/180;  % Inclination in radians
semi_major = norm([x0; y0; z0]);  % Orbit radius
vcircular = sqrt(mu / semi_major);  % Circular orbit velocity

ydot0 = vcircular * cos(inclination);  % y velocity
zdot0 = vcircular * sin(inclination);  % z velocity

% Initial Euler angles
phi0 = 0; theta0 = 0; psi0 = 0;
ptp0 = [phi0;theta0;psi0];
q0123_0 = EulerAngles2Quaternions(ptp0);  % Convert to quaternion

% Initial angular velocity (I estimated) (rad/s)
p0 = 0.05; q0 = -0.05; r0 = 0.05;

% Initial state vector: [position; velocity; quaternion; angular velocity]
state = [x0;y0;z0;xdot0;ydot0;zdot0;q0123_0;p0;q0;r0];

%% Define simulation time settings
period = 2*pi/sqrt(mu) * semi_major^(3/2);  % Orbital period
number_of_orbits = 2;       % Simulate 1 full orbit (can do more or less)
tfinal = period * number_of_orbits;
timestep = 0.1;             % Time step for integration (10hz just like flight code)
tout = 0:timestep:tfinal;   % Time vector
stateout = zeros(length(tout), length(state));  % Output state log

%% initial magnetic field output in body/inertial frame (for plotting)
BxBout = zeros(length(tout), 1); %magnetic field x in body frame output
ByBout = BxBout; %%magnetic field y in body frame output
BzBout = BxBout; %magnetic field z in body frame output

%% BDOT-related variables
mag_bf_prev = [0;0;0];    % initialize previous magnetic field (body frame)
mag_bf = [0;0;0];       % initialize current magnetic field (body frame)
out_u = [0;0;0];          % Initialize commanded magnetic dipole

LMN_magtorquers = [0;0;0];  % Initialize torque produced by rods

last_bdot_update = 0;
bdot_update_period = 1.0;  %update bdot at 1hz just like flight code

% simulation will call igrf at 10hz
nextMagUpdate = 0.1;
lastMagUpdate = 0;

% BDOT detumbling controller settings
detumble_gain = 5000;        % detumble gain for BDOT control
SENS_UPDATE_RATE = 0.1; % magnetometer 10hz update rate just like the flight loop

%% Track time progress and print in the command window
next = 100;  % seconds
lastPrint = 0;

%%     MAIN SIMULATION LOOP 
for idx = 1:length(tout)

    % Save current state in output log
    stateout(idx,:) = state';

    % Display simulation time progress in command window
    if tout(idx) > lastPrint
        disp(['Time = ', num2str(tout(idx)), ' out of ', num2str(tfinal)])
        lastPrint = lastPrint + next;
    end

    % Run BDOT controller periodically (10hz)
    if tout(idx) >= last_bdot_update + bdot_update_period
        mag_bf_rate = (mag_bf - mag_bf_prev) / SENS_UPDATE_RATE; %get mag field derivative
        out_u = ctl_bdot(detumble_gain, mag_bf_rate); %get dipole moments
        last_bdot_update = tout(idx); %reset bdot counter
    end

    % set the previous magnetic field before we calculate a new one in
    % state_update
    mag_bf_prev = mag_bf;

    % Run RK4 to update states 
    k1 = state_update(tout(idx), state);
    k2 = state_update(tout(idx)+timestep/2, state+k1*timestep/2);
    k3 = state_update(tout(idx)+timestep/2, state+k2*timestep/2);
    k4 = state_update(tout(idx)+timestep, state+k3*timestep);
    k = (1/6)*(k1 + 2*k2 + 2*k3 + k4);
    state = state + k*timestep;

    % Store body frame magnetic field output for plotting
    BxBout(idx) = mag_bf(1);
    ByBout(idx) = mag_bf(2);
    BzBout(idx) = mag_bf(3);
end

disp('Simulation Complete')

%% Extract useful state outputs for plotting
xout = stateout(:,1); yout = stateout(:,2); zout = stateout(:,3);
q0123out = stateout(:,7:10);
ptpout = Quaternions2EulerAngles(q0123out);
pqrout = stateout(:,11:13); % angular velocities over time

%% --- PLOTTING ---
% Magnetic Field in Body Frame over time
figure;
set(gcf,'color','white')
plot(tout, BxBout, 'b-', 'LineWidth', 2); hold on;
plot(tout, ByBout, 'y-', 'LineWidth', 2);
plot(tout, BzBout, 'g-', 'LineWidth', 2);
xlabel('Time (sec)');
ylabel('Magnetic Field (G)');
legend('Bx', 'By', 'Bz');
grid on;

% Angular Velocity over time
figure;
set(gcf,'color','white')
plot(tout, pqrout, 'LineWidth', 2);
xlabel('Time (sec)');
ylabel('Angular Velocity (rad/s)');
grid on;

% --- Plot Roll, Pitch, Yaw (ptpout) ---
figure;
plot(tout, ptpout, 'LineWidth', 2);
xlabel('Time [sec]');
ylabel('Angle [rad]');
title('Attitude: Roll, Pitch, Yaw');
legend('Roll', 'Pitch', 'Yaw');
grid on;

% --- Plot Quaternion Components (q0123out) ---
figure;
plot(tout, q0123out, 'LineWidth', 2);
xlabel('Time [sec]');
ylabel('Quaternion Value');
title('Quaternion Components');
legend('q0', 'q1', 'q2', 'q3');
grid on;

