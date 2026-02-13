%% This script setups up the workspace for the ADCS Simulink Model
% This is not a good practice way to do things but it'll do for now Donkey

dT_Control = 1/10; % 10Hz control rate
warning('If changing control rate need to ensure its an integer multiple of the sim rate');
