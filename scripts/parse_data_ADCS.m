%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ----------------- Edited by Ingrid Paska 3/26 ------------------ %
% Parses data using ADCSDataParseDir and Plot_ADCS_Data
% Input: file, written with the '10'

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear; 
clc; 
close all;

%% get the file
file = dir('10 (3)*');

if isempty(file)
    error('File not found');
end

%% actual parsing using ADCSDataParseDir
[adcs_data, filenames] = ADCSDataParseDir(file);

%% make sure output and that works
if isempty(adcs_data)
    error('Parsing failed — no data returned');
end

%% extract the data for gyro, mag, and the rw
gyro = adcs_data.GYRO.bmg250_gyro;
mag  = adcs_data.MAG.lis3mdl_1_mag;
rw   = adcs_data.RW.current_wheel_speeds_drpm;

%% time
t = 1:size(gyro,2);

%% plot
figure;
plot(t, gyro);
title('Gyro');
legend('X','Y','Z');

figure;
plot(t, mag);
title('Magnetometer');
legend('X','Y','Z');

figure;
plot(t, rw);
title('Reaction Wheels');
legend('1','2','3','4');