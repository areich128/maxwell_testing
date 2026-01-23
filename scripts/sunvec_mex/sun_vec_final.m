clear;
clc;
close all;

addpath('C:\Users\cdwy3\OneDrive\Desktop\MAXWELL\sun_vec_S_function');
%%plotting constants
EPOCH = 529800000;
raw_data = ADCSdataParse('C:\Users\cdwy3\OneDrive\Desktop\MAXWELL\maxwell_testing\maxwell_testing\scripts\QB50\tlm\tlm_061917c.out.clean.tlm');

%Time
seconds_d = double(raw_data.J2000_time);
frac_time_d = double(raw_data.J2000_frac_time);
time_v = ((seconds_d - EPOCH) + frac_time_d);
time_v = time_v - min(time_v);
sensors_v = double(raw_data.sensor_use);

%Sun Sensor Data
ad7991_d = double(raw_data.raw_css_ad7991);

%remove zero entries
rem_mask = (time_v~=0 & sensors_v~=0);
ad7991_d = ad7991_d(rem_mask,:);
time_v = time_v(rem_mask);
start_time = min(time_v);

%Normalize data
ss_p_m = ad7991_d;%./max(ad7991_d);
time_v = time_v - min(time_v);

sun_vec = zeros(327,3);
load("ss_p_m.mat");
for i = 1:size(ss_p_m,1)
    readings = ss_p_m(i,:);
    sun_vec(i,:) = find_sun_vec(readings);
    sun_vec(i,:) = sun_vec(i,:)./norm(sun_vec(i,:));
end

figure();
hold on;
scatter(time_v, sun_vec(:,1), '.r', 'LineWidth', 2);
scatter(time_v, sun_vec(:,2), '.g', 'LineWidth', 2);
scatter(time_v, sun_vec(:,3), '.b', 'LineWidth', 2);
grid on;
legend("x", "y", "z");
xlabel("Time (s)");
ylabel("Sun Unit Vector Component");
title("Plotted Sun Vector vs Time");
