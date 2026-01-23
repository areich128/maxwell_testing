clear;
clc;
close all;

addpath 'C:\Users\cdwy3\OneDrive\Desktop\MAXWELL\maxwell_testing\maxwell_testing\scripts\QB50\S_functions'
addpath 'C:\Users\cdwy3\OneDrive\Desktop\MAXWELL\maxwell_testing\maxwell_testing\scripts\QB50\tlm'
addpath 'C:\Users\cdwy3\OneDrive\Desktop\MAXWELL\maxwell_testing\maxwell_testing\scripts\QB50\S_functions\bin'


EPOCH = 529800000;
raw_data = ADCSdataParse("tlm/tlm_061917c.out.clean.tlm");

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

figure;
%axis_vec = [min(time_v),max(time_v),0,1.1];
axis_vec = [min(time_v),max(time_v),0,max(max(ss_p_m))*1.1];
subplot(5,1,1);hold on;
day = floor(start_time/86400);
hour = floor((start_time - day*86400)/3600);
minute = floor((start_time - day*86400 - hour*3600)/60);
submin = floor((start_time - day*86400 - hour*3600 - minute*60)/12);
title(sprintf("AD7991 Day: %.0f Hour: %.0f Min: %.0f SubMin: %.0f",...
                                                day,hour,minute,submin));
plot(time_v,ss_p_m(:,1),'r.');
plot(time_v,ss_p_m(:,2),'g.');
plot(time_v,ss_p_m(:,3),'b.');
legend('+X1','+X2','+X3');
axis(axis_vec);
subplot(5,1,2);hold on;
plot(time_v,ss_p_m(:,7),'r.');
plot(time_v,ss_p_m(:,6),'g.');
plot(time_v,ss_p_m(:,14),'b.');
plot(time_v,ss_p_m(:,8),'c.');
legend('+Y1','+Y2','+Y3','+Y4');
axis(axis_vec);
subplot(5,1,3);hold on;
plot(time_v,ss_p_m(:,11),'r.');
plot(time_v,ss_p_m(:,13),'g.');
plot(time_v,ss_p_m(:,12),'b.');
legend('-X1','-X2','-X3');
axis(axis_vec);
subplot(5,1,4);hold on;
plot(time_v,ss_p_m(:,5),'r.');
plot(time_v,ss_p_m(:,4),'g.');
plot(time_v,ss_p_m(:,15),'b.');
plot(time_v,ss_p_m(:,9),'c.');
legend('-Y1','-Y2','-Y3','-Y4');
axis(axis_vec);
subplot(5,1,5);hold on;
plot(time_v,ss_p_m(:,10),'r.');
legend('+Z1');
xlabel('Time (s)');
axis(axis_vec);

plot_css_pri("tlm/tlm_061917c.out.clean.tlm");

% addpath('S_functions/bin');
% rehash toolboxcache
% which att_sun_vec_cl -all