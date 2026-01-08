clear all; 
%close all;

EPOCH = 529800000;
raw_data = ADCSdataParse("tlm/tlm_061917c.out.clean.tlm");

%Time
seconds_d = double(raw_data.J2000_time);
frac_time_d = double(raw_data.J2000_frac_time);
time_v = ((seconds_d - EPOCH) + frac_time_d);
time_v = time_v - min(time_v);
sensors_v = double(raw_data.sensor_use);

%Sun Sensor Data
bmg160_d = double(raw_data.raw_bmg160);
max21000_d = double(raw_data.raw_max21000);

%remove zero entries
rem_mask = (time_v~=0 & sensors_v~=0);
bmg160_d = bmg160_d(rem_mask,:);
max21000_d = max21000_d(rem_mask,:);
time_v = time_v(rem_mask);
start_time = min(time_v);

%Normalize data
time_v = time_v - min(time_v);

figure;
subplot(2,1,1);
hold on;
day = floor(start_time/86400);
hour = floor((start_time - day*86400)/3600);
minute = floor((start_time - day*86400 - hour*3600)/60);
submin = floor((start_time - day*86400 - hour*3600 - minute*60)/5);
title(sprintf(" Day: %.0f Hour: %.0f Min: %.0f SubMin: %.0f\r\nMAX21000",...
                                                day,hour,minute,submin));
plot(time_v,max21000_d(:,1),'r.');
plot(time_v,max21000_d(:,2),'g.');
plot(time_v,max21000_d(:,3),'b.');
legend('1','2','3');
subplot(2,1,2);hold on;
title("STML3GD20H");
plot(time_v,bmg160_d(:,1),'r.');
plot(time_v,bmg160_d(:,2),'g.');
plot(time_v,bmg160_d(:,3),'b.');
% plot(time_v,lis3mdl2_d(:,1),'r.');
% plot(time_v,lis3mdl2_d(:,2),'g.');
% plot(time_v,lis3mdl2_d(:,3),'b.');
legend('1','2','3');
