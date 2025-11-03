clear all; 
%close all;

EPOCH = 529800000;
MAX21000_BIAS = [-721.4627, 150.6831, 21.7325,0];
MAX21000_SENS = [.0011,.0011,.0011,1];
raw_data = ADCSdataParse("tlm/tlm_061917c.out.clean.tlm");

%Time
seconds_d = double(raw_data.J2000_time);
frac_time_d = double(raw_data.J2000_frac_time);
time_v = ((seconds_d - EPOCH) + frac_time_d);
time_v = time_v - min(time_v);
sensors_v = double(raw_data.sensor_use);

%Sun Sensor Data
max21000_d = double(raw_data.raw_max21000);

%remove zero entries
rem_mask = (time_v~=0 & sensors_v~=0);
max21000_d = max21000_d(rem_mask,:);
time_v = time_v(rem_mask);
start_time = min(time_v);

%Normalize data
time_v = time_v - min(time_v);
max21000_d = (max21000_d + ones(length(max21000_d(:,1)),1)*MAX21000_BIAS)...
                           .*0.0011;
max21000_n = sqrt(sum(max21000_d'.^2))';
figure;
%subplot(2,1,1);
hold on;
day = floor(start_time/86400);
hour = floor((start_time - day*86400)/3600);
minute = floor((start_time - day*86400 - hour*3600)/60);
submin = floor((start_time - day*86400 - hour*3600 - minute*60)/5);
title(sprintf("MAX21000 Day: %.0f Hour: %.0f Min: %.0f SubMin: %.0f",...
                                                day,hour,minute,submin));
plot(time_v,max21000_d(:,1),'r.');
plot(time_v,max21000_d(:,2),'g.');
plot(time_v,max21000_d(:,3),'b.');
plot(time_v,max21000_n,'c.');
legend('1','2','3','Norm');
xlabel('Time (s)');
ylabel('Rates (dps)');
% subplot(2,1,2);hold on;
% plot(time_v,lis3mdl2_d(:,1),'r.');
% plot(time_v,lis3mdl2_d(:,2),'g.');
% plot(time_v,lis3mdl2_d(:,3),'b.');
% legend('2:1','2:2','2:3');
