clear all; close all;

raw_data = ADCSdataParse("tlm/tlm_061917c.out.clean.tlm");
time_i = ((raw_data.J2000_time - 529800000) + raw_data.J2000_frac_time/max(raw_data.J2000_frac_time));
time_v = double(time_i);

ss_p_m = raw_data.raw_css_ads7924./max(max(abs(raw_data.raw_css_ads7924)));

figure;
subplot(2,3,1);
hold on;
plot(time_v,ss_p_m(:,1),'r.');
plot(time_v,ss_p_m(:,2),'g.');
plot(time_v,ss_p_m(:,3),'b.');
subplot(2,3,2);
plot(time_v,ss_p_m(:,4),'r.');
plot(time_v,ss_p_m(:,5),'g.');
plot(time_v,ss_p_m(:,6),'b.');
subplot(2,3,3);
plot(time_v,ss_p_m(:,7),'r.');
plot(time_v,ss_p_m(:,8),'g.');
plot(time_v,ss_p_m(:,9),'b.');
subplot(2,3,4);
plot(time_v,ss_p_m(:,10),'r.');
plot(time_v,ss_p_m(:,11),'g.');
plot(time_v,ss_p_m(:,12),'b.');
subplot(2,3,5);
plot(time_v,ss_p_m(:,13),'r.');
plot(time_v,ss_p_m(:,14),'g.');
plot(time_v,ss_p_m(:,14),'b.');
subplot(2,3,5);
plot(time_v,ss_p_m(:,15),'r.');