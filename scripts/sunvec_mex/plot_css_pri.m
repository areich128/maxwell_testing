function [sun_vec_est, sun_sens_volt] = plot_css_pri( fileName )
%PLOT_CSS_PRIMARY Summary of this function goes here
%   Detailed explanation goes here

EPOCH = 529800000;
raw_data = ADCSdataParse(fileName);

%ss_cutoff = .259*3;
ss_cutoff = .42*3;
%No cal for 14/15, reuse 13
ad7991_bias = [-1078.0 -1078.0 -1076.0 ...
               -1039.0 -1047.0 -1038.0 ...
               -1045.0 -1078.0 -1078.0 ...
               -1079.0 -1016.0 -1016.0 ...
               -1016.0 ...
               -1016 -1016];
           
%AD7991_10_SENS = 0.0061/1.334;
AD7991_10_SENS = (0.0061/1.334)*.1973;
ad7991_scale = [1.409*AD7991_10_SENS 1.319*AD7991_10_SENS 1.409*AD7991_10_SENS ...
                0.922*AD7991_10_SENS 1.148*AD7991_10_SENS 1.192*AD7991_10_SENS ...
                1.319*AD7991_10_SENS 0.954*AD7991_10_SENS 0.785*AD7991_10_SENS ...
                AD7991_10_SENS 1.409*AD7991_10_SENS 1.512*AD7991_10_SENS ...
                1.409*AD7991_10_SENS ...
                1.409*AD7991_10_SENS 1.409*AD7991_10_SENS];


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

%Hide +Y
%ad7991_d(:,7) = zeros(length(ad7991_d(:,4)),1);

%Normalize data
css_cal = ad7991_d;%(ad7991_d + ones(length(ad7991_d(:,1)),1)*ad7991_bias)...
          % .*(ones(length(ad7991_d(:,1)),1)*ad7991_scale);

time_v = time_v - min(time_v);



% fig3 = figure; hold on;
% plot(time_v,css_cal(:,3),'b.');
% plot(time_v,3*cos(2*pi()*(1/60)*time_v - 2*pi()*(12.2/60)));
% axis(axis_vec);

%% Sun Vec Estimation


sun_vec_est = [0,0,0];
for ii = 1:length(css_cal(:,1))
    [sun_vec_est(ii,:), sun_sens_volt(ii,:)] = ...
            att_sun_vec_cl(...
                    ad7991_d(ii,1),ad7991_d(ii,2),ad7991_d(ii,3),...
                    ad7991_d(ii,4),ad7991_d(ii,5),ad7991_d(ii,6),...
                    ad7991_d(ii,7),ad7991_d(ii,8),ad7991_d(ii,9),...
                    ad7991_d(ii,10),ad7991_d(ii,11),ad7991_d(ii,12),...
                    ad7991_d(ii,13),ad7991_d(ii,14),ad7991_d(ii,15));
    
     sun_vec_norm(ii,:) = sun_vec_est(ii,:)./norm(sun_vec_est(ii,:));
end
 
sun_vec_norm


%clean data
for ii = 1:length(css_cal(:,1))
    for jj = 1:13
        if sun_sens_volt(ii,jj) > 5
            sun_sens_volt(ii,jj) = 0;
        end
    end
end

%%Plots

%%
fig3 = figure;
%axis_vec = [min(time_v),max(time_v),0,1.1];
axis_vec = [min(time_v),max(time_v),0,3];
subplot(5,1,1);hold on;
day = floor(start_time/86400);
hour = floor((start_time - day*86400)/3600);
minute = floor((start_time - day*86400 - hour*3600)/60);
submin = floor((start_time - day*86400 - hour*3600 - minute*60)/12);
title(sprintf("AD7991 Flight Cal\n Day: %.0f Hour: %.0f Min: %.0f SubMin: %.0f",...
                                                day,hour,minute,submin));
plot(time_v,sun_sens_volt(:,1),'r.');
plot(time_v,sun_sens_volt(:,2),'g.');
plot(time_v,sun_sens_volt(:,3),'b.');
refline(0,ss_cutoff);
legend('+X1','+X2','+X3');
axis(axis_vec);
subplot(5,1,2);hold on;
plot(time_v,sun_sens_volt(:,8),'r.');
plot(time_v,sun_sens_volt(:,7),'g.');
plot(time_v,sun_sens_volt(:,4),'b.');
plot(time_v,sun_sens_volt(:,9),'c.');
refline(0,ss_cutoff);
legend('+Y1','+Y2','+Y3','+Y4');
axis(axis_vec);
subplot(5,1,3);hold on;
plot(time_v,sun_sens_volt(:,13),'r.');
plot(time_v,sun_sens_volt(:,15),'g.');
plot(time_v,sun_sens_volt(:,14),'b.');
refline(0,ss_cutoff);
legend('-X1','-X2','-X3');
axis(axis_vec);
subplot(5,1,4);hold on;
plot(time_v,sun_sens_volt(:,6),'r.');
plot(time_v,sun_sens_volt(:,5),'g.');
plot(time_v,sun_sens_volt(:,10),'g.');
plot(time_v,sun_sens_volt(:,11),'c.');
refline(0,ss_cutoff);
legend('-Y1','-Y2','-Y3','-Y4');
axis(axis_vec);
subplot(5,1,5);hold on;
plot(time_v,sun_sens_volt(:,12),'r.');
refline(0,ss_cutoff);
legend('+Z1');
xlabel('Time (s)');
axis(axis_vec);
hold off;
plot_fname = sprintf("plots/css_fli_%.0f_%.0f_%.0f_%.0f",...
                        day,hour,minute,submin);
print(fig3,plot_fname{1},'-dpng');

fig2 = figure; 
% subplot(2,1,1);
% hold on;
% plot(time_v,sun_vec_est(:,1),'r.');
% plot(time_v,sun_vec_est(:,2),'g.');
% plot(time_v,sun_vec_est(:,3),'b.');
% xlabel('Time (s)');
% ylabel('Sun Vector Components');
% legend('X','Y','Z');

% subplot(2,1,2);
hold on;
plot(time_v,sun_vec_norm(:,1),'r.');
plot(time_v,sun_vec_norm(:,2),'g.');
plot(time_v,sun_vec_norm(:,3),'b.');
xlabel('Time (s)');
ylabel('Sun Vector Components');
legend('X','Y','Z');
title(sprintf("yyyEstimated Sun Vector\nDay: %.0f Hour: %.0f Min: %.0f SubMin: %.0f",...
                                                day,hour,minute,submin));
plot_fname = sprintf("plots/sun_vec_%.0f_%.0f_%.0f_%.0f",...
                        day,hour,minute,submin);
print(fig2,plot_fname{1},'-dpng');
fprintf("Saved Plots for Day: %.0f Hour: %.0f Min: %.0f SubMin: %.0f\n",...
                                                day,hour,minute,submin)


% 
% fig = figure;
% % axis_vec = [min(time_v),max(time_v),0,1.1];
% % axis_vec = [min(time_v),max(time_v),0,3];
% subplot(5,1,1);hold on;
% day = floor(start_time/86400);
% hour = floor((start_time - day*86400)/3600);
% minute = floor((start_time - day*86400 - hour*3600)/60);
% submin = floor((start_time - day*86400 - hour*3600 - minute*60)/12);
% title(sprintf("AD7991 Day: %.0f Hour: %.0f Min: %.0f SubMin: %.0f",...
%                                                 day,hour,minute,submin));
% plot(time_v,css_cal(:,1),'r.');
% plot(time_v,css_cal(:,2),'g.');
% plot(time_v,css_cal(:,3),'b.');
% % refline(0,ss_cutoff);
% legend('+X1','+X2','+X3');
% % axis(axis_vec);
% subplot(5,1,2);hold on;
% plot(time_v,css_cal(:,4),'r.');
% plot(time_v,css_cal(:,5),'g.');
% plot(time_v,css_cal(:,6),'b.');
% plot(time_v,css_cal(:,7),'c.');
% refline(0,ss_cutoff);
% legend('-Y2','-Y1','+Y2','+Y1');
% % axis(axis_vec);
% subplot(5,1,3);hold on;
% plot(time_v,css_cal(:,8),'r.');
% plot(time_v,css_cal(:,9),'g.');
% plot(time_v,css_cal(:,10),'b.');
% % refline(0,ss_cutoff);
% legend('+Y4','-Y4','+Z1');
% % axis(axis_vec);
% subplot(5,1,4);hold on;
% plot(time_v,css_cal(:,11),'r.');
% % refline(0,ss_cutoff);
% legend('-X1');
% % axis(axis_vec);
% subplot(5,1,5);hold on;
% plot(time_v,css_cal(:,12),'r.');
% plot(time_v,css_cal(:,13),'g.');
% plot(time_v,css_cal(:,14),'b.');
% plot(time_v,css_cal(:,15),'c.');
% % refline(0,ss_cutoff);
% legend('-X3','-X2','+Y','-Y');
% xlabel('Time (s)');
% % axis(axis_vec);
% hold off;
% plot_fname = sprintf("plots/css_pri_%.0f_%.0f_%.0f_%.0f",...
%                         day,hour,minute,submin);
% print(fig,plot_fname{1},'-dpng');

end
