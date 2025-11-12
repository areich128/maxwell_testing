%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ----------------- Created by Braden N. on 08/23/24 ------------------ %
% Plots adcs data after it is parsed
% Input:
%       adcs_data - structure holding all relevant adcs data (sensors, opmode, time, ect.)
% Usage:
%       For high resolution data:
%       See comments on inputs below. Note that high res can only plot 1 day's
%       worth of data
%       
%       For low resolution data:
%       Use commented out hour_path seen below.
%
% Notes: Each adcs_data input 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc;
close all;
clear all;

% cd ../data/'STOR folders'/
% Root_dir = pwd;
% cd ../../scripts/

Parameters = Data_Parameters();

%% Data Parsing
% Taking ADCS Data over specified time span
% (used for high-res data, don't use a large span of time)

Root_dir = pwd;

% Day = 484;
Day = 0;
% Use increments of 1 hour
First_Hour = 0;     % (0 to any number)
Last_Hour = 16;      % (0 to any number greater than first_hour)

% File range in each hour folder
First_File = 0;   % (0 is lowest)
Last_File = 302;   
% (302 is highest file seen as of 4/22/25, can make higher and it wont affect processes)

File_range = First_File:1:Last_File;
Time_range_hours = First_Hour:1:Last_Hour;

for i = 1:length(Time_range_hours)
    for j = 1:length(File_range)
        
        hour_path = fullfile(Root_dir,'STOR_folders','STOR_helm_mock',num2str(Day),num2str(Time_range_hours(i)));
        % hour_path = fullfile(Root_dir, 'STOR_YROT_5-19','LOW/'); % Can plot low res data
        FP = fullfile(hour_path,strcat(num2str(File_range(j))));
        Data_filepaths = dir(FP);
        
        adcs_data{i}(:,j) = ADCSDataParseDir(Data_filepaths);
    end
end

%% Declaring Structures
SYS_data.global_err = [];
SYS_data.op_mode = [];
SYS_data.sensor_use = [];

CSS_data.ad7991_1 = [];
CSS_data.ad7991_2 = [];
CSS_data.ad7991_3 = [];
CSS_data.ad7991_4 = [];
CSS_data.ads7924_1 = [];
CSS_data.ads7924_2 = [];
CSS_data.ads7924_3 = [];
CSS_data.ads7924_4 = [];

GYRO_data.bmg250_1_gyro = [];
GYRO_data.bmg250_1_temp = [];
GYRO_data.bmg250_2_gyro = [];
GYRO_data.bmg250_2_temp = [];

MAG_data.lis3mdl_1_mag = [];
MAG_data.lis3mdl_1_temp = [];
MAG_data.lis3mdl_2_mag = [];
MAG_data.lis3mdl_2_temp = [];
MAG_data.lis3mdl_3_mag = [];

GPS_data.J2000_time = [];
GPS_data.J2000_frac_time = [];
GPS_data.eci_pos = [];
GPS_data.eci_vel = [];

RW_data.ref_wheel_speeds_drpm = [];
RW_data.ref_wheel_speeds_drpm_ret = [];
RW_data.current_wheel_speeds_drpm = [];

%% Concatenate Data for std deviation and mean calculations
% ========================================================================
% May not need all this data concatenated, only need data that needs to be
% analyzed using std dev and mean
% NOTE: Maximum iteration is over 24 hours (cannot exceed one day using current implementation)
% ========================================================================
for i = 1:length(adcs_data) % i corresponds to hour
    for j = 1:length(adcs_data{i}) % j corresponds to minute (0 = 10, 1 = 20, ..., 5 = 60)
        % SYS
        SYS_data.global_err = [SYS_data.global_err adcs_data{i}(:,j).SYS.global_err];
        SYS_data.op_mode = [SYS_data.op_mode adcs_data{i}(:,j).SYS.op_mode];
        SYS_data.sensor_use = [SYS_data.sensor_use adcs_data{i}(:,j).SYS.sensor_use];

        % CSS
        CSS_data.ad7991_1 = [CSS_data.ad7991_1 adcs_data{i}(:,j).CSS.css_ad7991_1];
        CSS_data.ad7991_2 = [CSS_data.ad7991_2 adcs_data{i}(:,j).CSS.css_ad7991_2];
        CSS_data.ad7991_3 = [CSS_data.ad7991_3 adcs_data{i}(:,j).CSS.css_ad7991_3];
        CSS_data.ad7991_4 = [CSS_data.ad7991_4 adcs_data{i}(:,j).CSS.css_ad7991_4];
        CSS_data.ads7924_1 = [CSS_data.ads7924_1  adcs_data{i}(:,j).CSS.css_ads7924_1];
        CSS_data.ads7924_2 = [CSS_data.ads7924_2  adcs_data{i}(:,j).CSS.css_ads7924_2];
        CSS_data.ads7924_3 = [CSS_data.ads7924_3  adcs_data{i}(:,j).CSS.css_ads7924_3];
        CSS_data.ads7924_4 = [CSS_data.ads7924_4  adcs_data{i}(:,j).CSS.css_ads7924_4];

        % GYRO
        % NOTE: The l3gd20h gyro is no longer on the board, but the data
        % parse script logs it with that name (was replaced by a second
        % bmg250 gyro)
        GYRO_data.bmg250_2_gyro = [GYRO_data.bmg250_2_gyro adcs_data{i}(:,j).GYRO.bmg250_2_gyro];
        GYRO_data.bmg250_2_temp = [GYRO_data.bmg250_2_temp adcs_data{i}(:,j).GYRO.bmg250_2_temp];
        GYRO_data.bmg250_1_gyro = [GYRO_data.bmg250_1_gyro adcs_data{i}(:,j).GYRO.bmg250_gyro];
        GYRO_data.bmg250_1_temp = [GYRO_data.bmg250_1_temp adcs_data{i}(:,j).GYRO.bmg250_temp];
        
        % MAG
        MAG_data.lis3mdl_1_mag = [MAG_data.lis3mdl_1_mag adcs_data{i}(:,j).MAG.lis3mdl_1_mag];
        MAG_data.lis3mdl_1_temp = [MAG_data.lis3mdl_1_temp adcs_data{i}(:,j).MAG.lis3mdl_1_temp];
        MAG_data.lis3mdl_2_mag = [MAG_data.lis3mdl_2_mag adcs_data{i}(:,j).MAG.lis3mdl_2_mag];
        MAG_data.lis3mdl_2_temp = [MAG_data.lis3mdl_2_temp adcs_data{i}(:,j).MAG.lis3mdl_2_temp];
        MAG_data.lis3mdl_3_mag = [MAG_data.lis3mdl_3_mag adcs_data{i}(:,j).MAG.lis3mdl_3_mag];

        % GPS
        GPS_data.J2000_time = [GPS_data.J2000_time adcs_data{i}(:,j).GPS.J2000_time];
        GPS_data.J2000_frac_time = [GPS_data.J2000_frac_time adcs_data{i}(:,j).GPS.J2000_frac_time];
        GPS_data.eci_pos = [GPS_data.eci_pos adcs_data{i}(:,j).GPS.eci_pos];
        GPS_data.eci_vel = [GPS_data.eci_vel adcs_data{i}(:,j).GPS.eci_vel];

        % RW
        RW_data.ref_wheel_speeds_drpm = [RW_data.ref_wheel_speeds_drpm adcs_data{i}(:,j).RW.ref_wheel_speeds_drpm];
        RW_data.ref_wheel_speeds_drpm_ret = [RW_data.ref_wheel_speeds_drpm_ret adcs_data{i}(:,j).RW.ref_wheel_speeds_drpm_ret];
        RW_data.current_wheel_speeds_drpm = [RW_data.current_wheel_speeds_drpm adcs_data{i}(:,j).RW.current_wheel_speeds_drpm];
    end
end

%% Data Conversion to Readable Values
% Will convert data that is interpreted as unit8,int16,ect. into Gauss,
% Volts, ect.
% See ADCS sensor conversion document on how values were obtained

% CSS_data.ad7991_1 = CSS_data.ad7991_1 .* Parameters.CSS_conversion; % Currently in volts
% CSS_data.ad7991_2 = CSS_data.ad7991_2 .* Parameters.CSS_conversion; % Can normalize based on max int12 value
% CSS_data.ad7991_3 = CSS_data.ad7991_3 .* Parameters.CSS_conversion; 
% CSS_data.ad7991_4 = CSS_data.ad7991_4 .* Parameters.CSS_conversion;
% 
% CSS_data.ads7924_1 = CSS_data.ads7924_1 .* Parameters.CSS_conversion;
% CSS_data.ads7924_2 = CSS_data.ads7924_2 .* Parameters.CSS_conversion;
% CSS_data.ads7924_3 = CSS_data.ads7924_3 .* Parameters.CSS_conversion;
% CSS_data.ads7924_4 = CSS_data.ads7924_4 .* Parameters.CSS_conversion;

GYRO_data.bmg250_2_gyro = GYRO_data.bmg250_2_gyro .* Parameters.GYRO_conversion;
GYRO_data.bmg250_2_temp = GYRO_data.bmg250_2_temp .* Parameters.Temp_conversion;

GYRO_data.bmg250_1_gyro = GYRO_data.bmg250_1_gyro .* Parameters.GYRO_conversion;
GYRO_data.bmg250_1_temp = GYRO_data.bmg250_1_temp .* Parameters.Temp_conversion;

MAG_data.lis3mdl_1_mag = MAG_data.lis3mdl_1_mag .* Parameters.MAG_conversion;
MAG_data.lis3mdl_2_mag = MAG_data.lis3mdl_2_mag .* Parameters.MAG_conversion;
MAG_data.lis3mdl_3_mag = MAG_data.lis3mdl_3_mag .* Parameters.MAG_conversion;

%% Noise Measurements

% Calculate Std Deviation and Mean
% CSS AD7991
std_7991 = std(CSS_data.ad7991_1,0,2);  % std dev for each photodiode (1,2,3,4)
mean_7991 = mean(CSS_data.ad7991_1,2);  % mean for each photodiode

% CSS AD7924
std_7924 = std(CSS_data.ads7924_1,0,2);
mean_7924 = std(CSS_data.ads7924_1,0,2);

% GYRO 1 
std_bmg250 = std(GYRO_data.bmg250_1_gyro,0,2); % std dev for each axis (XYZ)
mean_bmg250 = mean(GYRO_data.bmg250_1_gyro,2);
std_bmg250_temp = std(GYRO_data.bmg250_1_temp,0,2);
mean_bmg250_temp = mean(GYRO_data.bmg250_1_temp,2);

% MAG 1
std_MAG = std(MAG_data.lis3mdl_1_mag,0,2);  % std dev for each axis (XYZ)
std_MAG_temp = std(MAG_data.lis3mdl_1_temp,0,2);
mean_MAG = mean(MAG_data.lis3mdl_1_mag,2);   % mean for each axis (XYS)
mean_MAG_temp = mean(MAG_data.lis3mdl_1_temp,2);

% GPS (eci pos/vel)
std_GPS_pos = std(GPS_data.eci_pos,0,2);
std_GPS_vel = std(GPS_data.eci_vel,0,2);
mean_GPS_pos = mean(GPS_data.eci_pos,2);
mean_GPS_vel = mean(GPS_data.eci_vel,2);

% Pulling in data to make into table
std_dev_array = [std_7991(1), std_7924(1), std_bmg250(1), std_MAG(1), std_GPS_pos(1), std_GPS_vel(1)];
mean_array = [mean_7991(1), mean_7924(1), mean_bmg250(1), mean_MAG(1), mean_GPS_pos(1), mean_GPS_vel(1)];
table_concat = [std_dev_array;mean_array];

% Set up Table for data
Row_Names = {'Std Deviation', 'Mean'};
Var_Names = {'CSS_7991','CSS_7924','GYRO','MAG','GPS_pos','GPS_vel'};

Noise_Table = array2table(table_concat,'VariableNames',Var_Names,'RowNames',Row_Names);

%% Convert J2000 to 'Normal' Time
% Takes in J2000 time (seconds) and declares test start time as 0

Time_shift = GPS_data.J2000_time - Parameters.EPOCH_START;
Time_shift = Time_shift - Time_shift(1);
Time_combine = Time_shift + GPS_data.J2000_frac_time;

% Poly fit a linear line between final points
t1 = Time_combine(1);
tf = Time_combine(end);
xtime1 = 0;
xtimef = length(Time_combine);
xtimetotal = xtime1:1:xtimef-1;

p = polyfit([xtime1 xtimef-1],[t1 tf],1);
Time_line = polyval(p,xtimetotal);

% Checking how long each loop runs
A = diff(Time_combine);
B = A > 0.2;
A = A(B);

%% Plot Calls
% Comment out when certain data is not wanted
plotSYS(Time_combine,SYS_data)
plotCSS(Time_combine,CSS_data)
plotGYRO(Time_combine,GYRO_data)
plotMAG(Time_combine,MAG_data)
plotGPS(Time_combine,GPS_data,Time_line)
plotRW(Time_combine, RW_data)

deltawheel1 = diff(RW_data.current_wheel_speeds_drpm(2,:));
deltawheel2 = diff(RW_data.ref_wheel_speeds_drpm(2,:));
deltat = diff(Time_combine);

dwheeldtime1 = deltawheel1 ./ deltat;
dwheeldtime2 = deltawheel2 ./ deltat;

dwheeldtime1 = lowpass(dwheeldtime1, 0.1);
dwheeldtime2 = lowpass(dwheeldtime2, 0.1);

% figure();
% hold on;
% plot(Time_combine(1:end-1), dwheeldtime1);
% plot(Time_combine(1:end-1), dwheeldtime2);
% legend("Current", "Ref");

%% Plotting Functions
function plotSYS(Time_combine,SYS_data)
    % SYS Data
    figure(1)
    plot(Time_combine,SYS_data.global_err)
    title('Global Error Reading')
    xlabel('Time (s)')
    ylabel('Error Number')
    
    figure(2)
    plot(Time_combine,SYS_data.op_mode)
    title('Op Mode')
    xlabel('Time (s)')
    ylabel('Mode Number')
    
    figure(3)
    plot(Time_combine,SYS_data.sensor_use)
    title('Sensor Use')
    xlabel('Time (s)')
    ylabel('Sensor Use?')

end

function plotCSS(Time_combine,CSS_data)
    % CSS Data AD7991
    figure(4)
    plot(Time_combine,CSS_data.ad7991_1)
    title('AD7991 - Board 1')
    xlabel('Time(s)')
    ylabel('ADC output (Normalized)')
    legend('PD 1','PD 2','PD 3','PD 4')
    ylim([0, 4095])
    
    figure(5)
    plot(Time_combine,CSS_data.ad7991_2)
    title('AD7991 - Board 2')
    xlabel('Time(s)')
    ylabel('ADC output (Normalized)')
    legend('PD 1','PD 2','PD 3','PD 4')
    ylim([0, 4095])
    
    figure(6)
    plot(Time_combine,CSS_data.ad7991_3)
    title('AD7991 - Board 3')
    xlabel('Time(s)')
    ylabel('ADC output (Normalized)')
    legend('PD 1','PD 2','PD 3','PD 4')
    ylim([0, 4095])
    
    figure(7)
    plot(Time_combine,CSS_data.ad7991_4)
    title('AD7991 - Board 4')
    xlabel('Time(s)')
    ylabel('ADC output (Normalized)')
    legend('PD 1','PD 2','PD 3','PD 4')
    ylim([0, 4095])
    
    % CSS Data ADS7924
    figure(8)
    plot(Time_combine,CSS_data.ads7924_1)
    title('ADS7924 - Board 1')
    xlabel('Time(s)')
    ylabel('ADC output (Normalized)')
    legend('PD 1','PD 2','PD 3','PD 4')
    ylim([0, 4095])
    
    figure(9)
    plot(Time_combine,CSS_data.ads7924_2)
    title('ADS7924 - Board 2')
    xlabel('Time(s)')
    ylabel('ADC output (Normalized)')
    legend('PD 1','PD 2','PD 3','PD 4')
    ylim([0, 4095])
    
    figure(10)
    plot(Time_combine,CSS_data.ads7924_3)
    title('ADS7924 - Board 3')
    xlabel('Time(s)')
    ylabel('ADC output (Normalized)')
    legend('PD 1','PD 2','PD 3','PD 4')
    ylim([0, 4095])
    
    figure(11)
    plot(Time_combine,CSS_data.ads7924_4)
    title('ADS7924 - Board 4')
    xlabel('Time(s)')
    ylabel('ADC output (Normalized)')
    legend('PD 1','PD 2','PD 3','PD 4')
    ylim([0, 4095])

    figure(12)
    plot(Time_combine,movmean(CSS_data.ads7924_4, 10));
    title('ADS7924 - Board 4')
    xlabel('Time(s)')
    ylabel('ADC output (Normalized)')
    legend('PD 1','PD 2','PD 3','PD 4')
    ylim([0, 4095])

end

function plotGYRO(Time_combine,GYRO_data)
    % GYRO Rate Data
    figure(12)
    plot(Time_combine,GYRO_data.bmg250_1_gyro)
    title('bmg250_1 - Rate Output')
    xlabel('Time (s)')
    ylabel('Angular Rate (deg/s)')
    legend('X','Y','Z')
    
    figure(13)
    plot(Time_combine,GYRO_data.bmg250_2_gyro)
    title('bmg250_2 - Rate Output')
    xlabel('Time (s)')
    ylabel('Angular Rate (deg/s)')
    legend('X','Y','Z')
    
    % GYRO Temp Data
    figure(14)
    plot(Time_combine,GYRO_data.bmg250_1_temp)
    title('bmg250_1 - Temp Output')
    xlabel('Time (s)')
    ylabel('Temperature ({\circ}C)')
    
    figure(15)
    plot(Time_combine,GYRO_data.bmg250_2_temp)
    title('bmg250_2 - Temp Output')
    xlabel('Time (s)')
    ylabel('Temperature ({\circ}C)')

end

function plotMAG(Time_combine,MAG_data)
    % lis3mdl Mag Field Data (MAG 1)
    figure(16)
    plot(Time_combine,MAG_data.lis3mdl_1_mag)
    title('lis3mdl Mag 1 Reading')
    xlabel('Time (s)')
    ylabel('Field Strength (Gauss)')
    legend('X','Y','Z')
    
    % lis3mdl Mag Field Data (MAG 2)
    figure(17)
    plot(Time_combine,MAG_data.lis3mdl_2_mag)
    title('lis3mdl Mag 2 Reading')
    xlabel('Time (s)')
    ylabel('Field Strength (Gauss)')
    legend('X','Y','Z')
    
    % lis3mdl Mag Field Data (MAG 3)
    figure(18)
    plot(Time_combine,MAG_data.lis3mdl_3_mag)
    title('lis3mdl Mag 3 Reading')
    xlabel('Time (s)')
    ylabel('Field Strength (Gauss)')
    legend('X','Y','Z')

    % MAG Temp Data
    figure(19)
    plot(Time_combine,MAG_data.lis3mdl_1_temp)
    title('lis3mdl Mag 1 Temp')
    xlabel('Time (s)')
    ylabel('Temp {\circ}C')
    
    figure(20)
    plot(Time_combine,MAG_data.lis3mdl_2_temp)
    title('lis3mdl Mag 2 Temp')
    xlabel('Time (s)')
    ylabel('Temp {\circ}C')

end

function plotGPS(Time_combine,GPS_data,Time_line)
    % GPS Time
    figure(21)
    plot(Time_line,'LineWidth',1.5,'Color','r','LineStyle','--')
    hold on
    plot(Time_combine,'LineWidth',1.5,'Color','b')
    title('Time of Test')
    xlabel('Data Point')
    ylabel('Time Since Test Start')
    legend('Linear Line','Recorded Time')
    hold off
    
    % GPS ECI Position
    figure(22)
    plot(Time_combine, GPS_data.eci_pos)
    title('ECI Position (GPS)')
    xlabel('Time (s)')
    ylabel('Position (coordinates??)')
    legend('X','Y','Z')
    
    % GPS ECI Velocity
    figure(23)
    plot(Time_combine, GPS_data.eci_vel)
    title('ECI Velocity (GPS)')
    xlabel('Time (s)')
    ylabel('Velocity (coordinates / sec??')
    legend('X','Z','Y')

end

function plotRW(Time_combine, RW_data)
    figure()
    hold on;
    plot(Time_combine,RW_data.ref_wheel_speeds_drpm)
    title('Ref Wheel Speeds DRPM')
    xlabel('Time (s)')
    ylabel('Ref Wheel Speeds (DRPM)')
    legend('1','2','3', '4')
    ylim([-100000, 100000]);

    figure()
    hold on;
    plot(Time_combine,RW_data.ref_wheel_speeds_drpm_ret)
    title('Ref Wheel Speeds DRPM?')
    xlabel('Time (s)')
    ylabel('Ref Wheel Speeds (DRPM)')
    legend('1','2','3', '4')
    ylim([-100000, 100000]);

    figure()
    hold on;
    plot(Time_combine,RW_data.current_wheel_speeds_drpm)
    title('Current Wheel Speeds DRPM')
    xlabel('Time (s)')
    ylabel('Current Wheel Speeds (DRPM)')
    legend('1','2','3', '4')
    ylim([-100000, 100000]);
end
