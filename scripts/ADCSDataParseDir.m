function [adcs_data,Filenames] = ADCSDataParseDir(files)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ----------------- Created by Anthony Zara on 10/1/22 ------------------ %
% Parses binary ADCS data file inside STOR folder
% Input:
%       DirName = Root directory for binary files
% Usage:
%       For high resolution data:
%       adcs_data = ADCSDataParseDir("STOR\0\0")
%       For low resolution data:
%       NOT CURRENTLY SUPPORTED
%
% Notes: If changing the structure of the data saved to the SD card, you
% should be able to accomplish all edits by editing the fields before line
% 165.  The structure below this line is adaptable to the fields above it,
% and most changes may be accomplished by simply editing those fields.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Used depending on how file is input -- dir function in Plot_ADCS_Data
% will create a '.' and '..' files if a folder is input rather than a file
if ~isempty(files)
    if files(1).bytes == 0
        files = files(3:end);
    end
end

EPOCH_START = 693662400; % From ADCS FSW 22.8.1 (defined in global.h)

%% --------------- File format definitions --------------- %%
% Total size of one frame of SD card data (Bytes)
% Should exactly match 'LOOP_STORAGE_LEN' in global.h
FRAME_SIZE = 200; % [Bytes]

%% Struct Strings - Should Match Variable Name Assignment Below
% Allows for looping over variable names to pull data

% Coarse sun sensors
CSS.varnames = {'css_ad7991_1','css_ad7991_2','css_ad7991_3','css_ad7991_4',...
            'css_ads7924_1','css_ads7924_2','css_ads7924_3','css_ads7924_4'};

% Rate Gyros
GYRO.varnames = {'bmg250_gyro','bmg250_temp',...
               'l3gd20h_gyro','l3gd20h_temp'}; % Can change to bmg250_2

% Magnetometers
MAG.varnames = {'lis3mdl_1_mag','lis3mdl_1_temp',...
               'lis3mdl_2_mag','lis3mdl_2_temp',...
               'lis3mdl_3_mag','lis3mdl_1_temp',};

% GPS position and time
GPS.varnames = {'J2000_time','J2000_frac_time','eci_pos','eci_vel'};

% System properties (global error mask, operational mode mask)
SYS.varnames = {'sensor_use','global_err','op_mode'};

% Reaction Wheels
RW.varnames = {'ref_wheel_speeds_drpm',...
              'ref_wheel_speeds_drpm_ret',...
              'current_wheel_speeds_drpm'};

%% OFFSETs - corresond to the first byte of each sensor's data
OFFSET.sensor_use = 1; % Sensors being used

% ad7991 sun sensors
OFFSET.css_ad7991_1 = 2;
OFFSET.css_ad7991_2 = 10;
OFFSET.css_ad7991_3 = 18;
OFFSET.css_ad7991_4 = 26;

% ads7924 sun sensors
OFFSET.css_ads7924_1 = 34;
OFFSET.css_ads7924_2 = 42;
OFFSET.css_ads7924_3 = 50;
OFFSET.css_ads7924_4 = 58;

% bmg250 rate gyro
OFFSET.bmg250_gyro = 66;    % rate gyro 1
OFFSET.bmg250_temp = 72;

% l3gd20h rate gyro (to be replaced with another bmg250, same structure)
OFFSET.l3gd20h_gyro = 74;   % rate gyro 2
OFFSET.l3gd20h_temp = 80;

% lis3mdl magnetometers
OFFSET.lis3mdl_1_mag = 82;
OFFSET.lis3mdl_1_temp = 88;
OFFSET.lis3mdl_2_mag = 90;
OFFSET.lis3mdl_2_temp = 96;
OFFSET.lis3mdl_3_mag = 98;
OFFSET.lis3mdl_3_temp = 104;

% GPS information
OFFSET.J2000_time = 106;
OFFSET.J2000_frac_time = 110;
OFFSET.eci_pos = 114;
OFFSET.eci_vel = 126;

% SYS information
OFFSET.global_err = 138; % error flag
OFFSET.op_mode = 146; % ADCS operational mode mask

% Reaction Wheel information
OFFSET.ref_wheel_speeds_drpm = 147;
OFFSET.ref_wheel_speeds_drpm_ret = 163;
OFFSET.current_wheel_speeds_drpm = 179;

%% Data Types
TYPE.sensor_use = 'uint8';
TYPE.global_err = 'uint64';
TYPE.op_mode = 'uint8';

% Same size for all CSS, can add fields for each if this changes
TYPE.css_all = 'int16';

% bmg250 rate gyro
TYPE.bmg250_gyro = 'int16';
TYPE.bmg250_temp = 'int16';

% l3gd20h rate gyro (to be replaced with another bmg250, same structure)
TYPE.l3gd20h_gyro = 'int16';
TYPE.l3gd20h_temp = 'int16';

% lis3mdl magnetometers
TYPE.lis3mdl_1_mag = 'int16';
TYPE.lis3mdl_1_temp = 'int16';
TYPE.lis3mdl_2_mag = 'int16';
TYPE.lis3mdl_2_temp = 'int16';
TYPE.lis3mdl_3_mag = 'int16';
TYPE.lis3mdl_3_temp = 'int16';

% GPS
TYPE.J2000_time = 'uint32';
TYPE.J2000_frac_time = 'single';    % 'float' in C
TYPE.eci_pos = 'single';            % 'float' in C
TYPE.eci_vel = 'single';            % 'float' in C

% Reaction Wheels
TYPE.ref_wheel_speeds_drpm = 'int32';
TYPE.ref_wheel_speeds_drpm_ret = 'int32';
TYPE.current_wheel_speeds_drpm = 'int32';

%% Number of entries each object has (i.e 4-element vector, scalar, etc.)
N_OBJ.sensor_use = 1;
N_OBJ.global_err = 1;
N_OBJ.op_mode = 1;

% Same # entries for all CSS, can add fields for each if this changes
N_OBJ.css_all = 4;

% bmg250 rate gyro
N_OBJ.bmg250_gyro = 3; % [x, y, z]
N_OBJ.bmg250_temp = 1;

% l3gd20h rate gyro (to be replaced with another bmg250, same structure)
N_OBJ.l3gd20h_gyro = 3;
N_OBJ.l3gd20h_temp = 1;

% lis3mdl magnetometers
N_OBJ.lis3mdl_1_mag = 3; % [x, y, z]
N_OBJ.lis3mdl_1_temp = 1;
N_OBJ.lis3mdl_2_mag = 3;
N_OBJ.lis3mdl_2_temp = 1;
N_OBJ.lis3mdl_3_mag = 3;
N_OBJ.lis3mdl_3_temp = 1;

% GPS
N_OBJ.J2000_time = 1;
N_OBJ.J2000_frac_time = 1;
N_OBJ.eci_pos = 3; % [x, y, z]
N_OBJ.eci_vel = 3;

% Reaction Wheels
N_OBJ.ref_wheel_speeds_drpm = 4; % [wheel 1, wheel 2, wheel 3, wheel 4]
N_OBJ.ref_wheel_speeds_drpm_ret = 4;
N_OBJ.current_wheel_speeds_drpm = 4;

% ---------------------------- CAUTION ---------------------------- %%
% DO NOT EDIT CODE BELOW THIS LINE UNLESS YOU KNOW WHAT YOU'RE DOING!
%% ----------------- Reading files from directory ----------------- %%
% files = dir(DirName); 
% files = files(3:end); % Old code when input was directory not files

n_files = length(files); % The number of files to parse
alldata_uint8 = [];
for file_idx = 1:n_files
    % Absolute path to file
    file_str = strcat(files(file_idx).folder,filesep,files(file_idx).name);
    
    % Opening the file and getting the handle
    fileID = fopen(file_str); % Opening current file

    % Reading data from file
    file_in_uint8 = fread(fileID); 
    % NOTE - file_in_uint8 contains all bytes in current file, with each
    % byte stored as one array entry expressed as an 8-bit unsigned integer
    
    % Closing File
    fclose(fileID);
    
    % Appending data
    alldata_uint8 = [alldata_uint8; file_in_uint8];
end

%% Segmenting data into frames
n_frames = length(alldata_uint8)/FRAME_SIZE;
if round(n_frames) ~= n_frames
    fprintf('ERROR: STOR File data is not a multiple of the frame size.\n')
end

% Frame array with each column corresponding to one frame in the SD data
frames = reshape(alldata_uint8,FRAME_SIZE,n_frames);

%% Removing bad Data
% Ensures 0xDEADBEEF are the last 4 bytes of data
j = 1;
while (j <= n_frames)
    % 0xDEADBEEF
%     if frames(197,j) ~= 222 || frames(198,j) ~= 173 || frames(199,j) ~= 190 || frames(200,j) ~= 239
    % 0xADBEEF
    if frames(198,j) ~= 173 || frames(199,j) ~= 190 || frames(200,j) ~= 239
        frames(:,j) = [];
        j = j - 1;
        n_frames = n_frames - 1;
    end
    j = j + 1;
end

%% Pre-allocating sensor data objects for speed
for i = 1:length(SYS.varnames)
    name = cell2mat(SYS.varnames(i));
    SYS.(name) = zeros(N_OBJ.(name),n_frames);
end
for i = 1:length(CSS.varnames)
    name = cell2mat(CSS.varnames(i));
    CSS.(name) = zeros(N_OBJ.css_all,n_frames); % Replace N_OBJ.css_all here
end
for i = 1:length(GYRO.varnames)
    name = cell2mat(GYRO.varnames(i));
    GYRO.(name) = zeros(N_OBJ.(name),n_frames);
end
for i = 1:length(MAG.varnames)
    name = cell2mat(MAG.varnames(i));
    MAG.(name) = zeros(N_OBJ.(name),n_frames);
end
for i = 1:length(GPS.varnames)
    name = cell2mat(GPS.varnames(i));
    GPS.(name) = zeros(N_OBJ.(name),n_frames);
end
for i = 1:length(RW.varnames)
    name = cell2mat(RW.varnames(i));
    RW.(name) = zeros(N_OBJ.(name),n_frames);
end

%% Looping through each frame and extracting data
% All modules are exactly the same with the exception of the struct name,
% CSS module is different as all objects have the same size/type
for frame_idx = 1:n_frames
    % SYS Data
    for k = 1:length(SYS.varnames)    
        datastr = cell2mat(SYS.varnames(k));
        data_idx = OFFSET.(datastr);
        for i = 1:N_OBJ.(datastr)
            val_size = sizeof(TYPE.(datastr));
            data_whole = frames(data_idx:data_idx+val_size-1,frame_idx);
            SYS.(datastr)(i,frame_idx) = ...
                typecast(uint8(data_whole),TYPE.(datastr));
            data_idx = data_idx + val_size;
        end
    end
    
    % CSS Data
    for k = 1:length(CSS.varnames)    
        datastr = cell2mat(CSS.varnames(k));
        data_idx = OFFSET.(datastr);
        for i = 1:N_OBJ.css_all % If differen CSS added, change here
            val_size = sizeof(TYPE.css_all); % and here
            data_whole = frames(data_idx:data_idx+val_size-1,frame_idx);
            CSS.(datastr)(i,frame_idx) = ...
                typecast(uint8(data_whole),TYPE.css_all); % and here!
            data_idx = data_idx + val_size;
        end
    end
    
    % Rate Gyro Data
    for k = 1:length(GYRO.varnames)
        datastr = cell2mat(GYRO.varnames(k));
        data_idx = OFFSET.(datastr);
        for i = 1:N_OBJ.(datastr)
            val_size = sizeof(TYPE.(datastr));
            data_whole = frames(data_idx:data_idx+val_size-1,frame_idx);
            GYRO.(datastr)(i,frame_idx) = ...
                typecast(uint8(data_whole),TYPE.(datastr));
            data_idx = data_idx + val_size;
        end
    end
    
    % Magnetometer Data
    for k = 1:length(MAG.varnames)
        datastr = cell2mat(MAG.varnames(k));
        data_idx = OFFSET.(datastr);
        for i = 1:N_OBJ.(datastr)
            val_size = sizeof(TYPE.(datastr));
            data_whole = frames(data_idx:data_idx+val_size-1,frame_idx);
            MAG.(datastr)(i,frame_idx) = ...
                typecast(uint8(data_whole),TYPE.(datastr));
            data_idx = data_idx + val_size;
        end
    end
    
    % GPS Data
    for k = 1:length(GPS.varnames)
        datastr = cell2mat(GPS.varnames(k));
        data_idx = OFFSET.(datastr);
        for i = 1:N_OBJ.(datastr)
            val_size = sizeof(TYPE.(datastr));
            data_whole = frames(data_idx:data_idx+val_size-1,frame_idx);
            GPS.(datastr)(i,frame_idx) = ...
                typecast(uint8(data_whole),TYPE.(datastr));
            data_idx = data_idx + val_size;
        end
    end
    
    % Reaction Wheel Data
    for k = 1:length(RW.varnames)    
        datastr = cell2mat(RW.varnames(k));
        data_idx = OFFSET.(datastr);
        for i = 1:N_OBJ.(datastr)
            val_size = sizeof(TYPE.(datastr));
            data_whole = frames(data_idx:data_idx+val_size-1,frame_idx);
            RW.(datastr)(i,frame_idx) = ...
                typecast(uint8(data_whole),TYPE.(datastr));
            data_idx = data_idx + val_size;
        end
    end
end

%% Copying data to master struct for output
adcs_data = ...
    struct('SYS',SYS,'CSS',CSS,'GYRO',GYRO,...
           'MAG',MAG,'GPS',GPS,'RW',RW);

end
% Function to compute size (in bytes) of object based on data type
function S = sizeof(V)
    switch lower(V)
      case {'double', 'int64', 'uint64'}
        S = 8;
      case {'single', 'int32', 'uint32'}
        S = 4;
      case {'char', 'int16', 'uint16'}
        S = 2;
      case {'logical', 'int8', 'uint8'}
        S = 1;
      otherwise
        warning('Jan:sizeof:BadClass', 'Class "%s" is not supported.', V);
        S = NaN;
    end
end