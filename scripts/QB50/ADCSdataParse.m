function [raw_parsed_data] = ADCSdataParse(fileName)
% Parses binary ADCS data file inside STOR folder to struct with all
% values available to matlab. The values are described in the drive
% 6171/ADCS_LOG.xlsx

%fileName = "tlm/tlm_061917c.out.clean.tlm";
EPOCH_START = 529800000;

% File format definitions
SECTOR_SIZE = 142;

% Offsets
OFFSET_sensor_use = 1;
OFFSET_raw_css_ad7991 = 2;
OFFSET_raw_css_ads7924 = 32;
OFFSET_raw_max21000 = 62;
OFFSET_raw_bmg160 = 70;
OFFSET_raw_lis3mdl1 = 78;
OFFSET_raw_lis3mdl2 = 86;
OFFSET_raw_lis3mdl3 = 94;
OFFSET_J2000_time = 102;
OFFSET_J2000_frac_time = 106;
OFFSET_eci_pos = 110;
OFFSET_eci_vel = 122;
OFFSET_g_err = 134;
OFFSET_op_mode = 142;

% Get raw data
data=[];
expected_time=[];

%Find length of file
fileID = fopen(fileName);
num_sec_file = size(fread(fileID))/SECTOR_SIZE;
num_sec_file = floor(num_sec_file(1));

fileID = fopen(fileName);
new_data = fread(fileID,[SECTOR_SIZE,num_sec_file]);
new_data = transpose(new_data);
data = [data;new_data];
fclose(fileID);


%% Reconstruct all data into appropriate vars

sensor_use = uint8(data(:,OFFSET_sensor_use));
raw_css_ad7991 = [];
raw_css_ads7924 = [];
raw_max21000 = [];
raw_bmg160 = [];
raw_lis3mdl1 = [];
raw_lis3mdl2 = [];
raw_lis3mdl3 = [];
J2000_time = [];
J2000_frac_time = [];
eci_pos = [];
eci_vel = [];
g_err = [];
op_mode = uint8(data(:,OFFSET_op_mode));

inv_iter = 1/length(expected_time);
for ii=1:length(data(:,1))
    tmp = [];
    tmp2 = [];
    for jj=1:15
        tmp = [tmp,typecast(uint8(data(ii,OFFSET_raw_css_ad7991+2*(jj-1):...
            OFFSET_raw_css_ad7991+2*jj-1)),'int16')];
        tmp2 = [tmp2,typecast(uint8(data(ii,OFFSET_raw_css_ads7924+2*(jj-1):...
            OFFSET_raw_css_ads7924+2*jj-1)),'int16')];        
    end
    raw_css_ad7991 = [raw_css_ad7991;tmp];
    raw_css_ads7924 = [raw_css_ads7924;tmp2];

    tmp = [];
    tmp2 = [];
    tmp3 = [];
    tmp4 = [];
    tmp5 = [];
    for jj=1:4
        tmp = [tmp,typecast(uint8(data(ii,OFFSET_raw_max21000+2*(jj-1):...
            OFFSET_raw_max21000+2*jj-1)),'int16')];
        tmp2 = [tmp2,typecast(uint8(data(ii,OFFSET_raw_bmg160+2*(jj-1):...
            OFFSET_raw_bmg160+2*jj-1)),'int16')];
        tmp3 = [tmp3,typecast(uint8(data(ii,OFFSET_raw_lis3mdl1+2*(jj-1):...
            OFFSET_raw_lis3mdl1+2*jj-1)),'int16')];  
        tmp4 = [tmp4,typecast(uint8(data(ii,OFFSET_raw_lis3mdl2+2*(jj-1):...
            OFFSET_raw_lis3mdl2+2*jj-1)),'int16')];
        tmp5 = [tmp5,typecast(uint8(data(ii,OFFSET_raw_lis3mdl3+2*(jj-1):...
            OFFSET_raw_lis3mdl3+2*jj-1)),'int16')];        
    end
    raw_max21000 = [raw_max21000;tmp];
    raw_bmg160 = [raw_bmg160;tmp2];
    raw_lis3mdl1 = [raw_lis3mdl1;tmp3];
    raw_lis3mdl2 = [raw_lis3mdl2;tmp4];
    raw_lis3mdl3 = [raw_lis3mdl3;tmp5];
    
    J2000_time = [J2000_time;...
        (typecast(uint8(data(ii,...
           OFFSET_J2000_time:OFFSET_J2000_time+4-1)),'uint32')...
        -EPOCH_START)];
    J2000_frac_time = [J2000_frac_time;...
        (typecast(uint8(data(ii, ...
           OFFSET_J2000_frac_time:OFFSET_J2000_frac_time+4-1)),'single'))];
    
    tmp = [];
    tmp2 = [];
    for jj=1:3
        tmp = [tmp,typecast(uint8(data(ii,OFFSET_eci_pos+4*(jj-1):...
            OFFSET_eci_pos+4*jj-1)),'single')];
        tmp2 = [tmp2,typecast(uint8(data(ii,OFFSET_eci_vel+4*(jj-1):...
            OFFSET_eci_vel+4*jj-1)),'single')];        
    end
    eci_pos = [eci_pos;tmp];
    eci_vel = [eci_vel;tmp2];
    
    g_err = [g_err;typecast(uint8(data(ii,OFFSET_g_err:...
            OFFSET_g_err+8-1)),'uint64')];
end

raw_parsed_data = struct('sensor_use',sensor_use,...
    'raw_css_ad7991',raw_css_ad7991,'raw_css_ads7924',raw_css_ads7924,...
    'raw_max21000',raw_max21000,'raw_bmg160',raw_bmg160,...
    'raw_lis3mdl1',raw_lis3mdl1,'raw_lis3mdl2',raw_lis3mdl2,...
    'raw_lis3mdl3',raw_lis3mdl3,'J2000_time',J2000_time,...
    'J2000_frac_time',J2000_frac_time,'eci_pos',eci_pos,'eci_vel',eci_vel,...
    'g_err',g_err,'op_mode',op_mode);

end