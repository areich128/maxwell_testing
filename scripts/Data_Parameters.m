function Parameters = Data_Parameters()
%% Loads sensor parameters into workspace

Parameters.EPOCH_START = 693662400; % From ADCS FSW 22.8.1 (defined in global.h)

% CSS Voltage conversion value: 0.0007324 Volts/bit
% Currently normalizing CSS value for basilisk noise sim
Parameters.CSS_conversion = (1/4096); % Divide by max value (12 bit ADC) to normalize between 0 and 1
Parameters.MAG_conversion = 0.00006103; % Gauss/bit
Parameters.GYRO_conversion = 0.003612; % (deg/s)/bit
% Temp conversion applies to all temp sensors (mag and gyro)
Parameters.Temp_conversion = 0.001297; % (Celsius)/bit --- NOTE: NOT ACCURATE as of 9/19 

end