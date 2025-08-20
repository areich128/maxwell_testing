function [speed, ramptime] = set_reference_speed(speed, ramptime, wheelnum)
% set_reference_speed 
%   MATLAB version of the C function
%
% Inputs:
%   speed     - commanded wheel speed 
%   ramptime  - ramp time [ms] 
%   wheelnum  - wheel index 
%
% Outputs:
%   
%   speed     - corrected speed command
%   ramptime  - corrected ramp time

    % --- constants (must define these globally or here) ---
    WHEELCOUNT       = 4;
    MAX_SPEED_TRPM   = 65000;   % 
    MIN_SPEED_TRPM   = 1200;     % 
    MAX_RAMPTIME_MS  = 10000;   % 10 sec
    MIN_RAMPTIME_MS  = 10;      % 10 ms

    % --- default return value (1 = error) ---

    % --- Error check: wheel number ---
    if wheelnum > WHEELCOUNT || wheelnum == 0
        fprintf('ERROR: INCORRECT WHEEL NUMBER, OUTSIDE INDEX. ABORTING.\n');
        return;
    end

    % --- Limit wheel speed to max ---
    if speed > MAX_SPEED_TRPM
        speed = MAX_SPEED_TRPM;
    elseif speed < -MAX_SPEED_TRPM
        speed = -MAX_SPEED_TRPM;
    end

    % --- Handle transition near 0 speed ---
    if abs(speed) < MIN_SPEED_TRPM
        if speed > 0
            speed = MIN_SPEED_TRPM;
        elseif speed < 0
            speed = -MIN_SPEED_TRPM;
        else
            speed = 0;
        end
    end

    % --- Limit ramp time ---
    if ramptime > MAX_RAMPTIME_MS
        ramptime = MAX_RAMPTIME_MS;
        fprintf('ERROR: RW RAMP TIME TOO HIGH, SETTING TO MAX_RAMPTIME_MS.\n');
    elseif ramptime < MIN_RAMPTIME_MS
        ramptime = MIN_RAMPTIME_MS;
        fprintf('ERROR: RW RAMP TIME TOO LOW, SETTING TO MIN_RAMPTIME_MS.\n');
    end


end
