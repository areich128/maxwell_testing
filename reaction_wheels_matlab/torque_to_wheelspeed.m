function [delta_wheelspeed] = torque_to_wheelspeed(GsValues, JsValues, torque_req, ramptime_ms, disabled_wheel)


% torque_to_wheelspeed
%   Maps body torque request into delta wheel speeds
%
% Inputs:
%   GsValues       - 3x4 reaction wheel spin axis matrix (body frame)
%   JsValues       - 1x4 wheel inertias (kg*m^2)
%   torque_req     - 3x1 requested body torque [Nm]  (NOTE: scale mNm->Nm before call!)
%   ramptime_ms    - ramp time (ms)
%   disabled_wheel - index (1..4) of disabled wheel, 0 if none
%
% Output:
%   delta_wheelspeed - 4x1 change in wheel speed [deci-RPM]

    % ---- constants ----
    US_MAX = 0.0032; % [Nm] max torque per wheel
    ramptime = ramptime_ms / 1000; % seconds

    % ---- Step 1: Negate torque vector (flight code convention) ----
    L_r = -torque_req(:); % make column vector

    % ---- Step 2: Apply disabled wheel mask ----
    G_s = reshape(GsValues, 3, 4);
    if disabled_wheel ~= 0
        if disabled_wheel >= 1 && disabled_wheel <= 4
            G_s(:, disabled_wheel) = 0; % zero out column
        else
            error('Disabled wheel index %d out of bounds (1:4)', disabled_wheel);
        end
    end

    % ---- Step 3: Compute pseudoinverse allocation ----
    % u_s = G_s' * inv(G_s*G_s') * L_r
    u_s = G_s' * ((G_s*G_s') \ L_r);

    % ---- Step 4: Torque saturation ----
    u_s = max(min(u_s, US_MAX), -US_MAX);

    % ---- Step 5: Torque -> angular acceleration ----
    rw_ang_acc = u_s(:) ./ JsValues(:);

    % ---- Step 6: Integrate over ramp time ----
    del_wheelspeed_radsec = rw_ang_acc * ramptime; % rad/s

    % ---- Step 7: Convert rad/s -> deci-RPM ----
    delta_wheelspeed = radsec_to_deciRPM(del_wheelspeed_radsec);

end

