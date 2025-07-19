function out_u = ctl_bdot(detumble_gain, mag_bf_rate)
% BDOT controller to compute magnetic dipole command


out_u = -detumble_gain * mag_bf_rate;  % Control law taken from flight code

% Saturate to ±100 mA·m^2 (0.1 A·m^2)
max_dipole = 100;  % milli-A·m^2
for i = 1:3
        if out_u(i) > max_dipole
            out_u(i) = max_dipole;
        elseif out_u(i) < -max_dipole
            out_u(i) = -max_dipole;
        end
end