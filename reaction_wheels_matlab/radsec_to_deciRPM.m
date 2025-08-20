function dRPM = radsec_to_deciRPM(radsec)
% rad/s to deci-RPM (0.1 RPM)
    RPM = radsec * 60 / (2*pi);  % rad/s -> RPM
    dRPM = round(RPM * 10);      % deci-RPM
end