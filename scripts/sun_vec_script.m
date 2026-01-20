% Script for sun vector algorithm instead of Simulink model bc Simulink
% model is annoying

addpath('QB50/S_functions/bin');
rehash toolboxcache
which att_sun_vec_cl

load("QB50/ss_p_m.mat");

norms = [0.866,  0.866,  0.866, 0.500, 0.500,  0.000,  0.500, 0.000,  0.000, 0.500,  0.000,  0.000, -0.866, -0.866, -0.866;
         0.000,  -0.433, 0.433, 0.866, -0.866, -0.866, 0.866, 0.866,  0.866, -0.866, -0.866, 0.000, -0.433, 0.000,  0.433;
         -0.500, 0.250,  0.250, 0.000, 0.000,  -0.500, 0.000, -0.500, 0.500, 0.000,  0.500,  1.000, -0.250, 0.500,  -0.250];

thresh_on  = 1500;
sun_vec    = nan(3, size(ss_p_m,1));   % NaN -> gaps like the C plot
active     = false(1, size(ss_p_m,2)); % persistent active mask
prev_good  = [1;0;0];
loop_cnt = 0;
ss_count = 0;
sun_sens_pair = [13,14,15,6,7,9,5,6,11,4,9,6,1,2,3];

for i = 1:size(ss_p_m,1)
    loop_cnt = loop_cnt+1;

    % calibrate/set invalid sensors to 0
    data = ss_p_m(i, :);
    for j = 1:length(data)
        if (data(j)>data(sun_sens_pair(j))) && (data(j)>thresh_on)
            data(j) = data(j);
        else
            data(j) = 0;
        end
    end

    for j = 1:15
        if data(j) ~= 0
            ss_count = ss_count+1;
            new_norms(:, ss_count) = norms(:,j);
            new_data(ss_count) = data(j);
        end
    end

    if ss_count < 3
        sun_vec(:, i) = [NaN; NaN; NaN];
    else
        size(new_norms*new_norms')
        size(new_norms*new_data')
        sun_vec(:, i) = (new_norms*new_norms')\(new_norms*new_data');
    end
end

figure();
hold on;
scatter(time_v, sun_vec(1, :), '.r', LineWidth=2);
scatter(time_v, sun_vec(2, :), '.g', LineWidth=2);
scatter(time_v, sun_vec(3, :), '.b', LineWidth=2);
legend("x", "y", "z");