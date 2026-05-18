function testResults = WheelTorque_Command_Test(doPlot)
    if nargin < 1
        doPlot = false;
    end

    modelName = 'sFunctionLibrary_WheelTorque_Command_Harness';

    load_system(modelName);

    % Setup
    nTestCases = 1000;
    setupSolver(nTestCases, modelName);
    tester = TestSupport(1E-6, 1E-6);

    % Inputs
    % Important:
    % time is used by Simulink/test harness.
    % profileTime is only used to create smooth input variation and plot nicely.
    time = 1:nTestCases;
    profileTime = linspace(0, 100, nTestCases);

    % Inputs must match port dimensions over time:
    % wheel_speeds   [4 x 1 x N]
    % mag_bf         [3 x 1 x N]
    % GsValues       [3 x 4 x N]
    % disabled_wheel [1 x 1 x N]
    % out_u          [3 x 1 x N]

    wheel_speeds   = single(zeros(4,1,nTestCases));
    mag_bf         = single(zeros(3,1,nTestCases));
    out_u          = single(zeros(3,1,nTestCases));
    disabled_wheel = int32(zeros(1,1,nTestCases));

    for k = 1:nTestCases
        t = profileTime(k);

        wheel_speeds(:,:,k) = single([
             20 + 5*sin(0.10*t);
            -15 + 4*cos(0.07*t);
             10 + 3*sin(0.05*t + 0.4);
             -5 + 2*cos(0.12*t)
        ]);

        mag_bf(:,:,k) = single([
             2e-5 + 0.5e-5*sin(0.03*t);
             1e-5 + 0.3e-5*cos(0.04*t);
            -3e-5 + 0.4e-5*sin(0.02*t + 0.5)
        ]);

        out_u(:,:,k) = single([
             0.0010 + 0.0003*sin(0.06*t);
            -0.0010 + 0.0002*cos(0.05*t);
             0.0005 + 0.0001*sin(0.08*t)
        ]);

        disabled_wheel(:,:,k) = int32(0);
    end

    % Reaction wheel geometry matrix [3 x 4]
    Gs = single([0.390731  0.390731  0.390731  0.390731;
                 0        -0.920505  0.920505  0;
                -0.920505  0         0         0.920505]);

    GsValues = single(zeros(3,4,nTestCases));
    for k = 1:nTestCases
        GsValues(:,:,k) = Gs;
    end

    %% Create external input dataset
    inports = createInputDataset(modelName, 'UpdateDiagram', false);

    inports{1} = timeseries(wheel_speeds,   time, 'Name', inports{1}.Name);
    inports{2} = timeseries(mag_bf,         time, 'Name', inports{2}.Name);
    inports{3} = timeseries(GsValues,       time, 'Name', inports{3}.Name);
    inports{4} = timeseries(disabled_wheel, time, 'Name', inports{4}.Name);
    inports{5} = timeseries(out_u,          time, 'Name', inports{5}.Name);

    %% Expected outputs from MATLAB reference implementation
    expected_pwm = single(zeros(3,nTestCases));
    expected_rw  = single(zeros(4,nTestCases));

    for k = 1:nTestCases
        [expected_rw(:,k), expected_pwm(:,k)] = wheelTorqueExpected( ...
            wheel_speeds(:,:,k), ...
            mag_bf(:,:,k), ...
            GsValues(:,:,k), ...
            disabled_wheel(:,:,k), ...
            out_u(:,:,k));
    end

    %% Run model
    simIn = Simulink.SimulationInput(modelName);
    simIn = simIn.setExternalInput(inports);
    simIn = simIn.setModelParameter( ...
        'SaveOutput', 'on', ...
        'OutputSaveName', 'yout', ...
        'SaveFormat', 'Dataset');

    out = sim(simIn);

    % Output order:
    % yout{1} = pwm_dipole
    % yout{2} = rw_torque
    pwm_out = normalizeOutput(out.yout{1}.Values.Data, 3);
    rw_out  = normalizeOutput(out.yout{2}.Values.Data, 4);

    %% Test results
    testResults = struct();
    testResults.passed = false(nTestCases,2);

    for k = 1:nTestCases
        testResults.passed(k,1) = tester.IS_EQUAL_ABS(expected_pwm(:,k), pwm_out(:,k));
        testResults.passed(k,2) = tester.IS_EQUAL_ABS(expected_rw(:,k),  rw_out(:,k));
    end

    %% Optional plots
    if doPlot
        figure;
        plot(profileTime, rw_out', 'LineWidth', 1.2);
        hold on;
        plot(profileTime, expected_rw', '--', 'LineWidth', 1.2);
        grid on;
        title('Reaction Wheel Torque: S-function vs MATLAB Expected');
        xlabel('Time [s]');
        ylabel('RW torque [Nm]');
        legend('RW1 S','RW2 S','RW3 S','RW4 S', ...
               'RW1 MATLAB','RW2 MATLAB','RW3 MATLAB','RW4 MATLAB', ...
               'Location','best');

        figure;
        plot(profileTime, expected_rw' - rw_out', 'LineWidth', 1.2);
        grid on;
        title('Reaction Wheel Torque Error');
        xlabel('Time [s]');
        ylabel('MATLAB - S-function [Nm]');
        legend('RW1 error','RW2 error','RW3 error','RW4 error', ...
               'Location','best');

        figure;
        plot(profileTime, pwm_out', 'LineWidth', 1.2);
        hold on;
        plot(profileTime, expected_pwm', '--', 'LineWidth', 1.2);
        grid on;
        title('MTB / PWM Output: S-function vs MATLAB Expected');
        xlabel('Time [s]');
        ylabel('MTB output');
        legend('MTB1 S','MTB2 S','MTB3 S', ...
               'MTB1 MATLAB','MTB2 MATLAB','MTB3 MATLAB', ...
               'Location','best');

        figure;
        plot(profileTime, expected_pwm' - pwm_out', 'LineWidth', 1.2);
        grid on;
        title('MTB / PWM Output Error');
        xlabel('Time [s]');
        ylabel('MATLAB - S-function');
        legend('MTB1 error','MTB2 error','MTB3 error', ...
               'Location','best');
    end

    close_system(modelName, 0);
end

function [us, tauMTB] = wheelTorqueExpected(wheel_speeds, mag_bf, GsValues, disabled_wheel, out_u)
    Iws = single(0.000029382450521);
    c   = single(0.05);

    OmegaD = single(250 * 2*pi/60) * ones(4,1,'single');
    Omega  = single(wheel_speeds(:));
    L_R    = single(out_u(:));

    Gs = single(GsValues);

    if disabled_wheel ~= 0 && disabled_wheel <= 4
        Gs(:,disabled_wheel) = 0;
    end

    Gs_pinv = Gs' * inv(Gs*Gs');

    usReq = Gs_pinv * (-L_R);

    usStar = -c * Iws * (Omega - OmegaD);

    Bx = mag_bf(1);
    By = mag_bf(2);
    Bz = mag_bf(3);

    B_tilde = single([  0   -Bz   By;
                        Bz   0   -Bx;
                       -By   Bx   0 ]);

    Bnorm2 = Bx^2 + By^2 + Bz^2;

    if Bnorm2 > single(1e-12)
        Btilde_pinv = -B_tilde / Bnorm2;
    else
        Btilde_pinv = zeros(3,3,'single');
    end

    muStar = -Btilde_pinv * Gs * usStar;

    tauMTB = -B_tilde * muStar;

    deltaU = Gs_pinv * (tauMTB - Gs*usStar);

    us = usReq + usStar + deltaU;

    us = single(us);
    tauMTB = single(tauMTB);
end

function y = normalizeOutput(rawData, nRows)
    % Converts Simulink output data into [nRows x nTime] format.
    y = squeeze(rawData);

    if isvector(y)
        y = reshape(y, nRows, []);
        return;
    end

    if size(y,1) == nRows
        % Already [nRows x nTime]
        return;
    elseif size(y,2) == nRows
        % Convert [nTime x nRows] to [nRows x nTime]
        y = y';
    else
        error('Unexpected output size. Expected one dimension to be %d.', nRows);
    end
end