function testResults = create_desRN_Test()
    modelName = 'sFunctionLibrary_create_desRN_Harness';
   
    load_system(modelName);
    
    % Setup
    nTestCases = 101;    
    setupSolver(nTestCases,modelName);
    tester = TestSupport(1E-7,1E-7); % ABS and Rel tol of 1E-7 to be close to single precision

    %% Inputs
    % Preallocate
    time = 1:1:nTestCases;
    opMode = uint8(zeros(1,nTestCases));
    desVector = single(zeros(3,nTestCases));
    
    % Subset 1 - Invalid Op Mode
    opMode(1) = 0x0;
    desVector(:,1) = [0,0,0]';

    % SubSet 2 - CTRL_SUN mode (with random vectors)
    opMode(2) = 0x2;
    angles = 0 + 2*pi* rand(100,3);
    for i = 1:100
        opMode(i+1) = 0x2;
        desVector(:,i+1) = angle2dcm(angles(i,3),angles(i,2),angles(i,1))*[1;0;0];
    end
   
    % Subset 3 - CSAC


    % Subset 4 - DCOMM

    %% TODO - add antenna modes

   % https://www.mathworks.com/matlabcentral/answers/458511-setexternalinput-the-number-of-external-inputs-must-be-equal-to-the-number-of-root-level-input-port
    inports=createInputDataset(modelName,'UpdateDiagram',false);
    inports{1} = timeseries(opMode, time, 'Name',inports{1}.Name);
    inports{2} = timeseries(desVector, time, 'Name',inports{2}.Name);
    
    %% Expected Outputs
    outputs =  single(zeros(4,nTestCases));
    outputs(:,1) = NaN(4,1);
    for i = 1:100
        outputs(:,i+1) = buildExpectedQuaternion(desVector(:,i+1));
    end
    
    %% Update Model
    % Set Inputs
    simIn = Simulink.SimulationInput(modelName);
    simIn = simIn.setExternalInput(inports);

    %% Run Model
    out = sim(simIn);
    outPort1 = out.yout{1}.Values.Data;

    %% Test Results
    testResults = struct();
    testResults.passed = NaN(nTestCases,1);

    for iTest = 1:nTestCases
        % Handle case where we expect NaN
        if any(isnan(outputs(:,iTest)))
            testResults.passed(iTest) = tester.IS_NAN(outPort1(:,:,iTest));
        else
            testResults.passed(iTest) = tester.IS_EQUAL_ABS(outputs(:,iTest),outPort1(:,:,iTest));
        end
    end   
    close_system(modelName);

    % figure(); 
    % subplot(4,1,1);
    % plot(2:11,zAngles);
    % title('Z Angles vs Step');
    % grid on;
    % 
    % subplot(4,1,2);
    % plot(2:11,desVector(:,2:11)');
    % legend('x','y','z');
    % title('Des Vector Components ECI');
    % grid on;
    % 
    % subplot(4,1,3);
    % plot(2:11,outputs(:,2:11)');
    % legend('w','x','y','z');
    % title('Expected Quaternion Out - Ref to ECI');
    % grid on;
    % 
    % subplot(4,1,4);
    % plot(2:11,squeeze(outPort1(:,2:11))');
    % legend('w','x','y','z');
    % title('Des Quaternion Out - Ref to ECI');
    % grid on;
    % 
    % figure();
    % plot(2:11, 2*acos(outPort1(1,2:11)));

end

function q_RN = buildExpectedQuaternion(des_vec)
    xECI = [1; 0; 0];
    yECI = [0; 1; 0];
    
    dotXdes = dot(des_vec,xECI);

    if abs(dotXdes) < 0.99
         yAxis = cross(des_vec,xECI);
    else
        yAxis = cross(des_vec,yECI);
    end

    zAxis = cross(des_vec,yAxis);

    DCM_NR = [des_vec/norm(des_vec), yAxis/norm(yAxis), zAxis/norm(zAxis)];
    DCM_RN = DCM_NR';
    q_RN = dcm2quat(DCM_RN);

    if (q_RN(1) < 0)
        q_RN = -q_RN;
    end


end