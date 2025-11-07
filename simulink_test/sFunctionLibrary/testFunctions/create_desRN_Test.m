function testResults = create_desRN_Test()
    modelName = 'sFunctionLibrary_create_desRN_Harness';
   
    load_system(modelName);
    
    % Setup
    nTestCases = 2;    
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

    % SubSet 1 - CTRL_SUN mode
    opMode(2) = 0x1;
    desVector(:,2) = [1,0,0]';
   
   % https://www.mathworks.com/matlabcentral/answers/458511-setexternalinput-the-number-of-external-inputs-must-be-equal-to-the-number-of-root-level-input-port
    inports=createInputDataset(modelName,'UpdateDiagram',false);
    inports{1} = timeseries(opMode, time, 'Name',inports{1}.Name);
    inports{2} = timeseries(desVector, time, 'Name',inports{2}.Name);
    
    %% Expected Outputs
    outputs =  single(zeros(4,nTestCases));
    outputs(:,1) = NaN(4,1);
    outputs(:,2) = [1; 0; 0; 0];
    
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

    testResults.passed = false;

    close_system(modelName);
end