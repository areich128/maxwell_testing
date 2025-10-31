function testResults = create_desRN_Test()
    modelName = 'sFunctionLibrary_create_desRN_Harness';
   
    load_system(modelName);
    
    % Setup
    nTestCases = 15;    
    setupSolver(nTestCases,modelName);
    tester = TestSupport(1E-7,1E-7); % ABS and Rel tol of 1E-7 to be close to single precision

    %% Inputs
    % Preallocate
    time = seconds(1:1:nTestCases)';
    opMode = uint8(zeros(nTestCases,1));
    desVector = single(zeros(nTestCases,3));

    % for iTest = 1:nTestCases
    %     data(iTest,:) = angle2quat(anglesToTest(iTest,1),...
    %         anglesToTest(iTest,2),anglesToTest(iTest,3),'ZYX');
    % end
    https://www.mathworks.com/matlabcentral/answers/458511-setexternalinput-the-number-of-external-inputs-must-be-equal-to-the-number-of-root-level-input-port
    inports=createInputDataset(modelName);
    inports{1} = timetable(time,opMode,'Name',inports{1}.Name);
    inports{2} = timetable(time,desVector,'Name',inports{2}.Name);
    
    %% Expected Outputs
    outputs =  single(zeros(3,nTestCases));
    for iTest = 1:nTestCases
        %outputs(:,iTest) = ;
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
        testResults.passed(iTest) = tester.IS_EQUAL_ABS(outputs(:,iTest),outPort1(:,iTest));
    end   

    close_system(modelName);
end