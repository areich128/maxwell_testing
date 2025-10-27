function testResults = dcm2quat_Test()
    modelName = 'sFunctionLibrary_dcm2quat_Harness';
   
    load_system(modelName);
    
    % Setup
    nTestCases = 1;    
    setupSolver(nTestCases,modelName);
    tester = TestSupport(1E-7,1E-7);

    %% Inputs
    % Preallocate
    time = seconds(1:1:nTestCases)';
    data = single(zeros(nTestCases,3,3));

    % Populate
    data(1,:,:) = [0 0 1; 0 1 0; -1 0 0];
    inData = timetable(time,data);

    %% Expected Outputs
    outputs = zeros(4,nTestCases);
    outputs(:,1) = [0; 1/sqrt(2); 0; 1/sqrt(2)];

    %% Update Model
    % Set Inputs
    simIn = Simulink.SimulationInput(modelName);
    simIn = simIn.setExternalInput(inData);

    %% Run Model
    out = sim(simIn);
    outPort1 = out.yout{1}.Values.Data;

    %% Test Results
    testResults = NaN(nTestCases,1);
    for iTest = 1:nTestCases
        testResults(iTest) = tester.IS_EQUAL_ABS(outputs(:,iTest),outPort1(:,iTest));
    end   

    close_system(modelName);
end