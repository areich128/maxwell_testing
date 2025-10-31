function testResults = q2dcm_Test()
    modelName = 'sFunctionLibrary_q2dcm_Harness';
   
    load_system(modelName);
    
    % Setup
    nTestCases = 15;    
    setupSolver(nTestCases,modelName);
    tester = TestSupport(1E-7,1E-7); % ABS and Rel tol of 1E-7 to be close to single precision

    %% Inputs
    % Preallocate
    time = seconds(1:1:nTestCases)';
    data = single(zeros(nTestCases,4));

    % Populate
    anglesToTest = [0,0,0;
                    pi/6,0,0;
                    -pi/6,0,0;
                    pi/6,pi/3,pi/4;
                    pi/6,-pi/3,pi/4;
                    -pi/6,pi/3,pi/4;
                    -pi/6,-pi/3,pi/4;
                    pi/6,pi/3,-pi/4;
                    pi/6,-pi/3,-pi/4;
                    -pi/6,pi/3,-pi/4;
                    -pi/6,-pi/3,-pi/4;
                    0,pi/2,0;
                    0,-pi/2,0;
                    0,0,pi/2;
                    0,0,-pi/2];

    for iTest = 1:nTestCases
        data(iTest,:) = angle2quat(anglesToTest(iTest,1),...
            anglesToTest(iTest,2),anglesToTest(iTest,3),'ZYX');
    end
    inData = timetable(time,data);

    %% Expected Outputs
    outputs =  single(zeros(3,3,nTestCases));
    for iTest = 1:nTestCases
        outputs(:,:,iTest) = angle2dcm(anglesToTest(iTest,1),...
            anglesToTest(iTest,2),anglesToTest(iTest,3),'ZYX');
    end

    %% Update Model
    % Set Inputs
    simIn = Simulink.SimulationInput(modelName);
    simIn = simIn.setExternalInput(inData);

    %% Run Model
    out = sim(simIn);
    outPort1 = out.yout{1}.Values.Data;

    %% Test Results
    testResults = struct();
    testResults.passed = NaN(nTestCases,1);

    for iTest = 1:nTestCases
        testResults.passed(iTest) = tester.IS_EQUAL_ABS(outputs(:,:,iTest),outPort1(:,:,iTest));
    end   

    close_system(modelName);
end