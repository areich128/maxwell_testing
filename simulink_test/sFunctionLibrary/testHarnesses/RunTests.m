%% Setup Paths
currentPath = matlab.desktop.editor.getActiveFilename;
currentPath = strsplit(currentPath,'\');
currentFolder = currentPath(1:end-1);

REPO_DIR = strjoin(currentFolder(1:end-3),'\');
TEST_DIR = fullfile(REPO_DIR,'simulink_test');
addpath(genpath(TEST_DIR));

%% Remove Existing Mex
clear mex; % not sure if need, but helps remove any stale mex you are holding onto

%% Get all tests
testPattern = fullfile(TEST_DIR, '**', '*_Test.m');
allTests = dir(testPattern);
nTests = numel(allTests);

%% Run Tests
testResults = struct();
for iTest = 1:numel(allTests)
    testName = strsplit(allTests(iTest).name,'.');
    testName = testName{1};

    % Display output
    disp(repelem('#',1,75));
    fprintf('Running Test: %s\n', testName);    

    % Run Test 
    testResults.(testName) = feval(testName);
    
    % Report Results
    testResultText = 'FAILED';
    if all(testResults.(testName).passed)
        testResultText = 'PASSED';
    end
    fprintf('%s\n',testResultText);
    disp(repelem('#',1,75));
end

