%% Setup Paths
currentPath = matlab.desktop.editor.getActiveFilename;
currentPath = strsplit(currentPath,'\');
currentFolder = currentPath(1:end-1);

REPO_DIR = strjoin(currentFolder(1:end-3),'\');
TEST_DIR = fullfile(REPO_DIR,'simulink_test');
addpath(genpath(TEST_DIR));

%% Get all tests
testPattern = fullfile(TEST_DIR, '**', '*_Test.m');
allTests = dir(testPattern);
nTests = numel(allTests);

%% Run Tests
testResults = struct();
for iTest = 1:numel(allTests)
    testName = strsplit(allTests(iTest).name,'.');
    testName = testName{1};
    testResults.(testName) = feval(testName);
end