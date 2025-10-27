function setupSolver(nTestCases,modelName)
    set_param(modelName,'StopTime',sprintf('%i',nTestCases));
    set_param(modelName,'StartTime','1');
    set_param(modelName, 'SolverType','Fixed-step');
    set_param(modelName,'FixedStep','1');
end