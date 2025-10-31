function setupSolver(nTestCases,modelName)
    % Sets up the solver settings within the test harness to make it
    % amienable to running lol
    set_param(modelName,'StopTime',sprintf('%i',nTestCases));
    set_param(modelName,'StartTime','1');
    set_param(modelName, 'SolverType','Fixed-step');
    set_param(modelName,'FixedStep','1');
    save_system(modelName); % FIXME: This is a bit bad to do, ideally 
    % should unload all of the changes above at the end of the test but
    % that was hard...
end