classdef TestSupport
    % TESTHELPER Summary of this class goes here
    %   Detailed explanation goes here

    properties
        ABS_TOL;
        REL_TOL;
    end

    methods
        function obj = TestSupport(absTol,relTol)
            obj.ABS_TOL = absTol;
            obj.REL_TOL = relTol;
        end

        function res = IS_EQUAL_ABS(obj,truth,meas)
            nx = numel(truth);
            ny = numel(meas);

            res = true;
            if (nx == ny)
                for i = 1:nx
                    compRes = (abs(truth(i) -meas(i)) < obj.ABS_TOL);
                    res = res & compRes;
                end
            else
                res = false;
            end
        end

        function res = IS_EQUAL_REL(obj,truth,meas)
            nx = numel(truth);
            ny = numel(meas);

            res = true;
            if (nx == ny)
                for i = 1:nx
                    compRes = (abs((truth(i) - meas(i)/truth(i))) < obj.REL_TOL);
                    res = res & compRes;
                end
            else
                res = false;
            end
        end
    end
end