function f = g06Objective(x)
    % Objective function for g06 problem
    f = (x(1) - 10)^3 + (x(2) - 20)^3;
end

function [c, ceq] = g06Constraints(x)
    % Constraints for g06 problem
    c(1) = -(x(1) - 5)^2 - (x(2) - 5)^2 + 100;
    c(2) = (x(1) - 6)^2 + (x(2) - 5)^2 - 82.81;
    ceq = [];
end
