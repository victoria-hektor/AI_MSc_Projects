function [f, c, ceq] = g06(x)
    % g06 Function for Constrained Optimisation
    % This function represents a nonlinear constrained optimisation problem.
    % It has two decision variables, two inequality constraints, and no equality constraints.
    
    % Objective function
    % The function to be minimised.
    f = (x(1) - 10)^3 + (x(2) - 20)^3;

    % Inequality constraints (c <= 0)
    % These are the constraints that the solution must satisfy.
    c = [(x(1) - 5)^2 + (x(2) - 5)^2 - 100, ...
         -((x(1) - 6)^2 + (x(2) - 5)^2 - 82.81)];

    % Equality constraints (ceq = 0)
    % There are no equality constraints for this problem.
    ceq = [];
end