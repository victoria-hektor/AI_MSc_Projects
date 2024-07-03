function [cost] = Himmelblau_Constrained(x)
    % Himmelblau_Constrained function definition with constraints
	% In this function, Himmelblau_Constrained, the input x is a vector containing the values of the decision variables. 
	% The function calculates the value of the objective function with constraints based on the Himmelblau function.
	% Constraints are implemented using penalty functions. Two constraints are defined:
	% Circle constraint: x1/2 + x2/2 ≤225
	% Linear constraint: x1+x2 ≤10
	% If a constraint is violated (i.e., if its value is greater than zero), a penalty is added to the objective function. 
	% The penalty parameter values k1 and k2 control the severity of the penalty for each constraint violation.
	    
    % Decision Variables
    x1 = x(1);
    x2 = x(2);
    
    % Constraints
    g1 = x1^2 + x2^2 - 225; % Circle constraint
    g2 = x1 + x2 - 10;      % Linear constraint
	g3 = 4*x1 - 3*x2 - 24;  % Debs constraint
    
    % Penalty Parameters
    k1 = 500; % Penalty parameter for g1 (circle constraint)
    k2 = 500; % Penalty parameter for g2 (linear constraint)
	k3 = 500; % Penalty parameter for g3 (Debs constraint)
    
    % Objective Function
    f = (x1^2 + x2 - 11)^2 + (x1 + x2^2 - 7)^2;
    
    % Penalty Function
    if g1 <= 0 && g2 <= 0 && g3 <= 0
        cost = f;
    else
        cost = f + k1 * max(0, g1)^2 + k2 * max(0, g2)^2 + k3 * max(0, g3)^2;
    end
end
