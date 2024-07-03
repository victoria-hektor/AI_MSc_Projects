function f = rosenbrock_with_constraints(x)
    % Rosenbrock function
    f = 100 * (x(2) - x(1)^2)^2 + (1 - x(1))^2;
    
    % Define box constraints
    % For example, set constraints such that:
    % -2 <= x(1) <= 2
    % -3 <= x(2) <= 3
    constraint_x0 = [-2, 2];
    constraint_x1 = [-3, 3];
    
    % Penalise violations of box constraints
    penalty = 0;
    if x(1) < constraint_x0(1) || x(1) > constraint_x0(2)
        penalty = penalty + 1e6 * (max(0, constraint_x0(1) - x(1))^2 + max(0, x(1) - constraint_x0(2))^2);
    end
    if x(2) < constraint_x1(1) || x(2) > constraint_x1(2)
        penalty = penalty + 1e6 * (max(0, constraint_x1(1) - x(2))^2 + max(0, x(2) - constraint_x1(2))^2);
    end
    
    % Add penalty to the objective function
    f = f + penalty;
end
