function f = g08Objective(x)
    % Objective function for g08 problem
    numerator = sin(2 * pi * x(1))^3 * sin(2 * pi * x(2));
    denominator = x(1)^3 * (x(1) + x(2));
    f = -numerator / denominator;
end
