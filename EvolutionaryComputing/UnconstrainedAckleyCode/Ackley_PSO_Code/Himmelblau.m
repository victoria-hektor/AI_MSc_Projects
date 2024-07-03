function f = Himmelblau(x)
% Himmelblau function implementation for PSO
% Input: x - A vector of parameters [x1, x2]
% Output: f - The value of the Himmelblau function at x

% Himmelblau formula: f(x) = (x1^2 + x2 - 11)^2 + (x1 + x2^2 - 7)^2
f = (x(1)^2 + x(2) - 11)^2 + (x(1) + x(2)^2 - 7)^2;
end
