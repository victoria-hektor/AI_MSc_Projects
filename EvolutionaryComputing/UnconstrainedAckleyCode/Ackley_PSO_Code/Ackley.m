function f = Ackley(x)
% Ackley function implementation for PSO
% Input: x - A vector of parameters
% Output: f - The value of the Ackley function at x

% Ackley formula: f(x) = -20*exp(-0.2*sqrt(0.5*(x1^2 + x2^2))) - exp(0.5*(cos(2*pi*x1) + cos(2*pi*x2))) + exp(1) + 20
c = 2 * pi;
a = 20;
b = 0.2;
f = -a * exp(-b * sqrt(0.5 * (x(1)^2 + x(2)^2))) - exp(0.5 * (cos(c * x(1)) + cos(c * x(2)))) + exp(1) + a;
end
