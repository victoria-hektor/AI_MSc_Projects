function f = g08(x)
    % g08 Function for Unconstrained Optimisation
    % This function represents an unconstrained optimisation problem.
    % It has two decision variables and is to be maximised. However, MATLAB 
    % optimisers typically minimise functions, so we return the negative.

    % Constants
    % The constants used in the function's formula.
    p = 10;
    q = 15;
    r = 0.1;

    % Objective function
    % The function to be maximised. Negate it for minimisation in MATLAB.
    f = -((sin(2*pi*x(1)))^(3) * sin(2*pi*x(2))) / ...
        ((x(1)^3)*(x(1) + x(2)));

    % Optimisation problems are generally set up to minimise the objective function. 
    % To maximise, you can minimise the negative of the objective function.
end