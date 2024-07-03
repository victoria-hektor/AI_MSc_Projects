function [c, ceq] = constraint_welded_beam(x)
    % Decompose the decision variables
    h = x(1); % Thickness of the weld
    l = x(2); % Length of the attached part of the beam
    t = x(3); % Thickness of the beam
    b = x(4); % Width of the beam
    
    % Constants and Parameters
    P = 6000; % Load in lbs
    L = 14; % Length of the beam in inches
    E = 30e6; % Modulus of elasticity in psi
    G = 12e6; % Shear modulus in psi
    
    % Derived Calculations
    Pc = (4.013 * E * sqrt((t^2 * b^6) / 36)) / (L^2) * (1 - t * L / (2 * pi) * sqrt(E / (4 * G)));
    M = P * (L + (l / 2));
    R = sqrt((l^2 / 4) + ((h + t)^2 / 4));
    J = 2 * (sqrt(2) * h * l * ((l^2 / 12) + ((h + t)^2 / 4)));
    
    % Calculating tau (shear stress), sigma (bending stress), delta (deflection)
    tau = (P / (sqrt(2) * h * l)) + ((M * R) / J);
    sigma = (6 * P * L) / (b * t^2);
    delta = (4 * P * L^3) / (E * b * t^3);
    
    % Constraints
    tau_max = 13600; % Maximum shear stress in psi
    sigma_max = 30000; % Maximum bending stress in psi
    delta_max = 0.25; % Maximum deflection in inches
    c = [tau - tau_max; sigma - sigma_max; Pc - P; delta - delta_max];
    
    % No equality constraints in this problem
    ceq = [];
end