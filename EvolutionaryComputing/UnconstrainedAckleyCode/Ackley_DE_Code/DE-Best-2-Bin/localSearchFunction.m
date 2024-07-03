function newPosition = localSearchFunction(currentPosition, VarMin, VarMax, CostFunction)
    % Generate random perturbation (Gaussian noise)
    perturbation = randn(size(currentPosition)); % Example: Gaussian noise

    % Add perturbation to the current position
    newPosition = currentPosition + perturbation;
    
    % Ensure newPosition stays within bounds
    newPosition = max(newPosition, VarMin);
    newPosition = min(newPosition, VarMax);

    % Optionally, could use CostFunction to evaluate newPosition and decide on further steps
end
