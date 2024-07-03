function pop = applyLocalSearch(pop, localSearchFunction, CostFunction, VarMin, VarMax)
    % Iterate over the population and apply local search
    for i = 1:length(pop)
        % Apply the local search function to find a new position within bounds
        new_position = localSearchFunction(pop(i).Position, VarMin, VarMax, CostFunction);
        
        % Update the individual's position and cost with the new values
        pop(i).Position = new_position;
        pop(i).Cost = CostFunction(new_position);
    end
end

