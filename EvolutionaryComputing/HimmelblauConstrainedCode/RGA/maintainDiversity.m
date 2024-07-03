function pop = maintainDiversity(pop, diversityThreshold, VarMin, VarMax, CostFunction)
    % Extract costs from the population structures for diversity calculation
    costs = [pop.Cost];
    
    % Calculate diversity as the standard deviation of the costs
    diversity = std(costs);
    
    if diversity < diversityThreshold
        % Determine the number of new individuals to introduce
        numNewIndividuals = ceil(0.1 * numel(pop));
        
        for i = 1:numNewIndividuals
            % Generate a new individual within the variable bounds
            pop(i).Position = VarMin + rand(1, size(VarMin, 2)) .* (VarMax - VarMin);
            % Calculate the Cost for the new individual
            pop(i).Cost = CostFunction(pop(i).Position); 
            % Reset the Age for the new individual
            pop(i).Age = 0;
        end
    end
end
