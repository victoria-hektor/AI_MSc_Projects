function [pop, noImproveCounter] = escapeLocalOptima(pop, bestCost, noImproveCounter, threshold, mutationRate, VarMin, VarMax, CostFunction)
    if noImproveCounter >= threshold
        for i = 1:numel(pop)
            % Here, create a new individual and assign its fields separately
            newIndividual.Position = VarMin + (VarMax - VarMin) .* rand(1, length(VarMin));
            newIndividual.Cost = CostFunction(newIndividual.Position);
            newIndividual.Age = 0;

            % Now assign the new individual to the population
            pop(i) = newIndividual;
        end
        % Reset the no improvement counter after the escape
        noImproveCounter = 0;
    end
end
