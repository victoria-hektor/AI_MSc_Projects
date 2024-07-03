function pop = AgeBasedSurvivor(pop, maxAge, nPop)
    % Eliminate individuals exceeding maxAge by setting their cost to infinity
    for i = 1:length(pop)
        if pop(i).Age > maxAge
            pop(i).Cost = inf;  % Mark for elimination
        end
    end

    % Sort the population based on cost, which moves individuals marked for elimination to the end
    pop = SortPopulation(pop);

    % Ensure the population size does not drop below nPop
    if length(pop) > nPop
        pop = pop(1:nPop); % Keep only the top nPop individuals
    end
end