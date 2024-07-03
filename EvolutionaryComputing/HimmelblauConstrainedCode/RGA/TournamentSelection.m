function i = TournamentSelection(pop, tournamentSize)
    % tournamentSize is the number of individuals to be selected for the tournament

    % Randomly select 'tournamentSize' individuals
    selectedIndices = randperm(length(pop), tournamentSize);
    tournamentIndividuals = pop(selectedIndices);

    % Find the best individual in the tournament
    [~, bestIndex] = min([tournamentIndividuals.Cost]);
    i = selectedIndices(bestIndex);
end
