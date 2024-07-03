% function pop = restartPopulation(pop, VarMin, VarMax, VarSize, restartFraction, CostFunction)
    % numRestart = round(length(pop) * restartFraction);
    % for i = 1:numRestart
        % pop(i).Position = unifrnd(VarMin, VarMax, VarSize);
        % pop(i).Cost = CostFunction(pop(i).Position);
    % end
% end

function pop = restartPopulation(pop, VarMin, VarMax, VarSize, restartFraction, CostFunction)
    numRestart = round(length(pop) * restartFraction);
    for i = 1:numRestart
        pop(end+1-i).Position = unifrnd(VarMin, VarMax, VarSize); % Correct usage of VarSize
        pop(end+1-i).Cost = CostFunction(pop(end+1-i).Position);
    end
end

