function pop = introduceRandomImmigrants(pop, CostFunction, VarMin, VarMax, VarSize, params)
    nImmigrants = round(params.immigrantFraction * length(pop)); % Adjusted to use length(pop) for generality
    for i = 1:nImmigrants
        idx = randi([1, length(pop)]);
        newImmigrant.Position = unifrnd(VarMin, VarMax, VarSize);
        newImmigrant.Cost = CostFunction(newImmigrant.Position); % Now this should work
        pop(idx) = newImmigrant;
    end
end
