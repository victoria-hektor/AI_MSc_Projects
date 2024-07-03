function out = RunGA(problem, params)

    % Problem
    CostFunction = problem.CostFunction;
    nVar = problem.nVar;
    
    % Params
    MaxIt = params.MaxIt;
    nPop = params.nPop;
    pC = params.pC;
    nC = round(pC*nPop/2)*2;
    mu = params.mu;
    maxAge = params.maxAge; % New parameter for AgeBasedSurvivor
    
    % Data Collection Initialisation
    diversity = zeros(MaxIt, 1);
    avgAge = zeros(MaxIt, 1);
    selectionEffectiveness = zeros(MaxIt, 2); % [average fitness of selected, average fitness of not selected]
    
    % Template for Empty Individuals
    empty_individual.Position = [];
    empty_individual.Cost = [];
    empty_individual.Age = 0;
    
    noImproveCounter = 0;
    bestsol.Cost = inf;
    
    % Population Initialisation
    pop = repmat(empty_individual, nPop, 1);
    for i = 1:nPop
        pop(i).Position = problem.VarMin + rand(1, problem.nVar) .* (problem.VarMax - problem.VarMin);
        pop(i).Cost = CostFunction(pop(i).Position);
        pop(i).Age = 0; % Initial age
        if pop(i).Cost < bestsol.Cost
            bestsol = pop(i);
            bestCost = pop(i).Cost; % Initialise the bestCost with the cost of the best initial individual
        end
    end
    
    % Best Cost of Iterations
    bestcost = nan(MaxIt, 1);
    
    % Main Loop
    for it = 1:MaxIt

        % Increment age of all individuals
        for i = 1:nPop
            pop(i).Age = pop(i).Age + 1;
        end
        
        % Initialise Offsprings Population
        popc = repmat(empty_individual, nC/2, 2);
        
        % Pre-selection for analysis purposes
        preSelectedFitness = [];
        notSelectedFitness = [];
        
        % Crossover (HUXCrossover Update)
        for k = 1:nC/2
            % Select Parents (TournamentSelection)
            p1 = TournamentSelection(pop, params.tournamentSize);
            p2 = TournamentSelection(pop, params.tournamentSize);
            
            % Collecting fitness data for selection analysis
            preSelectedFitness = [preSelectedFitness, pop(p1).Cost, pop(p2).Cost];
            
            % Perform HUXCrossover
            [popc(k, 1).Position, popc(k, 2).Position] = HUXCrossover(pop(p1).Position, pop(p2).Position);
        end
        
        % Post-selection analysis
        allIndices = 1:nPop;
        notSelectedIndices = setdiff(allIndices, [p1, p2]);
        for idx = notSelectedIndices
            notSelectedFitness = [notSelectedFitness, pop(idx).Cost];
        end
        
        selectionEffectiveness(it, :) = [mean(preSelectedFitness), mean(notSelectedFitness)];
        
        % Convert popc to Single-Column Matrix
        popc = popc(:);
        
        % Mutation
        for l = 1:nC
            popc(l).Position = NonUniformMutation(popc(l).Position, mu, it, params.MaxIt, problem.VarMin, problem.VarMax);
            popc(l).Cost = CostFunction(popc(l).Position);
            popc(l).Age = 0; % Reset age for new offspring
            if popc(l).Cost < bestsol.Cost
                bestsol = popc(l);
            end
        end
   
        % Merge Populations
        pop = [pop; popc];

        % Calculate diversity and average age
        currentFitness = [pop.Cost];
        diversity(it) = std(currentFitness);
        avgAge(it) = mean([pop.Age]);
		
		% Maintain diversity in the population
		pop = maintainDiversity(pop, 0.1, problem.VarMin, problem.VarMax, CostFunction);
		
		% Check for improvement and possibly escape local optima
		currentBestCost = min([pop.Cost]);
		if currentBestCost < bestCost
			bestCost = currentBestCost;
			noImproveCounter = 0;
		else
 			noImproveCounter = noImproveCounter + 1;
		end
		[pop, noImproveCounter] = escapeLocalOptima(pop, bestCost, noImproveCounter, 20, params.mu, problem.VarMin, problem.VarMax, CostFunction);
		
		% Adapt mutation and crossover rates
		params.mu = adaptiveRate(it, params.MaxIt, 0.1, 0.01);
		params.pC = adaptiveRate(it, params.MaxIt, 0.9, 0.6);
        
        % Age-Based Survivor Selection
        pop = AgeBasedSurvivor(pop, maxAge, nPop);
        
        % Update Best Cost of Iteration
        bestcost(it) = bestsol.Cost;

        % Display Iteration Information
        disp(['Iteration ' num2str(it) ': Best Cost = ' num2str(bestcost(it))]);
    end
    
    % Include collected data in the output
    out.diversity = diversity;
    out.avgAge = avgAge;
    out.selectionEffectiveness = selectionEffectiveness;
    
    % Results
    out.pop = pop;
    out.bestsol = bestsol;
    out.bestcost = bestcost;
    
end