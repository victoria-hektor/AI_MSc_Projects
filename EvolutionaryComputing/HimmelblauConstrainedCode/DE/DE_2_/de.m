%% DE /best/2/bin with Dynamic Parameter Testing incorporating a halt 10 iterations after achieving the global minimum within specified tolerance
clc; clear;

%% Problem Definition
CostFunction = @(x) Himmelblau_Constrained(x);  % Cost Function
nVar = 2;                          % Number of Decision Variables
VarSize = [1 nVar];                % Decision Variables Matrix Size
VarMin = -15.12;                    % Lower Bound of Decision Variables
VarMax = 15.12;                     % Upper Bound of Decision Variables
tolerance = 1e-6;                  % Tolerance for considering the global minimum achieved

immigrantFraction = 0.1;           % adjust as needed
prevBestCost = inf;                % Initialise with a high value
restartFraction = 0.2;             % Restart 20% of the population

%% Parameter Sets as Struct Array
parameterSets = [
    struct('MaxIt', 1000, 'nPop', 150, 'beta_min', 0.4, 'beta_max', 0.8, 'pCR', 0.4, 'immigrantFraction', 0.1, 'rateIncrease', 0.01, 'maxBetaMin', 0.1, 'minBetaMax', 0.7);  % Set 1
    struct('MaxIt', 900, 'nPop', 200, 'beta_min', 0.2, 'beta_max', 0.8, 'pCR', 0.2, 'immigrantFraction', 0.05, 'rateIncrease', 0.05, 'maxBetaMin', 0.2, 'minBetaMax', 0.8); % Set 2
    struct('MaxIt', 800, 'nPop', 150, 'beta_min', 0.1, 'beta_max', 0.9, 'pCR', 0.3, 'immigrantFraction', 0.2, 'rateIncrease', 0.09, 'maxBetaMin', 0.3, 'minBetaMax', 0.9); % Set 3
    struct('MaxIt', 700, 'nPop', 150, 'beta_min', 0.5, 'beta_max', 0.8, 'pCR', 0.1, 'immigrantFraction', 0.15, 'rateIncrease', 0.1, 'maxBetaMin', 0.4, 'minBetaMax', 0.7);  % Set 4
    struct('MaxIt', 600, 'nPop', 200, 'beta_min', 0.3, 'beta_max', 0.7, 'pCR', 0.6, 'immigrantFraction', 0.25, 'rateIncrease', 0.15, 'maxBetaMin', 0.5, 'minBetaMax', 0.8);% Set 5
    struct('MaxIt', 500, 'nPop', 150, 'beta_min', 0.6, 'beta_max', 0.8, 'pCR', 0.5, 'immigrantFraction', 0.4, 'rateIncrease', 0.2, 'maxBetaMin', 0.6, 'minBetaMax', 0.9)   % Set 6
];

%% Initialise Results Storage
allBestCosts = zeros(max([parameterSets.MaxIt]), length(parameterSets) + 1); % +1 for iteration numbers
allBestCosts(:, 1) = 1:max([parameterSets.MaxIt]); % Column 1 is iteration numbers
globalMinIter = zeros(1, length(parameterSets)); % Initialise globalMinIter
maxIterations = max([parameterSets.MaxIt]);
particlePositions = cell(length(parameterSets), 1);

%% Main Loop Over Parameter Sets
for run = 1:length(parameterSets)
    params = parameterSets(run);
    adaptive_beta_min = params.beta_min;
    adaptive_beta_max = params.beta_max;
    previousBestCost = inf; % For tracking improvement
    particlePositions{run} = zeros(parameterSets(run).nPop, nVar, parameterSets(run).MaxIt);
    
    % Initialise Population
    pop = struct('Position', [], 'Cost', inf(params.nPop, 1));
    for i = 1:params.nPop
        pop(i).Position = unifrnd(VarMin, VarMax, VarSize);
        pop(i).Cost = CostFunction(pop(i).Position);
    end
    [BestCost, index] = min([pop.Cost]);
    BestSol = pop(index);
    
    %% DE Main Loop
    for it = 1:params.MaxIt
		for i = 1:params.nPop
            % Choose five random individuals from the population, distinct from i
            candidates = randperm(params.nPop);
            candidates(candidates == i) = [];  % Exclude the target individual
            r = candidates(1:4);  % Select the first four for mutation

            % Mutation: DE/best/2/bin strategy
            % y = BestIndividual + F * (individual_r1 - individual_r2) + F * (individual_r3 - individual_r4)
            y = BestSol.Position + params.beta_min * (pop(r(1)).Position - pop(r(2)).Position) + ...
                params.beta_min * (pop(r(3)).Position - pop(r(4)).Position);
            y = max(y, VarMin); % Make sure solutions stay within bounds
            y = min(y, VarMax);

			% Crossover
			z = pop(i).Position;
			jRand = randi([1 numel(z)]);
			for j = 1:numel(z)
				if rand <= params.pCR || j == jRand
					z(j) = y(j);
				end
			end

			% Selection
			NewSol.Position = z;
			NewSol.Cost = CostFunction(NewSol.Position);
			if NewSol.Cost < pop(i).Cost
				pop(i) = NewSol;
				if NewSol.Cost < BestSol.Cost
					BestSol = NewSol;
				end
			end
		end
        
        % Introduce Random Immigrants
        pop = introduceRandomImmigrants(pop, CostFunction, VarMin, VarMax, VarSize, params);

        % Update BestSol and BestCost logic...
        [currentBestCost, index] = min([pop.Cost]);  % Find the current best cost and index
        if currentBestCost < BestSol.Cost  % If the current best is better than the overall best
            BestSol = pop(index);  % Update BestSol
            BestCost = currentBestCost;  % Update BestCost with the current iteration best
        end
        
        % Use the updated BestCost for adaptMutationRates
        [adaptive_beta_min, adaptive_beta_max] = adaptMutationRates(adaptive_beta_min, adaptive_beta_max, params, BestCost, previousBestCost);
		
		pop = restartPopulation(pop, VarMin, VarMax, VarSize, restartFraction, CostFunction);
        
		% Apply local search if required
        if mod(it, 50) == 0
            pop = applyLocalSearch(pop, @localSearchFunction, CostFunction, VarMin, VarMax);
        end

        improvementRate = abs(prevBestCost - BestCost) / prevBestCost;
        if improvementRate < 0.01
            pop = applyLocalSearch(pop, @localSearchFunction, CostFunction, VarMin, VarMax);
        end
        
        % Update Best Cost Record
        allBestCosts(it, run + 1) = BestCost;

        % Store positions for this iteration
        for i = 1:parameterSets(run).nPop
            particlePositions{run}(i, :, it) = pop(i).Position;
        end
        
		% Check for Global Minimum Achievement
	    if abs(BestSol.Cost) <= tolerance
			globalMinReached = true;
			globalMinIter(run) = it;
			if postGlobalMinIterations > 10
				break; % Exit after 10 iterations post global minimum
            end
        end

        % Update prevBestCost for the next iteration
        prevBestCost = BestCost;
        
        disp(['Iteration ' num2str(it) ' in Run ' num2str(run) ': Best Cost = ' num2str(BestCost)]);
    end
end

%% Save Iteration Data
csvwrite('DE_AllRunsBestCosts.csv', allBestCosts);

%% Visualisation: Trajectory of Each Run's Journey up to Global Minimum
% New figure for the line graph
figure;
hold on;
colors = lines(length(parameterSets));

% Variable to store the minimum iteration for each parameter set
minIterations = zeros(length(parameterSets), 1);

for run = 1:length(parameterSets)
    params = parameterSets(run);
    itData = allBestCosts(:, run + 1);
    minIt = find(itData == min(itData), 1, 'first');
    minIterations(run) = minIt; % Store the minimum iteration
    plot(1:minIt, itData(1:minIt), 'LineWidth', 2, 'Color', colors(run, :));
end

legend(arrayfun(@(x) ['Set ' num2str(x)], 1:length(parameterSets), 'UniformOutput', false), 'Location', 'best');
xlabel('Iteration');
ylabel('Best Cost');
title('Best Cost Trajectory for Each Run');

hold off;

%% Create a 3D Surface Plot for Best Costs Over Iterations and Parameter Sets
% Determine the maximum number of iterations to display based on when global minima were reached
maxDisplayIterations = max(globalMinIter);

% If no global minima were reached, use the maximum number of iterations for any run
if maxDisplayIterations == 0
    maxDisplayIterations = maxIterations;
end

% Initialise the TruncatedBestCostsGrid with NaN values
TruncatedBestCostsGrid = NaN(length(parameterSets), maxDisplayIterations);

% Loop through each parameter set to fill in the best costs
for run = 1:length(parameterSets)
    if globalMinIter(run) > 0
        % Use only up to the iteration when the global minimum was reached
        TruncatedBestCostsGrid(run, 1:globalMinIter(run)) = allBestCosts(1:globalMinIter(run), run + 1)';
    else
        % Use all iterations if the global minimum was not reached
        TruncatedBestCostsGrid(run, :) = allBestCosts(:, run + 1)';
    end
end

% Truncate the IterationGrid and ParameterSetGrid for display using the maxDisplayIterations
[TruncatedIterationGrid, TruncatedParameterSetGrid] = meshgrid(1:maxDisplayIterations, 1:length(parameterSets));

% Create the truncated surface plot
figure;
surf(TruncatedIterationGrid, TruncatedParameterSetGrid, TruncatedBestCostsGrid, 'EdgeColor', 'none');
xlabel('Iteration');
ylabel('Parameter Set');
zlabel('Best Cost');
title('Best Cost Surface Up to Global Minimum');
colorbar;
view(-45, 45);
colormap(jet);
legend(arrayfun(@(x) ['Set ' num2str(x) 'All Runs'], 1:length(parameterSets), 'UniformOutput', false), 'Location', 'best');

%% Contour Plot of Particle Journeys for a Selected Parameter Set
% Find the best solution from the final population of each run
bestPositions = zeros(length(parameterSets), nVar);
for run = 1:length(parameterSets)
    params = parameterSets(run);  % Use params specific for the current run
    finalPopCosts = arrayfun(@(i) CostFunction(particlePositions{run}(i, :, end)), 1:params.nPop);
    [~, idx] = min(finalPopCosts);  % Find the index of the best solution in the final population
    bestPositions(run, :) = particlePositions{run}(idx, :, end);
end

% Create a meshgrid for the contour plot of the Rastrigin function
x = linspace(VarMin, VarMax, 100);
y = linspace(VarMin, VarMax, 100);
[X, Y] = meshgrid(x, y);
Z = arrayfun(@(x, y) CostFunction([x y]), X, Y);

%% Contour Plot:
figure;
% Reduce the number of contour lines to make the plot less busy
contourf(X, Y, Z, 20); % Adjust the number to get the desired granularity
hold on;
colormap('parula'); % Set a colormap that provides a clear background

% Plot the best solutions as red balls
% Adjust the MarkerSize if needed
for i = 1:size(bestPositions, 1)
    plot(bestPositions(i, 1), bestPositions(i, 2), 'ro', 'MarkerSize', 10, ...
        'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'w'); % White edge for better visibility
end

% Label the plot
xlabel('x_1');
ylabel('x_2');
title('Best Particle Positions Contour');
hold off;

%% Side By Side Iterations isualisation:
% Select iterations to display: early, mid, and late convergence stages
selectedIterations = [100, 250, 900]; % For example

% Create a new figure
figure;

% Define colors for each run
colors = lines(length(parameterSets)); 

% Plot for each run at different stages of convergence
for run = 1:length(parameterSets)
    % Extract MaxIt for the current run
    MaxIt = parameterSets(run).MaxIt;
    
    % Define stages based on the number of iterations of the current run
    earlyIter = max(10, round(MaxIt * 0.1)); % 10% into the run
    midIter = round(MaxIt / 2); % 50% into the run
    lateIter = MaxIt - 10; % Near the end of the run

    % Create subplots for early, mid, and late iterations
    for k = 1:3
        subplot(length(parameterSets), 3, (run-1)*3 + k);
        hold on;
        
        if k == 1
            iter = earlyIter;
        elseif k == 2
            iter = midIter;
        else
            iter = lateIter;
        end
        
        % Check if the iteration exists for the current run
        if iter <= MaxIt
            % Extract the positions for the current iteration of this run
            positions = reshape(particlePositions{run}(:,:,iter), [], nVar);
            scatter(positions(:,1), positions(:,2), 36, colors(run, :), 'filled');
        end
        
        % Formatting the plot
        title(sprintf('Run %d - Iteration %d', run, iter));
        xlabel('x_1');
        ylabel('x_2');
        xlim([VarMin, VarMax]);
        ylim([VarMin, VarMax]);
        hold off; 
    end
end

% Add a super title to the entire figure
sgtitle('Particle Positions from Different Runs at Selected Iterations');

%% Diversity Over Time Visualisation:
% Preallocate an array to hold the diversity measure for each run and iteration
diversityOverTime = zeros(maxIterations, length(parameterSets));

% Loop over each parameter set
for run = 1:length(parameterSets)
    % Calculate the diversity for each iteration
    for it = 1:parameterSets(run).MaxIt
        % Get the particle positions for the current iteration and run
        positions = particlePositions{run}(:, :, it);
        
        % Calculate the standard deviation of the positions
        diversityOverTime(it, run) = std(positions(:));
    end
end

% Now, let's plot the diversity over time
figure;
hold on;
colors = lines(length(parameterSets));
for run = 1:length(parameterSets)
    plot(1:parameterSets(run).MaxIt, diversityOverTime(1:parameterSets(run).MaxIt, run), 'Color', colors(run, :), 'LineWidth', 2);
end
hold off;

xlabel('Iteration');
ylabel('Diversity (Standard Deviation)');
title('Diversity of the Population Over Time');
legend(arrayfun(@(x) ['Run ' num2str(x)], 1:length(parameterSets), 'UniformOutput', false), 'Location', 'best');

%% Compare Diversity Vs BestCosts:
% Create a new figure
figure;

% Loop over each parameter set
for run = 1:length(parameterSets)
    subplot(length(parameterSets), 1, run);
    
    % Plot diversity on the left y-axis
    yyaxis left;
    plot(diversityOverTime(1:parameterSets(run).MaxIt, run), 'LineWidth', 2);
    ylabel('Diversity');
    
    % Plot best cost on the right y-axis
    yyaxis right;
    plot(allBestCosts(1:parameterSets(run).MaxIt, run + 1), 'LineWidth', 2); % +1 to skip the iteration count column
    ylabel('Best Cost');
    
    % Add labels and title
    xlabel('Iteration');
    title(sprintf('Run %d - Diversity and Best Cost Over Time', run));
    
    % Add legend
    legend('Diversity', 'Best Cost', 'Location', 'best');
end

% Add an overall title
sgtitle('Comparison of Diversity and Best Cost Over Time for Each Run');
