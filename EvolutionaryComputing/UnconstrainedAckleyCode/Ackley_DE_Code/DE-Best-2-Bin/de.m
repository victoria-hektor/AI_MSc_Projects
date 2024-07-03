%% DE /best/2/bin with Dynamic Parameter Testing incorporating a halt 10 iterations after achieving the global minimum within specified tolerance
clc; clear;

%% Problem Definition
CostFunction = @(x) Himmelblau_Constrained(x);  % Cost Function
nVar = 2;                          % Number of Decision Variables
VarSize = [1 nVar];                % Decision Variables Matrix Size
VarMin = -5.12;                    % Lower Bound of Decision Variables
VarMax = 5.12;                     % Upper Bound of Decision Variables
tolerance = 1e-5;                  % Tolerance for considering the global minimum achieved

% DE parameters
F = 0.5; % value for scaling factor F
params_F = F;
immigrantFraction = 0.1;           % adjust as needed
prevBestCost = inf;                % Initialise with a high value assuming minimisation
restartFraction = 0.2;             % Restart 20% of the population

% Initialisation
BestSol = struct('Cost', inf, 'Position', [], 'Index', 0);

%% Parameter Sets as Struct Array
parameterSets = [
    struct('MaxIt', 1000, 'nPop', 50, 'beta_min', 0.2, 'beta_max', 0.8, 'pCR', 0.2, 'immigrantFraction', 0.1, 'rateIncrease', 0.01, 'maxBetaMin', 0.5, 'minBetaMax', 0.7);  % Set 1
    struct('MaxIt', 500, 'nPop', 100, 'beta_min', 0.2, 'beta_max', 0.8, 'pCR', 0.2, 'immigrantFraction', 0.05, 'rateIncrease', 0.05, 'maxBetaMin', 0.5, 'minBetaMax', 0.7); % Set 2
    struct('MaxIt', 1000, 'nPop', 50, 'beta_min', 0.1, 'beta_max', 0.9, 'pCR', 0.3, 'immigrantFraction', 0.2, 'rateIncrease', 0.09, 'maxBetaMin', 0.5, 'minBetaMax', 0.7); % Set 3
    struct('MaxIt', 500, 'nPop', 50, 'beta_min', 0.2, 'beta_max', 0.8, 'pCR', 0.4, 'immigrantFraction', 0.15, 'rateIncrease', 0.1, 'maxBetaMin', 0.5, 'minBetaMax', 0.7);  % Set 4
    struct('MaxIt', 1000, 'nPop', 100, 'beta_min', 0.3, 'beta_max', 0.7, 'pCR', 0.2, 'immigrantFraction', 0.25, 'rateIncrease', 0.15, 'maxBetaMin', 0.5, 'minBetaMax', 0.7);% Set 5
    struct('MaxIt', 1500, 'nPop', 50, 'beta_min', 0.2, 'beta_max', 0.6, 'pCR', 0.5, 'immigrantFraction', 0.4, 'rateIncrease', 0.2, 'maxBetaMin', 0.5, 'minBetaMax', 0.7)   % Set 6
];

%% Initialise Results Storage
allBestCosts = zeros(max([parameterSets.MaxIt]), length(parameterSets) + 1); % +1 for iteration numbers
allBestCosts(:, 1) = 1:max([parameterSets.MaxIt]); % Column 1 is iteration numbers
globalMinIter = zeros(1, length(parameterSets)); % Initialise globalMinIter
%maxIterations = max([parameterSets.MaxIt]);
particlePositions = cell(length(parameterSets), 1);

%% Snapshot Variable Initialisations
% Determine the maximum number of iterations from all parameter sets
maxIterations = max([parameterSets.MaxIt]);

% Set the iteration points at which you want to take snapshots
% Specify the iteration points for taking snapshots dynamically
snapshotIterations = [100, 350, maxIterations];  % Adjust as needed
%snapshotIterations = [round(maxIterations * 0.25), round(maxIterations * 0.5), round(maxIterations * 0.75)];

% Initialise a structure to hold the snapshots for each parameter set
snapshots = repmat(struct('Iteration', num2cell(snapshotIterations), 'Population', []), length(parameterSets), 1);

%% Initialise Summary table before the loop starts
Summary = table();

%% Main Loop Over Parameter Sets
for run = 1:length(parameterSets)
    params = parameterSets(run);
    adaptive_beta_min = params.beta_min;
    adaptive_beta_max = params.beta_max;
    previousBestCost = inf; % For tracking improvement
    particlePositions{run} = zeros(parameterSets(run).nPop, nVar, parameterSets(run).MaxIt);
    CR = params.pCR; % Define the crossover rate for the current run
    
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
            pop(i).Cost = CostFunction(pop(i).Position);
            % Compare individual cost to the best cost found so far
            if pop(i).Cost < BestSol.Cost
                BestSol.Cost = pop(i).Cost;
                BestSol.Position = pop(i).Position;
                BestSol.Index = i;  % Store the index of the best solution
            end
            % Choose two distinct random vector indices from the population (not including the best individual)
            % Ensure unique indices for a, b, c, and d that are not equal to BestSol.Cost
            indices = randperm(params.nPop);
            indices(indices == BestSol.Cost) = []; % Exclude BestSol.Cost
            a = indices(1);
            b = indices(2);
            c = indices(3);
            d = indices(4);
    
            % Mutation using best individual and two difference vectors
            y = BestSol.Position + params_F * (pop(a).Position - pop(b).Position) ...
                + params_F * (pop(c).Position - pop(d).Position);
            y = max(y, VarMin);
            y = min(y, VarMax);
    
            % Crossover (binomial)
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
        
        if abs(BestSol.Cost) <= tolerance
            globalMinReached = true;
            globalMinIter(run) = it;
            if postGlobalMinIterations > 10
                break;
            end
        end

        % Update prevBestCost for the next iteration
        prevBestCost = BestCost;

        % After finding the best solution, update the BestSol structure
        [BestCost, BestIndex] = min([pop.Cost]);
        BestSol.Cost = BestCost;
        BestSol.Position = pop(BestIndex).Position;
        BestSol.Index = BestIndex; % Store the index of the best solution
        
        disp(['Iteration ' num2str(it) ' in Run ' num2str(run) ': Best Cost = ' num2str(BestCost)]);

        % Take snapshots at specified iterations
        if ismember(it, snapshotIterations)
            % Store population positions for the snapshot
            popPositions = reshape([pop.Position], nVar, [])';
            snapshots(run).Population = popPositions;  % Store snapshot for the current parameter set 'run'
            disp(['Snapshot taken at Iteration ' num2str(it) ' for Parameter Set ' num2str(run)]);
            disp(['Population size: ' num2str(size(popPositions))]);  % Print out population size for debugging
        end
    end

    % At the end of each run, create a new row in the summary table
    newSummary = table(F, CR, params.MaxIt, params.nPop, BestSol.Position(1), BestSol.Position(2), BestSol.Cost, ...
        'VariableNames', {'F', 'CR', 'MaxIt', 'nPop', 'Best_X1', 'Best_X2', 'Best_Cost'});
    
    Summary = [Summary; newSummary];  % Append the new row to the summary table
end

%% Save summary Data:
disp(Summary);

% Save the table to a CSV file
writetable(Summary, 'DE_Summary.csv');

%% Save Iteration Data
csvwrite('DE_AllRunsBestCosts.csv', allBestCosts);

%% Save Medians in Table:
% Display a table of results
disp('Final Solution:');
disp(['Position = [' num2str(BestSol.Position) ']']);
disp(['Cost = ' num2str(BestSol.Cost)]);

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
legend(arrayfun(@(x) ['Set ' num2str(x) ' (global min)'], 1:length(parameterSets), 'UniformOutput', false), 'Location', 'best');

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

% Plot the contour of the objective function
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

numRuns = length(snapshots);
numIterations = size(snapshots(1).Population, 3); % snapshots have the same number of iterations

for j = 1:numIterations
    figure; % Create a new figure for each iteration
    for i = 1:numRuns
        subplot(1, numRuns, i);
        scatter(snapshots(i).Population(:, 1, j), snapshots(i).Population(:, 2, j));
        title(sprintf('Run %d', i));
        xlabel('x1');
        ylabel('x2');
        axis([VarMin VarMax VarMin VarMax]);
        grid on;
    end
    sgtitle(sprintf('Population Snapshots at Iteration %d', j));
end