%% Problem Definition
problem.CostFunction = @(x) Himmelblau_Constrained(x); % Himmelblau_Constrained
problem.nVar = 2; % Number of variables
problem.VarMin = [-15, -15];  % Lower Bound for each variable
problem.VarMax = [15, 15];    % Upper Bound for each variable

%% GA Parameters
params.MaxIt = 100; % Maximum number of iterations
params.nPop = 250; % Population size
pop = zeros(numIndividuals, 4); % Initialise matrix for population for Welded Beam Only, comment out when change
params.pC = 0.7; % Crossover probability
params.mu = 0.02; % Mutation rate
params.maxAge = 5; % Maximum age for AgeBasedSurvivor
params.tournamentSize = 3; % Tournament size for selection

% Number of runs
numRuns = 6;

% Perform the runs for each combination of parameters
runCounter = 0; % Initialise run counter

% Initialise matrix to hold all runs
BestCostsMatrix = zeros(params.MaxIt, numRuns);

% Define a broader and more varied range of parameters for mutation and crossover
mutationRates = [0.01, 0.015, 0.02, 0.025, 0.03, 0.035, 0.04, 0.045, 0.05, 0.06, 0.07, 0.08, 0.09, 0.1];
crossoverRates = [0.5, 0.55, 0.6, 0.65, 0.7, 0.75, 0.8, 0.85, 0.9, 0.95, 1.0];

% Prepare a structure to store the results
results = struct();

% Perform the runs for each combination of parameters
for mu = mutationRates
    for pc = crossoverRates
        if runCounter >= numRuns
            break; % Exit the inner loop if we've reached the desired number of runs
        end
        params.mu = mu; % Set the mutation rate
        params.pC = pc; % Set the crossover probability

        % Run the GA with the current set of parameters
        out = RunGA(problem, params);

        % Generate a valid field name by formatting numbers to avoid scientific notation
        mu_str = strrep(num2str(mu, '%.6f'), '.', 'p'); % Replace period with 'p'
        pc_str = strrep(num2str(pc, '%.6f'), '.', 'p'); % Replace period with 'p'

        runID = sprintf('run_mu%s_pc%s', mu_str, pc_str);

        % Store the results with the valid field name
        results.(runID).bestcost = out.bestcost;
        results.(runID).diversity = out.diversity;
        results.(runID).avgAge = out.avgAge;
        results.(runID).selectionEffectiveness = out.selectionEffectiveness;

        % Store the best solution
        results.(runID).bestSolution = out.bestsol.Position;

        runCounter = runCounter + 1; % Increment run counter after each GA run
    end
    if runCounter >= numRuns
        break; % Exit the outer loop if we've reached the desired number of runs
    end
end

%% Save to spreadsheet
% Prepare the matrix to hold the best costs from each run
numRuns = numel(fieldnames(results));
numIters = params.MaxIt;
BestCostsMatrix = zeros(numIters, numRuns);

% Fill the matrix with the best cost data from each run
runCounter = 1;
for runID = fieldnames(results)'
    BestCostsMatrix(:, runCounter) = results.(runID{1}).bestcost;
    runCounter = runCounter + 1;
end

% Define filename for the Excel file
filename = 'BestCostsMultipleRuns.xlsx';

% Save the matrix to Excel
writematrix(BestCostsMatrix, filename);

%% Run GA
out = RunGA(problem, params);

%% Results Visualisations
runIDs = fieldnames(results);
numRuns = length(runIDs);

% Initialise an array to hold the final best cost of each run
finalBestCosts = zeros(1, numRuns);

% Populate the array with the final best cost of each run
for i = 1:numRuns
    finalBestCosts(i) = results.(runIDs{i}).bestcost(end);
end

% Sort the runs by their final best cost in ascending order
[sortedCosts, sortIndex] = sort(finalBestCosts);

% Select the indices of the 10 best runs, ensuring not to exceed the total number of runs
topRunsIndex = sortIndex(1:min(10, numel(sortedCosts)));

% Initialise the figure for plotting
figure;

% Adjust the loop to iterate over indices of the selected runs
for idx = 1:length(topRunsIndex)
    selectedRunID = runIDs{topRunsIndex(idx)};
    subplot(2, 5, idx);  % Arrange the subplots in 2 rows and 5 columns
    plot(results.(selectedRunID).bestSolution, 'LineWidth', 2);
    
    % Use regular expression to extract mutation and crossover values
    tokens = regexp(selectedRunID, 'run_mu(\d+p\d+)_pc(\d+p\d+)', 'tokens');
    if isempty(tokens)
        error('The runID format does not match the expected pattern.');
    end
    
    % Convert the mutation and crossover values back to numeric format
    muValue = str2double(strrep(tokens{1}{1}, 'p', '.'));  % Replace 'p' back with period
    pcValue = str2double(strrep(tokens{1}{2}, 'p', '.'));  % Replace 'p' back with period
    title(sprintf('Mutation: %.4f, Crossover: %.2f', muValue, pcValue));
    
    xlabel('Gene Position');
    ylabel('Gene Expression');
    grid on;
end
annotation('textbox', [0.5, 0.9, 0.1, 0.1], 'String', 'Mutation vs Crossover Run Comparison', ...
        'EdgeColor', 'none', 'HorizontalAlignment', 'center', 'FontSize', 14);

% Best Cost Evolution
figure;
semilogy(out.bestcost, 'LineWidth', 2);
title('Best Cost Evolution');
xlabel('Iteration');
ylabel('Best Cost');
grid on;

% Diversity Measurement
figure;
plot(out.diversity, 'LineWidth', 2);
title('Population Diversity Over Generations');
xlabel('Generation');
ylabel('Diversity (Standard Deviation of Fitness)');
grid on;

% Average Age Distribution
figure;
plot(out.avgAge, 'LineWidth', 2);
title('Average Age of Individuals Over Generations');
xlabel('Generation');
ylabel('Average Age');
grid on;

% Selection Effectiveness
figure;
plot(out.selectionEffectiveness, 'LineWidth', 2);
legend({'Selected Individuals', 'Not Selected Individuals'}, 'Location', 'Best');
title('Selection Effectiveness Over Generations');
xlabel('Generation');
ylabel('Average Fitness');
grid on;

%% New visualisations:
% Plot the best cost evolution for each parameter set on the same graph
figure;
hold on;
for runID = fieldnames(results)'
    % Extract the actual mutation and crossover values from runID for the legend
    tokens = regexp(runID{1}, 'run_mu(\d+p\d+)_pc(\d+p\d+)', 'tokens');
    if isempty(tokens)
        error('The runID format does not match the expected pattern.');
    end
    
    muValue = str2double(strrep(tokens{1}{1}, 'p', '.'));  % Convert 'p' back to period
    pcValue = str2double(strrep(tokens{1}{2}, 'p', '.'));  % Convert 'p' back to period
    descriptiveName = sprintf('Mutation: %.4f, Crossover: %.2f', muValue, pcValue);
    
    % Plot the best cost evolution with a semilogarithmic scale
    semilogy(results.(runID{1}).bestcost, 'LineWidth', 2, 'DisplayName', descriptiveName);
end

title('Best Cost Evolution Across Parameter Sets');
xlabel('Iteration');
ylabel('Best Cost');
legend('show');
grid on;
hold off;

%% Surf Visualisation:
% Initialise the matrix for storing best costs
bestCostsMatrix = zeros(length(mutationRates), length(crossoverRates));

% Iterating over mutation and crossover rates to run the GA
for i = 1:length(mutationRates)
    for j = 1:length(crossoverRates)
        params.mu = mutationRates(i); % Set the mutation rate
        params.pC = crossoverRates(j); % Set the crossover probability

        % Run the GA with the current set of parameters
        out = RunGA(problem, params);

        % Store the best cost in the matrix
        bestCostsMatrix(i, j) = out.bestcost(end); % Assuming 'out.bestcost' gives the final best cost of the run
    end
end

% Create meshgrid for mutation and crossover rates
[X, Y] = meshgrid(mutationRates, crossoverRates);

% Generating the surf plot
figure;
surf(X, Y, bestCostsMatrix'); % Transpose bestCostsMatrix to match the orientation of X and Y in the plot
xlabel('Mutation Rate');
ylabel('Crossover Rate');
zlabel('Best Cost');
title('Performance Landscape of Mutation and Crossover Rates Vs Best Cost');

