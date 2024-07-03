clc; clear;

%% Problem Definition
problem.CostFunction = @(x) Ackley(x);  % Problem function
problem.nVar = 2;
problem.VarMin = -5;  % Lower Bound
problem.VarMax = 5; % Upper Bound

%% Parameters of PSO

% Constriction Coefficients
kappa = 1;
phi1 = 2.05;
phi2 = 2.05;
phi = phi1 + phi2;
chi = 2*kappa/abs(2-phi-sqrt(phi^2-4*phi));

params.w = chi;             % Intertia Coefficient
params.wdamp = 1;           % Damping Ratio of Inertia Coefficient
params.c1 = chi*phi1;       % Personal Acceleration Coefficient
params.c2 = chi*phi2;       % Social Acceleration Coefficient
params.ShowIterInfo = true; % Flag for Showing Iteration Information  

% Tolerance and global minimum
tolerance = 1e-2;
globalMin = [0, 0];  % Theoretical global minimum

% Array of parameter sets for different runs
paramSets = [
    struct('MaxIt', 1000, 'nPop', 50, 'w', 1.00, 'wdamp', 0.99, 'c1', 2.00, 'c2', 2.00, 'ShowIterInfo', true);
    struct('MaxIt', 800, 'nPop', 60, 'w', 0.99, 'wdamp', 0.75, 'c1', 1.00, 'c2', 1.00, 'ShowIterInfo', true);
    struct('MaxIt', 600, 'nPop', 70, 'w', 1.00, 'wdamp', 0.99, 'c1', 2.00, 'c2', 2.00, 'ShowIterInfo', true);
    struct('MaxIt', 400, 'nPop', 80, 'w', 1.00, 'wdamp', 0.99, 'c1', 2.00, 'c2', 2.00, 'ShowIterInfo', true);
    struct('MaxIt', 300, 'nPop', 150, 'w', 1.00, 'wdamp', 0.99, 'c1', 2.00, 'c2', 2.00, 'ShowIterInfo', true);
];

% Initialise arrays to store results
allBestSolutions = zeros(length(paramSets), problem.nVar);
allBestCosts = zeros(length(paramSets), 1);

% Loop through each parameter set
for i = 1:length(paramSets)
    params = paramSets(i);
    
    % PSO Main Body
    out = PSO(problem, params);  % Use the PSO function with the current parameter set
    
    % Store Results
    allBestSolutions(i, :) = out.BestSol.Position;
    allBestCosts(i) = out.BestSol.Cost;
    BestSol = out.BestSol;
    BestCosts = out.BestCosts;
    particle_history = out.particle_history;
    particleHistory = out.particleHistory;
    
    % Tolerance Check
    error = max(abs(out.BestSol.Position - globalMin));
    if error < tolerance
        fprintf('Run %d: Solution found within the acceptable tolerance.\n', i);
    else
        fprintf('Run %d: Best Cost = %.4f, not within the acceptable tolerance after %d iterations.\n', i, allBestCosts(i), params.MaxIt);
    end
    
    % Save Results to Spreadsheet for each parameter set
    saveHistoricalData(out.particleHistory, sprintf('historicalData_Run%d.xlsx', i));
    
    % Visualisation for each parameter set
    figure; % Create a new figure for each set of results
    plotBestCostOverIterations(out.BestCosts);
    title(sprintf('Best Cost Over Iterations (Run %d)', i));
    drawnow; % Ensure plots are updated before the next iteration
    
    figure; % Create a new figure for contour plot
    plotPopulationContour(problem.CostFunction, out.pop, problem.VarMin, problem.VarMax, params.nPop);
    title(sprintf('Population Contour (Run %d)', i));
    drawnow; % Ensure plots are updated before the next iteration
    
    % Optional: animateParticles function can be called here if needed
    animateParticles(out.particleHistory, out.BestCosts, params.MaxIt, params.nPop);
end

% Compile all results into a table after all runs are complete
allEntries = cell(length(paramSets), 9); % 9 for each of the parameters and results

for i = 1:length(paramSets)
    allEntries(i,:) = {paramSets(i).MaxIt, paramSets(i).nPop, paramSets(i).w, ...
        paramSets(i).wdamp, paramSets(i).c1, paramSets(i).c2, ...
        allBestSolutions(i, 1), allBestSolutions(i, 2), allBestCosts(i)};
end

resultsTable = cell2table(allEntries, 'VariableNames', ...
    {'MaxIt', 'nPop', 'w', 'wdamp', 'c1', 'c2', 'Var1', 'Var2', 'BestCost'});

% Write the compiled results to the spreadsheet
writetable(resultsTable, 'PSO_Results.xlsx');

% Plot comparison of Best Costs for all parameter sets
figure; % Create a new figure for comparison plot
PlotBestCostsComparison(allBestCosts, paramSets);
title('Comparison of Best Costs for Different Parameter Sets');

%% Visualisations
% Call the visualisation functions
plotBestCostOverIterations(BestCost);
plotPopulationContour(problem.CostFunction, out.pop, problem.VarMin, problem.VarMax, params.nPop);
animateParticles(particleHistory, BestCosts, params.MaxIt, params.nPop);
PlotBestCostsComparison(allBestCosts, paramSets);