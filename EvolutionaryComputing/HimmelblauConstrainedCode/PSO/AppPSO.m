clc; clear;

%% Problem Definition
problem.CostFunction = @(x) Himmelblau_Constrained(x);  % Problem function
problem.nVar = 2;
problem.VarMin = [-15, -15];  % Lower Bound for each variable
problem.VarMax = [15, 15];    % Upper Bound for each variable
globalOptimumCost = 0;

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

% Initialise the `results` structure
results = struct();

% Define the number of runs you want to perform
numberOfRuns = 1;

% Array of parameter sets for different runs
paramSets = [
     struct('MaxIt', 1000, 'nPop', 250, 'w', 0.20, 'wdamp', 0.20, 'c1', 0.10, 'c2', 0.25, 'ShowIterInfo', true);
     struct('MaxIt', 1000, 'nPop', 100, 'w', 0.40, 'wdamp', 0.45, 'c1', 0.30, 'c2', 0.50, 'ShowIterInfo', true);
     struct('MaxIt', 1000, 'nPop', 70, 'w', 0.60, 'wdamp', 0.65, 'c1', 0.50, 'c2', 0.75, 'ShowIterInfo', true);
     struct('MaxIt', 1000, 'nPop', 40, 'w', 0.80, 'wdamp', 0.79, 'c1', 0.70, 'c2', 0.85, 'ShowIterInfo', true);
     struct('MaxIt', 1000, 'nPop', 150, 'w', 1.00, 'wdamp', 0.99, 'c1', 0.90, 'c2', 0.99, 'ShowIterInfo', true);
];

% Initialise arrays to store results
allBestSolutions = zeros(length(paramSets), problem.nVar);
allBestCosts = zeros(length(paramSets), numberOfRuns);

% Loop through each parameter set
for runIndex = 1:numberOfRuns
    for i = 1:length(paramSets)
        params = paramSets(i);
        
        % PSO Main Body
        out = PSO(problem, params);  % Use the PSO function with the current parameter set
        
        % Store Results
        allBestSolutions(i, :) = out.BestSol.Position;
        allBestCosts(i, runIndex) = out.BestSol.Cost;
        BestSol = out.BestSol;
        BestCosts = out.BestCosts;
        particle_history = out.particle_history;
        particleHistory = out.particleHistory;
        allVelocityHistory = out.velocityHistory;  % Store the velocity history for this run
        
        % Tolerance Check
        error = max(abs(out.BestSol.Position - globalMin));
        if error < tolerance
            fprintf('Run %d: Solution found within the acceptable tolerance.\n', i);
        else
            fprintf('Run %d: Best Cost = %.4f, not within the acceptable tolerance after %d iterations.\n', i, allBestCosts(i), params.MaxIt);
        end
        
        % Save Results to Spreadsheet for each parameter set
        saveHistoricalData(out.particleHistory, sprintf('historicalData_Run%d.xlsx', i));
               
        figure; % Create a new figure for contour plot
        plotPopulationContour(problem.CostFunction, out.pop, problem.VarMin, problem.VarMax, params.nPop);
        title(sprintf('Population Contour (Run %d)', i));
        drawnow; % Ensure plots are updated before the next iteration
    
        runID = sprintf('run%d', runIndex);
        results.(runID) = out;

    end
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

% Debug:
disp(size(allBestCosts));
disp(allBestCosts);

%% Visualisations
plotVelocityMagnitudes(out.velocityHistory);
plotDiversity(particle_history);
plotConvergence(BestCosts, globalOptimumCost);
plotPopulationContour(problem.CostFunction, out.pop, problem.VarMin, problem.VarMax, params.nPop);
% Call the animateParticles function
animateParticles(particleHistory, BestCosts, params.MaxIt, params.nPop);