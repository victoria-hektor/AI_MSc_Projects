%% To suppress deprecation warnings
warning('off','fuzzy:general:warnDeprecation_Newfis') 
warning('off','fuzzy:general:warnDeprecation_Addvar')
warning('off','fuzzy:general:warnDeprecation_Addmf')
warning('off','fuzzy:general:warnDeprecation_Evalfis')

%% Clear the Command Window to remove previous runs' clutter
clc

%% Creating a new Fuzzy Inference System (FIS) with specified parameters
%                                       AND OR    Impl Agg  Defuzzification
fis = newfis('DarkMatterDetection','mamdani','min','max', 'min','max','centroid');
%fis = newfis('DarkMatterDetection','mamdani','min','max', 'min','max','mom');
%fis = newfis('DarkMatterDetection','mamdani','min','max', 'min','max','lom');
%fis = newfis('DarkMatterDetection','mamdani','min','max', 'min','max','som');
%fis = newfis('DarkMatterDetection','mamdani','min','max', 'min','max','bisector');

%                                               AND OR    Impl Agg  Defuzzification
%fis = newfis('DarkMatterDetection','mamdani','prod','probor', 'prod','max','centroid');
%fis = newfis('DarkMatterDetection','mamdani','prod','probor', 'prod','max','mom');
%fis = newfis('DarkMatterDetection','mamdani','prod','probor', 'prod','max','lom');
%fis = newfis('DarkMatterDetection','mamdani','prod','probor', 'prod','max','som');
%fis = newfis('DarkMatterDetection','mamdani','prod','probor', 'prod','max','bisector');

% Get the number of input variables in the FIS
numInputs = length(fis.input);

% Initialize a variable to store the index of the 'Angular Resolution' variable
inputIndex = [];

% Iterate through the input variables to find the 'Angular Resolution' variable
for i = 1:numInputs
    if strcmp(fis.input(i).name, 'Angular Resolution (milli-arcseconds)')
        inputIndex = i;
        break;
    end
end

% Check if the 'Angular Resolution' variable was found
if isempty(inputIndex)
    % If it wasn't found, add the variable
    fis = addvar(fis, 'input', 'Angular Resolution (milli-arcseconds)', [0 1]);
    % Retrieve its index
    inputIndex = length(fis.input);
end

% Convert inputIndex to a string
inputIndexStr = num2str(inputIndex);

% Construct the variable name with 'input' prefix
varName = ['input', inputIndexStr];

% Now proceed to add membership functions for 'Angular Resolution'
fis = addmf(fis, varName, 1, 'Low', @wrapAroundMembershipFunction, 'trapmf', [0 0 0.3 0.6]);
fis = addmf(fis, varName, 1, 'Medium', @wrapAroundMembershipFunction, 'trapmf', [0.3 0.5 0.7 0]);
fis = addmf(fis, varName, 1, 'High', @wrapAroundMembershipFunction, 'trapmf', [0.4 0.7 1 0.1]);

%% Adding 'Density of the Gravitational Lens' as an input variable
fis = addvar(fis, 'input', 'Density', [0 1]);

% Defining membership functions for 'Density'
fis = addmf(fis, 'input', 2, 'Low', 'trapmf', [0 0 0.3 0.6]);
fis = addmf(fis, 'input', 2, 'Medium', 'trimf', [0.3 0.5 0.7]);
fis = addmf(fis, 'input', 2, 'High', 'trapmf', [0.4 0.7 1 1]);

%% Adding 'Width and Distribution of Gravitational Arcs' as an input variable
fis = addvar(fis, 'input', 'Width of Arcs', [0 1]);

% Defining membership functions for 'Width of Arcs'
fis = addmf(fis, 'input', 3, 'Narrow', 'trapmf', [0 0 0.3 0.5]);
fis = addmf(fis, 'input', 3, 'Average', 'trimf', [0.3 0.5 0.7]);
fis = addmf(fis, 'input', 3, 'Wide', 'trapmf', [0.5 0.7 1 1]);

%% Adding an output variable for the estimated mass of dark matter
fis = addvar(fis, 'output', 'Estimated Mass (eV)', [0 10^(-21)]);

% Defining membership functions for the output
fis = addmf(fis, 'output', 1, 'Light', 'trapmf', [0 0 2*10^(-22) 5*10^(-22)]);
fis = addmf(fis, 'output', 1, 'Medium', 'trimf', [2*10^(-22) 5*10^(-22) 8*10^(-22)]);
fis = addmf(fis, 'output', 1, 'Heavy', 'trapmf', [5*10^(-22) 8*10^(-22) 10^(-21) 10^(-21)]);

% Full set of rules for the FIS
rules = [
    % Antecedent (Input 1, Input 2, Input 3), Consequent (Output), Weight, Operator (AND=1)
    1 1 1 1 1 1;  % If AR is Low and Density is Low and Width is Narrow, then Mass is Light
    1 1 2 1 1 1;  % If AR is Low and Density is Low and Width is Medium, then Mass is Light
    1 1 3 1 1 1;  % If AR is Low and Density is Low and Width is Wide, then Mass is Light
    1 2 1 2 1 1;  % If AR is Low and Density is Medium and Width is Narrow, then Mass is Medium
    1 2 2 2 1 1;  % If AR is Low and Density is Medium and Width is Medium, then Mass is Medium
    1 2 3 1 1 1;  % If AR is Low and Density is Medium and Width is Wide, then Mass is Light
    1 3 1 3 1 1;  % If AR is Low and Density is High and Width is Narrow, then Mass is Heavy
    1 3 2 2 1 1;  % If AR is Low and Density is High and Width is Medium, then Mass is Medium
    1 3 3 2 1 1;  % If AR is Low and Density is High and Width is Wide, then Mass is Medium
    2 1 1 1 1 1;  % If AR is Medium and Density is Low and Width is Narrow, then Mass is Light
    2 1 2 1 1 1;  % If AR is Medium and Density is Low and Width is Medium, then Mass is Light
    2 1 3 1 1 1;  % If AR is Medium and Density is Low and Width is Wide, then Mass is Light
    2 2 1 2 1 1;  % If AR is Medium and Density is Medium and Width is Narrow, then Mass is Medium
    2 2 2 2 1 1;  % If AR is Medium and Density is Medium and Width is Medium, then Mass is Medium
    2 2 3 2 1 1;  % If AR is Medium and Density is Medium and Width is Wide, then Mass is Medium
    2 3 1 3 1 1;  % If AR is Medium and Density is High and Width is Narrow, then Mass is Heavy
    2 3 2 2 1 1;  % If AR is Medium and Density is High and Width is Medium, then Mass is Medium
    2 3 3 3 1 1;  % If AR is Medium and Density is High and Width is Wide, then Mass is Heavy
    3 1 1 2 1 1;  % If AR is High and Density is Low and Width is Narrow, then Mass is Medium
    3 1 2 1 1 1;  % If AR is High and Density is Low and Width is Medium, then Mass is Light
    3 1 3 1 1 1;  % If AR is High and Density is Low and Width is Wide, then Mass is Light
    3 2 1 3 1 1;  % If AR is High and Density is Medium and Width is Narrow, then Mass is Heavy
    3 2 2 2 1 1;  % If AR is High and Density is Medium and Width is Medium, then Mass is Medium
    3 2 3 2 1 1;  % If AR is High and Density is Medium and Width is Wide, then Mass is Medium
    3 3 1 3 1 1;  % If AR is High and Density is High and Width is Narrow, then Mass is Heavy
    3 3 2 3 1 1;  % If AR is High and Density is High and Width is Medium, then Mass is Heavy
    3 3 3 3 1 1;  % If AR is High and Density is High and Width is Wide, then Mass is Heavy
];

% Add rules to the FIS
fis = addrule(fis, rules);

%% Read, Normalise and save data
% Define the file and sheet name
dataFile = 'DarkMatterData.xlsx';
sheetName = 'Sheet1';

% Read the normalized data directly from the specified columns
normalisedData = readmatrix(dataFile, 'Sheet', sheetName, 'Range', 'B3:F52');

% Debug: Display the size of the normalisedData matrix
disp('Size of normalisedData matrix:');
disp(size(normalisedData));

% Extract only the normalized columns, which are columns 1, 3, and 5 of the normalisedData matrix
inputData = normalisedData(:, [1, 3, 5]);

% Debug: Display the inputData to ensure it's correct
disp('Input data to be used:');
disp(inputData);

%% Defuzz Method
% Now inputData contains normalised values that you can pass to your FIS
evaluateAndWriteFIS(fis, inputData, dataFile, sheetName, 'H3');

%% Visualising the system - for a detailed analysis and debugging
figure('Name','Input 1 - Angular Resolution');
subplot(3,1,1), plotmf(fis, 'input', 1);
title('Angular Resolution Membership Functions');

subplot(3,1,2), plotmf(fis, 'input', 2);
title('Density Membership Functions');

subplot(3,1,3), plotmf(fis, 'input', 3);
title('Width of Arcs Membership Functions');

figure('Name','Output - Estimated Mass');
subplot(1,1,1), plotmf(fis, 'output', 1);
title('Estimated Mass Membership Functions');

%% Surf Plot
% Define the fixed width value
fixedWidthValue = 0.5; % adjust as necessary [0 1]

% Call the generateSurfPlot function
generateSurfPlot(fis, fixedWidthValue);

%% Rule Viewer Plot
ruleview(fis);

%% Sensitivity Analysis:
% Define ranges for each input variable with appropriate resolution
arRange = linspace(0, 0.4, 100);  % Will generate 100 points
densityRange = linspace(0, 0.3, 1);  % Assuming constant for analysis
widthRange = linspace(0, 0.2, 1);  % Assuming constant for analysis

% Keeping density and width constant at their median values
densityMedian = median(densityRange);
widthMedian = median(widthRange);

% Evaluate the FIS output while varying Angular Resolution only
output = zeros(size(arRange));  % Pre-allocate for performance
for i = 1:length(arRange)
    output(i) = evalfis([arRange(i), densityMedian, widthMedian], fis);
end

% Plot the results
figure;
plot(arRange, output);
xlabel('Angular Resolution (normalised)');
ylabel('FIS Output (eV)');
title('Sensitivity Analysis for Angular Resolution');

%% Principal Component Analysis (PCA):
% Prepare:
% Define the minimum and maximum values for each input
minAngularResolution = 0;  
maxAngularResolution = 1; 
minDensity = 0;            
maxDensity = 1;          
minWidth = 0;              
maxWidth = 1;              

% Define the range for each input
numSamples = 100; % For example, 100 samples for each input
angularResolutionRange = linspace(minAngularResolution, maxAngularResolution, numSamples);
densityRange = linspace(minDensity, maxDensity, numSamples);
widthRange = linspace(minWidth, maxWidth, numSamples);

% Generate a grid of input combinations
[ARgrid, DensityGrid, WidthGrid] = ndgrid(angularResolutionRange, densityRange, widthRange);

% Flatten the grids into vectors for evalfis
ARvector = ARgrid(:);
DensityVector = DensityGrid(:);
WidthVector = WidthGrid(:);

% Combine into a single matrix for evalfis
inputMatrix = [ARvector, DensityVector, WidthVector];

% Evaluate the FIS for each combination of inputs
outputVector = evalfis(fis, inputMatrix);

% Specify the file and sheet name for reading normalized data
dataFile = 'DarkMatterData.xlsx';
sheetName = 'Sheet1';

% Read the normalised data from specified columns and rows
normalisedData = readmatrix(dataFile, 'Sheet', sheetName, 'Range', 'B3:F52', 'UseExcel', false); % 'H3:L52'

% Extract columns B, D, and F only for PCA
inputDataForPCA = normalisedData(:, [1, 3, 5]); % 1, 2, 3, 4, 5

% Execute: 
% where the last column is the output
% coeff contains the coefficients for each principal component
% score contains the observations projected into the principal component space
% latent contains the variance explained by each principal component
[coeff, score, latent] = pca(inputDataForPCA);

% Plot the variance explained by each principal component
figure;
explainedVar = cumsum(latent)./sum(latent) * 100;
bar(explainedVar);
xlabel('Principal Components');
ylabel('Variance Explained (%)');
title('PCA Variance Explained');