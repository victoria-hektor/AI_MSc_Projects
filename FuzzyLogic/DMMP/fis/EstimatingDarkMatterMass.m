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

%% Adding 'Galaxy Rotation' as an input variable
fis = addvar(fis, 'input', 'Galaxy Rotation', [0 1]);

% Defining membership functions for 'Galaxy Rotation'
fis = addmf(fis, 'input', 1, 'Low', 'trapmf', [0 0 0.35 0.6]);
fis = addmf(fis, 'input', 1, 'Medium', 'trimf', [0.25 0.6 0.8]);
fis = addmf(fis, 'input', 1, 'High', 'trapmf', [0.4 0.85 1 1]);

%% Adding 'Gravitational Lensing Effects' as an input variable
fis = addvar(fis, 'input', 'Gravitational Lensing Effects', [0 1]);

% Defining membership functions for 'Gravitational Lensing Effects'
fis = addmf(fis, 'input', 2, 'Low', 'trapmf', [0 0 0.35 0.6]);
fis = addmf(fis, 'input', 2, 'Medium', 'trimf', [0.25 0.6 0.8]);
fis = addmf(fis, 'input', 2, 'High', 'trapmf', [0.4 0.85 1 1]);

%% Adding 'CMB Measurements' as an input variable
fis = addvar(fis, 'input', 'CMB Measurements', [0 1]);

% Defining membership functions for 'CMB Measurements'
fis = addmf(fis, 'input', 3, 'Low', 'trapmf', [0 0 0.35 0.6]);
fis = addmf(fis, 'input', 3, 'Medium', 'trimf', [0.25 0.6 0.8]);
fis = addmf(fis, 'input', 3, 'High', 'trapmf', [0.4 0.85 1 1]);

%% Adding an output variable for the estimated mass of dark matter
fis = addvar(fis, 'output', 'Estimated Mass (eV)', [0 10^(-21)]);

% Defining membership functions for the output
fis = addmf(fis, 'output', 1, 'Light', 'trapmf', [0 0 2*10^(-22) 5*10^(-22)]);
fis = addmf(fis, 'output', 1, 'Medium', 'trimf', [2*10^(-22) 5*10^(-22) 8*10^(-22)]);
fis = addmf(fis, 'output', 1, 'Heavy', 'trapmf', [5*10^(-22) 8*10^(-22) 10^(-21) 10^(-21)]);

% Full set of rules for the FIS
rules = [
    % Antecedent (Input 1, Input 2, Input 3), Consequent (Output), Weight, Operator (AND=1)
    1 1 1 1 1 1; % Rule 1: If galaxy rotation is low AND lensing effects are low AND CMB is low, THEN mass is light.
    1 1 2 1 1 1; % Rule 2: If galaxy rotation is low AND lensing effects are low AND CMB is medium, THEN mass is light.
    1 1 3 1 1 1; % Rule 3: If galaxy rotation is low AND lensing effects are low AND CMB is high, THEN mass is light.
    1 2 1 1 1 1; % Rule 4: If galaxy rotation is low AND lensing effects are medium AND CMB is low, THEN mass is light.
    1 2 2 2 1 1; % Rule 5: If galaxy rotation is low AND lensing effects are medium AND CMB is medium, THEN mass is medium.
    1 2 3 1 1 1; % Rule 6: If galaxy rotation is low AND lensing effects are medium AND CMB is high, THEN mass is light.
    1 3 1 2 1 1; % Rule 7: If galaxy rotation is low AND lensing effects are high AND CMB is low, THEN mass is medium.
    1 3 2 2 1 1; % Rule 8: If galaxy rotation is low AND lensing effects are high AND CMB is medium, THEN mass is medium.
    1 3 3 2 1 1; % Rule 9: If galaxy rotation is low AND lensing effects are high AND CMB is high, THEN mass is medium.
    2 1 1 1 1 1; % Rule 10: If galaxy rotation is medium AND lensing effects are low AND CMB is low, THEN mass is light.
    2 1 2 1 1 1; % Rule 11: If galaxy rotation is medium AND lensing effects are low AND CMB is medium, THEN mass is light.
    2 1 3 1 1 1; % Rule 12: If galaxy rotation is medium AND lensing effects are low AND CMB is high, THEN mass is light.
    2 2 1 2 1 1; % Rule 13: If galaxy rotation is medium AND lensing effects are medium AND CMB is low, THEN mass is medium.
    2 2 2 2 1 1; % Rule 14: If galaxy rotation is medium AND lensing effects are medium AND CMB is medium, THEN mass is medium.
    2 2 3 2 1 1; % Rule 15: If galaxy rotation is medium AND lensing effects are medium AND CMB is high, THEN mass is medium.
    2 3 1 3 1 1; % Rule 16: If galaxy rotation is medium AND lensing effects are high AND CMB is low, THEN mass is heavy.
    2 3 2 2 1 1; % Rule 17: If galaxy rotation is medium AND lensing effects are high AND CMB is medium, THEN mass is medium.
    2 3 3 3 1 1; % Rule 18: If galaxy rotation is medium AND lensing effects are high AND CMB is high, THEN mass is heavy.
    3 1 1 2 1 1; % Rule 19: If galaxy rotation is high AND lensing effects are low AND CMB is low, THEN mass is medium.
    3 1 2 1 1 1; % Rule 20: If galaxy rotation is high AND lensing effects are low AND CMB is medium, THEN mass is light.
    3 1 3 1 1 1; % Rule 21: If galaxy rotation is high AND lensing effects are low AND CMB is high, THEN mass is light.
    3 2 1 3 1 1; % Rule 22: If galaxy rotation is high AND lensing effects are medium AND CMB is low, THEN mass is heavy.
    3 2 2 2 1 1; % Rule 23: If galaxy rotation is high AND lensing effects are medium AND CMB is medium, THEN mass is medium.
    3 2 3 2 1 1; % Rule 24: If galaxy rotation is high AND lensing effects are medium AND CMB is high, THEN mass is medium.
    3 3 1 3 1 1; % Rule 25: If galaxy rotation is high AND lensing effects are high AND CMB is low, THEN mass is heavy.
    3 3 2 3 1 1; % Rule 26: If galaxy rotation is high AND lensing effects are high AND CMB is medium, THEN mass is heavy.
    3 3 3 3 1 1; % Rule 27: If galaxy rotation is high AND lensing effects are high AND CMB is high, THEN mass is heavy.
];

% Add rules to the FIS
fis = addrule(fis, rules);

%% Read and save data
% Define the file and sheet name
dataFile = 'astronomy_data.xlsx';
sheetName = 'Sheet1';

% Read the data directly from the specified columns
data = readmatrix(dataFile, 'Sheet', sheetName, 'Range', 'A2:C152');

% Debug: Display the size of the data matrix
disp('Size of data matrix:');
disp(size(data));

% row of inputs and writes the defuzzified outputs to the specified Excel file.
evaluateAndWriteFIS(fis, data, dataFile, sheetName, 'E2:I152');

%% Visualising the system - for detailed analysis and debugging
figure('Name','Input Visualisation');
subplot(3,1,1), plotmf(fis, 'input', 1);
title('Galaxy Rotation Curve Membership Functions');

subplot(3,1,2), plotmf(fis, 'input', 2);
title('Cosmic Microwave Background Membership Functions');

subplot(3,1,3), plotmf(fis, 'input', 3);
title('Gravitational Lensing Membership Functions');

figure('Name','Output Visualisation');
subplot(1,1,1), plotmf(fis, 'output', 1);
title('Dark Matter Mass Estimation Membership Functions');

%% Surf Plot
% Define the fixed width value
fixedWidthValue = 0.5; % adjust as necessary [0 1]

% Call the generateSurfPlot function
generateSurfPlot(fis, fixedWidthValue);

%% Rule Viewer Plot
ruleview(fis);

%% Sensitivity Analysis for new input variables
% Define ranges for each input variable with appropriate resolution
rotationCurveRange = linspace(0, 1, 100);  % Range for Galaxy Rotation Curve
cmbRange = linspace(0, 1, 100);           % Range for Cosmic Microwave Background
lensingWidthRange = linspace(0, 1, 100);  % Range for Gravitational Lensing Width

% Evaluate the FIS output while varying each input individually
rotationCurveOutput = zeros(size(rotationCurveRange));
cmbOutput = zeros(size(cmbRange));
lensingWidthOutput = zeros(size(lensingWidthRange));

% Keeping the other two inputs at their median values
for i = 1:length(rotationCurveRange)
    rotationCurveOutput(i) = evalfis(fis, [rotationCurveRange(i), median(cmbRange), median(lensingWidthRange)]);
    cmbOutput(i) = evalfis(fis, [median(rotationCurveRange), cmbRange(i), median(lensingWidthRange)]);
    lensingWidthOutput(i) = evalfis(fis, [median(rotationCurveRange), median(cmbRange), lensingWidthRange(i)]);
end

% Plot the results for Galaxy Rotation Curve
figure;
plot(rotationCurveRange, rotationCurveOutput);
xlabel('Galaxy Rotation Curve');
ylabel('FIS Output (eV)');
title('Sensitivity Analysis for Galaxy Rotation Curve');

% Plot the results for Cosmic Microwave Background
figure;
plot(cmbRange, cmbOutput);
xlabel('Cosmic Microwave Background');
ylabel('FIS Output (eV)');
title('Sensitivity Analysis for Cosmic Microwave Background');

% Plot the results for Gravitational Lensing Width
figure;
plot(lensingWidthRange, lensingWidthOutput);
xlabel('Gravitational Lensing Width');
ylabel('FIS Output (eV)');
title('Sensitivity Analysis for Gravitational Lensing Width');

%% Principal Component Analysis (PCA):
% Prepare:
% Define the minimum and maximum values for each input
minGalaxyVelocity = 0;
maxGalaxyVelocity = 1;
minLensingMass = 0;
maxLensingMass = 1;
minCosmicShear = 0;
maxCosmicShear = 1;

% Define the range for each input
numSamples = 150; 
galaxyVelocityRange = linspace(minGalaxyVelocity, maxGalaxyVelocity, numSamples);
lensingMassRange = linspace(minLensingMass, maxLensingMass, numSamples);
cosmicShearRange = linspace(minCosmicShear, maxCosmicShear, numSamples);

% Generate a grid of input combinations
[VelocityGrid, MassGrid, ShearGrid] = ndgrid(galaxyVelocityRange, lensingMassRange, cosmicShearRange);

% Flatten the grids into vectors for evalfis
VelocityVector = VelocityGrid(:);
MassVector = MassGrid(:);
ShearVector = ShearGrid(:);

% Combine into a single matrix for evalfis
inputMatrix = [VelocityVector, MassVector, ShearVector];

% Evaluate the FIS for each combination of inputs
outputVector = evalfis(fis, inputMatrix);

% Specify the file and sheet name for reading normalised data
dataFile = 'astronomy_data.xlsx';
sheetName = 'Sheet1';

% Read the normalised data from specified columns and rows
normalisedData = readmatrix(dataFile, 'Sheet', sheetName, 'Range', 'A2:C152', 'UseExcel', false);

% Extract the relevant columns for PCA
inputDataForPCA = normalisedData(:, [1, 2, 3]); % Assuming the first three columns are for PCA

% Execute:
% Compute PCA on the inputDataForPCA
[coeff, score, latent] = pca(inputDataForPCA);

% Plot the variance explained by each principal component
figure;
explainedVar = cumsum(latent)./sum(latent) * 100;
bar(explainedVar);
xlabel('Principal Components');
ylabel('Variance Explained (%)');
title('PCA Variance Explained');