%% To suppress deprecation warnings
warning('off','fuzzy:general:warnDeprecation_Newfis') 
warning('off','fuzzy:general:warnDeprecation_Addvar')
warning('off','fuzzy:general:warnDeprecation_Addmf')
warning('off','fuzzy:general:warnDeprecation_Evalfis')

%% Clear the Command Window to remove previous runs' clutter
clc

%% Data Setup
% File and Sheet Setup
dataFile = 'pseudo_patient_data_1000dp.xlsx';
sheetName = 'Sheet1'; 

% Read Data Once
inputData = readmatrix(dataFile, 'Sheet', sheetName, 'Range', 'A2:C1001');
disp('Size of inputData matrix:');
disp(size(inputData));

%% Define the standard deviation and mean for the Gaussian membership functions
% The standard deviation controls the width of the curve
% The mean determines the centre of the curve

% values for 'Very Unlikely' outcome
sigma_very_unlikely = 1;
mean_very_unlikely = 10;

% values for 'Unlikely' outcome
sigma_unlikely = 1;
mean_unlikely = 30;

% values for 'Possible' outcome
sigma_possible = 1;
mean_possible = 50;

% Example values for 'Probable' outcome
sigma_probable = 1;
mean_probable = 70;

% values for 'Very Likely' outcome
sigma_very_likely = 1;
mean_very_likely = 90;

% Compute Mean Input for Use in Visualisations
meanInput = mean(inputData);

%% Creating a new Fuzzy Inference System (FIS) with specified parameters
%                                       AND OR    Impl Agg  Defuzzification
%fis = newfis('MedicalDiagnosis','mamdani','min','max', 'min','max','centroid');
%fis = newfis('MedicalDiagnosis','mamdani','min','max', 'min','max','mom');
%fis = newfis('MedicalDiagnosis','mamdani','min','max', 'min','max','lom');
fis = newfis('MedicalDiagnosis','mamdani','min','max', 'min','max','som');
%fis = newfis('MedicalDiagnosis','mamdani','min','max', 'min','max','bisector');

%                                               AND OR    Impl Agg  Defuzzification
%fis = newfis('MedicalDiagnosis','mamdani','prod','probor', 'prod','max','centroid');
%fis = newfis('MedicalDiagnosis','mamdani','prod','probor', 'prod','max','mom');
%fis = newfis('MedicalDiagnosis','mamdani','prod','probor', 'prod','max','lom');
%fis = newfis('MedicalDiagnosis','mamdani','prod','probor', 'prod','max','som');
%fis = newfis('MedicalDiagnosis','mamdani','prod','probor', 'prod','max','bisector');

%% Adding 'Symptom Severity' as an input variable
fis = addvar(fis, 'input', 'Symptom Severity', [0 10]);

% Defining more nuanced membership functions for 'Symptom Severity'
fis = addmf(fis, 'input', 1, 'Very Low', 'gaussmf', [1 0]);
fis = addmf(fis, 'input', 1, 'Low', 'gaussmf', [1 2.5]);
fis = addmf(fis, 'input', 1, 'Medium Low', 'gaussmf', [1 5]);
fis = addmf(fis, 'input', 1, 'Medium High', 'gaussmf', [1 7.5]);
fis = addmf(fis, 'input', 1, 'High', 'gaussmf', [1 10]);

%% Adding 'Test Result Consistency' as an input variable
fis = addvar(fis, 'input', 'Test Result Consistency', [0 10]);

% Defining more nuanced membership functions for 'Test Result Consistency'
fis = addmf(fis, 'input', 2, 'Very Inconsistent', 'gaussmf', [1 0]);
fis = addmf(fis, 'input', 2, 'Inconsistent', 'gaussmf', [1 2.5]);
fis = addmf(fis, 'input', 2, 'Moderately Consistent', 'gaussmf', [1 5]);
fis = addmf(fis, 'input', 2, 'Consistent', 'gaussmf', [1 7.5]);
fis = addmf(fis, 'input', 2, 'Very Consistent', 'gaussmf', [1 10]);

%% Adding 'Patient History Relevance' as an input variable
fis = addvar(fis, 'input', 'Patient History Relevance', [0 10]);

% Defining more nuanced membership functions for 'Patient History Relevance'
fis = addmf(fis, 'input', 3, 'Very Irrelevant', 'gaussmf', [1 0]);
fis = addmf(fis, 'input', 3, 'Irrelevant', 'gaussmf', [1 2.5]);
fis = addmf(fis, 'input', 3, 'Somewhat Relevant', 'gaussmf', [1 5]);
fis = addmf(fis, 'input', 3, 'Relevant', 'gaussmf', [1 7.5]);
fis = addmf(fis, 'input', 3, 'Highly Relevant', 'gaussmf', [1 10]);

%% Define output variable with appropriate membership functions
fis = addvar(fis, 'output', 'Likelihood of Disease Diagnosis', [0 100]);

% Adding membership functions for 'Likelihood of Disease Diagnosis' output variable
fis = addmf(fis, 'output', 1, 'Very Unlikely', 'gaussmf', [sigma_very_unlikely mean_very_unlikely]);
fis = addmf(fis, 'output', 1, 'Unlikely', 'gaussmf', [sigma_unlikely mean_unlikely]);
fis = addmf(fis, 'output', 1, 'Possible', 'gaussmf', [sigma_possible mean_possible]);
fis = addmf(fis, 'output', 1, 'Probable', 'gaussmf', [sigma_probable mean_probable]);
fis = addmf(fis, 'output', 1, 'Very Likely', 'gaussmf', [sigma_very_likely mean_very_likely]);

%% Full set of rules for the Medical Diagnosis FIS
% Define a matrix to store all the rules
numLevels = 5; % Number of levels for each input and output variable
numInputs = 3; % Number of input variables
rules = zeros(numLevels^numInputs, numInputs+3); % Preallocate the rules matrix

% Generate all possible combinations of rules
ruleCounter = 1;
for i = 1:numLevels % Symptom Severity levels
    for j = 1:numLevels % Test Result Consistency levels
        for k = 1:numLevels % Patient History Relevance levels
            % The consequent (output) can be decided based on a strategy.
            % Here, we take a simple average of the input levels
            consequent = round(mean([i, j, k])); 
            rules(ruleCounter, :) = [i j k consequent 1 1]; % Last two are weight and operator
            ruleCounter = ruleCounter + 1;
        end
    end
end

% Add rules to FIS
fis = addrule(fis, rules);

%% Evaluate FIS and Write Results
evaluateAndWriteFIS(fis, inputData, dataFile, sheetName, 'E2');

%% Visualising the system - for a detailed analysis and debugging
figure('Name','Input - Symptom Severity');
subplot(3,1,1), plotmf(fis, 'input', 1);
title('Symptom Severity Membership Functions');

subplot(3,1,2), plotmf(fis, 'input', 2);
title('Test Result Consistency Membership Functions');

subplot(3,1,3), plotmf(fis, 'input', 3);
title('Patient History Relevance Membership Functions');

figure('Name','Output - Likelihood of Disease Diagnosis');
subplot(1,1,1), plotmf(fis, 'output', 1);
title('Likelihood of Disease Diagnosis Membership Functions');

%% Surface Plot
figure;
[X, Y] = meshgrid(linspace(0, 10, 50), linspace(0, 10, 50)); 
Z = zeros(size(X));
for i = 1:numel(X)
    Z(i) = evalfis([X(i), Y(i), meanInput(3)], fis);
end
surf(X, Y, Z);
xlabel('Symptom Severity'); ylabel('Test Result Consistency'); zlabel('FIS Output');
title('FIS Output Surface');

%% Rule Viewer Plot
figure;
ruleview(fis);

%% Sensitivity Analysis
symptomSeverityRange = linspace(0, 10, 100);
fisOutputForSensitivity = arrayfun(@(x) evalfis([x, meanInput(2:3)], fis), symptomSeverityRange);
figure;
plot(symptomSeverityRange, fisOutputForSensitivity);
title('Sensitivity Analysis for Symptom Severity');
xlabel('Symptom Severity'); ylabel('Likelihood of Disease Diagnosis');

%% Principal Component Analysis (PCA)
standardisedData = (inputData - mean(inputData)) ./ std(inputData);
[coeff, score, latent] = pca(standardisedData);
explainedVariance = cumsum(latent) / sum(latent) * 100;
figure; bar(explainedVariance);
title('PCA - Explained Variance');
xlabel('Principal Components'); ylabel('Percentage of Variance Explained');
figure; scatter(score(:,1), score(:,2));
title('PCA - Scatter Plot of the First Two Principal Components');
xlabel('Principal Component 1'); ylabel('Principal Component 2');

%% Function to Evaluate FIS and Write Results
function evaluateAndWriteFIS(fis, inputData, dataFile, sheetName, outputStartCell)
    defuzzMethods = {'centroid', 'mom', 'lom', 'som', 'bisector'};
    numDataPoints = size(inputData, 1);
    results = zeros(numDataPoints, numel(defuzzMethods));
    for i = 1:numel(defuzzMethods)
        fis.DefuzzificationMethod = defuzzMethods{i};
        results(:, i) = evalfis(inputData(:, 1:3), fis);
    end
    % Writing results to Excel
    endRow = num2str(numDataPoints + 1);
    outputRange = [outputStartCell ':' char('E' + numel(defuzzMethods) - 1) endRow];
    writematrix(results, dataFile, 'Sheet', sheetName, 'Range', outputRange);
end