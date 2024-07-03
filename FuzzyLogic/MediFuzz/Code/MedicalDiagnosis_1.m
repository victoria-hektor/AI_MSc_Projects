%% To suppress deprecation warnings
warning('off','fuzzy:general:warnDeprecation_Newfis') 
warning('off','fuzzy:general:warnDeprecation_Addvar')
warning('off','fuzzy:general:warnDeprecation_Addmf')
warning('off','fuzzy:general:warnDeprecation_Evalfis')

%% Clear the Command Window to remove previous runs' clutter
clc

%% Data Setup
% File and Sheet Setup
dataFile = 'rule_activation_data.xlsx';
sheetName = 'Sheet1'; 

% Read Data Once
inputData = readmatrix(dataFile, 'Sheet', sheetName, 'Range', 'A2:C126');
disp('Size of inputData matrix:');
disp(size(inputData));

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

% Adding 'Symptom Severity' as an input variable
fis = addvar(fis, 'input', 'Symptom Severity', [0 10]);

% Defining triangular membership functions for 'Symptom Severity'
fis = addmf(fis, 'input', 1, 'Very Low', 'trapmf', [0 0 1 3]);
fis = addmf(fis, 'input', 1, 'Low', 'trapmf', [2 3 4 6]);
fis = addmf(fis, 'input', 1, 'Medium', 'trapmf', [5 6 7 8]);
fis = addmf(fis, 'input', 1, 'High', 'trapmf', [7 8 9 10]);
fis = addmf(fis, 'input', 1, 'Very High', 'trapmf', [9 10 10 10]);

% Adding 'Test Result Consistency' as an input variable
fis = addvar(fis, 'input', 'Test Result Consistency', [0 10]);

% Defining triangular membership functions for 'Test Result Consistency'
fis = addmf(fis, 'input', 2, 'Very Inconsistent', 'trapmf', [0 0 1 3]);
fis = addmf(fis, 'input', 2, 'Inconsistent', 'trapmf', [2 3 4 6]);
fis = addmf(fis, 'input', 2, 'Moderately Consistent', 'trapmf', [5 6 7 8]);
fis = addmf(fis, 'input', 2, 'Consistent', 'trapmf', [7 8 9 10]);
fis = addmf(fis, 'input', 2, 'Very Consistent', 'trapmf', [9 10 10 10]);

% Adding 'Patient History Relevance' as an input variable
fis = addvar(fis, 'input', 'Patient History Relevance', [0 10]);

% Defining triangular membership functions for 'Patient History Relevance'
fis = addmf(fis, 'input', 3, 'Very Irrelevant', 'trapmf', [0 0 1 3]);
fis = addmf(fis, 'input', 3, 'Irrelevant', 'trapmf', [2 3 4 6]);
fis = addmf(fis, 'input', 3, 'Somewhat Relevant', 'trapmf', [5 6 7 8]);
fis = addmf(fis, 'input', 3, 'Relevant', 'trapmf', [7 8 9 10]);
fis = addmf(fis, 'input', 3, 'Highly Relevant', 'trapmf', [9 10 10 10]);

%% Adding 'Likelihood of Disease Diagnosis' as an output variable
fis = addvar(fis, 'output', 'Likelihood of Disease Diagnosis', [0 100]);

% Defining membership functions for the output
fis = addmf(fis, 'output', 1, 'Very Unlikely', 'trapmf', [0 0 10 30]);
fis = addmf(fis, 'output', 1, 'Unlikely', 'trimf', [20 40 60]);
fis = addmf(fis, 'output', 1, 'Possible', 'trimf', [50 60 70]);
fis = addmf(fis, 'output', 1, 'Likely', 'trimf', [60 80 90]);
fis = addmf(fis, 'output', 1, 'Very Likely', 'trapmf', [80 90 100 100]);

% Full set of rules for the Medical Diagnosis FIS - set 1
%  rules = [
%      % Antecedent (Input 1, Input 2, Input 3), Consequent (Output), Weight, Operator (AND=1)
%      1 1 1 1 1 1; % If Symptom Severity is Low and Test Result Consistency is Inconsistent and Patient History Relevance is Irrelevant, then Likelihood of Disease Diagnosis is Unlikely
%      1 1 2 1 1 1; % If Symptom Severity is Low and Test Result Consistency is Inconsistent and Patient History Relevance is Somewhat Relevant, then Likelihood of Disease Diagnosis is Unlikely
%      1 1 3 2 1 1; % If Symptom Severity is Low and Test Result Consistency is Inconsistent and Patient History Relevance is Highly Relevant, then Likelihood of Disease Diagnosis is Possible
%      1 2 1 1 1 1; % If Symptom Severity is Low and Test Result Consistency is Moderately Consistent and Patient History Relevance is Irrelevant, then Likelihood of Disease Diagnosis is Unlikely
%      1 2 2 2 1 1; % If Symptom Severity is Low and Test Result Consistency is Moderately Consistent and Patient History Relevance is Somewhat Relevant, then Likelihood of Disease Diagnosis is Possible
%      1 2 3 2 1 1; % If Symptom Severity is Low and Test Result Consistency is Moderately Consistent and Patient History Relevance is Highly Relevant, then Likelihood of Disease Diagnosis is Possible
%      1 3 1 1 1 1; % If Symptom Severity is Low and Test Result Consistency is Consistent and Patient History Relevance is Irrelevant, then Likelihood of Disease Diagnosis is Unlikely
%      1 3 2 2 1 1; % If Symptom Severity is Low and Test Result Consistency is Consistent and Patient History Relevance is Somewhat Relevant, then Likelihood of Disease Diagnosis is Possible
%      1 3 3 3 1 1; % If Symptom Severity is Low and Test Result Consistency is Consistent and Patient History Relevance is Highly Relevant, then Likelihood of Disease Diagnosis is Probable
%      2 1 1 1 1 1; % If Symptom Severity is Moderate and Test Result Consistency is Inconsistent and Patient History Relevance is Irrelevant, then Likelihood of Disease Diagnosis is Unlikely
%      2 1 2 2 1 1; % If Symptom Severity is Moderate and Test Result Consistency is Inconsistent and Patient History Relevance is Somewhat Relevant, then Likelihood of Disease Diagnosis is Possible
%      2 1 3 2 1 1; % If Symptom Severity is Moderate and Test Result Consistency is Inconsistent and Patient History Relevance is Highly Relevant, then Likelihood of Disease Diagnosis is Possible
%      2 2 1 2 1 1; % If Symptom Severity is Moderate and Test Result Consistency is Moderately Consistent and Patient History Relevance is Irrelevant, then Likelihood of Disease Diagnosis is Possible
%      2 2 2 3 1 1; % If Symptom Severity is Moderate and Test Result Consistency is Moderately Consistent and Patient History Relevance is Somewhat Relevant, then Likelihood of Disease Diagnosis is Probable
%      2 2 3 3 1 1; % If Symptom Severity is Moderate and Test Result Consistency is Moderately Consistent and Patient History Relevance is Highly Relevant, then Likelihood of Disease Diagnosis is Probable
%      2 3 1 2 1 1; % If Symptom Severity is Moderate and Test Result Consistency is Consistent and Patient History Relevance is Irrelevant, then Likelihood of Disease Diagnosis is Possible
%      2 3 2 3 1 1; % If Symptom Severity is Moderate and Test Result Consistency is Consistent and Patient History Relevance is Somewhat Relevant, then Likelihood of Disease Diagnosis is Probable
%      2 3 3 4 1 1; % If Symptom Severity is Moderate and Test Result Consistency is Consistent and Patient History Relevance is Highly Relevant, then Likelihood of Disease Diagnosis is Highly Probable
%      3 1 1 2 1 1; % If Symptom Severity is High and Test Result Consistency is Inconsistent and Patient History Relevance is Irrelevant, then Likelihood of Disease Diagnosis is Possible
%      3 1 1 2 1 1;  % If Symptom Severity is High and Test Result Consistency is Low and Patient History Relevance is Irrelevant, then Likelihood of Disease Diagnosis is Possible
%      3 1 2 1 1 1;  % If Symptom Severity is High and Test Result Consistency is Low and Patient History Relevance is Somewhat Relevant, then Likelihood of Disease Diagnosis is Possible
%      3 1 3 1 1 1;  % If Symptom Severity is High and Test Result Consistency is Low and Patient History Relevance is Highly Relevant, then Likelihood of Disease Diagnosis is Possible
%      3 2 1 3 1 1;  % If Symptom Severity is High and Test Result Consistency is Moderate and Patient History Relevance is Irrelevant, then Likelihood of Disease Diagnosis is Probable
%      3 2 2 2 1 1;  % If Symptom Severity is High and Test Result Consistency is Moderate and Patient History Relevance is Somewhat Relevant, then Likelihood of Disease Diagnosis is Probable
%      3 2 3 2 1 1;  % If Symptom Severity is High and Test Result Consistency is Moderate and Patient History Relevance is Highly Relevant, then Likelihood of Disease Diagnosis is Probable
%      3 3 1 3 1 1;  % If Symptom Severity is High and Test Result Consistency is High and Patient History Relevance is Irrelevant, then Likelihood of Disease Diagnosis is Highly Probable
%      3 3 2 3 1 1;  % If Symptom Severity is High and Test Result Consistency is High and Patient History Relevance is Somewhat Relevant, then Likelihood of Disease Diagnosis is Highly Probable
%      3 3 3 3 1 1;  % If Symptom Severity is High and Test Result Consistency is High and Patient History Relevance is Highly Relevant, then Likelihood of Disease Diagnosis is Highly Probable
%  ];

% Full set of revised rules for the Medical Diagnosis FIS - Set 2
% revised_rules = [
%     % Antecedent (Input 1, Input 2, Input 3), Consequent (Output), Weight, Operator (AND=1)
%     1 1 1 1 2 1; % If Symptom Severity is Low and Test Result Consistency is Inconsistent and Patient History Relevance is Irrelevant, then Likelihood of Disease Diagnosis is Unlikely
%     1 1 2 2 2 1; % If Symptom Severity is Low and Test Result Consistency is Inconsistent and Patient History Relevance is Somewhat Relevant, then Likelihood of Disease Diagnosis is Unlikely
%     1 1 3 3 3 1; % If Symptom Severity is Low and Test Result Consistency is Inconsistent and Patient History Relevance is Highly Relevant, then Likelihood of Disease Diagnosis is Possible
%     1 2 1 1 2 1; % If Symptom Severity is Low and Test Result Consistency is Moderately Consistent and Patient History Relevance is Irrelevant, then Likelihood of Disease Diagnosis is Unlikely
%     1 2 2 2 3 1; % If Symptom Severity is Low and Test Result Consistency is Moderately Consistent and Patient History Relevance is Somewhat Relevant, then Likelihood of Disease Diagnosis is Possible
%     1 2 3 3 3 1; % If Symptom Severity is Low and Test Result Consistency is Moderately Consistent and Patient History Relevance is Highly Relevant, then Likelihood of Disease Diagnosis is Possible
%     1 3 1 1 2 1; % If Symptom Severity is Low and Test Result Consistency is Consistent and Patient History Relevance is Irrelevant, then Likelihood of Disease Diagnosis is Unlikely
%     1 3 2 2 3 1; % If Symptom Severity is Low and Test Result Consistency is Consistent and Patient History Relevance is Somewhat Relevant, then Likelihood of Disease Diagnosis is Possible
%     1 3 3 4 3 1; % If Symptom Severity is Low and Test Result Consistency is Consistent and Patient History Relevance is Highly Relevant, then Likelihood of Disease Diagnosis is Probable
%     2 1 1 1 2 1; % If Symptom Severity is Moderate and Test Result Consistency is Inconsistent and Patient History Relevance is Irrelevant, then Likelihood of Disease Diagnosis is Unlikely
%     2 1 2 2 3 1; % If Symptom Severity is Moderate and Test Result Consistency is Inconsistent and Patient History Relevance is Somewhat Relevant, then Likelihood of Disease Diagnosis is Possible
%     2 1 3 3 3 1; % If Symptom Severity is Moderate and Test Result Consistency is Inconsistent and Patient History Relevance is Highly Relevant, then Likelihood of Disease Diagnosis is Possible
%     2 2 1 2 3 1; % If Symptom Severity is Moderate and Test Result Consistency is Moderately Consistent and Patient History Relevance is Irrelevant, then Likelihood of Disease Diagnosis is Possible
%     2 2 2 3 3 1; % If Symptom Severity is Moderate and Test Result Consistency is Moderately Consistent and Patient History Relevance is Somewhat Relevant, then Likelihood of Disease Diagnosis is Probable
%     2 2 3 4 3 1; % If Symptom Severity is Moderate and Test Result Consistency is Moderately Consistent and Patient History Relevance is Highly Relevant, then Likelihood of Disease Diagnosis is Probable
%     2 3 1 2 3 1; % If Symptom Severity is Moderate and Test Result Consistency is Consistent and Patient History Relevance is Irrelevant, then Likelihood of Disease Diagnosis is Possible
%     2 3 2 3 3 1; % If Symptom Severity is Moderate and Test Result Consistency is Consistent and Patient History Relevance is Somewhat Relevant, then Likelihood of Disease Diagnosis is Probable
%     2 3 3 4 4 1; % If Symptom Severity is Moderate and Test Result Consistency is Consistent and Patient History Relevance is Highly Relevant, then Likelihood of Disease Diagnosis is Highly Probable
%     3 1 1 2 2 1; % If Symptom Severity is High and Test Result Consistency is Inconsistent and Patient History Relevance is Irrelevant, then Likelihood of Disease Diagnosis is Possible
%     3 1 1 3 2 1;  % If Symptom Severity is High and Test Result Consistency is Low and Patient History Relevance is Irrelevant, then Likelihood of Disease Diagnosis is Possible
%     3 1 2 2 3 1;  % If Symptom Severity is High and Test Result Consistency is Low and Patient History Relevance is Somewhat Relevant, then Likelihood of Disease Diagnosis is Possible
%     3 1 3 3 3 1;  % If Symptom Severity is High and Test Result Consistency is Low and Patient History Relevance is Highly Relevant, then Likelihood of Disease Diagnosis is Possible
%     3 2 1 3 3 1;  % If Symptom Severity is High and Test Result Consistency is Moderate and Patient History Relevance is Irrelevant, then Likelihood of Disease Diagnosis is Probable
%     3 2 2 3 3 1;  % If Symptom Severity is High and Test Result Consistency is Moderate and Patient History Relevance is Somewhat Relevant, then Likelihood of Disease Diagnosis is Probable
%     3 2 3 3 3 1;  % If Symptom Severity is High and Test Result Consistency is Moderate and Patient History Relevance is Highly Relevant, then Likelihood of Disease Diagnosis is Probable
%     3 3 1 4 3 1;  % If Symptom Severity is High and Test Result Consistency is High and Patient History Relevance is Irrelevant, then Likelihood of Disease Diagnosis is Highly Probable
%     3 3 2 4 4 1;  % If Symptom Severity is High and Test Result Consistency is High and Patient History Relevance is Somewhat Relevant, then Likelihood of Disease Diagnosis is Highly Probable
%     3 3 3 4 4 1;  % If Symptom Severity is High and Test Result Consistency is High and Patient History Relevance is Highly Relevant, then Likelihood of Disease Diagnosis is Highly Probable
% ];

% Rules for the Medical Diagnosis FIS
rules = [
    % Antecedent (Input 1, Input 2, Input 3), Consequent (Output), Weight, Operator (AND=1)
    % Very Low Severity, Inconsistency, and Irrelevance
    1 1 1 1 1 1; % Unlikely
    1 1 2 1 1 1; % Unlikely
    1 1 3 2 1 1; % Possible
    1 2 1 1 1 1; % Unlikely
    1 2 2 2 1 1; % Possible
    1 2 3 2 1 1; % Possible
    1 3 1 1 1 1; % Unlikely
    1 3 2 2 1 1; % Possible
    1 3 3 3 1 1; % Probable

    % Low Severity, Varying Consistency and Relevance
    2 1 1 1 1 1; % Unlikely
    2 1 2 2 1 1; % Possible
    2 1 3 2 1 1; % Possible
    2 2 1 2 1 1; % Possible
    2 2 2 3 1 1; % Probable
    2 2 3 3 1 1; % Probable
    2 3 1 2 1 1; % Possible
    2 3 2 3 1 1; % Probable
    2 3 3 4 1 1; % Likely

    % Medium Severity, Varying Consistency and Relevance
    3 1 1 2 1 1; % Possible
    3 1 2 2 1 1; % Possible
    3 1 3 3 1 1; % Probable
    3 2 1 2 1 1; % Possible
    3 2 2 3 1 1; % Probable
    3 2 3 3 1 1; % Probable
    3 3 1 2 1 1; % Possible
    3 3 2 3 1 1; % Probable
    3 3 3 4 1 1; % Likely

    % High Severity, Varying Consistency and Relevance
    4 1 1 2 1 1; % Possible
    4 1 2 2 1 1; % Possible
    4 1 3 3 1 1; % Probable
    4 2 1 2 1 1; % Possible
    4 2 2 3 1 1; % Probable
    4 2 3 4 1 1; % Likely
    4 3 1 3 1 1; % Probable
    4 3 2 4 1 1; % Likely
    4 3 3 5 1 1; % Very Likely

    % Very High Severity, Varying Consistency and Relevance
    5 1 1 3 1 1; % Probable
    5 1 2 3 1 1; % Probable
    5 1 3 4 1 1; % Likely
    5 2 1 3 1 1; % Probable
    5 2 2 4 1 1; % Likely
    5 2 3 5 1 1; % Very Likely
    5 3 1 4 1 1; % Likely
    5 3 2 5 1 1; % Very Likely
    5 3 3 5 1 1; % Very Likely
];

% Add rules to the FIS
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
% symptomSeverityRange = linspace(0, 10, 100);
% fisOutputForSensitivity = arrayfun(@(x) evalfis([x, meanInput(2:3)], fis), symptomSeverityRange);
% figure;
% plot(symptomSeverityRange, fisOutputForSensitivity);
% title('Sensitivity Analysis for Symptom Severity');
% xlabel('Symptom Severity'); ylabel('Likelihood of Disease Diagnosis');

%% Principal Component Analysis (PCA)
% standardisedData = (inputData - mean(inputData)) ./ std(inputData);
% [coeff, score, latent] = pca(standardisedData);
% explainedVariance = cumsum(latent) / sum(latent) * 100;
% figure; bar(explainedVariance);
% title('PCA - Explained Variance');
% xlabel('Principal Components'); ylabel('Percentage of Variance Explained');
% figure; scatter(score(:,1), score(:,2));
% title('PCA - Scatter Plot of the First Two Principal Components');
% xlabel('Principal Component 1'); ylabel('Principal Component 2');

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