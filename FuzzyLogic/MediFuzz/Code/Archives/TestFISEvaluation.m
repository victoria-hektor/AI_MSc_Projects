%% Data Setup
% File and Sheet Setup
dataFile = 'pseudo_patient_data_1000dp.xlsx';
sheetName = 'Sheet1'; 

% Read Data Once
inputData = readmatrix(dataFile, 'Sheet', sheetName, 'Range', 'A2:C1001');

% Create a new fuzzy inference system called 'medicalFIS'
fis = newfis('medicalFIS');

% Adding 'Symptom Severity' as an input variable
fis = addvar(fis, 'input', 'Symptom Severity', [0 10]);

% Defining trapezoidal membership functions for 'Symptom Severity'
fis = addmf(fis, 'input', 1, 'Very Low', 'trapmf', [0 1 2.5 5]);
fis = addmf(fis, 'input', 1, 'Low', 'trapmf', [1 2 2.5 6]);
fis = addmf(fis, 'input', 1, 'Medium', 'trapmf', [2 2.5 5 7.5]);
fis = addmf(fis, 'input', 1, 'High', 'trapmf', [4 5 7.5 10]);
fis = addmf(fis, 'input', 1, 'Very High', 'trapmf', [4.5 5 8 10]);

% Adding 'Test Result Consistency' as an input variable
fis = addvar(fis, 'input', 'Test Result Consistency', [0 10]);

% Defining trapezoidal membership functions for 'Test Result Consistency'
fis = addmf(fis, 'input', 2, 'Very Inconsistent', 'trapmf', [0 1 2.5 5]);
fis = addmf(fis, 'input', 2, 'Inconsistent', 'trapmf', [0 2 2.5 6]);
fis = addmf(fis, 'input', 2, 'Moderately Consistent', 'trapmf', [2.5 3 5 7.5]);
fis = addmf(fis, 'input', 2, 'Consistent', 'trapmf', [4 5 7.5 10]);
fis = addmf(fis, 'input', 2, 'Very Consistent', 'trapmf', [4.5 5 8 10]);

% Adding 'Patient History Relevance' as an input variable
fis = addvar(fis, 'input', 'Patient History Relevance', [0 10]);

% Defining trapezoidal membership functions for 'Patient History Relevance'
fis = addmf(fis, 'input', 3, 'Very Irrelevant', 'trapmf', [0 1 2.5 5]);
fis = addmf(fis, 'input', 3, 'Irrelevant', 'trapmf', [0 1.5 2.5 6]);
fis = addmf(fis, 'input', 3, 'Somewhat Relevant', 'trapmf', [2 2.5 5 7.5]);
fis = addmf(fis, 'input', 3, 'Relevant', 'trapmf', [4 5 7.5 10]);
fis = addmf(fis, 'input', 3, 'Highly Relevant', 'trapmf', [4.5 5 8 10]);

% Generate input data for testing
numSamples = 100;
symptomSeverityRange = linspace(0, 10, numSamples);
testResultConsistencyRange = linspace(0, 10, numSamples);
patientHistoryRelevanceRange = linspace(0, 10, numSamples);
[SymptomSeverityGrid, TestResultConsistencyGrid, PatientHistoryRelevanceGrid] = ...
    meshgrid(symptomSeverityRange, testResultConsistencyRange, patientHistoryRelevanceRange);
inputData = [SymptomSeverityGrid(:), TestResultConsistencyGrid(:), PatientHistoryRelevanceGrid(:)];

% Evaluate the FIS with the generated data
outputData = evalfis(fis, inputData);

% Visualizing the results
figure;
scatter3(inputData(:,1), inputData(:,2), inputData(:,3), 36, outputData, 'filled');
title('FIS Output Activation');
xlabel('Symptom Severity');
ylabel('Test Result Consistency');
zlabel('Patient History Relevance');
colorbar;