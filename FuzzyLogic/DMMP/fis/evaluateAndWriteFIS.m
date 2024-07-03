function evaluateAndWriteFIS(fis, inputData, dataFile, sheetName, outputStartCell)
    % Define the different defuzzification methods
    defuzzMethods = {'centroid', 'mom', 'lom', 'som', 'bisector'};

    % Initialise a matrix to hold the results
    numDataPoints = size(inputData, 1);
    results = zeros(numDataPoints, numel(defuzzMethods));
    
    % Loop through each defuzzification method
    for i = 1:numel(defuzzMethods)
        % Update the FIS to use the current defuzzification method
        fis.DefuzzificationMethod = defuzzMethods{i};

        % Evaluate the FIS for all input data
        for j = 1:numDataPoints
            results(j, i) = evalfis(fis, inputData(j, :));
        end
    end
    
    % Write the results to the Excel file
    % Assuming outputStartCell is the starting point 'E2'
    endRow = num2str(size(inputData, 1) + 1); % Since starting from 2, add 1
    outputRange = strcat('E2:I', endRow);
    writematrix(results, dataFile, 'Sheet', sheetName, 'Range', outputRange);
end