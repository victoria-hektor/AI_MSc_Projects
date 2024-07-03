function normalisedData = normaliseAndSaveData(rawData, filename, sheetname)
    % Initialise the normalised data matrix with the same size as rawData
    normalisedData = zeros(size(rawData));
    
    % Define the original and target columns for normalisation
    originalColumns = {'A', 'C', 'E'}; % Columns with raw data
    targetColumns = {'B', 'D', 'F'};   % Columns where normalised data will be written
    
    % Loop over each column to normalise
    for i = 1:length(originalColumns)
        % Normalise the column
        col = rawData(:, i);
        minCol = min(col);
        maxCol = max(col);
        % Avoid division by zero in case maxCol is equal to minCol
        if maxCol == minCol
            normalisedCol = zeros(size(col)); % or ones(size(col)) if you want to set them to the max value
        else
            normalisedCol = (col - minCol) / (maxCol - minCol);
        end
        normalisedData(:, i) = normalisedCol;
        
        % Write normalised data to the target column in Excel
        range = strcat(targetColumns{i}, '3:', targetColumns{i}, num2str(size(rawData, 1)+2));
        writematrix(normalisedCol, filename, 'Sheet', sheetname, 'Range', range);
    end
end