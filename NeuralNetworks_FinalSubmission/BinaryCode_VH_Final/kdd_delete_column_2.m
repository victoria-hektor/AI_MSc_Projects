% Load the data from the Excel file using readtable
data = readtable('normalised_scaled.xlsx');

% Get the number of columns
numColumns = width(data);

% Rename columns from 1 to numColumns
data.Properties.VariableNames = strcat('Column_', arrayfun(@num2str, 1:numColumns, 'UniformOutput', false));

% List of column indices to delete
columnsToDelete = [20, 25, 26];

% Delete the specified columns
data(:, columnsToDelete) = [];

% Save the preprocessed data to a new file
writetable(data, 'kdd_normalised.xlsx');

fprintf('Data normalisation completed successfully.\n');
