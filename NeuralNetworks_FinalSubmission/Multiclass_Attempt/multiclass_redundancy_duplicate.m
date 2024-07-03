% Load your dataset into a variable 'data'
data = readtable('kdd_scaled_encoded.xlsx');

% Check for duplicate rows
[~, uniqueIdx, ~] = unique(data, 'rows');
isDuplicate = false(size(data, 1), 1);
isDuplicate(uniqueIdx) = true;

% Find rows that are duplicated
duplicateIndices = find(isDuplicate);

% Create a new dataset without duplicates
dataWithoutDuplicates = data(~isDuplicate, :);

% Display the number of duplicate rows:
numDuplicateRows = sum(isDuplicate);
fprintf('Found %d duplicate rows.\n', numDuplicateRows);

% Use the unique function to find unique rows
[uniqueData, ~, uniqueIndices] = unique(dataWithoutDuplicates, 'rows');

% Count the occurrences of each unique row
counts = hist(uniqueIndices, unique(uniqueIndices));

% Find rows that appear more than once (redundant rows)
redundantIndices = find(counts > 1);

% Create a new dataset with redundant rows removed
dataWithoutRedundancy = uniqueData;

% Display the number of redundant rows:
numRedundantRows = sum(counts(redundantIndices));
fprintf('Found %d redundant rows.\n', numRedundantRows);

% Save the new dataset without redundancy and duplicates
writetable(dataWithoutRedundancy, 'kdd_no_redundancy_and_duplicates.xlsx');
