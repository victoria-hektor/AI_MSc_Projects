try
    % Read the xlsx file into a table
    data_table = readtable('kdd_data_multi_class.xlsx');
    
    % Define the categorical column names to exclude from scaling
    categorical_column_names = {'protocol_type', 'service', 'flag', 'attack'}; 
    
    % Get the indices of numerical columns
    numerical_column_indices = ~ismember(data_table.Properties.VariableNames, categorical_column_names);

    % Scale the numerical columns only
    numerical_data = data_table{:, numerical_column_indices};
    min_data = min(numerical_data);
    max_data = max(numerical_data);
    scaled_data = (numerical_data - min_data) ./ (max_data - min_data);

    % Replace the scaled data in the table
    data_table{:, numerical_column_indices} = scaled_data;

    % Perform one-hot encoding for the 'attack' column
    encodedLabels = dummyvar(categorical(data_table.attack));
    
    % Append the one-hot encoded columns to the table
    data_table = [data_table, array2table(encodedLabels)];

    % Remove the original 'attack' column
    data_table.attack = [];

    % Save the scaled and encoded data to 'kdd_scaled_encoded.xlsx' (tried xlsx but had errors, worked when made to a csv)
    writetable(data_table, 'kdd_scaled_encoded.csv');

    disp('Data has been successfully scaled and encoded, and saved to kdd_scaled_encoded.csv.');
catch ME
    error_msg = ['Error: ', ME.message];
    disp(error_msg);
end
