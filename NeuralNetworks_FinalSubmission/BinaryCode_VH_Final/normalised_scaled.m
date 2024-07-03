try
    % Read the xlsx file into a table
    data_table = readtable('kdd_data.xlsx');

    % Exclude the 'attack' column
    data_table = data_table(:, ~strcmp(data_table.Properties.VariableNames, 'attack'));

    % Convert the table to a matrix (numerical data only)
    data_matrix = table2array(data_table);

    % Define the maximum and minimum values for scaling
    max_val = 1;
    min_val = 0;

    % Min-max scaling to the range of -2 to 2 for each feature
    min_data = min(data_matrix);
    max_data = max(data_matrix);
    scaled_data = ((data_matrix - min_data) ./ (max_data - min_data)) * (max_val - min_val) + min_val;

    % Save the scaled data to 'normalised_scaled.xlsx'
    csvwrite('normalised_scaled.xlsx', scaled_data);

    disp('Data has been successfully scaled and saved to normalised_scaled.xlsx.');
catch ME
    error_msg = ['Error: ', ME.message];
    disp(error_msg);
end
