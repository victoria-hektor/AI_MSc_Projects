% Load the preprocessed data from the Excel file
data = readtable('kdd_normalised.xlsx');

% Define the desired column names
desiredColumnNames = {
    'duration', 'protocol_type', 'service', 'flag', ...
    'src_bytes', 'dst_bytes', 'land', 'wrong_fragment', ...
    'urgent', 'hot', 'num_failed_logins', 'logged_in', ...
    'num_compromised', 'root_shell', 'su_attempted', 'num_root', ...
    'num_file_creations', 'num_shells', 'num_access_files' ...
    'is_hot_login', 'is_guest_login', 'count', 'srv_count', ...
    'serror_rate', 'srv_serror_rate', 'rerror_rate', 'srv_rerror_rate', ...
    'same_srv_rate', 'diff_srv_rate', 'srv_diff_host_rate', 'dst_host_count', ...
    'dst_host_srv_count', 'dst_host_same_srv_rate', 'dst_host_diff_srv_rate', ...
    'dst_host_same_src_port_rate', 'dst_host_srv_diff_host_rate', 'dst_host_serror_rate', 'dst_host_srv_serror_rate', ...
    'dst_host_rerror_rate', 'dst_host_srv_rerror_rate', 'attack'
};

% Set the new column names
data.Properties.VariableNames = desiredColumnNames;

% Save the updated data to a new Excel file
updatedFileName = 'kdd_normalised.xlsx';
writetable(data, updatedFileName);

% Success Message
fprintf('Column renaming completed successfully.\n');