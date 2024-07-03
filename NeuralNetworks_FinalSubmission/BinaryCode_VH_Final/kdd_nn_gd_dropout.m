% Load dataset with appropriate error handling
try
    data = readtable('kdd_no_redundancy_and_duplicates.xlsx');
catch
    error('Error loading the dataset. Make sure the file "kdd_no_redundancy_and_duplicates.xlsx" is in the current directory.'); % Error message
end

% Set inputs & outputs with appropriate error handling
try
    input = table2array(data(:,1:40));
    output = table2array(data(:,41));
catch
    error('Error setting inputs and outputs. Make sure the dataset columns are correctly defined.'); % Error message
end

% Define the size of the rolling window
try
    windowSize = round(size(input, 1) / 10); % 10% of the data in each window
    numWindows = 2; % Number of windows
catch
    error('Error calculating the window size and number of windows.'); % Error message
end

perfArray = zeros(1, numWindows); % Initialise an array for storing performance values

bestConfig = struct('layers', 0, 'neurons', 0); % Struct that is used to store information about the best configuration found during execution
lowestError = inf; % variable is initialised with the value inf (positive infinity)

for window = 1:numWindows % This is based on the amount of windows set earlier
    try
        disp(['Window: ', num2str(window)]); % This displays a message indicating the current window
        
        % Calculate the starting and ending indices for the current window
        startIndex = (window - 1) * windowSize + 1;
        endIndex = window * windowSize;
        
        % Split the data into training and testing sets for this window
        % Create Training data for this window
        inputTrain = input([1:startIndex - 1, endIndex + 1:end], :);
        outputTrain = output([1:startIndex - 1, endIndex + 1:end], :);
        
        % Create Testing data for this window
        inputTest = input(startIndex:endIndex, :);
        outputTest = output(startIndex:endIndex, :);
    catch
        error(['Error splitting data for window ', num2str(window)]); % Error Handling
    end

    % Initiates a for loop for 10 iterations
    for r = 1:10
        try
            disp('Step:')
            disp(r)
            % Create a feedforward neural network
            net = feedforwardnet(r*2);

            % Display Hidden Units & r*2
            disp('HU')
            disp(r*2)
            % Set ANN parameters
            net.trainFcn = 'traingd'; % gradient descent (gd)
            net.trainParam.epochs = r * 10; % Set the maximum number of training iterations
            net.trainParam.lr = 0.06; % Learning rate
            net.performFcn = 'mse'; % Set the performance function (Mean Squared Error)

            % Initialise the network
            net = init(net);

			% Configure dropout for hidden layers
			dropoutRate = 0.2; % Adjust the dropout rate as needed
			for i = 1:numel(net.layers)-1
				net.layers{i}.transferFcn = 'tansig'; % Use 'tansig' for dropout
				net.layers{i}.dropoutFraction = dropoutRate;
			end

            % Train the network with the input and output data for this window
            [net, tr] = train(net, inputTrain', outputTrain');

            % Testing
            % Calculate the network's outputs based on the input for this window
            annoutputs = net(inputTest');
            % Calculate the performance of the network using Mean Squared Error for this window
            perf = perform(net, outputTest, annoutputs);
            perfArray(window, r) = perf;
            
            % If loop to track the lowest error 
            if perf < lowestError % If better than previous, updates below
                bestConfig.layers = r;
                bestConfig.neurons = r * 2;
                lowestError = perf;
            end

            view(net)

        catch
            error(['Error in window ', num2str(window), ', step ', num2str(r)]); % Error message
        end
    end
end

% Block to display info on the best nn
try
    disp(['Best Configuration - Hidden Layers: ', num2str(bestConfig.layers), ', Neurons: ', num2str(bestConfig.neurons)]);
    disp(['Lowest Error: ', num2str(lowestError)]);
catch % Any errors caught here
    error('Error displaying best configuration and lowest error.'); % Error message
end

% Block to calculate the average performance for each step over all windows, calculate the average performance using the mean function on the perfArray array
try
    averagePerformance = mean(perfArray, 1); % Calculated along the first dimension of the perfArray
    disp(['Average Performance for Each Step:']); % Display Message
    disp(averagePerformance); % Calls on the calculated perf for display 
catch
    error('Error calculating average performance.'); % Error message
end

% Create a line plot to depict the performance for each step
x = 2:2:20; % Assuming there's 10 iterations with r ranging from 1 to 10 - change this if params change

% Define styles for better visualization
lineStyle = '-'; % Solid line
markerStyle = 'o'; % Circle markers
lineColor = 'r'; % color

% Define labels and plot with error handling
try
    figure;
    plot(x, averagePerformance, [lineStyle, markerStyle], 'Color', lineColor, 'LineWidth', 3);
    title('Performance vs. Hidden Layers');
    xlabel('Number of Hidden Layers');
    ylabel('Average Performance (MSE)');
    grid on;
catch
    error('Error creating the line plot for performance.'); % Error message
end