% Define constants
numIterations = 10;
numWindows = 2;

% Load dataset with appropriate error handling
try
    data = readtable('kdd_no_redundancy_and_duplicates.xlsx');
catch
    error('Error loading the dataset. Make sure the file "kdd_no_redundancy_and_duplicates.xlsx" is in the current directory.');
end

% Set inputs & outputs with appropriate error handling
try
    input = table2array(data(:,1:40));
    output = table2array(data(:,41));
catch
    error('Error setting inputs and outputs. Make sure the dataset columns are correctly defined.');
end

% Define the size of the rolling window
try
    windowSize = round(size(input, 1) / numWindows); % 10% of the data in each window
catch
    error('Error calculating the window size and number of windows.');
end

% Preallocate perfArray
perfArray = zeros(numWindows, numIterations);

bestConfig = struct('layers', 0, 'neurons', 0);
lowestError = inf;

% Main loop
for window = 1:numWindows
    try
        disp(['Window: ', num2str(window)]);

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

        % Loop for different numbers of hidden layers
        for r = 1:numIterations
            try
                disp('Step:');
                disp(r);

                % Create and configure an Artificial Neural Network (ANN)
                net = fitnet(r*2); % Use fitnet for greater control with partial connectivity

                % Set ANN parameters
                net.trainFcn = 'traingd'; % Use gradient descent (gd)
                net.trainParam.epochs = r * 10; % Set the maximum number of training iterations
                net.trainParam.lr = 0.06; % Learning rate
                net.performFcn = 'mse'; % Set the performance function (Mean Squared Error)

                % Configure weight decay
                weightDecay = 0.001; % Adjust this if needed
                net.performParam.regularization = weightDecay;

                % Initialize the network
                net = init(net);

                % Set the number of hidden layers and neurons
                net.layers{1}.size = r * 2;

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

                view(net);

            catch exception
                disp(['Error in window ', num2str(window), ', step ', num2str(r)]);
                disp(getReport(exception, 'extended'));
            end
        end

    catch
        error(['Error splitting data for window ', num2str(window)]);
    end
end

% Block to display info on the best nn
try
    disp(['Best Configuration - Hidden Layers: ', num2str(bestConfig.layers), ', Neurons: ', num2str(bestConfig.neurons)]);
    disp(['Lowest Error: ', num2str(lowestError)]);
catch
    error('Error displaying best configuration and lowest error.');
end

% Block to calculate the average performance for each step over all windows
try
    averagePerformance = mean(perfArray, 1);
    disp(['Average Performance for Each Step:']);
    disp(averagePerformance);
catch
    error('Error calculating average performance.');
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
    error('Error creating the line plot for performance.');
end
