% Load dataset
data = readtable('kdd_no_redundancy_and_duplicates.xlsx');

% Set inputs & outputs
input = table2array(data(:,1:40));  % Extract the first 40 columns as input data
output = table2array(data(:,41));   % Extract the 41st column as output data

% Define the size of the rolling window
windowSize = round(size(input, 1) / 10);  % 10% of the data in each window
numWindows = 2;  % Number of windows

perfArray = zeros(1, numWindows);  % Initialise an array for storing performance values

%lr = 0.05;          % Set the learning rate

for window = 1:numWindows 	% This is based on the amount of windows set earlier (10)
    disp(['Window: ', num2str(window)]); % This displays message indicating current window 
    
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

    for r = 1:10
        disp('Step:')
        disp(r)
        % Create an Artificial Neural Network (ANN) with r*2 hidden layers and r*2 nodes
        net = patternnet(r*2);    % Define an ANN with varying hidden layers and nodes

        % Display Hidden Units & r*2
        disp('HU')
        disp(r*2)
        % Set ANN parameters
        net.trainFcn = 'traingd';          % Set the training function (Gradient Descent)
        net.trainParam.epochs = r*10;      % Set the maximum number of training iterations
		net.trainParam.lr = 0.05; % Learning rate
        net.performFcn = 'mse';            % Set the performance function (Mean Squared Error)

        % Initialise the network
        net = init(net);
        % Train the network with the input and output data for this window
        [net, tr] = train(net, inputTrain', outputTrain');

        % Testing
        % Calculate the network's outputs based on the input for this window
        annoutputs = net(inputTest');
        % Calculate the performance of the network using Mean Squared Error for this window
        perf = perform(net, outputTest, annoutputs);
        perfArray(window, r) = perf;

        % View the neural network
        view(net)

        % Decrease the learning rate (lr) by 0.01
        %net.trainParam.lr = lr - 0.01;
    end
end

% Calculate the average performance for each step over all windows
averagePerformance = mean(perfArray, 1);
disp(['Average Performance for Each Step:']);
disp(averagePerformance);

% Create a line plot to depict the performance for each step
x = 2:2:20; % Assuming there's 10 iterations with r ranging from 1 to 10 - change this if params change

% Define styles for better visualisation
lineStyle = '-'; % Solid line
markerStyle = 'o'; % Circle markers
lineColor = 'r'; % color

% Define labels and plot
figure;
plot(x, averagePerformance, [lineStyle, markerStyle], 'Color', lineColor, 'LineWidth', 3);
title('Performance vs. Hidden Layers');
xlabel('Number of Hidden Layers');
ylabel('Average Performance (MSE)');
grid on;
