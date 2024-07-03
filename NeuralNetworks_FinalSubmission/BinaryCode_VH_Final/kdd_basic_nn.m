% Load dataset
data = readtable('kdd_no_redundancy_and_duplicates.xlsx');

% Set inputs & outputs
input = table2array(data(:,1:40));  % Extract the first 40 columns as input data
output = table2array(data(:,41));   % Extract the 41st column as output data
perfArray = [];                      % Initialise an empty array for performance values

lr = 0.05;          % Set the learning rate

for r = 1:10
    disp('Step:')
    disp(r)
    % Create an Artificial Neural Network (ANN) with r*2 hidden layers and r*2 nodes
    net = patternnet([r*2, r*2]);    % Define an ANN with varying hidden layers and nodes
    
	% Display Hidden Units & r*2
    disp('HU')
    disp(r*2)
    % Set ANN parameters
    net.trainFcn = 'traingd';          % Set the training function (Gradient Descent)
    net.trainParam.epochs = r*1000;      % Set the maximum number of training iterations
    net.performFcn = 'mse';            % Set the performance function (Mean Squared Error)
    
    % Specify how to divide the data for training, testing, and validation
    net.divideFcn = 'dividerand';       
    net.divideParam.trainRatio = 70/100;  % 70% of data for training
    net.divideParam.testRatio = 15/100;   % 15% of data for testing
    net.divideParam.valRatio = 15/100;    % 15% of data for validation
    
    % Initialise the network
    net = init(net);
    % Train the network with the input and output data
    [net, tr] = train(net, input', output');
    
    % Testing
    % Calculate the network's outputs based on the input
    annoutputs = net(input');
    % Calculate the performance of the network using Mean Squared Error
    perf = perform(net, output, annoutputs);
    % Add the performance value to the perfArray
    perfArray = [perfArray, perf];
    
    % Calculate the error between the expected output and the network's output
    err = gsubtract(output, annoutputs);
    
    % View the neural network
    view(net)
    
    % Decrease the learning rate (lr) by 0.01
    net.trainParam.lr = lr - 0.01;
end

% Create a line plot to depict the performance
x = 2:2:20; % Assuming there's 10 iterations with r ranging from 1 to 10 - change this if params change

% Define some custom line and marker styles for better visualisation
lineStyle = '-'; % Solid line
markerStyle = 'o'; % Circle markers
lineColor = 'r'; % color

plot(x, perfArray, [lineStyle, markerStyle], 'Color', lineColor, 'LineWidth', 2);
title('Performance vs. Hidden Layers');
xlabel('Number of Hidden Layers');
ylabel('Performance (MSE)');
grid on;

% Adding a legend to the plot
legend('Performance', 'Location', 'Best'); 

% Plot the input data against the output data
figure(1); % Display figure 1

