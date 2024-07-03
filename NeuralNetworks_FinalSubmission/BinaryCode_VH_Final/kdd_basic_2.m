% Load dataset
data = readtable('kdd_no_redundancy_and_duplicates.xlsx');

% Set inputs & outputs
input = table2array(data(:,1:40));
output = table2array(data(:,41));

% Create a new architecture with n hidden layers
net = feedforwardnet(5);

% Set training parameters
net.trainFcn = 'trainlm'; % Levenberg-Marquardt algorithm for training % 'trainlm';
net.trainParam.epochs = 1000; % Max iterations
net.trainParam.lr = 0.05; % Learning rate
net.performFcn = 'mse'; % Use 'crossentropy'; for classification tasks

% Divide data more representatively
net.divideFcn = 'dividerand';
net.divideParam.trainRatio = 60/100;
net.divideParam.valRatio = 20/100;
net.divideParam.testRatio = 20/100;

% Training
[net,tr] = train(net, input', output');

% Testing
annoutputs = net(input');
mse = perform(net, output, annoutputs); % Mean Squared Error
classification_error = sum(abs(round(annoutputs) - output)) / length(output); % Classification error

% Save the trained network
save('trained_network.mat', 'net');

% Additional performance evaluation metric
fprintf('Mean Squared Error: %.4f\n', mse);
fprintf('Classification Error: %.4f\n', classification_error);

% View the network (optional)
view(net)
