function PlotAckleySurface()
% PlotAckleySurface generates a surface plot of the Ackley function.

% Define the domain for the plot
x = linspace(-5, 5, 400);
y = linspace(-5, 5, 400);

[X, Y] = meshgrid(x, y);

% Preallocate Z for speed
Z = zeros(size(X));

% Evaluate the Ackley function at each point (x,y)
for i = 1:size(X, 1)
    for j = 1:size(X, 2)
        Z(i, j) = Ackley([X(i, j), Y(i, j)]);
    end
end

% Generate the surface plot
figure;
surf(X, Y, Z, 'EdgeColor', 'none');
title('Surface Plot of the Ackley Function');
xlabel('x');
ylabel('y');
zlabel('Ackley(x, y)');
colorbar; % Adds a color bar to indicate the scale
view(-45, 45); % Adjusts the view angle for better visualisation
end