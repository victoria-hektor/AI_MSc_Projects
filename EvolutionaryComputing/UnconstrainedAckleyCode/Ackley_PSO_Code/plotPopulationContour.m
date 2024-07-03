function plotPopulationContour(CostFunction, particle, VarMin, VarMax, nPop)
    % Prepare Mesh for Contour Plots
    x1 = linspace(VarMin, VarMax, 100); % Adjust the granularity as needed
    x2 = linspace(VarMin, VarMax, 100);
    [X1, X2] = meshgrid(x1, x2);

    % Evaluate the Cost Function on the grid
    Z = arrayfun(@(x, y) CostFunction([x, y]), X1, X2);

    % Clear and hold the figure
    clf; % Clears current figure window and resets hold state
    hold on;

    % Contour Plot of Cost Function
    contourf(X1, X2, log(Z+1), 20); % Filled contour plot with more levels
    colormap(jet); % Change to a more colorful colormap
    colorbar; % Optionally add a color bar to indicate the scale

    % Plot whole Population as red stars
    for i = 1:nPop
        plot(particle(i).Position(1), particle(i).Position(2), 'r*', 'LineWidth', 2);
    end

    % Enhancements
    title('Particle Positions and Cost Function Contour');
    xlabel('Variable x1');
    ylabel('Variable x2');
    axis equal;
    grid on; % Optionally add a grid
    hold off;
end