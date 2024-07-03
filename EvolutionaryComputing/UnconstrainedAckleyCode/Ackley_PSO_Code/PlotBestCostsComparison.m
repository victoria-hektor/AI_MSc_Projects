function PlotBestCostsComparison(allBestCosts, paramSets)
    figure;
    hold on;
    for i = 1:size(allBestCosts, 2) % each column represents a run's best costs over iterations
        plot(allBestCosts(:, i), 'LineWidth', 2, 'DisplayName', sprintf('Pop Size = %d', paramSets(i).nPop));
    end
    hold off;
    title('Best Costs Over Iterations for Different Population Sizes');
    xlabel('Iteration');
    ylabel('Best Cost');
    legend show;
    grid on;
end