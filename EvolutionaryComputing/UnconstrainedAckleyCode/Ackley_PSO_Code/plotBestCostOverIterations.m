function plotBestCostOverIterations(BestCosts)
    figure;
    semilogy(BestCosts, 'LineWidth', 2);
    xlabel('Iteration');
    ylabel('Best Cost');
    title('Best Cost Over Iterations');
    grid on;
end
