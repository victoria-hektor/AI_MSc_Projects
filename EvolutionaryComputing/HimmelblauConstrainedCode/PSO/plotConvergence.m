function plotConvergence(bestCosts, globalOptimumCost)
    figure;
    % Calculate the absolute difference between the best cost of each iteration and the global optimum cost
    convergenceRate = abs(bestCosts - globalOptimumCost);
    
    plot(convergenceRate, 'LineWidth', 2);
    title('Convergence Plot');
    xlabel('Iteration');
    ylabel('Absolute Error from Global Optimum');
    grid on;
end