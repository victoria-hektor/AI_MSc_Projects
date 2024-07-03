function plotDiversity(particle_history)
    figure;
    numIterations = size(particle_history, 1); % to match the first dimension
    diversity = zeros(1, numIterations);
    
    for i = 1:numIterations
        % The dimensions are reordered to match particle_history structure
        currentPositions = squeeze(particle_history(i, :, :));
        % Calculate pairwise distances between all particles
        distances = pdist(currentPositions, 'euclidean');
        % Average distance as a measure of diversity
        diversity(i) = mean(distances);
    end
    
    plot(diversity, 'LineWidth', 2);
    title('Diversity Measurement Over Time');
    xlabel('Iteration');
    ylabel('Average Distance Between Particles');
    grid on;
end