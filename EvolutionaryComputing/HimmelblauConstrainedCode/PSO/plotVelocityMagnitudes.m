function plotVelocityMagnitudes(particleVelocityHistory)
    figure;
    % Calculate the magnitude of velocity for each particle at each iteration
    velocityMagnitudes = squeeze(sqrt(sum(particleVelocityHistory.^2, 3)));
    
    plot(velocityMagnitudes, 'LineWidth', 1);
    title('Velocity Magnitudes Over Time');
    xlabel('Iteration');
    ylabel('Velocity Magnitude');
    legend('Particle 1', 'Particle 2', 'Location', 'Best'); % Adjust the legend accordingly
    grid on;
end