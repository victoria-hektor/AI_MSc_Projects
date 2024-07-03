function animateParticles(particleHistory, BestCosts, MaxIt, nPop)
    % Set up the figure
    hFig = figure('Name', 'Particle Swarm Optimization Progression', 'NumberTitle', 'off');
    plotHandle = scatter3(NaN, NaN, NaN, 'o', 'MarkerEdgeColor', 'r', 'MarkerFaceColor', 'r');
    xlabel('Position X');
    ylabel('Position Y');
    zlabel('BestCosts');
    
    % Determine the global minimum and maximum particle positions across all iterations
    minX = min(particleHistory(:, 1, :), [], 'all');
    maxX = max(particleHistory(:, 1, :), [], 'all');
    minY = min(particleHistory(:, 2, :), [], 'all');
    maxY = max(particleHistory(:, 2, :), [], 'all');
    minZ = min(BestCosts, [], 'all');
    maxZ = max(BestCosts, [], 'all');

    % Set the axes limits
    xlim([minX, maxX]);
    ylim([minY, maxY]);
    zlim([minZ, maxZ]);
    
    % Prepare the video writer object
    v = VideoWriter('pso_animation.avi');
    open(v);
    
    for it = 1:MaxIt
        xPos = particleHistory(:, 1, it);
        yPos = particleHistory(:, 2, it);
        zPos = repmat(BestCosts(it), nPop, 1);
        set(plotHandle, 'XData', xPos, 'YData', yPos, 'ZData', zPos);
        drawnow;
        
        % Capture the frame for the video
        frame = getframe(hFig);
        writeVideo(v, frame);
        
        pause(0.1);  % Pause to control the speed of animation
    end
    
    close(v);
    close(hFig);
end
