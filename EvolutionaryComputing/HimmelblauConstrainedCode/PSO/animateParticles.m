function animateParticles(particleHistory, BestCosts, MaxIt, nPop)
    figure(5);
    plotHandle = scatter3(NaN, NaN, NaN, 'o', 'MarkerEdgeColor', 'r', 'MarkerFaceColor', 'r');
    xlabel('Position X');
    ylabel('Position Y');
    zlabel('BestCosts');
    
    % Create a VideoWriter object to save the animation
    writerObj = VideoWriter('particle_animation.avi');
    writerObj.FrameRate = 10; % Set the frame rate (frames per second)
    open(writerObj); % Open the VideoWriter object
    
    for it = 1:MaxIt
        xPos = particleHistory(:, 1, it);
        yPos = particleHistory(:, 2, it);
        zPos = repmat(BestCosts(it), 1, nPop);
        set(plotHandle, 'XData', xPos, 'YData', yPos, 'ZData', zPos);
        drawnow;
        
        % Get the current frame
        frame = getframe(gcf);
        
        % Write the current frame to the video
        writeVideo(writerObj, frame);
        
        pause(0.1);
    end
    
    % Close the VideoWriter object
    close(writerObj);
end