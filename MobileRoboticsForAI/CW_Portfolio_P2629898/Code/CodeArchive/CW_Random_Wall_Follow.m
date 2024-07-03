function CW_Code(serPort)
% Robot moves randomly around the environment, avoiding getting stuck.
% serPort is the serial port number (for controlling the actual robot).

%%%% DO NOT MODIFY CODE ABOVE %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

disp('==================')
disp('Program Starting')
disp('------------------')

% Start the timer
startTime = tic;

% Set thresholds for obstacle detection
obstacleThreshold = 0.5; % Distance to consider an obstacle is close
stuckThreshold = 5; % Time in seconds to consider the robot might be stuck

% Initialize variables to track time spent near an obstacle
timeNearObstacle = 0;
lastObstacleTime = tic;

% Main control loop
while true
    % Check if 5 minutes have passed
    if toc(startTime) > 300
        break; % Stop the robot after 5 minutes
    end

    % Read front sonar value
    SonFF = ReadSonar(serPort, 2);
    
    % Ensure SonFF is a scalar logical value
    SonFF = SonFF(:); % Convert SonFF to a column vector to handle multiple values if necessary
    isObstacleDetected = any(SonFF < obstacleThreshold);
    
    % If there's an obstacle in front or the robot is potentially stuck,
    % turn to a new random direction
    if isObstacleDetected || timeNearObstacle > stuckThreshold
        randomAngle = rand() * 180 - 90; % Random angle between -90 and 90 degrees
        turnAngle(serPort, 0.2, randomAngle); % Turn at 0.2 rad/s
        lastObstacleTime = tic; % Reset the timer since we've just changed direction
    else
        % No obstacle detected, move forward
        SetDriveWheelsCreate(serPort, 0.5, 0.5);
    end
    
    % Update the time spent near an obstacle
    if isObstacleDetected
        timeNearObstacle = toc(lastObstacleTime);
    else
        timeNearObstacle = 0;
    end
    
    pause(0.1); % Small pause to prevent overloading the CPU
end

% Stop the robot
SetDriveWheelsCreate(serPort, 0, 0);
disp('Program Ending - Time Limit Reached')
end
