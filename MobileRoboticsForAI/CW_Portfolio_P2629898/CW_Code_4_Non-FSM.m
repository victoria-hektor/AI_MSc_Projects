function CW_Code_4(serPort)
% Combined behavior for wall following and random wandering.
% serPort is the serial port number (for controlling the actual robot).

%%%% DO NOT MODIFY CODE ABOVE %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

disp('==================')
disp('Combined Program Starting')
disp('------------------')

% Start the timer
startTime = tic;

% Set thresholds
wallFollowDistance = 0.5; % Distance to follow the wall (0.5 meters)
obstacleThreshold = 0.5; % Distance to consider an obstacle is close
stuckThreshold = 5; % Time in seconds to consider the robot might be stuck

% Initialise state variables
isFollowingWall = false;
timeNearObstacle = 0;
lastObstacleTime = tic;

% Initialise odometry data
x = 0; y = 0; angle = 0;
positionLog = [x, y, angle];

% Main control loop
while true
    % Check if 5 minutes have passed
    if toc(startTime) > 300
        break; % Stop the robot after 5 minutes
    end

    % Read sonar values
    SonFF = ReadSonar(serPort, 2); % Front sonar
    SonRight = ReadSonar(serPort, 3); % Right sonar

    % Check if near a wall
    isNearWall = any(SonFF < wallFollowDistance) || any(SonRight < wallFollowDistance);

    % Switch to wall following if near a wall, else random wander
    if isNearWall && ~isFollowingWall
        isFollowingWall = true;
    elseif ~isNearWall && isFollowingWall
        isFollowingWall = false;
    end
	
	% Random wandering logic
	if ~isFollowingWall
		% Perform a random action occasionally
		if rand() < 0.1  % 10% chance to perform a random action
			% Randomly choose an action: turn or change speed
			action = randi([1, 3]);
			switch action
				case 1  % Random turn
					randomAngle = rand() * 360 - 180;  % Full range turn
					turnAngle(serPort, 0.2, randomAngle);
				case 2  % Change speed randomly
					leftSpeed = rand();
					rightSpeed = rand();
					SetDriveWheelsCreate(serPort, leftSpeed, rightSpeed);
				case 3  % Reverse or stop for a moment
					SetDriveWheelsCreate(serPort, -0.2, -0.2);
					pause(1);
					SetDriveWheelsCreate(serPort, 0, 0);
					pause(0.5);
			end
			lastObstacleTime = tic; % Reset the stuck timer
		else
			% Continue moving forward
			SetDriveWheelsCreate(serPort, 0.5, 0.5);
		end
	end
	
	% Update odometry
    distance = DistanceSensorRoomba(serPort); % Get distance traveled since last call
    turn = AngleSensorRoomba(serPort); % Get angle turned since last call

    % Update position and orientation
    angle = angle + turn;
    x = x + distance * cos(angle);
    y = y + distance * sin(angle);

    % Log the current position and orientation
    positionLog = [positionLog; x, y, angle];

    pause(0.1); % Small pause to prevent overloading the CPU
end

% Stop the robot
SetDriveWheelsCreate(serPort, 0, 0);

% After exiting the main loop, smooth the path
windowSize = 10;  % Set the window size for the moving average
smoothedPositionLog = smoothPath(positionLog(:, 1:2), windowSize);

% Convert the smoothed path data to a table
smoothedPathTable = array2table(smoothedPositionLog, 'VariableNames', {'X', 'Y'});

% Write the table to an Excel file
filename = 'SmoothedPathTraveledData.xlsx';
writetable(smoothedPathTable, filename);

disp(['Smoothed path data written to ', filename]);

function smoothedPath = smoothPath(rawPath, windowSize)
    % Smooths the path data using a moving average filter.
    %
    % Parameters:
    %   rawPath - The original path data as an Nx2 matrix, where N is the number of points.
    %             Each row represents [X, Y] coordinates.
    %   windowSize - The size of the moving average window. This is the number of
    %                adjacent points used to compute the average.

    % Initialise the smoothed path with the same size as the raw path
    smoothedPath = rawPath;

    % Apply the moving average filter to each dimension independently
    for i = 1:size(rawPath, 2)
        smoothedPath(:, i) = movmean(rawPath(:, i), windowSize);
    end
end

% Output the position log
disp('Path Traveled:');
disp(positionLog);

% Plot the path
figure;
plot(positionLog(:,1), positionLog(:,2));
title('Path Traveled by the Robot');
xlabel('X Position');
ylabel('Y Position');
axis equal;

disp('Combined Program Ending - Time Limit Reached')
end