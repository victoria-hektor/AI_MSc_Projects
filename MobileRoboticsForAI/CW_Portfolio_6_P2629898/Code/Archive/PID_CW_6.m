function PID_CW_6(serPort)
% Combined behaviour for wall following and random wandering.
% serPort is the serial port number (for controlling the actual robot).

%%%% DO NOT MODIFY CODE ABOVE %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

disp('==================')
disp('PID Programme Starting')
disp('------------------')

% Start the timer
startTime = tic;

% Set thresholds
wallFollowDistance = 0.5; % Distance to follow the wall (0.5 metres)
obstacleThreshold = 0.5; % Distance to consider an obstacle is close
stuckThreshold = 5; % Time in seconds to consider the robot might be stuck

% Define states
STATE_RANDOM_WANDERING = 1;
STATE_WALL_FOLLOWING = 2;
STATE_TURNING_AWAY = 3; % New state for turning after wall following

% Define the base speed for the robot
baseSpeed = 0.2;

% Initialise state variable
currentState = STATE_RANDOM_WANDERING;
wallFollowStartTime = 0; % Timer to keep track of wall following duration

% Initialise odometry data
x = 0; y = 0; angle = 0;
positionLog = [x, y, angle];

% Define PID constants
Kp = 0.5;  % Proportional gain
Ki = 0.05;  % Integral gain
Kd = 0.1;  % Derivative gain

% Initialise PID variables
integral = 0;
lastError = 0;

    % Main control loop
    while toc(startTime) < 300  % Run for 5 minutes
        % Read sonar values
        SonFF = ReadSonar(serPort, 2); % Front sonar
        SonRight = ReadSonar(serPort, 3); % Right sonar

        % FSM logic for transitioning between states
        switch currentState
            case STATE_RANDOM_WANDERING
                disp('Current State: Random Wandering');
                % Random wandering logic
                if rand() < 0.1  % 10% chance to perform a random action
                    action = randi([1, 3]);  % Including the case for reversing/stopping
                    switch action
                        case 1
                            % Random turn
                            randomAngle = rand() * 90 - 45;  % Limited turn range
                            turnAngle(serPort, 0.2, randomAngle);
                        case 2
                            % Continue moving forward for a longer period
                            straightRunTime = rand() * 2 + 1;  % 1 to 3 seconds
                            SetDriveWheelsCreate(serPort, 0.3, 0.3);
                            pause(straightRunTime);  % Move straight for a longer period
                        case 3
                            % Reverse or stop for a moment
                            SetDriveWheelsCreate(serPort, -0.2, -0.2);
                            pause(1);
                            SetDriveWheelsCreate(serPort, 0, 0);
                            pause(0.5);
                    end
                else
                    % Continue moving forward
                    SetDriveWheelsCreate(serPort, 0.3, 0.3);
                    pause(1);  % Move straight for 1 second
                end

                % Check for state transition to wall following
                SonFF = ReadSonar(serPort, 2);  % Re-check front sonar
                SonRight = ReadSonar(serPort, 3);  % Re-check right sonar
                isNearWall = any(SonFF < wallFollowDistance) || any(SonRight < wallFollowDistance);
                if isNearWall
                    disp('Transitioning to Wall Following');
                    currentState = STATE_WALL_FOLLOWING;
					wallFollowStartTime = tic; % Start timing wall follow NEWLINE#
                end

            case STATE_WALL_FOLLOWING
                disp('Current State: Wall Following');
                % Wall following logic
                % If the robot is too close to the wall, steer away
                while any(SonFF < wallFollowDistance) || any(SonRight < wallFollowDistance)
                    if SonFF < wallFollowDistance
                        SetDriveWheelsCreate(serPort, 0.4, 0);  % Turn in place
                    elseif SonRight < wallFollowDistance
                        SetDriveWheelsCreate(serPort, 0, 0.4);  % Turn in place
                    end
					
					% Measure the current distance from the wall
					measuredDistance = min(SonFF, SonRight);  % Assuming closer reading is more relevant
					
					% Initialise desiredDistance
					desiredDistance = 0.5;

					% Calculate error
					error = desiredDistance - measuredDistance;

					% Update integral and derivative
					integral = integral + error;
					derivative = error - lastError;

					% Calculate control signal
					controlSignal = Kp*error + Ki*integral + Kd*derivative;

					% Ensure controlSignal does not cause the speeds to go out of bounds
					% Clamp control signal if necessary
					controlSignal = max(min(controlSignal, 0.5 - baseSpeed), -0.5 + baseSpeed);

					% Calculate the wheel speeds, ensuring they are within the valid range [-0.5, 0.5]
					leftWheelSpeed = baseSpeed + controlSignal;
					rightWheelSpeed = baseSpeed - controlSignal;

					% Apply the wheel speeds to the robot's wheels
					SetDriveWheelsCreate(serPort, leftWheelSpeed, rightWheelSpeed);

					% Update lastError for next iteration
					lastError = error;
					
					% % SetDriveWheelsCreate(serPort, baseSpeed + controlSignal, baseSpeed - controlSignal);

                    SonFF = ReadSonar(serPort, 2);  % Re-check front sonar
                    SonRight = ReadSonar(serPort, 3);  % Re-check right sonar
                end
				
                % Once a safe distance from the wall is achieved, move forward
                SetDriveWheelsCreate(serPort, 0.3, 0.3);

                % Check for state transition back to random wandering
                pause(1);  % Allow some time to move away from the wall
                SonFF = ReadSonar(serPort, 2);  % Re-check front sonar
                SonRight = ReadSonar(serPort, 3);  % Re-check right sonar
				
                isNearWall = any(SonFF < wallFollowDistance) || any(SonRight < wallFollowDistance);
				
				            if toc(wallFollowStartTime) >= 3
                disp('Transitioning to Turning Away');
                currentState = STATE_TURNING_AWAY;
            end

			case STATE_TURNING_AWAY
				% Turn 45 degrees away from the wall
				turnAngle(serPort, 0.2, 45);
				pause(1); % Allow time for the turn to complete
				currentState = STATE_RANDOM_WANDERING; % Switch back to wandering
				disp('Transitioning to Random Wandering');
			end
				
                %if ~isNearWall
                %    disp('Transitioning to Random Wandering');
                %    currentState = STATE_RANDOM_WANDERING;
                %end
    %end

    % Update odometry
    distance = DistanceSensorRoomba(serPort); % Get distance travelled since last call
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
disp('Path Travelled:');
disp(positionLog);

% Plot the path
figure;
plot(positionLog(:,1), positionLog(:,2));
title('Path Travelled by the Robot');
xlabel('X Position');
ylabel('Y Position');
axis equal;

disp('Combined Programme Ending - Time Limit Reached')
end