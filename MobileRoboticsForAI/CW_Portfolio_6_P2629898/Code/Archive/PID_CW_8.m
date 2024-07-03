function PID_CW_8(serPort)
% Controls a robot to random wander and follow walls using PID control.
% serPort is the serial port number (for controlling the actual robot).

%%%% Initial Setup %%%%
disp('PID Random Wander and Wall Following Programme Starting');

% Start the timer
startTime = tic;

% Define robot's behavior state
STATE_RANDOM_WANDERING = 1;
STATE_WALL_FOLLOWING = 2;
currentState = STATE_RANDOM_WANDERING;
% Define the duration for wall following (in seconds)
wallFollowDuration = 10;
wallFollowDistance = 0.25;

% PID constants based on "PID without a PhD"
Kp = 0.4;  % Proportional gain
Ki = 0.001; % Integral gain
Kd = 0.4; % Derivative gain

% Initialise PID variables
integral = 0;      % Integral term
lastError = 0;     % Last error term for derivative
desiredDistance = 0.5; % Desired distance from the wall

% Define the base speed for the robot
baseSpeed = 0.1;

% Control loop execution time
executionTime = 300; % Run for 5 minutes

% Initialise logging variables
errorLog = [];        % Log for PID errors
pidTermsLog = [];     % Log for P, I, and D terms
stateLog = [];        % Log for state transitions
positionLog = [];     % Initialise position log
heatMap = zeros(10, 10); % Adjust size based on expected range of x and y

% Assume initial position (x, y, theta)
x = 0;
y = 0;
theta = 0;

%%%% Main Control Loop %%%%
while toc(startTime) < executionTime
    % Read sensors
    [SonFF, SonRight] = readSensors(serPort);

    % Finite State Machine for behavior transitions
    switch currentState
        case STATE_RANDOM_WANDERING
            % Perform random wandering
            randomWandering(serPort);
            % Check for walls and switch state if necessary
            if isWallNearby(SonFF, SonRight, desiredDistance)
                currentState = STATE_WALL_FOLLOWING;
                wallFollowStartTime = tic; % Start the wall follow timer
                disp('Switching to Wall Following');
            end

        case STATE_WALL_FOLLOWING
			[SonFF, SonRight] = readSensors(serPort);
			
			if isCorner(SonFF, SonRight, wallFollowDistance)
                handleCorner(serPort);
            elseif isStuck(positionLog)
                escapeStuck(serPort);
            else
			
				if any(SonFF < wallFollowDistance) && any(SonRight < wallFollowDistance)
					handleCorner(serPort);
				else
					% Normal wall following
					[integral, lastError, error] = followWall(serPort, SonRight, Kp, Ki, Kd, integral, lastError, desiredDistance, baseSpeed);
					% Log the error and PID terms
					errorLog = [errorLog; error];
					pidTermsLog = [pidTermsLog; Kp*error, Ki*integral, Kd*(error - lastError)];
					% Check if the wall following duration has elapsed
					if toc(wallFollowStartTime) > wallFollowDuration
						disp('Switching to Random Wandering');
						currentState = STATE_RANDOM_WANDERING;
					end
				end
			end
		end

    % Update odometry
    distance = DistanceSensorRoomba(serPort); % Get distance travelled since last call
    angle = AngleSensorRoomba(serPort); % Get angle turned since last call

    % Update position and orientation
    theta = theta + deg2rad(angle);
    x = x + distance * cos(theta);
    y = y + distance * sin(theta);

    % Log the current position and orientation
    positionLog = [positionLog; x, y];
    stateLog = [stateLog; currentState];

    % Update heat map
    % This assumes x and y range from -25 to 25
    heatMapIdxX = round(x) + size(heatMap, 2)/2;
    heatMapIdxY = round(y) + size(heatMap, 1)/2;
    if heatMapIdxX >= 1 && heatMapIdxX <= size(heatMap, 2) && heatMapIdxY >= 1 && heatMapIdxY <= size(heatMap, 1)
        heatMap(heatMapIdxY, heatMapIdxX) = heatMap(heatMapIdxY, heatMapIdxX) + 1;
    end

    pause(0.1); % Small pause to prevent overloading the CPU
end

% Stop the robot after exiting main loop
SetDriveWheelsCreate(serPort, 0, 0);
disp('Programme Ending - Time Limit Reached');

%%%% Plotting %%%%
% Plotting the path followed by the robot
figure; % Creates a new figure window
plot(positionLog(:, 1), positionLog(:, 2), 'r-'); % Plot x and y from positionLog with a red line
title('Path Followed by the Robot');
xlabel('X Position');
ylabel('Y Position');
%grid on; % Adds a grid to the plot for better readability
%axis equal; % Ensures that the scale of the plot is equal in both X and Y directions

% Error Plot
figure;
plot(errorLog);
title('PID Error Over Time');
xlabel('Time Step');
ylabel('Error');

% Control Signal Plot
figure;
hold on;
plot(pidTermsLog(:, 1), 'r-'); % Proportional
plot(pidTermsLog(:, 2), 'g-'); % Integral
plot(pidTermsLog(:, 3), 'b-'); % Derivative
hold off;
legend('Proportional', 'Integral', 'Derivative');
title('PID Control Terms Over Time');
xlabel('Time Step');
ylabel('Control Terms');

% Heat Map Visualization
figure;
imagesc(heatMap);
colorbar;
title('Heat Map of Robot Position');
xlabel('X Position');
ylabel('Y Position');
axis equal;

% State Transition Diagram
figure;
stairs(stateLog);
title('State Transitions Over Time');
xlabel('Time Step');
ylabel('State');
ylim([0 max(stateLog) + 1]); % Adjust based on state values
yticks(unique(stateLog));
yticklabels({'Random Wandering', 'Wall Following'}); % Add more labels if you have more states

end

%%%% Helper Functions %%%%
function handleCorner(serPort)
    % Reverse a bit
    SetDriveWheelsCreate(serPort, -0.2, -0.2);
    pause(1); % Reverse for 1 second

    % Turn a larger angle
    turnAngle(serPort, 0.2, 90); % Turn 90 degrees
end

function [SonFF, SonRight, SonLeft] = readSensors(serPort)
    % Reads the front and right sonar sensors
    SonFF = ReadSonar(serPort, 2);    % Front sonar
    SonRight = ReadSonar(serPort, 3); % Right sonar
	SonLeft = ReadSonar(serPort, 1); % Left sonar
end

function isCorner = isCorner(SonFF, SonRight, wallFollowDistance)
    isCorner = SonFF < wallFollowDistance && SonRight < wallFollowDistance;
end


function isStuck = isStuck(positionLog)
    % Determine if the robot is stuck
    % Check if there has been little movement over the last few readings
    if size(positionLog, 1) > 5
        movement = sqrt(diff(positionLog(end-5:end, 1)).^2 + diff(positionLog(end-5:end, 2)).^2);
        isStuck = all(movement < someSmallValue);
    else
        isStuck = false;
    end
end

function escapeStuck(serPort)
    % Maneuver to escape from being stuck
    SetDriveWheelsCreate(serPort, -baseSpeed, -baseSpeed); % Reverse
    pause(1);
    SetDriveWheelsCreate(serPort, baseSpeed, -baseSpeed); % Spin
    pause(1);
end

function randomWandering(serPort)
    % Random wandering behavior
    % Robot moves forward; occasionally turns randomly
    SetDriveWheelsCreate(serPort, 0.3, 0.3); % Move forward
    if rand() < 0.1  % 10% chance to turn randomly
        randomTurnAngle = rand() * 90 - 45; % Random turn between -45 and 45 degrees
        turnAngle(serPort, 0.2, randomTurnAngle); % Pass the random turn angle to the turnAngle function
    end
end

function [integral, lastError, error] = followWall(serPort, SonRight, Kp, Ki, Kd, integral, lastError, desiredDistance, baseSpeed)
    % Ensure that SonRight is a valid numerical value
    if isempty(SonRight) || isnan(SonRight)
        SonRight = desiredDistance; % Default to desired distance if no valid reading
    end

    % PID wall following behavior
    error = desiredDistance - SonRight;
    integral = integral + error;
    derivative = error - lastError;

    % Calculate control signal
    controlSignal = Kp * error + Ki * integral + Kd * derivative;

    % Apply a fraction of the control signal for more gradual correction
    controlSignal = controlSignal * 0.5; % Adjust this factor as needed

    % Calculate the wheel speeds based on the control signal
    leftWheelSpeed = baseSpeed + controlSignal;
    rightWheelSpeed = baseSpeed - controlSignal;

    % Clamp the wheel speeds to be within the valid range
    leftWheelSpeed = max(min(leftWheelSpeed, 0.5), -0.5);
    rightWheelSpeed = max(min(rightWheelSpeed, 0.5), -0.5);

    % Print the wheel speeds for debugging
    %disp(['Left Wheel Speed: ', num2str(leftWheelSpeed), ', Right Wheel Speed: ', num2str(rightWheelSpeed)]);

    % Apply the wheel speeds to the robot's wheels
    SetDriveWheelsCreate(serPort, leftWheelSpeed, rightWheelSpeed);

    % Update lastError for next iteration
    lastError = error;
	
	% Return the current error as well for logging
    error = desiredDistance - SonRight; % Actual error calculation
end

function isWall = isWallNearby(SonFF, SonRight, threshold)
    % Check if there is a wall within the threshold distance
    % Handle the case where SonFF or SonRight might be empty
    isWallFF = ~isempty(SonFF) && SonFF < threshold;
    isWallRight = ~isempty(SonRight) && SonRight < threshold;
    
    isWall = isWallFF || isWallRight;
end