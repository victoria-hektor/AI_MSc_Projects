function PID_Follow_Wall(serPort)
% Controls a robot to random wander and follow walls using PID control.
% serPort is the serial port number (for controlling the actual robot).

%%%% Initial Setup %%%%
disp('PID Wall Following Programme Starting');

% Start the timer
startTime = tic;

% Define robot's behavior state as wall following from the start
STATE_WALL_FOLLOWING = 1;
currentState = STATE_WALL_FOLLOWING;

% PID constants based on "PID without a PhD"
Kp = 0.4;  % Proportional gain
Ki = 0.001; % Integral gain
Kd = 0.4; % Derivative gain

% Initialise PID variables
integral = 0;      % Integral term
lastError = 0;     % Last error term for derivative
desiredDistance = 0.5; % Desired distance from the wall
maxSensorRange = 2; % Need to check what this is...?

% PID control signal scaling factor
pidScalingFactor = 0.5; % Adjust this factor as needed

% Define turning parameters for corners
turnSpeed = 0.2; % Speed for turning
turnAngle = 90; % Angle to turn at corners, in degrees

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
	% Initialise error value at the start of each loop iteration
	error = 0;
    % Read sensors
    [SonFF, SonRight] = readSensors(serPort);

    % Wall Following Logic
    if isempty(SonRight) || isnan(SonRight)
        % If no valid reading, assume maximum sensor range
        SonRight = maxSensorRange;
    end
    
    if SonRight > desiredDistance
        % Robot is too far from the wall, move closer
        handleApproachWall(serPort, Kp, desiredDistance, baseSpeed);
    elseif SonRight < desiredDistance
        % Robot is too close to the wall, move away
        handleAwayWall(serPort, Kp, desiredDistance, baseSpeed);
    else
        % Maintain a fixed distance from the wall
        [integral, lastError, error] = followWall(serPort, SonRight, Kp, Ki, Kd, integral, lastError, desiredDistance, baseSpeed);
    end

    % Log the error and PID terms
    errorLog = [errorLog; error];
    pidTermsLog = [pidTermsLog; Kp*error, Ki*integral, Kd*(error - lastError)];

    % Check for corners
    if SonFF < desiredDistance
        % If a wall is detected in front, handle the corner
        handleCorner(serPort);
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

% Heat Map Visualisation
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
yticklabels({'Wall Following'}); % Add more labels if more states

end

%%%% Helper Functions %%%%
function handleApproachWall(serPort, Kp, desiredDistance, baseSpeed)
    % Function to handle approaching the wall
    % Kp: proportional gain for approaching wall
    % desiredDistance: the desired distance to maintain from the wall
    % baseSpeed: the base speed of the robot
    
    % Get the current distance from the right wall
    [~, SonRight] = readSensors(serPort);
    
    % Calculate the error
    error = desiredDistance - SonRight;
    
    % Calculate the control signal for approaching the wall
    controlSignal = Kp * error;

    % Scale down the control signal even further if necessary
    controlSignal = min(max(controlSignal, -0.5), 0.5); % Limit the control signal range
    
    % Calculate wheel speeds to approach the wall
    leftWheelSpeed = baseSpeed - controlSignal;
    rightWheelSpeed = baseSpeed + controlSignal;
	
	% Debugging: print the calculated wheel speeds
    disp(['Left Wheel Speed (before clamping): ', num2str(leftWheelSpeed)]);
    disp(['Right Wheel Speed (before clamping): ', num2str(rightWheelSpeed)]);

    % Clamp the wheel speeds to be within the valid range
    leftWheelSpeed = max(min(leftWheelSpeed, 0.5), -0.5);
    rightWheelSpeed = max(min(rightWheelSpeed, 0.5), -0.5);

    % Debugging: print the clamped wheel speeds
    disp(['Left Wheel Speed (after clamping): ', num2str(leftWheelSpeed)]);
    disp(['Right Wheel Speed (after clamping): ', num2str(rightWheelSpeed)]);

    % Apply the wheel speeds
    SetDriveWheelsCreate(serPort, leftWheelSpeed, rightWheelSpeed);
end

function handleAwayWall(serPort, Kp, desiredDistance, baseSpeed)
    % Function to handle moving away from the wall
    % Kp: proportional gain for moving away from wall
    % desiredDistance: the desired distance to maintain from the wall
    % baseSpeed: the base speed of the robot
    
    % Get the current distance from the right wall
    [~, SonRight] = readSensors(serPort);
    
    % Calculate the error
    error = SonRight - desiredDistance;
    
    % Calculate the control signal for moving away from the wall
    controlSignal = Kp * error;
	
	% Scale down the control signal
	controlSignal = controlSignal * 0.5; % Adjust this factor as needed
    
    % Calculate wheel speeds to move away from the wall
    leftWheelSpeed = baseSpeed + controlSignal;
    rightWheelSpeed = baseSpeed - controlSignal;
	
	% Clamp the wheel speeds to be within the valid range
    leftWheelSpeed = max(min(leftWheelSpeed, 0.5), -0.5);
    rightWheelSpeed = max(min(rightWheelSpeed, 0.5), -0.5);
    
    % Apply the wheel speeds
    SetDriveWheelsCreate(serPort, leftWheelSpeed, rightWheelSpeed);
end

function handleCorner(serPort, turnSpeed, turnAngle)
    % Function to handle maneuvering around corners
    % turnSpeed: the speed to use while turning
    % turnAngle: the angle to turn, typically 90 degrees for a right angle turn
    
    % Stop the robot before making a turn
    SetDriveWheelsCreate(serPort, 0, 0);
    pause(0.5); % Short pause to stabilize the robot
    
    % Turn right to align with the next wall
    TurnAngle(serPort, turnSpeed, turnAngle);
    
    % Brief pause to complete the turn
    pause(1);
    
    % Move forward a little to prevent the robot from turning in place
    SetDriveWheelsCreate(serPort, baseSpeed, baseSpeed);
    pause(1);
end

function [SonFF, SonRight] = readSensors(serPort)
    % Reads the front and right sonar sensors
    SonFF = ReadSonar(serPort, 2);    % Front sonar
    SonRight = ReadSonar(serPort, 3); % Right sonar
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