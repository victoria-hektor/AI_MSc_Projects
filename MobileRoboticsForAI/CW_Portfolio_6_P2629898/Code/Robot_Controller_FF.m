function Robot_Controller_FF(serPort)
% Controls a robot to follow walls using PID control.
% serPort is the serial port number (for controlling the actual robot).

%%%% Initial Setup %%%%
disp('PID Wall Following Programme Starting');

% Start the timer
startTime = tic;

% Define robot's behaviour state
STATE_WALL_FOLLOWING = 2;
STATE_SEEKING_WALL = 1;
currentState = STATE_SEEKING_WALL;  % Start with seeking wall

% PID constants based on "PID without a PhD"
Kp = 0.1;  % Proportional gain
Ki = 0.005; % Integral gain
Kd = 0.1; % Derivative gain

% Initialise PID variables
integral = 0;      % Integral term
lastError = 0;     % Last error term for derivative
desiredDistance = 0.5; % Desired distance from the wall

% Define the base speed for the robot
baseSpeed = 0.1;

% Define the threshold for detecting a wall (adjust as needed)
wallDetectionThreshold = 0.5;

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
lastTime = tic;  % Initialise last time before entering the loop

while toc(startTime) < executionTime
    currentTime = tic;  % Get current time
    dt = toc(lastTime);  % Calculate elapsed time since last iteration
    lastTime = currentTime;  % Update last time for next iteration
    % Read sensors
    [SonFF, SonRight] = readSensors(serPort);
	
	% This section handles it as a feed forward control
	currentTime = tic;  % Get current time
    dt = toc(lastTime);  % Calculate elapsed time since last iteration
    lastTime = currentTime;  % Update last time for next iteration

    % Wall Seeking and Following Logic
    switch currentState
        case STATE_SEEKING_WALL
            % Move forward until a wall is detected on the right or in front
            if any(SonRight < wallDetectionThreshold) || any(SonFF < wallDetectionThreshold)
                currentState = STATE_WALL_FOLLOWING;  % Switch to wall following
            else
                SetDriveWheelsCreate(serPort, baseSpeed, baseSpeed);  % Keep moving forward
            end
        case STATE_WALL_FOLLOWING
            % Normal wall following with corner handling
            [integral, lastError, error] = followWall(serPort, SonFF, SonRight, Kp, Ki, Kd, integral, lastError, desiredDistance, baseSpeed);
            % Log the error and PID terms
            errorLog = [errorLog; error];
            pidTermsLog = [pidTermsLog; Kp*error, Ki*integral, Kd*(error - lastError)];
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
yticks([STATE_SEEKING_WALL, STATE_WALL_FOLLOWING]);
yticklabels({'Seeking Wall', 'Wall Following'});

end

%%%% Helper Functions %%%%
function [SonFF, SonRight] = readSensors(serPort)
    % Reads the front and right sonar sensors
    SonFF = ReadSonar(serPort, 2);    % Front sonar
    SonRight = ReadSonar(serPort, 1); % Right sonar
	% Debug:
	disp(['SonFF: ', num2str(SonFF), ' SonRight: ', num2str(SonRight)]);
end

function [integral, lastError, error] = followWall(serPort, SonFF, SonRight, Kp, Ki, Kd, integral, lastError, desiredDistance, baseSpeed, dt)
    % Ensure that SonRight and SonFF are valid numerical values
    if isempty(SonRight) || isnan(SonRight)
        SonRight = inf; % Use infinity to indicate no wall detected
    end
    if isempty(SonFF) || isnan(SonFF)
        SonFF = inf; % Use infinity to indicate no obstacle in front
    end

    % PID wall following behavior
    error = desiredDistance - SonRight;
    integral = integral + error;
    derivative = error - lastError;

    % Calculate control signal
    %controlSignal = Kp * error + Ki * integral + Kd * derivative;
	
	% Calculate the PID control signal
    pidControlSignal = Kp * error + Ki * integral + Kd * derivative;
	
	%%% Feedforward control %%%
    % Assume a desired constant distance to the wall. If the robot is moving
    % straight, the distance to the wall should not change. Any deviation from
    % this is considered in the feedforward control.
    % In this simple case, just assume feedforwardControlSignal is 0.
    feedforwardControlSignal = 0;

    % Combine PID control signal with Feedforward control signal
    controlSignal = pidControlSignal + feedforwardControlSignal;

    % Calculate control signal with saturation
	maxControlSignal = 0.1; % Maximum allowable control signal
	%controlSignal = max(min(Kp * error + Ki * integral + Kd * derivative, maxControlSignal), -maxControlSignal);
	
	% Inside followWall function, adjust control signal calculation
	controlSignal = Kp * error + Ki * integral + Kd * derivative;
	if error < 0
		% If too close to the wall, limit the control signal to prevent sharp turns
		controlSignal = max(controlSignal, -0.05);
	end

    % Calculate the wheel speeds based on the control signal
    leftWheelSpeed = baseSpeed + controlSignal;
    rightWheelSpeed = baseSpeed - controlSignal;

    % Clamp the wheel speeds to be within the valid range
    leftWheelSpeed = max(min(leftWheelSpeed, 0.5), -0.5);
    rightWheelSpeed = max(min(rightWheelSpeed, 0.5), -0.5);

    % Determine if there is no wall on the right and no corner ahead
    wallAbsentThreshold = 2.0; % Define the threshold for determining if there is no wall on the right
    cornerThreshold = 0.3; % Define the threshold distance for detecting a corner

    if SonRight > wallAbsentThreshold && SonFF > cornerThreshold
        % No wall on the right and no corner ahead, move forward
        SetDriveWheelsCreate(serPort, baseSpeed, baseSpeed);
    elseif SonFF < cornerThreshold
        % Corner detected, execute turn maneuver
        handleCorner(serPort, baseSpeed);  % Passing baseSpeed to handleCorner
    else
        % Apply the wheel speeds to the robot's wheels for wall following
        SetDriveWheelsCreate(serPort, leftWheelSpeed, rightWheelSpeed);
    end

    % Update lastError for next iteration
    lastError = error;
    
    % Return the current error as well for logging
    error = desiredDistance - SonRight; % Actual error calculation
end

function handleCorner(serPort, baseSpeed)
    % Define turning parameters
    % Speeds for turning: positive for left wheel, negative for right wheel to turn clockwise
    leftWheelSpeed = 0.1;
    rightWheelSpeed = -0.1;

    % Stop the robot before making a turn
    SetDriveWheelsCreate(serPort, 0, 0);
    pause(0.5); % Short pause to stabilise the robot

    % Start turning
    SetDriveWheelsCreate(serPort, leftWheelSpeed, rightWheelSpeed);
    
    % Assuming it takes approximately 2 seconds to turn 90 degrees
    pause(2);

    % Stop the turn
    SetDriveWheelsCreate(serPort, 0, 0);
    pause(0.5); % Short pause after the turn

    % Move forward a little to align with the next wall
    SetDriveWheelsCreate(serPort, baseSpeed, baseSpeed);
    pause(1);
end