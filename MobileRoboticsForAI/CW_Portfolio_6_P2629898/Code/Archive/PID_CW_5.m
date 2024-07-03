function PID_CW_2(serPort)
    % Robot moves along a path, part blindly, part sensing.
    % serPort is the serial port number (for controlling the actual robot).

    disp('==================')
    disp('Program Starting')
    disp('------------------')

    % Initialize variables
    isWandering = true;
    baseSpeed = 0.4; % Default speed
    stuckThresholdTime = 10; % Time in seconds to consider if stuck
    lastUnstuckTime = tic; % Timer to check stuck condition
    stuck = false; % Flag to indicate if robot is stuck

    % Start the timer for the entire program
    startTime = tic;

    % PID Controller Parameters
    Kp = 0.5;
    Ki = 0.05;
    Kd = 0.1;

    % Initialize PID variables
    integralError = 0;
    previousError = 0;
    currentError = 0;

    % Initialize arrays for logging
    timeLog = [];
    leftSonarLog = [];
    rightSonarLog = [];
    frontSonarLog = [];
    modeLog = []; % 1 for wandering, 2 for wall-following

    % Main control loop
    while true
        % Check if 5 minutes have passed
        if toc(startTime) > 300
            break; % Exit the loop to stop the robot
        end

        % Read sonar values
        SonL = ReadSonarOrDefault(serPort, 1, 100);
        SonR = ReadSonarOrDefault(serPort, 3, 100);
        SonFF = ReadSonarOrDefault(serPort, 2, 100);

        % Check if the robot is stuck
        if toc(lastUnstuckTime) > stuckThresholdTime && not(stuck)
            % Perform actions to unstuck the robot
            stuck = true; % Set the stuck flag
            turnAngle(serPort, 0.2, 180); % Turn around completely
            travelDist(serPort, 0.5, 1); % Move away from the current spot
        elseif SonFF > 1.5
            % If the robot has a clear path, reset the stuck condition
            stuck = false;
            lastUnstuckTime = tic;
        end

        % Wall and corner detection and handling
        if SonFF < 1 && SonL < 0.5
            % Corner detected
            isWandering = true;
            travelDist(serPort, -0.2, 0.5); % Move backwards
            turnAngle(serPort, 0.2, 135); % Turn away from the corner
        elseif SonFF < 1.5
            if SonL > 1.5
                turnAngle(serPort, 0.2, 90); % Turn to the side
            else
                turnAngleValue = (1.5 - SonFF) * 90;
                turnAngle(serPort, 0.2, turnAngleValue); % Gradual turn
            end
            baseSpeed = 0.2; % Slow down when close to a wall
        else
            baseSpeed = 0.4; % Default speed
        end

        % Randomized movement if the front sonar keeps detecting obstacles
        if SonFF < 1.5 && not(stuck)
            randomTurnAngle = rand() * 180 - 90;
            turnAngle(serPort, 0.2, randomTurnAngle);
        end

        % Wandering and wall-following logic
        if isWandering
            % Wandering logic
            randomDistance = rand() * 10 + 3;
            randomAngle = rand() * 60 - 30;
            travelDist(serPort, 0.5, randomDistance);
            turnAngle(serPort, 0.2, randomAngle);
        else
            % Wall-following logic
            desiredState = 0.5; % Ideal distance from the wall
            currentState = SonL;
            if currentState < 0.3
                turnAngle(serPort, 0.2, 15); % Adjust if too close to the wall
            end

            % PID control logic
            currentError = desiredState - currentState;
            integralError = integralError + currentError; % Update integral error
            P = Kp * currentError;
            I = Ki * integralError;
            D = Kd * (currentError - previousError);
            PID_output = P + I + D;

            % Calculate wheel speeds
            leftWheelSpeed = baseSpeed - PID_output;
            rightWheelSpeed = baseSpeed + PID_output;

            % Ensure speeds are within limits
            maxSpeed = 0.5;
            minSpeed = 0;
            leftWheelSpeed = max(min(leftWheelSpeed, maxSpeed), minSpeed);
            rightWheelSpeed = max(min(rightWheelSpeed, maxSpeed), minSpeed);

            % Set wheel speeds
            SetDriveWheelsCreate(serPort, leftWheelSpeed, rightWheelSpeed);

            % Switch to wandering if there's enough space
            if SonFF > 1.5
                isWandering = true;
            end
        end

        % Logging data
        currentTime = toc(startTime);
        timeLog = [timeLog, currentTime];
        leftSonarLog = [leftSonarLog, SonL];
        rightSonarLog = [rightSonarLog, SonR];
        frontSonarLog = [frontSonarLog, SonFF];
        modeLog = [modeLog, isWandering + 1];

        % Update previous error for next iteration
        previousError = currentError;
    end

    % Stop the robot and plot data
    SetDriveWheelsCreate(serPort, 0, 0);
    disp('Program Ending - Time Limit Reached');
    PlotSensorData(timeLog, leftSonarLog, rightSonarLog, frontSonarLog, modeLog);
end

% Helper function to read a sonar value or return a default if no object is detected
function value = ReadSonarOrDefault(serPort, sonarIndex, default)
    value = ReadSonar(serPort, sonarIndex);
    if isempty(value)
        value = default;
    end
end

% Helper function to plot the sensor data
function PlotSensorData(timeLog, leftSonarLog, rightSonarLog, frontSonarLog, modeLog)
    figure;
    subplot(4,1,1);
    plot(timeLog, leftSonarLog);
    title('Left Sonar Readings');
    xlabel('Time (s)');
    ylabel('Distance (m)');
    
    subplot(4,1,2);
    plot(timeLog, rightSonarLog);
    title('Right Sonar Readings');
    xlabel('Time (s)');
    ylabel('Distance (m)');
    
    subplot(4,1,3);
    plot(timeLog, frontSonarLog);
    title('Front Sonar Readings');
    xlabel('Time (s)');
    ylabel('Distance (m)');
    
    subplot(4,1,4);
    stairs(timeLog, modeLog);
    title('Robot Mode');
    xlabel('Time (s)');
    ylabel('Mode (1-Wandering, 2-Wall Following)');
    
    % Ensure each subplot has the same x-axis
    linkaxes(findall(gcf, 'Type', 'axes'), 'x');
end
