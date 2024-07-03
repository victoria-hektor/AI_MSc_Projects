function PID_CW(serPort)
    % Robot moves along a path, part blindly, part sensing.
    % serPort is the serial port number (for controlling the actual robot).

    %%%% DO NOT MODIFY CODE ABOVE %%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    disp('==================')
    disp('Program Starting')
    disp('------------------')

    % State Variable to switch between wandering and wall following
    isWandering = true;

    % Start the timer
    startTime = tic; % Initialise the timer here

    % PID Controller Parameters
    Kp = 1; % Proportional gain
    Ki = 0; % Integral gain
    Kd = 0; % Derivative gain

    % Initialise PID variables
    integralError = 0;
    previousError = 0;
	currentError = 0;

    % Main control loop
    while true
        % Check if 5 minutes have passed
        if toc(startTime) > 300 % 300 seconds = 5 minutes
            break; % Exit the loop to stop the robot
        end

        if isWandering
            % Random Wandering Logic
            randomDistance = rand() * 10 + 3; % Random distance between 3 and 13 meters
            randomAngle = rand() * 60 - 30; % Random angle between -30 and 30 degrees
            
            % Move the robot for the randomly determined distance and angle
            travelDist(serPort, 0.5, randomDistance); % Travel at 0.5 m/s
            turnAngle(serPort, 0.2, randomAngle); % Turn at 0.2 rad/s

            % Read front sonar value
            SonFF = ReadSonar(serPort, 2);
            if ~any(SonFF) 
                SonFF = 100;
            end

            % Check for wall
            if SonFF < 1 % Assuming wall is detected within 1 meter
                isWandering = false;
            end
			
			currentError = 0;
        else
            % Wall Following Logic
            % Define the ideal distance from the wall (desiredState)
            desiredState = 0.5; % meters
    
            % Read the current distance from the wall (currentState)
            currentState = ReadSonar(serPort, 1); % Assuming left sonar sensor
    
            % PID error calculation
            currentError = desiredState - currentState;

            % Recalculate PID values
            P = Kp * currentError;
            I = Ki * integralError;
            D = Kd * (currentError - previousError);
            PID_output = P + I + D;
    
            % Adjust robot's speed based on PID output
            baseSpeed = 0.4; % Base speed for both wheels
            correction = PID_output; % Correction factor from PID controller
            leftWheelSpeed = baseSpeed - correction;
            rightWheelSpeed = baseSpeed + correction;
    
            % Ensure the wheel speeds are within the robot's operational limits
            maxSpeed = 0.5; % Define maximum speed
            minSpeed = 0;   % Define minimum speed
            leftWheelSpeed = max(min(leftWheelSpeed, maxSpeed), minSpeed);
            rightWheelSpeed = max(min(rightWheelSpeed, maxSpeed), minSpeed);
    
            % Set the robot's wheel speeds
            SetDriveWheelsCreate(serPort, leftWheelSpeed, rightWheelSpeed);

            % Check for open space to switch back to wandering
            SonFF = ReadSonar(serPort, 2); % Read front sonar value
            if ~any(SonFF) 
                SonFF = 100;
            end
            if SonFF > 1.5 % If enough space is detected in front, switch to wandering
                isWandering = true;
            end
        end

        % Update previous error for next iteration
        previousError = currentError;
    end % End of main control loop

    % Stop the robot
    SetDriveWheelsCreate(serPort, 0, 0);
    disp('Program Ending - Time Limit Reached')
end % End of function