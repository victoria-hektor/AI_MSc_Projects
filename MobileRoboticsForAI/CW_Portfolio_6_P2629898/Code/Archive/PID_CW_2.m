function PID_CW_2(serPort)
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

    % PID Controller Parameters - Refine these based on experimentation
    Kp = 0.5; % Proportional gain
    Ki = 0.05; % Integral gain
    Kd = 0.1; % Derivative gain

    % Initialise PID variables
    integralError = 0;
    previousError = 0;
    currentError = 0;
	
	% Initialise arrays for logging
	timeLog = [];
	leftSonarLog = [];
	rightSonarLog = [];
	frontSonarLog = [];
	modeLog = []; % 1 for wandering, 2 for wall-following


    % Main control loop
    while true
        % Check if 5 minutes have passed
        if toc(startTime) > 300 % 300 seconds = 5 minutes
            break; % Exit the loop to stop the robot
        end

        % Enhanced Obstacle Detection
        % Read left, right, and front sonar values
        SonL = ReadSonar(serPort, 1); % Left sonar
        SonR = ReadSonar(serPort, 3); % Right sonar
        SonFF = ReadSonar(serPort, 2); % Front sonar
		
        % Assign a default value if no object is detected
        if ~any(SonL) 
            SonL = 100;
        end
        if ~any(SonR) 
            SonR = 100;
        end
        if ~any(SonFF) 
            SonFF = 100;
        end
		
		% Increase the front sonar threshold
		if SonFF < 1.5 % Adjust this value as needed
			% Logic to move away from the wall or turn
		end
		
		% Subtle turning away from the wall
		if SonFF < 1.5 % Front sonar detects wall close
			turnAngle(serPort, 0.2, 45); % Adjust angle as needed
		end
		
		% This structure first checks if the front sonar detects an obstacle within 1.5 meters. If so, it then checks the left sonar:
		% If the left side is clear (SonL > 1.5), it turns to the side.
		% If the left side is not clear, it performs a gradual turn based on how close the front obstacle is.
		if SonFF < 1.5
			if SonL > 1.5
				turnAngle(serPort, 0.2, 90); % Turn to the side
			else
				turnAngleValue = (1.5 - SonFF) * 90; % Gradual turn
				turnAngle(serPort, 0.2, turnAngleValue);
			end
		end

		% Randomised movement if stuck in a pattern
		if SonFF < 1.5 % Repeatedly detecting an obstacle
			randomTurnAngle = rand() * 180 - 90; % Larger random turn
			turnAngle(serPort, 0.2, randomTurnAngle);
		end
		
        if isWandering
            % Random Wandering Logic
            randomDistance = rand() * 10 + 3; % Random distance between 3 and 13 meters
            randomAngle = rand() * 60 - 30; % Random angle between -30 and 30 degrees
            
            % Move the robot for the randomly determined distance and angle
            travelDist(serPort, 0.5, randomDistance); % Travel at 0.5 m/s
            turnAngle(serPort, 0.2, randomAngle); % Turn at 0.2 rad/s
			
		% Check for a corner and perform a significant turn if detected
		if SonFF < 1 && SonL < 0.5 % Corner detected
			travelDist(serPort, -0.2, 0.5); % Back up a little
			turnAngle(serPort, 0.2, 135); % Turn away from the corner more significantly
			isWandering = true; % Switch to wandering mode
		end

		% Implement a 'stuck' detection mechanism
		if stuckCondition % Define your stuck condition based on time or lack of movement
			turnAngle(serPort, 0.2, 180); % Turn around completely
			travelDist(serPort, 0.5, 1); % Move away from the current spot
		end

		% Variable speed control based on proximity to walls
		if SonFF < 1.5 % Close to a wall
			baseSpeed = 0.2; % Slow down when close to a wall
		else
			baseSpeed = 0.4; % Speed up when farther away from a wall
		end

		% Corner Detection and Escape Maneuver
		if SonFF < 1 && SonL < 0.5 % Assuming a corner is detected
			% Escape Maneuver
			travelDist(serPort, -0.2, 0.5); % Move backwards
			turnAngle(serPort, 0.2, 120); % Turn more sharply
			isWandering = true;
		elseif SonFF < 1 % Wall detected in front but not necessarily a corner
			% Regular Obstacle Avoidance Logic
			travelDist(serPort, -0.2, 0.5); % Move backwards
			randomTurnAngle = rand() * 180 - 90; % Regular turn
			turnAngle(serPort, 0.2, randomTurnAngle);
		end

        else
            % Wall Following Logic
            % Define the ideal distance from the wall (desiredState)
            desiredState = 0.5; % meters
    
            % Read the current distance from the wall (currentState)
            currentState = SonL; % Using left sonar sensor

            % Adjust Wall-Following Behavior
            % If too close to a wall, adjust position
            if currentState < 0.3 % If too close to the wall
                turnAngle(serPort, 0.2, 15); % Turn right slightly
            end
    
            % PID error calculation
            currentError = desiredState - currentState;

            % Recalculate PID values
            P = Kp * currentError;
            I = Ki * integralError + currentError;
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
            if SonFF > 1.5 % If enough space is detected in front, switch to wandering
                isWandering = true;
            end
        end
		
		currentTime = toc(startTime);
		timeLog = [timeLog, currentTime];
		leftSonarLog = [leftSonarLog, SonL];
		rightSonarLog = [rightSonarLog, SonR];
		frontSonarLog = [frontSonarLog, SonFF];
		modeLog = [modeLog, isWandering + 1]; % 1 for wandering, 2 for wall-following
		
        % Update previous error for next iteration
        previousError = currentError;
        integralError = integralError + currentError; % Update integral error
    end % End of main control loop

    % Stop the robot
    SetDriveWheelsCreate(serPort, 0, 0);
    disp('Program Ending - Time Limit Reached')

% Plotting the sensor data
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

end % End of function
