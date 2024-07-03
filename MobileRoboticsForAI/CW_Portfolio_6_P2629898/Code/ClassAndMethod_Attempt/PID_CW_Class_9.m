
classdef RobotController
    properties
        serPort;         % Serial port for communication
        currentState;    % Current state of the robot
        Kp = 0.4;        % Proportional gain
        Ki = 0.001;      % Integral gain
        Kd = 0.4;        % Derivative gain
        integral = 0;    % Integral term for PID
        lastError = 0;   % Last error term for PID
        desiredDistance = 0.5; % Desired distance from the wall
        baseSpeed = 0.1; % Base speed of the robot
        executionTime = 600; % Execution time (10 minutes)
        startTime;       % Start time of the operation
		
		% Odometry
		x = 0;
        y = 0;
        theta = 0;
        positionLog = [];
        % Additional properties for logging, odometry, etc. can be added
    end
    
	methods
		% Constructor
		function obj = RobotController(serPort)
			obj.serPort = serPort;
			obj.currentState = 'seekingWall'; % Initial state
			obj.startTime = tic; % Start the timer
		end

		% Constructor
		function obj = RobotController(serPort)
			obj.serPort = serPort;
			obj.currentState = 1; % For example, 1 could be random wandering
			obj.integral = 0;
			obj.lastError = 0;
			% Initialise properties here...
		end

		function startControlLoop(obj)
			while toc(obj.startTime) < obj.executionTime
				switch obj.currentState
					case 'seekingWall'
						obj.seekWall();
					case 'followingWall'
						obj.followWall();
				end
				
				% Update odometry and handle stuck situations
				obj.updateOdometry();
				if obj.isStuck()
					obj.handleStuck();
				end
				
				pause(0.1); % To prevent overloading
			end
		end

		function seekWall(obj)
			[SonFF, SonRight, SonLeft] = obj.readSensors();

			% Logic to actively seek a wall
			if isempty(SonFF) || SonFF > obj.desiredDistance
				% If no wall in front, move forward
				SetDriveWheelsCreate(obj.serPort, obj.baseSpeed, obj.baseSpeed);
			elseif ~isempty(SonFF) && SonFF <= obj.desiredDistance
				% If a wall is detected in front, decide which way to turn
				if (isempty(SonRight) || SonRight > obj.desiredDistance) && ...
				   (isempty(SonLeft) || SonLeft > obj.desiredDistance)
					% If both sides are clear, choose a side to turn randomly
					turnDirection = randi([0, 1]) * 2 - 1; % -1 for left, 1 for right
					turnAngle(obj.serPort, 0.2, 90 * turnDirection);
				elseif ~isempty(SonRight) && SonRight <= obj.desiredDistance
					% If right side is blocked, turn left
					turnAngle(obj.serPort, 0.2, -90);
				else
					% If left side is blocked (or both sides), turn right
					turnAngle(obj.serPort, 0.2, 90);
				end
			end

			% Check if the wall is close enough to switch to following
			if obj.isWallNearby(SonFF, SonRight, SonLeft, obj.desiredDistance)
				obj.currentState = 'followingWall';
			end
		end

		function followWall(obj)
			[SonFF, SonRight, SonLeft] = obj.readSensors();

			% Implement PID control for wall following
			if ~isempty(SonRight)
				error = obj.desiredDistance - SonRight;
				obj.integral = obj.integral + error;
				derivative = error - obj.lastError;

				% Calculate control signal
				controlSignal = obj.Kp * error + obj.Ki * obj.integral + obj.Kd * derivative;

				% Apply control signal to wheel speeds
				leftWheelSpeed = obj.baseSpeed + controlSignal;
				rightWheelSpeed = obj.baseSpeed - controlSignal;

				% Clamp the wheel speeds to be within the valid range
				leftWheelSpeed = max(min(leftWheelSpeed, 0.5), -0.5);
				rightWheelSpeed = max(min(rightWheelSpeed, 0.5), -0.5);

				% Apply the wheel speeds to the robot's wheels
				SetDriveWheelsCreate(obj.serPort, leftWheelSpeed, rightWheelSpeed);

				% Update lastError for next iteration
				obj.lastError = error;
			end
		end

		function stuck = isStuck(obj)
			% Logic to determine if the robot is stuck
			% This could be based on the lack of change in the robot's position
			% over a certain period or other criteria depending on available sensors
			% For example, checking if the robot hasn't moved much in the last few readings

			% Using positionLog as a property storing the robot's recent positions
			if size(obj.positionLog, 1) > 10
				recentMovements = diff(obj.positionLog(end-10:end, :));
				distanceMoved = sqrt(sum(recentMovements.^2, 2));
				stuck = all(distanceMoved < someSmallThreshold); % someSmallThreshold needs to be defined
			else
				stuck = false;
			end
		end

		function handleStuck(obj)
			obj.escapeStuck(); % Call to escape if stuck
			obj.currentState = 'seekingWall'; % Resume seeking wall after escaping
		end

		function escapeStuck(obj)
			% Reverse a bit
			SetDriveWheelsCreate(obj.serPort, -obj.baseSpeed, -obj.baseSpeed);
			pause(1); % Reverse for 1 second

			% Turn a random angle
			randomTurnAngle = rand() * 180 - 90; % Random turn between -90 and 90 degrees
			turnAngle(obj.serPort, 0.2, randomTurnAngle);

			% Optionally, resume moving forward after turning
			SetDriveWheelsCreate(obj.serPort, obj.baseSpeed, obj.baseSpeed);
			pause(1); % Move forward for 1 second
		end

		function updateOdometry(obj)
			% Get distance travelled and angle turned since last call
			distance = DistanceSensorRoomba(obj.serPort);
			angle = AngleSensorRoomba(obj.serPort);

			% Update position and orientation
			obj.theta = obj.theta + deg2rad(angle);
			obj.x = obj.x + distance * cos(obj.theta);
			obj.y = obj.y + distance * sin(obj.theta);

			% Log the current position and orientation
			obj.positionLog = [obj.positionLog; obj.x, obj.y];
		end
		
		% Helper methods: below, isWall, isStuck
		function [SonFF, SonRight, SonLeft] = readSensors(obj)
			% Reads the front, right, and left sonar sensors
			SonFF = ReadSonar(obj.serPort, 2);    % Front sonar
			SonRight = ReadSonar(obj.serPort, 3); % Right sonar
			SonLeft = ReadSonar(obj.serPort, 1);  % Left sonar
		end

		function isWall = isWallNearby(obj, SonFF, SonRight, SonLeft, threshold)
			% Check if there is a wall within the threshold distance
			isWallFF = ~isempty(SonFF) && SonFF < threshold;
			isWallRight = ~isempty(SonRight) && SonRight < threshold;
			isWallLeft = ~isempty(SonLeft) && SonLeft < threshold;

			isWall = isWallFF || isWallRight || isWallLeft;
		end
		
		%%%% Plotting Methods %%%%
		function plotPathFollowed(obj)
			% Plotting the path followed by the robot
			figure; % Creates a new figure window
			plot(positionLog(:, 1), positionLog(:, 2), 'r-'); % Plot x and y from positionLog with a red line
			title('Path Followed by the Robot');
			xlabel('X Position');
			ylabel('Y Position');
			%grid on; % Adds a grid to the plot for better readability
			%axis equal; % Ensures that the scale of the plot is equal in both X and Y directions
		end
		
		function plotErrorLog(obj)
			% Error Plot
			figure;
			plot(errorLog);
			title('PID Error Over Time');
			xlabel('Time Step');
			ylabel('Error');
		end
		
		function plotControlTerms(obj)
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
		end
		
		function plotHeatMap(obj)
			% Heat Map Visualization
			figure;
			imagesc(heatMap);
			colorbar;
			title('Heat Map of Robot Position');
			xlabel('X Position');
			ylabel('Y Position');
			axis equal;
		end
		
		function plotStateTransitions(obj)
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
		
    end
end
