%% Object Oriented Method of Implementing a PID Controller Using iRobot Create Toolbox %%
%% VH - P2629898 %%

classdef RobotController
    properties
        serPort;                  % Serial port for communication
        currentState;             % Current state of the robot
        Kp = 0.5;                 % Proportional gain
        Ki = 0.01;                % Integral gain
        Kd = 0.2;                 % Derivative gain
        integral = 0;             % Integral term for PID
        lastError = 0;            % Last error term for PID
        desiredDistance = 1;      % Desired distance from the wall
        baseSpeed = 0.2;          % Base speed of the robot
        executionTime = 300;      % Execution time (5 minutes)
        startTime;                % Start time of the operation
        someMargin = 0.1;         % Margin of the desired distance

        %%%%%% Odometry %%%%%%
        x = 0;
        y = 0;
        theta = 0;
        positionLog = [];

        %%%%%% Logging variables %%%%%%
        errorLog = [];            % Log for PID errors
        pidTermsLog = [];         % Log for P, I, and D terms
        stateLog = [];            % Log for state transitions
        heatMap;                  % Heat map for position tracking
    end

    methods
        %%%%%% Constructor %%%%%
        function obj = RobotController(serPort)
            obj.serPort = serPort;
            obj.currentState = 'seekingWall'; % Initial state
            obj.startTime = tic; % Start the timer
            obj.heatMap = zeros(10, 10); % Initialise heat map
        end

		%%%%%% Main control loop %%%%%
		function startControlLoop(obj)
			disp(['Control loop started at ', datestr(now)]);
			while toc(obj.startTime) < obj.executionTime
				disp(['Current state: ', obj.currentState]);
				switch obj.currentState
					case 'seekingWall'
						obj.seekWall();
					case 'followingWall'
						obj.followWall();
				end
				obj.updateOdometry();
				pause(0.1); % To prevent overloading
			end

			disp('Control loop ended, generating plots...');
			obj.plotPathFollowed();
			obj.plotErrorLog();
			obj.plotControlTerms();
			obj.plotHeatMap();
			obj.plotStateTransitions();
			disp(['Control loop ended at ', datestr(now)]);
		end

        function switchToRandomWandering(obj)
            obj.currentState = 'seekingWall';
            disp('Switching to Random Wandering');
        end

        function switchToWallFollowing(obj)
            obj.currentState = 'followingWall';
            disp('Switching to Wall Following');
        end
		
		%%%%%% Wall finder logic %%%%%%
		function seekWall(obj)
			[SonFF, SonRight, SonLeft] = obj.readSensors();

			if isempty(SonFF) || SonFF > obj.desiredDistance
				% Move forward if no wall is detected directly in front
				SetDriveWheelsCreate(obj.serPort, obj.baseSpeed, obj.baseSpeed);
			elseif ~isempty(SonRight) && SonRight <= obj.desiredDistance + obj.someMargin
				% Wall detected on the right side, start following it
				obj.switchToWallFollowing();
			elseif ~isempty(SonFF) && SonFF <= obj.desiredDistance
				% Wall detected directly in front but not on the right, turn towards right
				obj.turnRightToFindWall();
			end

			% Check if the robot is stuck and handle accordingly
			obj.checkAndHandleStuck();
		end

		%%%%%% Wall Following Logic %%%%%%
		function followWall(obj)
			[SonFF, SonRight, SonLeft] = obj.readSensors();

			% Calculate the rate of change of the distance to the right wall
			if ~isempty(obj.errorLog)
				rateOfChange = SonRight - obj.errorLog(end);
			else
				rateOfChange = 0;
			end

			% Add the current distance to the errorLog for future rate of change calculations
			obj.errorLog = [obj.errorLog; SonRight];

			% Adjust speed to maintain desired distance from the right wall
			if ~isempty(SonRight) && SonRight <= obj.desiredDistance + obj.someMargin
				obj.adjustSpeedBasedOnPID(SonRight - obj.desiredDistance);
				obj.cornerCount = 0; % Reset corner count after successful wall following
			elseif isempty(SonRight) || SonRight > obj.desiredDistance + obj.someMargin || abs(rateOfChange) > someRateThreshold
				% If the wall on the right is lost, or the rate of change is high, handle corner
				obj.handleCorner();
			else
				% If no significant rate of change, continue following the wall
				obj.turnRightToFindWall();
			end

			% Check and handle the wall on the left side
			if ~isempty(SonLeft) && SonLeft <= obj.desiredDistance + obj.someMargin
				% If a wall is detected on the left within the margin, adjust orientation
				obj.handleLeftWall(SonLeft);
			end

			% If a wall is detected in front, handle a potential left corner
			if ~isempty(SonFF) && SonFF <= obj.desiredDistance
				obj.handleLeftCorner();
			end

			% Check if the robot is stuck and handle accordingly
			obj.checkAndHandleStuck();
		end

		
		%%%%%% Corner handling logic %%%%%%
		function handleCorner(obj)
			[SonFF, SonRight, SonLeft] = obj.readSensors();
			
			% Slow down as the robot approaches the wall
			slowApproachSpeed = obj.baseSpeed * 0.5; % Slow down to half speed
			SetDriveWheelsCreate(obj.serPort, slowApproachSpeed, slowApproachSpeed);
			
			% If close to a wall in front, stop and turn
			if ~isempty(SonFF) && SonFF < obj.desiredDistance
				% Stop the robot before turning
				SetDriveWheelsCreate(obj.serPort, 0, 0);
				pause(1); % Pause for 1 second to ensure the robot has stopped
				
				% Determine direction to turn based on wall position
				if ~isempty(SonRight) && SonRight < SonFF
					% If the wall is closer on the right, turn left (anticlockwise)
					turnAngle(obj.serPort, obj.baseSpeed, 90);
				elseif ~isempty(SonLeft) && SonLeft < SonFF
					% If the wall is closer on the left, turn right (clockwise)
					turnAngle(obj.serPort, obj.baseSpeed, -90);
				end
				
				% Assume robot is now aligned with the new wall and reset cornering
				obj.isCornering = false;
				
				% Adjust the state to follow the new wall
				obj.currentState = 'followingWall';
			end
		end

		%%%%%% Handle Left Wall Logic %%%%%%
		function handleLeftWall(obj, SonLeft)
			% If the left wall is closer than the right wall, adjust to follow the left wall
			if SonLeft < obj.desiredDistance
				% Slow down the left wheel to turn towards the left wall
				SetDriveWheelsCreate(obj.serPort, obj.baseSpeed * 0.8, obj.baseSpeed);
			else
				% If the left wall is at an ideal distance, maintain current speed
				SetDriveWheelsCreate(obj.serPort, obj.baseSpeed, obj.baseSpeed);
			end
		end

        function turnRightToFindWall(obj)
            % Turn right to find the wall
            turnAngle(obj.serPort, 0.2, -30);
        end

        function turnLeftSlightly(obj)
            % Turn left slightly to align with the wall
            turnAngle(obj.serPort, 0.2, 30);
        end

		function adjustSpeedBasedOnPID(obj, error)
			% Calculate PID control signal
			obj.integral = obj.integral + error;
			derivative = error - obj.lastError;
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
		
		%%%%%% ODOMETRY %%%%%%
		function updateOdometry(obj)
			% Get distance travelled and angle turned since last call
			distance = DistanceSensorRoomba(obj.serPort);
			angle = AngleSensorRoomba(obj.serPort);

			% Debug print
			disp(['Distance: ', num2str(distance), ', Angle: ', num2str(angle)]);

			% Update position and orientation
			obj.theta = obj.theta + deg2rad(angle);
			obj.x = obj.x + distance * cos(obj.theta);
			obj.y = obj.y + distance * sin(obj.theta);

			% Log the current position and orientation
			obj.positionLog = [obj.positionLog; obj.x, obj.y];

			% Debug print
			disp(['Updated Position: ', num2str(obj.x), ', ', num2str(obj.y)]);
		end
		
		%%%%%% Functions for handling walls %%%%%%
		function isWallNearby(obj)
			% Check for walls using sensors and navigate closer to one if found
			[SonFF, SonRight, SonLeft] = obj.readSensors();
			
			% Determine if walls are close on either side or front
			hasWallRight = ~isempty(SonRight) && SonRight <= obj.desiredDistance;
			hasWallLeft = ~isempty(SonLeft) && SonLeft <= obj.desiredDistance;
			hasWallFront = ~isempty(SonFF) && SonFF <= obj.desiredDistance;
			
			% If walls are detected on both sides, randomly choose one to navigate closer to
			if hasWallRight && hasWallLeft
				if rand() < 0.5
					obj.turnTowardsWall('right');
				else
					obj.turnTowardsWall('left');
				end
			elseif hasWallRight
				obj.turnTowardsWall('right');
			elseif hasWallLeft
				obj.turnTowardsWall('left');
			end

			% If there is a wall in front, prepare to turn based on other wall positions
			if hasWallFront
				if hasWallRight
					obj.turnAwayFromWall('right');
				elseif hasWallLeft
					obj.turnAwayFromWall('left');
				else
					% If no walls are detected on either side, randomly choose a direction to turn
					obj.turnRandomDirection();
				end
			end
		end

		function turnTowardsWall(obj, direction)
			% Adjust the robot to move closer to the wall on the specified direction
			if strcmp(direction, 'right')
				% If the wall is on the right, slow down the right wheel to turn towards the wall
				SetDriveWheelsCreate(obj.serPort, obj.baseSpeed, obj.baseSpeed * 0.8);
			elseif strcmp(direction, 'left')
				% If the wall is on the left, slow down the left wheel to turn towards the wall
				SetDriveWheelsCreate(obj.serPort, obj.baseSpeed * 0.8, obj.baseSpeed);
			end
		end

		function turnAwayFromWall(obj, direction)
			% Turn away from the wall in the specified direction
			if strcmp(direction, 'right')
				turnAngle(obj.serPort, obj.baseSpeed, 90); % Turn left by 90 degrees
			elseif strcmp(direction, 'left')
				turnAngle(obj.serPort, obj.baseSpeed, -90); % Turn right by 90 degrees
			end
		end

		function turnRandomDirection(obj)
			% Turn a random angle between -90 and 90 degrees
			randomTurnAngle = (rand() * 180) - 90;
			turnAngle(obj.serPort, obj.baseSpeed, randomTurnAngle);
		end
		
		%%%%%% Read the sensors %%%%%%
		function [SonFF, SonRight, SonLeft] = readSensors(obj)
			% Reads the front, right, and left sonar sensors
			SonFF = ReadSonar(obj.serPort, 2);    % Front sonar
			SonRight = ReadSonar(obj.serPort, 3); % Right sonar
			SonLeft = ReadSonar(obj.serPort, 1);  % Left sonar
		end
		
		%%%%%% Helper Functions for getting out of a stuck position %%%%%%
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

		function checkAndHandleStuck(obj)
			% Check if the robot is stuck by seeing if it has moved significantly in the last 5 seconds.
			% This requires having a timestamp and position log updated regularly in your updateOdometry method.

			% If the robot hasn't moved much, it's considered stuck
			if obj.isStuck()
				disp('Robot is stuck, attempting to free itself.');

				% Check for walls
				[SonFF, SonRight, SonLeft] = obj.readSensors();
				if ~isempty(SonRight) && SonRight < obj.desiredDistance
					% If there's a wall on the right, turn left (anticlockwise)
					turnAngle(obj.serPort, obj.baseSpeed, 90);
				elseif ~isempty(SonLeft) && SonLeft < obj.desiredDistance
					% If there's a wall on the left, turn right (clockwise)
					turnAngle(obj.serPort, obj.baseSpeed, -90);
				else
					% If no wall is detected, turn a random angle
					if rand() > 0.5
						turnAngle(obj.serPort, obj.baseSpeed, 90); % Turn right (clockwise)
					else
						turnAngle(obj.serPort, obj.baseSpeed, -90); % Turn left (anticlockwise)
					end
				end

				% Resume seeking or following the wall
				obj.currentState = 'seekingWall';
			end
		end
		
		%%%%%% Plotting Methods %%%%%%
		% if statement for debugging.
		function plotPathFollowed(obj)
			if ~isempty(obj.positionLog)
				figure; % Creates a new figure window
				plot(obj.positionLog(:, 1), obj.positionLog(:, 2), 'r-'); % Plot x and y from positionLog with a red line
				title('Path Followed by the Robot');
				xlabel('X Position');
				ylabel('Y Position');
				grid on;
				axis equal;
			else
				disp('No path data available for plotting.');
			end
		end

		function plotErrorLog(obj)
			% Error Plot
			figure;
			plot(obj.errorLog);
			title('PID Error Over Time');
			xlabel('Time Step');
			ylabel('Error');
		end

		function plotControlTerms(obj)
			% Control Signal Plot
			figure;
			hold on;
			plot(obj.pidTermsLog(:, 1), 'r-'); % Proportional
			plot(obj.pidTermsLog(:, 2), 'g-'); % Integral
			plot(obj.pidTermsLog(:, 3), 'b-'); % Derivative
			hold off;
			legend('Proportional', 'Integral', 'Derivative');
			title('PID Control Terms Over Time');
			xlabel('Time Step');
			ylabel('Control Terms');
		end

		function plotHeatMap(obj)
			% Heat Map Visualization
			figure;
			imagesc(obj.heatMap);
			colorbar;
			title('Heat Map of Robot Position');
			xlabel('X Position');
			ylabel('Y Position');
			axis equal;
		end

		function plotStateTransitions(obj)
			% State Transition Diagram
			figure;
			stairs(obj.stateLog);
			title('State Transitions Over Time');
			xlabel('Time Step');
			ylabel('State');
			ylim([0 max(obj.stateLog) + 1]); % Adjust based on state values
			yticks(unique(obj.stateLog));
			yticklabels({'Random Wandering', 'Wall Following'}); % Add more labels if you have more states
		end
    end
end
