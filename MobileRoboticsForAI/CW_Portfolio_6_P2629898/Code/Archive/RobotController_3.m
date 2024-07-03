classdef RobotController_3
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
            disp(['Control loop ended at ', datestr(now)]);
        end
		
		% Method to get the numeric value of a state for logging
        function value = getStateValue(obj, stateName)
            switch stateName
                case 'seekWall'
                    value = 1;
                case 'followingWall'
                    value = 2;
                otherwise
                    value = 0; % Default or unknown state
            end
        end		

        %%%%%% Wall finder logic %%%%%%
        function seekWall(obj)
            [SonFF, SonRight, SonLeft] = obj.readSensors();

            if isempty(SonFF) || SonFF > obj.desiredDistance
                DriveStraight(obj.serPort, obj.baseSpeed);
            elseif ~isempty(SonRight) && SonRight <= obj.desiredDistance + obj.someMargin
                obj.switchToWallFollowing();
            elseif ~isempty(SonFF) && SonFF <= obj.desiredDistance
                turnAngle(obj.serPort, obj.baseSpeed, -90); % Turn right
            end
			% Logging state transition
            obj.stateLog = [obj.stateLog; obj.getStateValue('seekWall')];
        end
		
        %%%%%% Wall Following Logic %%%%%%
        function followWall(obj)
            [SonFF, SonRight, SonLeft] = obj.readSensors();

            if ~isempty(SonRight) && SonRight <= obj.desiredDistance + obj.someMargin
                error = SonRight - obj.desiredDistance;
                obj.integral = obj.integral + error;
                derivative = error - obj.lastError;
                obj.lastError = error;
                controlSignal = obj.Kp * error + obj.Ki * obj.integral + obj.Kd * derivative;

                leftWheelSpeed = obj.baseSpeed + controlSignal;
                rightWheelSpeed = obj.baseSpeed - controlSignal;

                leftWheelSpeed = max(min(leftWheelSpeed, 0.5), -0.5);
                rightWheelSpeed = max(min(rightWheelSpeed, 0.5), -0.5);

                SetDriveWheelsCreate(obj.serPort, leftWheelSpeed, rightWheelSpeed);
                
                % Update PID log
                obj.errorLog = [obj.errorLog; error];
                obj.pidTermsLog = [obj.pidTermsLog; error, obj.integral, derivative];

                % Update heatmap
                obj.heatMap(obj.y + 1, obj.x + 1) = obj.heatMap(obj.y + 1, obj.x + 1) + 1;

                % Log state transition
                obj.stateLog = [obj.stateLog; obj.getStateValue('followWall')];
            else
                obj.switchToRandomWandering();
            end
        end

        %%%%%% Sensor reading %%%%%%
        function [SonFF, SonRight, SonLeft] = readSensors(obj)
            SonFF = ReadSonar(obj.serPort, 2);    % Front sonar
            SonRight = ReadSonar(obj.serPort, 3); % Right sonar
            SonLeft = ReadSonar(obj.serPort, 1);  % Left sonar
        end

        %%%%%% Odometer update %%%%%%
        function updateOdometry(obj)
            distance = DistanceSensorRoomba(obj.serPort);
            angle = AngleSensorRoomba(obj.serPort);

            obj.theta = obj.theta + deg2rad(angle);
            obj.x = obj.x + distance * cos(obj.theta);
            obj.y = obj.y + distance * sin(obj.theta);
            obj.positionLog = [obj.positionLog; obj.x, obj.y];
        end

        %%%%%% Utility functions %%%%%%
        function switchToRandomWandering(obj)
            obj.currentState = 'seekingWall';
        end

        function switchToWallFollowing(obj)
            obj.currentState = 'followingWall';
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
