classdef RobotController_2
    properties
        serPort;                  % Serial port for communication
        desiredDistance = 1;      % Desired distance from the wall
        % Sensor readings
        frontSensor;
        leftSensor;
        rightSensor;
    end

    methods
        function obj = RobotController(serPort)
            obj.serPort = serPort;
            % Initialise sensor readings
            obj.frontSensor = 0;
            obj.leftSensor = 0;
            obj.rightSensor = 0;
        end

        % Update sensor readings using the SonFF function
        function updateSensors(obj)
            obj.frontSensor = SonFF(obj.serPort, 'front');
            obj.leftSensor = SonFF(obj.serPort, 'left');
            obj.rightSensor = SonFF(obj.serPort, 'right');
        end

        % Function to turn left incrementally until the robot aligns with a wall on the right
        function turnLeftToAlignWithWall(obj)
            while ~isWallOnRight(obj)
                turnLeft(obj, 10);  % Turning left by a small angle, e.g., 10 degrees
                pause(0.5);         % Pausing briefly to allow the robot to execute the turn
                updateSensors(obj); % Update sensor readings
            end
        end

        % Function to check if there is a wall on the right
        function result = isWallOnRight(obj)
            % Checking the right sensor to determine if there is a wall
            result = obj.rightSensor < obj.desiredDistance;
        end

        % Function to execute a left turn
        function turnLeft(obj, angle)
            % Command to turn the robot left by the specified angle
            % Adjust the velocity and radius as needed for your specific robot
            velocity = 0.2; % Set a positive velocity
            radius = 0.2;   % Set a positive radius to turn left
            setFwdVelRadiusRoomba(obj.serPort, velocity, radius);
            pause(angle / 45); % Adjust pause time based on the desired angle (example calculation)
            setFwdVelRadiusRoomba(obj.serPort, 0, inf); % Stop the robot after turning
        end
    end
end
