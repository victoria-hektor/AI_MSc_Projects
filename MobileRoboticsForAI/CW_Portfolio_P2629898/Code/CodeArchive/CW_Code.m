function TestProg_1(serPort)
% Robot moves along a path, part blindly, part sensing.
% serPort is the serial port number (for controlling the actual robot).

%%%% DO NOT MODIFY CODE ABOVE %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

disp('==================')
disp('Program Starting')
disp('------------------')

% State Variable to switch between wandering and wall following
isWandering = true;

% Main control loop
while true
    if isWandering
        % Random Wandering Logic
        randomDistance = rand() * 2; % Random distance between 0 and 2 meters
        randomAngle = rand() * 360 - 180; % Random angle between -180 and 180 degrees
        
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
    else
        % Wall Following Logic
        % Assuming left sonar is used for wall following (change as needed)
        SonLeft = ReadSonar(serPort, 1);
        if ~any(SonLeft)
            SonLeft = 100;
        end
        
        % Maintain a set distance (e.g., 0.5 meters) from the wall
        if SonLeft < 0.5
            % Too close to the wall, turn right slightly
            SetDriveWheelsCreate(serPort, 0.3, 0.4);
        elseif SonLeft > 0.5
            % Too far from the wall, turn left slightly
            SetDriveWheelsCreate(serPort, 0.4, 0.3);
        else
            % Maintain straight line
            SetDriveWheelsCreate(serPort, 0.4, 0.4);
        end
        
        % Optionally: Condition to switch back to wandering
        % ...
    end
    
    pause(0.1); % Small pause to prevent overloading the CPU
end

% Function to stop the robot if needed
% SetDriveWheelsCreate(serPort, 0, 0);
end