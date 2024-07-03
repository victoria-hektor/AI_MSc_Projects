function CW_Code(serPort)
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
    else
        % Wall Following Logic
        SonLeft = ReadSonar(serPort, 1);
        if ~any(SonLeft)
            SonLeft = 100;
        end
        
        % Maintain a set distance (e.g., 0.5 meters) from the wall
        if SonLeft < 0.5
            SetDriveWheelsCreate(serPort, 0.3, 0.4); % Turn right slightly
        elseif SonLeft > 0.5
            SetDriveWheelsCreate(serPort, 0.4, 0.3); % Turn left slightly
        else
            SetDriveWheelsCreate(serPort, 0.4, 0.4); % Maintain straight line
        end

        % Check for open space to switch back to wandering
        if SonFF > 1.5 % If enough space is detected in front, switch to wandering
            isWandering = true;
        end
    end
    
    pause(0.1); % Small pause to prevent overloading the CPU
end

% Stop the robot
SetDriveWheelsCreate(serPort, 0, 0);
disp('Program Ending - Time Limit Reached')
end
