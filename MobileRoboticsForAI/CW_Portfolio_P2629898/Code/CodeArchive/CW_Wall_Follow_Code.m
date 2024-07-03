function CW_Code(serPort)
% Robot moves along a path, following the wall.
% serPort is the serial port number (for controlling the actual robot).

%%%% DO NOT MODIFY CODE ABOVE %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

disp('==================')
disp('Program Starting')
disp('------------------')

% Start the timer
startTime = tic; % Initialize the timer here

% Define a set distance from the wall
wallFollowDistance = 0.5; % 0.5 meters from the wall

% Main control loop
while true
    % Check if 5 minutes have passed
    if toc(startTime) > 300 % 300 seconds = 5 minutes
        break; % Exit the loop to stop the robot
    end
	
    % Read front and right sonar values
    SonFF = ReadSonar(serPort, 2); % Front sonar
    SonRight = ReadSonar(serPort, 3); % Right sonar
    
    % If a wall is detected in front, turn left until it's clear
    while SonFF < wallFollowDistance
        turnAngle(serPort, 0.2, -90); % Turn left 90 degrees
        SonFF = ReadSonar(serPort, 2);
    end
    
    % Wall Following Logic
    % Adjust the robot's movement to follow the wall on its right
    if SonRight > wallFollowDistance
        % Too far from the wall, turn right slightly
        SetDriveWheelsCreate(serPort, 0.4, 0.3);
    elseif SonRight < wallFollowDistance
        % Too close to the wall, turn left slightly
        SetDriveWheelsCreate(serPort, 0.3, 0.4);
    else
        % Maintain a straight line
        SetDriveWheelsCreate(serPort, 0.4, 0.4);
    end
    
    pause(0.1); % Small pause to prevent overloading the CPU
end

% Stop the robot
SetDriveWheelsCreate(serPort, 0, 0);
disp('Program Ending - Time Limit Reached')
end
