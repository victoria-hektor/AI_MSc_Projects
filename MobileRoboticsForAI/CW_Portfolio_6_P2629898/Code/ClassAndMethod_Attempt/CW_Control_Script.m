function CW_Control_Script(serPort)
    % CW_Control_Script: Controls the Robot using the RobotController class

    % Check if serPort is provided
    if nargin < 1 || isempty(serPort)
        error('serPort is required as an input argument.');
    end

    % Create an instance of the RobotController class
    robot = RobotController_3(serPort);

    % Start the control loop
    robot.startControlLoop();
end