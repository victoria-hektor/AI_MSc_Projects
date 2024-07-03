function PID_Controller_P(serPort)
    % Initialize variables
    Kp = 0.5; % Proportional gain
    desiredState = 0.5; % Desired distance from the wall or setpoint
    maxSpeed = 0.5; % Maximum wheel speed

    % Start the timer for operation
    startTime = tic;

% Main control loop
    while toc(startTime) <= 300 % Run for 5 minutes
        % Read the current state
        currentState = ReadSonar(serPort, 1); % Ensure '1' is a valid sensor index

        % Debug: Print current state
        disp(['Current State: ', num2str(currentState)]);

        % Calculate the error
        currentError = desiredState - currentState;

        % Debug: Print current error
        disp(['Current Error: ', num2str(currentError)]);

        % Proportional control
        controlSignal = Kp * currentError;

        % Debug: Print control signal before clamping
        disp(['Control Signal (pre-clamp): ', num2str(controlSignal)]);

        % Clamp controlSignal within operational limits
        controlSignal = max(min(controlSignal, maxSpeed), -maxSpeed);

        % Debug: Print control signal after clamping
        disp(['Control Signal (post-clamp): ', num2str(controlSignal)]);

        % Actuate the robot
        SetDriveWheelsCreate(serPort, controlSignal, controlSignal);

        % Add a small delay
        pause(0.1);
    end
end
