function [NewPosition] = NonUniformMutation(OldPosition, mu, CurrentIteration, MaxIterations, VarMin, VarMax)
    % NonUniformMutation Applies non-uniform mutation to a candidate solution
    %   OldPosition: the current position of the solution
    %   mu: mutation rate
    %   CurrentIteration: the current iteration number
    %   MaxIterations: the maximum number of iterations
    %   VarMin: the minimum bounds for the decision variables
    %   VarMax: the maximum bounds for the decision variables

    % Initialise the new position with the old position
    NewPosition = OldPosition;
    
    % Number of decision variables
    nVar = numel(OldPosition);

    % Mutation parameter 'b' determines the degree of non-uniformity
    % Typically set to a value between 2 and 5
    b = 3;

    % Apply non-uniform mutation to each decision variable
    for j = 1:nVar
        % Only mutate with a probability of 'mu'
        if rand() <= mu
            % Choose the direction of mutation randomly
            if rand() < 0.5
                % Mutate towards upper bound
                delta = VarMax(j) - OldPosition(j);
                NewPosition(j) = OldPosition(j) + delta * (1 - rand()^((1 - CurrentIteration / MaxIterations)^b));
            else
                % Mutate towards lower bound
                delta = OldPosition(j) - VarMin(j);
                NewPosition(j) = OldPosition(j) - delta * (1 - rand()^((1 - CurrentIteration / MaxIterations)^b));
            end

            % Ensure the new position respects the variable bounds
            NewPosition(j) = max(NewPosition(j), VarMin(j));
            NewPosition(j) = min(NewPosition(j), VarMax(j));
        end
    end
end