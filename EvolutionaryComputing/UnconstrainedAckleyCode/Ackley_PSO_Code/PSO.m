function out = PSO(problem, params)
    %% Problem Definiton
    CostFunction = problem.CostFunction;  % Cost Function
    nVar = problem.nVar;        % Number of Unknown (Decision) Variables
    VarSize = [1 nVar];         % Matrix Size of Decision Variables
    VarMin = problem.VarMin;	% Lower Bound of Decision Variables
    VarMax = problem.VarMax;    % Upper Bound of Decision Variables


    %% Parameters of PSO
    MaxIt = params.MaxIt;   % Maximum Number of Iterations
    nPop = params.nPop;     % Population Size (Swarm Size)
    w = params.w;           % Intertia Coefficient
    wdamp = params.wdamp;   % Damping Ratio of Inertia Coefficient
    c1 = params.c1;         % Personal Acceleration Coefficient
    c2 = params.c2;         % Social Acceleration Coefficient

    % The Flag for Showing Iteration Information
    ShowIterInfo = params.ShowIterInfo;    

    MaxVelocity = 0.2*(VarMax-VarMin);
    MinVelocity = -MaxVelocity;
    
    particle_history = zeros(MaxIt, nPop, nVar);
    % Initialise a matrix to store particle history (needed for animation)
    particleHistory = zeros(nPop, nVar, MaxIt);

    %% Initialisation
    % The Particle Template
    empty_particle.Position = [];
    empty_particle.Velocity = [];
    empty_particle.Cost = [];
    empty_particle.Best.Position = [];
    empty_particle.Best.Cost = [];

    % Create Population Array
    particle = repmat(empty_particle, nPop, 1);

    % Initialise Global Best
    GlobalBest.Cost = inf;

    % Initialise Population Members
    for i=1:nPop

        % Generate Random Solution
        particle(i).Position = unifrnd(VarMin, VarMax, VarSize);

        % Initialise Velocity
        particle(i).Velocity = zeros(VarSize);

        % Evaluation
        particle(i).Cost = CostFunction(particle(i).Position);

        % Update the Personal Best
        particle(i).Best.Position = particle(i).Position;
        particle(i).Best.Cost = particle(i).Cost;

        % Update Global Best
        if particle(i).Best.Cost < GlobalBest.Cost
            GlobalBest = particle(i).Best;
        end

    end

    % Array to Hold Best Cost Value on Each Iteration
    BestCosts = zeros(MaxIt, 1);

    %% Main Loop of PSO

    for it=1:MaxIt

        for i=1:nPop

            % Update Velocity
            particle(i).Velocity = w*particle(i).Velocity ...
                + c1*rand(VarSize).*(particle(i).Best.Position - particle(i).Position) ...
                + c2*rand(VarSize).*(GlobalBest.Position - particle(i).Position);

            % Apply Velocity Limits
            particle(i).Velocity = max(particle(i).Velocity, MinVelocity);
            particle(i).Velocity = min(particle(i).Velocity, MaxVelocity);
            
            % Update Position
            particle(i).Position = particle(i).Position + particle(i).Velocity;
            
            % Apply Lower and Upper Bound Limits
            particle(i).Position = max(particle(i).Position, VarMin);
            particle(i).Position = min(particle(i).Position, VarMax);

            % Evaluation
            particle(i).Cost = CostFunction(particle(i).Position);

            % Store the history
            particle_history(it, i, :) = particle(i).Position;

            % Update Personal Best
            if particle(i).Cost < particle(i).Best.Cost

                particle(i).Best.Position = particle(i).Position;
                particle(i).Best.Cost = particle(i).Cost;

                % Update Global Best
                if particle(i).Best.Cost < GlobalBest.Cost
                    GlobalBest = particle(i).Best;
                end            

            end
            
        end

        % Store positions for animation
        for i = 1:nPop
            particleHistory(i, :, it) = particle(i).Position;
        end

        % Store the Best Cost Value
        BestCosts(it) = GlobalBest.Cost;
		
		%% Enhanced Display Iteration Information
		if ShowIterInfo
			disp(['Iteration ' num2str(it) ': Best Cost = ' num2str(BestCosts(it))]);
			fprintf('Current Best Position: [%f, %f]\n', GlobalBest.Position); % Prints the current best position
		end

        % Damping Inertia Coefficient
        w = w * wdamp;

    end
    
    out.pop = particle;
    out.BestSol = GlobalBest;
    out.BestCosts = BestCosts;
    out.particle_history = particle_history;
    out.particleHistory = particleHistory;
    out.particle_final_positions = [particle.Position];
    
end