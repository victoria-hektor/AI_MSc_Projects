% Create new FIS for UFO classification
ufoFIS = newfis('UFO Classification');

% Define linguistic variables for UFO shape
ufoFIS = addvar(ufoFIS, 'input', 'Shape', [0 10]);
ufoFIS = addmf(ufoFIS, 'input', 1, 'Saucer-shaped', 'gaussmf', [1.5 0]);
ufoFIS = addmf(ufoFIS, 'input', 1, 'Triangular-shaped', 'gaussmf', [1.5 3]);
ufoFIS = addmf(ufoFIS, 'input', 1, 'Cigar-shaped', 'gaussmf', [1.5 6]);
ufoFIS = addmf(ufoFIS, 'input', 1, 'Spherical-shaped', 'gaussmf', [1.5 8]);
ufoFIS = addmf(ufoFIS, 'input', 1, 'Irregular-shaped', 'gaussmf', [1.5 10]);

% Define linguistic variables for UFO motion
ufoFIS = addvar(ufoFIS, 'input', 'Motion', [0 10]);
ufoFIS = addmf(ufoFIS, 'input', 2, 'Manoeuvrable', 'gaussmf', [1.5 0]);
ufoFIS = addmf(ufoFIS, 'input', 2, 'High-speed', 'gaussmf', [1.5 3]);
ufoFIS = addmf(ufoFIS, 'input', 2, 'Stationary', 'gaussmf', [1.5 6]);
ufoFIS = addmf(ufoFIS, 'input', 2, 'Sudden movement', 'gaussmf', [1.5 8]);
ufoFIS = addmf(ufoFIS, 'input', 2, 'Linear motion', 'gaussmf', [1.5 10]);

% Define linguistic variables for UFO lighting
ufoFIS = addvar(ufoFIS, 'input', 'Lighting', [0 10]);
ufoFIS = addmf(ufoFIS, 'input', 3, 'Light-emitting', 'gaussmf', [1.5 0]);
ufoFIS = addmf(ufoFIS, 'input', 3, 'Dark', 'gaussmf', [1.5 3]);
ufoFIS = addmf(ufoFIS, 'input', 3, 'Glowing', 'gaussmf', [1.5 6]);
ufoFIS = addmf(ufoFIS, 'input', 3, 'Multi-coloured', 'gaussmf', [1.5 8]);

% Define linguistic variables for UFO size and distance
ufoFIS = addvar(ufoFIS, 'input', 'Size and Distance', [0 10]);
ufoFIS = addmf(ufoFIS, 'input', 4, 'Close-range', 'gaussmf', [1.5 0]);
ufoFIS = addmf(ufoFIS, 'input', 4, 'Medium-range', 'gaussmf', [1.5 5]);
ufoFIS = addmf(ufoFIS, 'input', 4, 'Long-range', 'gaussmf', [1.5 10]);

% Define linguistic variables for witness credibility
ufoFIS = addvar(ufoFIS, 'input', 'Witness Credibility', [0 10]);
ufoFIS = addmf(ufoFIS, 'input', 5, 'Credible', 'gaussmf', [1.5 0]);
ufoFIS = addmf(ufoFIS, 'input', 5, 'Unverified', 'gaussmf', [1.5 10]);

%% Define output linguistic variable for UFO classification
ufoFIS = addvar(ufoFIS, 'output', 'UFO Classification', [0 1]);
ufoFIS = addmf(ufoFIS, 'output', 1, 'Saucer-shaped UFO', 'gaussmf', [1.5 0]);
ufoFIS = addmf(ufoFIS, 'output', 1, 'Triangular-shaped UFO', 'gaussmf', [1.5 0]);
ufoFIS = addmf(ufoFIS, 'output', 1, 'Cigar-shaped UFO', 'gaussmf', [1.5 0]);
ufoFIS = addmf(ufoFIS, 'output', 1, 'Spherical-shaped UFO', 'gaussmf', [1.5 0]);
ufoFIS = addmf(ufoFIS, 'output', 1, 'Irregular-shaped UFO', 'gaussmf', [1.5 0]);
ufoFIS = addmf(ufoFIS, 'output', 1, 'Manoeuvrable UFO', 'gaussmf', [1.5 0]);
ufoFIS = addmf(ufoFIS, 'output', 1, 'High-speed UFO', 'gaussmf', [1.5 0]);
ufoFIS = addmf(ufoFIS, 'output', 1, 'Stationary UFO', 'gaussmf', [1.5 0]);
ufoFIS = addmf(ufoFIS, 'output', 1, 'Sudden movement UFO', 'gaussmf', [1.5 0]);
ufoFIS = addmf(ufoFIS, 'output', 1, 'Linear motion UFO', 'gaussmf', [1.5 0]);
ufoFIS = addmf(ufoFIS, 'output', 1, 'Light-emitting UFO', 'gaussmf', [1.5 0]);
ufoFIS = addmf(ufoFIS, 'output', 1, 'Dark UFO', 'gaussmf', [1.5 0]);
ufoFIS = addmf(ufoFIS, 'output', 1, 'Glowing UFO', 'gaussmf', [1.5 0]);
ufoFIS = addmf(ufoFIS, 'output', 1, 'Multi-coloured UFO', 'gaussmf', [1.5 0]);
ufoFIS = addmf(ufoFIS, 'output', 1, 'Close-range UFO', 'gaussmf', [1.5 0]);
ufoFIS = addmf(ufoFIS, 'output', 1, 'Medium-range UFO', 'gaussmf', [1.5 0]);
ufoFIS = addmf(ufoFIS, 'output', 1, 'Long-range UFO', 'gaussmf', [1.5 0]);
ufoFIS = addmf(ufoFIS, 'output', 1, 'Credible UFO', 'gaussmf', [1.5 0]);
ufoFIS = addmf(ufoFIS, 'output', 1, 'Unverified UFO', 'gaussmf', [1.5 0]);

% Rulebase (UFO’s):
% Shape:
ruleList = [
    % Saucer-shaped UFO
    1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0; 
    
    % Triangular-shaped UFO
    0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
    
    % Cigar-shaped UFO
    0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
    
    % Spherical-shaped UFO
    0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
    
    % Irregular-shaped UFO
    0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0;
    
    % Manoeuvrable UFO
    0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0;
    
    % High-speed UFO
    0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0;
    
    % Stationary UFO
    0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0;
    
    % Sudden movement UFO
    0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0;
    
    % Linear motion UFO
    0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0;
];

% Lighting:
ruleList = [
    % Light-emitting UFO
    ruleList;
    1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
    
    % Dark UFO
    0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
    
    % Glowing UFO
    0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
    
    % Multi-coloured UFO
    0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
];

% Size and Distance:
ruleList = [
    % Close-range UFO
    ruleList;
    1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
    
    % Medium-range UFO
    0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
    
    % Long-range UFO
    0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
];

% Witness Credibility:
ruleList = [
    % Credible UFO
    ruleList;
    1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
    
    % Unverified UFO
    0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
];

% Plot membership functions for UFO shape
figure;
subplot(2, 3, 1);
plotmf(ufoFIS, 'input', 1);
title('Shape');

% Plot membership functions for UFO motion
subplot(2, 3, 2);
plotmf(ufoFIS, 'input', 2);
title('Motion');

% Plot membership functions for UFO lighting
subplot(2, 3, 3);
plotmf(ufoFIS, 'input', 3);
title('Lighting');

% Plot membership functions for UFO size and distance
subplot(2, 3, 4);
plotmf(ufoFIS, 'input', 4);
title('Size and Distance');

% Plot membership functions for witness credibility
subplot(2, 3, 5);
plotmf(ufoFIS, 'input', 5);
title('Witness Credibility');

% Adjust layout
sgtitle('Membership Functions for UFO Classification');

% Adjust figure size and spacing
set(gcf, 'Position', [100, 100, 1200, 800]);