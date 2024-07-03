% Create a new FIS for actions
actionFIS = newfis('Action Classification');

% Linguistic variable: Orbiting the Sun
actionFIS = addvar(actionFIS, 'input', 'Orbiting the Sun', [0 10]);
actionFIS = addmf(actionFIS, 'input', 1, 'Far', 'gaussmf', [1.5 0]);
actionFIS = addmf(actionFIS, 'input', 1, 'Moderate', 'gaussmf', [1.5 5]);
actionFIS = addmf(actionFIS, 'input', 1, 'Close', 'gaussmf', [1.5 10]);

% Linguistic variable: Approaching the Sun
actionFIS = addvar(actionFIS, 'input', 'Approaching the Sun', [0 10]);
actionFIS = addmf(actionFIS, 'input', 2, 'Far', 'gaussmf', [1.5 0]);
actionFIS = addmf(actionFIS, 'input', 2, 'Moderate', 'gaussmf', [1.5 5]);
actionFIS = addmf(actionFIS, 'input', 2, 'Close', 'gaussmf', [1.5 10]);

% Linguistic variable: Interstellar Travel
actionFIS = addvar(actionFIS, 'input', 'Interstellar Travel', [0 10]);
actionFIS = addmf(actionFIS, 'input', 3, 'Low', 'trimf', [0 3 6]);
actionFIS = addmf(actionFIS, 'input', 3, 'Moderate', 'trimf', [4 6 8]);
actionFIS = addmf(actionFIS, 'input', 3, 'High', 'trimf', [6 8 10]);

% Linguistic variable: Orbiting a Planet
actionFIS = addvar(actionFIS, 'input', 'Orbiting a Planet', [0 10]);
actionFIS = addmf(actionFIS, 'input', 4, 'Far', 'gaussmf', [1.5 0]);
actionFIS = addmf(actionFIS, 'input', 4, 'Moderate', 'gaussmf', [1.5 5]);
actionFIS = addmf(actionFIS, 'input', 4, 'Close', 'gaussmf', [1.5 10]);

% Linguistic variable: Volcanic Activity
actionFIS = addvar(actionFIS, 'input', 'Volcanic Activity', [0 10]);
actionFIS = addmf(actionFIS, 'input', 5, 'Low', 'trimf', [0 2 4]);
actionFIS = addmf(actionFIS, 'input', 5, 'Moderate', 'trimf', [3 5 7]);
actionFIS = addmf(actionFIS, 'input', 5, 'High', 'trimf', [6 8 10]);

% Linguistic variable: Solar Flare Activity
actionFIS = addvar(actionFIS, 'input', 'Solar Flare Activity', [0 10]);
actionFIS = addmf(actionFIS, 'input', 6, 'Low', 'trimf', [0 2 4]);
actionFIS = addmf(actionFIS, 'input', 6, 'Moderate', 'trimf', [3 5 7]);
actionFIS = addmf(actionFIS, 'input', 6, 'High', 'trimf', [6 8 10]);

% Linguistic variable: Close Encounter with a Celestial Body
actionFIS = addvar(actionFIS, 'input', 'Close Encounter with a Celestial Body', [0 10]);
actionFIS = addmf(actionFIS, 'input', 7, 'Weak', 'gaussmf', [1.5 0]);
actionFIS = addmf(actionFIS, 'input', 7, 'Moderate', 'gaussmf', [1.5 5]);
actionFIS = addmf(actionFIS, 'input', 7, 'Strong', 'gaussmf', [1.5 10]);

% Linguistic variable: Atmospheric Entry
actionFIS = addvar(actionFIS, 'input', 'Atmospheric Entry', [0 10]);
actionFIS = addmf(actionFIS, 'input', 8, 'Weak', 'gaussmf', [1.5 0]);
actionFIS = addmf(actionFIS, 'input', 8, 'Moderate', 'gaussmf', [1.5 5]);
actionFIS = addmf(actionFIS, 'input', 8, 'Strong', 'gaussmf', [1.5 10]);

% Linguistic variable: Impact Event
actionFIS = addvar(actionFIS, 'input', 'Impact Event', [0 10]);
actionFIS = addmf(actionFIS, 'input', 9, 'Low', 'trimf', [0 2 4]);
actionFIS = addmf(actionFIS, 'input', 9, 'Moderate', 'trimf', [3 5 7]);
actionFIS = addmf(actionFIS, 'input', 9, 'High', 'trimf', [6 8 10]);

%% Define output linguistic variable for action classification
actionFIS = addvar(actionFIS, 'output', 'Action Classification', [0 1]);
actionFIS = addmf(actionFIS, 'output', 1, 'Sungrazing Comet', 'trimf', [0 0.5 1]);
actionFIS = addmf(actionFIS, 'output', 1, 'Interstellar Comet', 'trimf', [0.5 1 1]);

actionFIS = addmf(actionFIS, 'output', 2, 'Comet with Active Coma', 'trimf', [0 0.5 1]);
actionFIS = addmf(actionFIS, 'output', 2, 'Planet in Stable Orbit', 'trimf', [0.5 1 1]);

actionFIS = addmf(actionFIS, 'output', 3, 'Magnetically Active Planet', 'trimf', [0 0.5 1]);
actionFIS = addmf(actionFIS, 'output', 3, 'Moon in Stable Orbit', 'trimf', [0.5 1 1]);

actionFIS = addmf(actionFIS, 'output', 4, 'Tidally Disrupted Moon', 'trimf', [0 0.5 1]);
actionFIS = addmf(actionFIS, 'output', 4, 'Geologically Active Moon', 'trimf', [0.5 1 1]);

actionFIS = addmf(actionFIS, 'output', 5, 'Potentially Hazardous Asteroid', 'trimf', [0 0.5 1]);
actionFIS = addmf(actionFIS, 'output', 5, 'Captured Asteroid', 'trimf', [0.5 1 1]);

actionFIS = addmf(actionFIS, 'output', 6, 'Interstellar Probe', 'trimf', [0 0.5 1]);
actionFIS = addmf(actionFIS, 'output', 6, 'Space Capsule Re-entering Earth\'s Atmosphere', 'trimf', [0.5 1 1]);

actionFIS = addmf(actionFIS, 'output', 7, 'Space Probe in Orbit', 'trimf', [0 0.5 1]);
actionFIS = addmf(actionFIS, 'output', 7, 'Meteor', 'trimf', [0.5 1 1]);

actionFIS = addmf(actionFIS, 'output', 8, 'Natural Satellite', 'trimf', [0 0.5 1]);
actionFIS = addmf(actionFIS, 'output', 8, 'Potential Impactor', 'trimf', [0.5 1 1]);

actionFIS = addmf(actionFIS, 'output', 9, 'Terrestrial Planet', 'trimf', [0 0.5 1]);
actionFIS = addmf(actionFIS, 'output', 9, 'Rogue Planet', 'trimf', [0.5 1 1]);

actionFIS = addmf(actionFIS, 'output', 10, 'Gas Giant with Comet-like Tail', 'trimf', [0 0.5 1]);
actionFIS = addmf(actionFIS, 'output', 10, 'Gas Giant with Magnetic Disturbances', 'trimf', [0.5 1 1]);

actionFIS = addmf(actionFIS, 'output', 11, 'Terrestrial Planet with Active Volcanism', 'trimf', [0 0.5 1]);
actionFIS = addmf(actionFIS, 'output', 11, 'Terrestrial Planet with Tectonic Activity', 'trimf', [0.5 1 1]);

actionFIS = addmf(actionFIS, 'output', 12, 'Icy Moon with Cryovolcanic Activity', 'trimf', [0 0.5 1]);
actionFIS = addmf(actionFIS, 'output', 12, 'Icy Moon with Internal Heating', 'trimf', [0.5 1 1]);

actionFIS = addmf(actionFIS, 'output', 13, 'Rubble-pile Asteroid', 'trimf', [0 0.5 1]);
actionFIS = addmf(actionFIS, 'output', 13, 'Potentially Hazardous Asteroid with Orbital Perturbations', 'trimf', [0.5 1 1]);

actionFIS = addmf(actionFIS, 'output', 14, 'Comet with Sublimation-driven Activity', 'trimf', [0 0.5 1]);
actionFIS = addmf(actionFIS, 'output', 14, 'Comet with Nucleus Breakup', 'trimf', [0.5 1 1]);

actionFIS = addmf(actionFIS, 'output', 15, 'Spacecraft Engaged in Space Rendezvous', 'trimf', [0 0.5 1]);
actionFIS = addmf(actionFIS, 'output', 15, 'Spacecraft Utilising Gravitational Slingshot', 'trimf', [0.5 1 1]);

%% Rulebase
ruleList = [
    % Comet rules
    1 0 0 0 0 0 0 0 0; % Rule 1
    1 0 0 0 0 0 0 0 0; % Rule 2
    1 0 0 0 0 0 0 0 0; % Rule 3
    
    % Planet rules
    0 1 0 0 0 0 0 0 0; % Rule 4
    0 0 0 0 1 0 0 0 0; % Rule 5
    
    % Moon rules
    0 0 0 1 0 0 0 0 0; % Rule 6
    0 0 0 0 0 0 0 0 0; % Rule 7
    0 0 0 0 0 0 0 0 0; % Rule 8
    
    % Asteroid rules
    0 0 0 0 0 0 0 0 0; % Rule 9
    0 0 0 0 0 0 0 0 0; % Rule 10
    
    % Spacecraft rules
    0 0 1 0 0 0 0 0 0; % Rule 11
    0 0 0 0 0 1 0 0 0; % Rule 12
    0 0 0 0 0 0 0 0 0; % Rule 13
    
    % Meteoroid rule
    0 0 0 0 0 0 0 1 0; % Rule 14
    
    % Other planet rules
    0 1 0 0 0 0 0 0 0; % Rule 15
    0 0 0 0 0 0 0 0 0; % Rule 16
    0 0 0 1 0 0 0 0 0; % Rule 17
    0 0 0 0 0 0 0 0 0; % Rule 18
    0 0 0 0 0 0 0 0 0; % Rule 19
    0 0 0 0 0 0 0 0 0; % Rule 20
    0 0 0 0 0 0 0 0 0; % Rule 21
    0 0 0 0 0 0 0 0 0; % Rule 22
    
    % Other moon rules
    0 0 0 0 0 0 0 0 0; % Rule 23
    0 0 0 0 0 0 0 0 0; % Rule 24
    
    % Other asteroid rules
    0 0 0 0 0 0 0 0 0; % Rule 25
    0 0 0 0 0 0 0 0 0; % Rule 26
    
    % Other comet rules
    0 0 0 0 0 0 0 0 0; % Rule 27
    0 0 0 0 0 0 0 0 0; % Rule 28
    
    % Other spacecraft rules
    0 0 0 0 0 0 0 0 0; % Rule 29
    0 0 0 0 0 0 0 0 0; % Rule 30
    ];

%% Plot the membership functions
figure;
subplot(3,3,1);
plotmf(actionFIS, 'input', 1);
title('Orbiting the Sun');
subplot(3,3,2);
plotmf(actionFIS, 'input', 2);
title('Approaching the Sun');
subplot(3,3,3);
plotmf(actionFIS, 'input', 3);
title('Interstellar Travel');
subplot(3,3,4);
plotmf(actionFIS, 'input', 4);
title('Orbiting a Planet');
subplot(3,3,5);
plotmf(actionFIS, 'input', 5);
title('Volcanic Activity');
subplot(3,3,6);
plotmf(actionFIS, 'input', 6);
title('Solar Flare Activity');
subplot(3,3,7);
plotmf(actionFIS, 'input', 7);
title('Close Encounter with a Celestial Body');
subplot(3,3,8);
plotmf(actionFIS, 'input', 8);
title('Atmospheric Entry');
subplot(3,3,9);
plotmf(actionFIS, 'input', 9);
title('Impact Event');
