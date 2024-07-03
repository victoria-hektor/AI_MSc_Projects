%% Create a new FIS for attributes
attributeFIS = newfis('Attribute Classification');

%% Linguistic variable: Size
attributeFIS = addvar(attributeFIS, 'input', 'Size', [0 10]);
attributeFIS = addmf(attributeFIS, 'input', 1, 'Small', 'gaussmf', [1.5 0]);
attributeFIS = addmf(attributeFIS, 'input', 1, 'Medium', 'gaussmf', [1.5 5]);
attributeFIS = addmf(attributeFIS, 'input', 1, 'Large', 'gaussmf', [1.5 10]);

%% Linguistic variable: Rotation Speed
attributeFIS = addvar(attributeFIS, 'input', 'Rotation Speed', [0 10]);
attributeFIS = addmf(attributeFIS, 'input', 2, 'Slow', 'gaussmf', [1.5 0]);
attributeFIS = addmf(attributeFIS, 'input', 2, 'Moderate', 'gaussmf', [1.5 5]);
attributeFIS = addmf(attributeFIS, 'input', 2, 'Fast', 'gaussmf', [1.5 10]);

%% Linguistic variable: Mass
attributeFIS = addvar(attributeFIS, 'input', 'Mass', [0 10]);
attributeFIS = addmf(attributeFIS, 'input', 3, 'Light', 'gaussmf', [1.5 0]);
attributeFIS = addmf(attributeFIS, 'input', 3, 'Medium', 'gaussmf', [1.5 5]);
attributeFIS = addmf(attributeFIS, 'input', 3, 'Heavy', 'gaussmf', [1.5 10]);

%% Linguistic variable: Surface Features
attributeFIS = addvar(attributeFIS, 'input', 'Surface Features', [0 10]);
attributeFIS = addmf(attributeFIS, 'input', 4, 'Cratered', 'gaussmf', [1.5 0]);
attributeFIS = addmf(attributeFIS, 'input', 4, 'Smooth', 'gaussmf', [1.5 5]);
attributeFIS = addmf(attributeFIS, 'input', 4, 'Mountainous', 'gaussmf', [1.5 10]);

%% Linguistic variable: Distance from the Sun
attributeFIS = addvar(attributeFIS, 'input', 'Distance from the Sun', [0 10]);
attributeFIS = addmf(attributeFIS, 'input', 5, 'Close', 'gaussmf', [1.5 0]);
attributeFIS = addmf(attributeFIS, 'input', 5, 'Moderate', 'gaussmf', [1.5 5]);
attributeFIS = addmf(attributeFIS, 'input', 5, 'Far', 'gaussmf', [1.5 10]);

%% Linguistic variable: Orbital Period
attributeFIS = addvar(attributeFIS, 'input', 'Orbital Period', [0 10]);
attributeFIS = addmf(attributeFIS, 'input', 6, 'Short', 'gaussmf', [1.5 0]);
attributeFIS = addmf(attributeFIS, 'input', 6, 'Moderate', 'gaussmf', [1.5 5]);
attributeFIS = addmf(attributeFIS, 'input', 6, 'Long', 'gaussmf', [1.5 10]);

%% Linguistic variable: Composition
attributeFIS = addvar(attributeFIS, 'input', 'Composition', [0 10]);
attributeFIS = addmf(attributeFIS, 'input', 7, 'Rocky', 'gaussmf', [1.5 0]);
attributeFIS = addmf(attributeFIS, 'input', 7, 'Gaseous', 'gaussmf', [1.5 5]);
attributeFIS = addmf(attributeFIS, 'input', 7, 'Ice', 'gaussmf', [1.5 10]);

%% Linguistic variable: Axial Tilt
attributeFIS = addvar(attributeFIS, 'input', 'Axial Tilt', [0 10]);
attributeFIS = addmf(attributeFIS, 'input', 8, 'Low', 'gaussmf', [1.5 0]);
attributeFIS = addmf(attributeFIS, 'input', 8, 'Moderate', 'gaussmf', [1.5 5]);
attributeFIS = addmf(attributeFIS, 'input', 8, 'High', 'gaussmf', [1.5 10]);

%% Linguistic variable: Surface Temperature
attributeFIS = addvar(attributeFIS, 'input', 'Surface Temperature', [0 10]);
attributeFIS = addmf(attributeFIS, 'input', 9, 'Cold', 'gaussmf', [1.5 0]);
attributeFIS = addmf(attributeFIS, 'input', 9, 'Moderate', 'gaussmf', [1.5 5]);
attributeFIS = addmf(attributeFIS, 'input', 9, 'Hot', 'gaussmf', [1.5 10]);

%% Linguistic variable: Magnetic Field Strength
attributeFIS = addvar(attributeFIS, 'input', 'Magnetic Field Strength', [0 10]);
attributeFIS = addmf(attributeFIS, 'input', 10, 'Weak', 'gaussmf', [1.5 0]);
attributeFIS = addmf(attributeFIS, 'input', 10, 'Moderate', 'gaussmf', [1.5 5]);
attributeFIS = addmf(attributeFIS, 'input', 10, 'Strong', 'gaussmf', [1.5 10]);

%% Linguistic variable: Atmospheric Pressure
attributeFIS = addvar(attributeFIS, 'input', 'Atmospheric Pressure', [0 10]);
attributeFIS = addmf(attributeFIS, 'input', 11, 'Low', 'gaussmf', [1.5 0]);
attributeFIS = addmf(attributeFIS, 'input', 11, 'Moderate', 'gaussmf', [1.5 5]);
attributeFIS = addmf(attributeFIS, 'input', 11, 'High', 'gaussmf', [1.5 10]);

%% Linguistic variable: Eccentricity of Orbit
attributeFIS = addvar(attributeFIS, 'input', 'Eccentricity of Orbit', [0 10]);
attributeFIS = addmf(attributeFIS, 'input', 12, 'Circular', 'gaussmf', [1.5 0]);
attributeFIS = addmf(attributeFIS, 'input', 12, 'Elliptical', 'gaussmf', [1.5 10]);

%% Linguistic variable: Albedo (Reflectivity)
attributeFIS = addvar(attributeFIS, 'input', 'Albedo (Reflectivity)', [0 10]);
attributeFIS = addmf(attributeFIS, 'input', 13, 'Low', 'gaussmf', [1.5 0]);
attributeFIS = addmf(attributeFIS, 'input', 13, 'Moderate', 'gaussmf', [1.5 5]);
attributeFIS = addmf(attributeFIS, 'input', 13, 'High', 'gaussmf', [1.5 10]);

%% Linguistic variable: Atmospheric Composition
attributeFIS = addvar(attributeFIS, 'input', 'Atmospheric Composition', [0 10]);
attributeFIS = addmf(attributeFIS, 'input', 14, 'Thin', 'gaussmf', [1.5 0]);
attributeFIS = addmf(attributeFIS, 'input', 14, 'Moderate', 'gaussmf', [1.5 5]);
attributeFIS = addmf(attributeFIS, 'input', 14, 'Dense', 'gaussmf', [1.5 10]);

%% Linguistic variable: Tidal Forces
attributeFIS = addvar(attributeFIS, 'input', 'Tidal Forces', [0 10]);
attributeFIS = addmf(attributeFIS, 'input', 15, 'Weak', 'gaussmf', [1.5 0]);
attributeFIS = addmf(attributeFIS, 'input', 15, 'Moderate', 'gaussmf', [1.5 5]);
attributeFIS = addmf(attributeFIS, 'input', 15, 'Strong', 'gaussmf', [1.5 10]);

%% Linguistic variable: Surface Gravity
attributeFIS = addvar(attributeFIS, 'input', 'Surface Gravity', [0 10]);
attributeFIS = addmf(attributeFIS, 'input', 16, 'Low', 'gaussmf', [1.5 0]);
attributeFIS = addmf(attributeFIS, 'input', 16, 'Moderate', 'gaussmf', [1.5 5]);
attributeFIS = addmf(attributeFIS, 'input', 16, 'High', 'gaussmf', [1.5 10]);

%% Linguistic variable: Ring System Presence
attributeFIS = addvar(attributeFIS, 'input', 'Ring System Presence', [0 10]);
attributeFIS = addmf(attributeFIS, 'input', 17, 'None', 'gaussmf', [1.5 0]);
attributeFIS = addmf(attributeFIS, 'input', 17, 'Partial', 'gaussmf', [1.5 5]);
attributeFIS = addmf(attributeFIS, 'input', 17, 'Complete', 'gaussmf', [1.5 10]);

%% Linguistic variable: Magnetosphere Strength
attributeFIS = addvar(attributeFIS, 'input', 'Magnetosphere Strength', [0 10]);
attributeFIS = addmf(attributeFIS, 'input', 18, 'Weak', 'gaussmf', [1.5 0]);
attributeFIS = addmf(attributeFIS, 'input', 18, 'Moderate', 'gaussmf', [1.5 5]);
attributeFIS = addmf(attributeFIS, 'input', 18, 'Strong', 'gaussmf', [1.5 10]);

%% Linguistic variable: Geological Activity
attributeFIS = addvar(attributeFIS, 'input', 'Geological Activity', [0 10]);
attributeFIS = addmf(attributeFIS, 'input', 19, 'Inactive', 'gaussmf', [1.5 0]);
attributeFIS = addmf(attributeFIS, 'input', 19, 'Moderate', 'gaussmf', [1.5 5]);
attributeFIS = addmf(attributeFIS, 'input', 19, 'Active', 'gaussmf', [1.5 10]);

%% Linguistic variable: Orbital Inclination
attributeFIS = addvar(attributeFIS, 'input', 'Orbital Inclination', [0 10]);
attributeFIS = addmf(attributeFIS, 'input', 20, 'Low', 'gaussmf', [1.5 0]);
attributeFIS = addmf(attributeFIS, 'input', 20, 'Moderate', 'gaussmf', [1.5 5]);
attributeFIS = addmf(attributeFIS, 'input', 20, 'High', 'gaussmf', [1.5 10]);

%% Define output linguistic variable for planet classification
planetFIS = addvar(planetFIS, 'output', 'Planetary Classification', [0 1]);

planetFIS = addmf(planetFIS, 'output', 1, 'Mercury', 'trimf', [0 0.2 0.4]);
planetFIS = addmf(planetFIS, 'output', 1, 'Venus', 'trimf', [0.2 0.4 0.6]);
planetFIS = addmf(planetFIS, 'output', 1, 'Earth', 'trimf', [0.4 0.6 0.8]);
planetFIS = addmf(planetFIS, 'output', 1, 'Mars', 'trimf', [0.6 0.8 1]);
planetFIS = addmf(planetFIS, 'output', 1, 'Jupiter', 'trimf', [0 0.2 0.4]);
planetFIS = addmf(planetFIS, 'output', 1, 'Saturn', 'trimf', [0.2 0.4 0.6]);
planetFIS = addmf(planetFIS, 'output', 1, 'Uranus', 'trimf', [0.4 0.6 0.8]);
planetFIS = addmf(planetFIS, 'output', 1, 'Neptune', 'trimf', [0.6 0.8 1]);
planetFIS = addmf(planetFIS, 'output', 1, 'Pluto', 'trimf', [0 0.2 0.4]);

planetFIS = addmf(planetFIS, 'output', 2, 'Earth-like Planet', 'trimf', [0 0.5 1]);
planetFIS = addmf(planetFIS, 'output', 2, 'Gas Giant like Jupiter', 'trimf', [0.5 1 1]);
planetFIS = addmf(planetFIS, 'output', 2, 'Ice Giant like Uranus', 'trimf', [0 0.5 1]);
planetFIS = addmf(planetFIS, 'output', 2, 'Dwarf Planet like Ceres', 'trimf', [0.5 1 1]);
planetFIS = addmf(planetFIS, 'output', 2, 'Kuiper Belt Object like Pluto', 'trimf', [0 0.5 1]);
planetFIS = addmf(planetFIS, 'output', 2, 'Terrestrial Planet like Venus', 'trimf', [0.5 1 1]);
planetFIS = addmf(planetFIS, 'output', 2, 'Gas Giant like Saturn', 'trimf', [0 0.5 1]);
planetFIS = addmf(planetFIS, 'output', 2, 'Terrestrial Planet like Mars', 'trimf', [0.5 1 1]);
planetFIS = addmf(planetFIS, 'output', 2, 'Terrestrial Planet like Mercury', 'trimf', [0 0.5 1]);
planetFIS = addmf(planetFIS, 'output', 2, 'Gas Giant like Neptune', 'trimf', [0.5 1 1]);

%% Rulebase:
%% Define rules for planet classification
ruleList = [
    % Mercury rule
    1 0 1 1 3 3 1 1 2 3 1 3 1 1 1 1 1 1 1 1; % Heavily cratered surface, no significant atmosphere, low mass, closest to the sun
    
    % Venus rule
    0 0 0 1 3 3 1 1 3 3 3 1 3 3 1 1 1 1 1 1; % Dense atmosphere mainly CO2, high surface temperature, second closest to the sun
    
    % Earth rule
    0 0 0 2 2 2 2 2 2 2 2 2 2 2 2 2 2 2 2 2; % Moderate axial tilt, liquid water, diverse ecosystem, only known planet with life
    
    % Mars rule
    0 0 0 1 3 3 1 1 2 1 1 2 1 3 2 2 2 2 1 2; % Thin atmosphere mainly nitrogen, polar ice caps, extensive volcanic plains, often referred to as the "Red Planet"
    
    % Jupiter rule
    0 0 0 1 1 1 3 1 3 3 1 3 3 3 3 3 3 3 3 3; % Predominantly gaseous composition, extensive ring system, strong magnetic field, largest planet
    
    % Saturn rule
    0 0 0 1 2 2 3 2 2 3 1 2 1 3 1 3 3 3 2 3; % Large visible ring system, low surface temperature, extensive moons, distinctive cloud bands
    
    % Uranus rule
    0 0 0 2 2 2 2 2 2 3 1 2 1 1 1 2 2 2 1 3; % Predominantly icy composition, low surface temperature, low surface gravity, tilted on its side
    
    % Neptune rule
    0 0 0 1 3 3 1 1 2 3 1 3 1 3 3 3 3 3 2 3; % Thick atmosphere mainly hydrogen and helium, high surface temperature, rapid cloud movements, farthest known planet
    
    % Pluto rule
    0 0 0 2 2 2 2 2 1 1 1 1 2 2 1 2 1 2 2 1; % Highly eccentric orbit, rapid rotation, located in Kuiper Belt
];

% Create FIS
planetFIS = addrule(planetFIS, ruleList);


%% Possible Plot Visualisations:
% Create figure for plotting
figure;

% Plot membership functions for Size
subplot(4, 5, 1);
plotmf(attributeFIS, 'input', 1);
title('Size');

% Plot membership functions for Rotation Speed
subplot(4, 5, 2);
plotmf(attributeFIS, 'input', 2);
title('Rotation Speed');

% Plot membership functions for Mass
subplot(4, 5, 3);
plotmf(attributeFIS, 'input', 3);
title('Mass');

% Plot membership functions for Surface Features
subplot(4, 5, 4);
plotmf(attributeFIS, 'input', 4);
title('Surface Features');

% Plot membership functions for Distance from the Sun
subplot(4, 5, 5);
plotmf(attributeFIS, 'input', 5);
title('Distance from the Sun');

% Plot membership functions for Composition
subplot(4, 5, 6);
plotmf(attributeFIS, 'input', 6);
title('Composition');

% Plot membership functions for Axial Tilt
subplot(4, 5, 7);
plotmf(attributeFIS, 'input', 7);
title('Axial Tilt');

% Plot membership functions for Surface Temperature
subplot(4, 5, 8);
plotmf(attributeFIS, 'input', 8);
title('Surface Temperature');

% Plot membership functions for Magnetic Field Strength
subplot(4, 5, 9);
plotmf(attributeFIS, 'input', 9);
title('Magnetic Field Strength');

% Plot membership functions for Atmospheric Pressure
subplot(4, 5, 10);
plotmf(attributeFIS, 'input', 10);
title('Atmospheric Pressure');

% Plot membership functions for Eccentricity of Orbit
subplot(3, 4, 1);
plotmf(attributeFIS, 'input', 12);
title('Eccentricity of Orbit');

% Plot membership functions for Albedo (Reflectivity)
subplot(3, 4, 2);
plotmf(attributeFIS, 'input', 13);
title('Albedo (Reflectivity)');

% Plot membership functions for Atmospheric Composition
subplot(3, 4, 3);
plotmf(attributeFIS, 'input', 14);
title('Atmospheric Composition');

% Plot membership functions for Tidal Forces
subplot(3, 4, 4);
plotmf(attributeFIS, 'input', 15);
title('Tidal Forces');

% Plot membership functions for Surface Gravity
subplot(3, 4, 5);
plotmf(attributeFIS, 'input', 16);
title('Surface Gravity');

% Plot membership functions for Ring System Presence
subplot(3, 4, 6);
plotmf(attributeFIS, 'input', 17);
title('Ring System Presence');

% Plot membership functions for Magnetosphere Strength
subplot(3, 4, 7);
plotmf(attributeFIS, 'input', 18);
title('Magnetosphere Strength');

% Plot membership functions for Geological Activity
subplot(3, 4, 8);
plotmf(attributeFIS, 'input', 19);
title('Geological Activity');

% Plot membership functions for Orbital Inclination
subplot(3, 4, 9);
plotmf(attributeFIS, 'input', 20);
title('Orbital Inclination');

% Adjust layout
sgtitle('Membership Functions for Linguistic Variables');

% Adjust figure size and spacing
set(gcf, 'Position', [100, 100, 1200, 800]);