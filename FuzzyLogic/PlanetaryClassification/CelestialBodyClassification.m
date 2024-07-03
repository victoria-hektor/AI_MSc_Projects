%% Define fis
fis = mamfis('Name', 'CelestialBodyClassification');

fis = addInput(fis, [0 1], 'Name', 'Size'); % Assuming normalised values: Small [0, 0.33], Medium [0.33, 0.66], Large [0.66, 1]
fis = addInput(fis, [0 1], 'Name', 'Composition'); % Rocky [0, 0.33], Gaseous [0.33, 0.66], Icy [0.66, 1]
fis = addInput(fis, [0 1], 'Name', 'OrbitalZone'); % Inner [0, 0.33], AsteroidBelt [0.33, 0.66], Outer [0.66, 1]

fis = addOutput(fis, [0 1], 'Name', 'Classification'); % The range is for confidence scoring

% Adding Size membership functions
fis = addMF(fis, 'Size', 'gaussmf', [0.1 0.16], 'Name', 'Small');
fis = addMF(fis, 'Size', 'gaussmf', [0.1 0.5], 'Name', 'Medium');
fis = addMF(fis, 'Size', 'gaussmf', [0.1 0.83], 'Name', 'Large');

% Adding Composition membership functions
fis = addMF(fis, 'Composition', 'trimf', [0 0.33 0.66], 'Name', 'Rocky');
fis = addMF(fis, 'Composition', 'trimf', [0.33 0.66 1], 'Name', 'Gaseous');
fis = addMF(fis, 'Composition', 'trimf', [0.66 1 1], 'Name', 'Icy');

% Adding OrbitalZone membership functions
fis = addMF(fis, 'OrbitalZone', 'trimf', [0 0.33 0.66], 'Name', 'Inner');
fis = addMF(fis, 'OrbitalZone', 'trimf', [0.33 0.66 1], 'Name', 'AsteroidBelt');
fis = addMF(fis, 'OrbitalZone', 'trimf', [0.66 1 1], 'Name', 'Outer');

%% Adding Output Membership Functions for Classification
% Distinct Classifications
fis = addMF(fis, 'Classification', 'trimf', [0 0.0625 0.125], 'Name', 'DwarfPlanet');
fis = addMF(fis, 'Classification', 'trimf', [0.0625 0.125 0.1875], 'Name', 'TerrestrialPlanet');
fis = addMF(fis, 'Classification', 'trimf', [0.125 0.1875 0.25], 'Name', 'GasGiant');
fis = addMF(fis, 'Classification', 'trimf', [0.1875 0.25 0.3125], 'Name', 'IcyBody');
fis = addMF(fis, 'Classification', 'trimf', [0.25 0.3125 0.375], 'Name', 'IcyMoon');
fis = addMF(fis, 'Classification', 'trimf', [0.3125 0.375 0.4375], 'Name', 'Asteroid');

% Overlapping/Intermediate Classifications
fis = addMF(fis, 'Classification', 'trapmf', [0.375 0.4375 0.5 0.5625], 'Name', 'DwarfOrTerrestrial');
fis = addMF(fis, 'Classification', 'trapmf', [0.5 0.5625 0.625 0.6875], 'Name', 'MiniGasOrLargeIcyMoon');
fis = addMF(fis, 'Classification', 'trapmf', [0.625 0.6875 0.75 0.8125], 'Name', 'SuperEarthOrTerrestrial');

% Advanced Classifications and Anomalies
fis = addMF(fis, 'Classification', 'trapmf', [0.75 0.8125 0.875 0.9375], 'Name', 'RoguePlanetOrExoplanet');
fis = addMF(fis, 'Classification', 'trapmf', [0.8125 0.875 0.9375 1], 'Name', 'LostGasGiantOrHotNeptune');
fis = addMF(fis, 'Classification', 'trapmf', [0.4375 0.5 0.5625 0.625], 'Name', 'CometOrKuiperBeltObject');
fis = addMF(fis, 'Classification', 'trapmf', [0.5625 0.625 0.6875 0.75], 'Name', 'ProtoplanetOrLargeAsteroid');
fis = addMF(fis, 'Classification', 'trapmf', [0.6875 0.75 0.8125 0.875], 'Name', 'TransitionalObject');

%% Rules
% Rule Set 1: Basic Classification
rules = [
    % Size, Composition, OrbitalZone, Classification, Weight, Operator
    1 1 0 1 1 1; % Dwarf Planet
    2 1 1 2 1 1; % Terrestrial Planet
    3 2 0 3 1 1; % Gas Giant
    0 3 3 4 1 1; % Icy Body
    2 3 3 5 1 1; % Icy Moon
    1 0 2 6 1 1; % Asteroid
    
    % Rule Set 2: Intermediate Classification with Overlaps
    % Approximation: Using a "central" output MF to represent the overlap or the decision point.
    1 1 0 7 1 1; % Dwarf Planet or Terrestrial Planet, assuming 7 represents this overlap
    2 2 3 8 1 1; % Mini Gas Giant or Large Icy Moon, assuming 8 represents this overlap
    3 1 1 9 1 1; % Super-Earth or Terrestrial Planet, assuming 9 represents this overlap
    2 3 0 10 1 1; % Icy Moon or Water World, assuming 10 represents this overlap
    2 3 3 11 1 1; % Icy Giant or Gas Giant, assuming 11 represents this overlap
    3 3 3 11 1 1; % Icy Giant or Gas Giant, for larger size
    
    % Rule Set 3: Advanced Classification Considering Anomalies
    3 1 3 12 1 1; % Rogue Planet or Exoplanet
    2 2 1 13 1 1; % Lost Gas Giant or Hot Neptune
    1 3 1 14 1 1; % Comet or Captured Kuiper Belt Object
    3 1 2 15 1 1; % Protoplanet or Large Asteroid
    0 0 0 16 1 1; % Transitional Object, representing a highly general case
];

% 'fis' is your Fuzzy Inference System variable a
fis = addRule(fis, rules);

inputSize = 0.5; % Medium
inputComposition = 0.1; % Rocky
inputOrbitalZone = 0.2; % Inner Solar System

evalFis = evalfis(fis, [inputSize, inputComposition, inputOrbitalZone]);
disp(['Classification Confidence: ', num2str(evalFis)]);

%%% Visualisations %%%
% Visualise the Fuzzy Rules:
rulesview(fis);

%% Visualise Input Membership Functions
figure('Name', 'Input Membership Functions');

subplot(3, 1, 1);
plotmf(fis, 'input', 1); % For 'Size'
title('Membership Functions for Size');

subplot(3, 1, 2);
plotmf(fis, 'input', 2); % For 'Composition'
title('Membership Functions for Composition');

subplot(3, 1, 3);
plotmf(fis, 'input', 3); % For 'Orbital Zone'
title('Membership Functions for Orbital Zone');

%% Visualise the Membership Functions for Classification:
figure('Name', 'Output Membership Functions');
plotmf(fis, 'output', 1); % For 'Classification'
title('Membership Functions for Classification');

%% Output Surface Views
figure;
gensurf(fis, [1 2]); % Surface for 'Size' and 'Composition'
title('Output Surface for Size and Composition');

figure;
gensurf(fis, [1 3]); % Surface for 'Size' and 'OrbitalZone'
title('Output Surface for Size and Orbital Zone');

% Advanced Surf Plots:
% Visualise Output Surfaces for All Input Combinations
figure('Name', 'FIS Output Surfaces for All Input Combinations');

% Surface for 'Size' and 'Composition'
subplot(2, 2, 1);
gensurf(fis, [1 2], 1); % Indices [1 2] correspond to 'Size' and 'Composition'
title('Output Surface for Size and Composition');

% Surface for 'Size' and 'Orbital Zone'
subplot(2, 2, 2);
gensurf(fis, [1 3], 1); % Indices [1 3] correspond to 'Size' and 'Orbital Zone'
title('Output Surface for Size and Orbital Zone');

% Surface for 'Composition' and 'Orbital Zone'
subplot(2, 2, 3);
gensurf(fis, [2 3], 1); % Indices [2 3] correspond to 'Composition' and 'Orbital Zone'
title('Output Surface for Composition and Orbital Zone');

% Optionally, if you want to see the effect of varying 'Size' while holding 'Composition' and 'Orbital Zone' constant:
% You would set constant values for 'Composition' and 'Orbital Zone' and vary 'Size'.
% For example, if you set 'Composition' = 0.5 (middle value for Gaseous) and 'Orbital Zone' = 0.5 (Asteroid Belt),
% you can create a plot like this:

% Specify the fixed input values for 'Composition' and 'Orbital Zone'
fixedComposition = 0.5;
fixedOrbitalZone = 0.5;

% Create an input matrix where each row represents a set of input conditions
% 'Size' varies from its min to max value, 'Composition' and 'Orbital Zone' are held constant
sizeRange = linspace(0, 1, 100);
inputMatrix = [sizeRange(:), repmat(fixedComposition, length(sizeRange), 1), repmat(fixedOrbitalZone, length(sizeRange), 1)];

% Evaluate the FIS for these input conditions
outputValues = evalfis(fis, inputMatrix);

% Plot the result
subplot(2, 2, 4);
plot(sizeRange, outputValues);
xlabel('Size');
ylabel('FIS Output');
title('FIS Output for Size with Fixed Composition and Orbital Zone');

%  Plot Input/Output Relationships
% For 'Size' input variable
inputRange = getfis(fis, 'Size', 1).Range; % Get the range for the input
inputValues = linspace(inputRange(1), inputRange(2), 100); % Generate 100 points within this range
outputValues = evalfis(fis, inputValues); % Evaluate the FIS for these input values

% For 'Composition' input variable
inputRange = getfis(fis, 'Composition', 1).Range; % Get the range for the input
inputValues = linspace(inputRange(1), inputRange(2), 100); % Generate 100 points within this range
outputValues = evalfis(fis, inputValues); % Evaluate the FIS for these input values

% For 'OrbitalZone' input variable
inputRange = getfis(fis, 'OrbitalZone', 1).Range; % Get the range for the input
inputValues = linspace(inputRange(1), inputRange(2), 100); % Generate 100 points within this range
outputValues = evalfis(fis, inputValues); % Evaluate the FIS for these input values

figure;
plot(inputValues, outputValues);
xlabel('Size');
ylabel('FIS Output');
title('FIS Output for Size');
