function generateSurfPlot(fis, fixedWidthValue)
    % Generate input grids for Galaxy Rotation Curve and Cosmic Microwave Background
    rotationCurveRange = linspace(0, 1, 100); % Range for Galaxy Rotation Curve
    cmbRange = linspace(0, 1, 100); % Range for Cosmic Microwave Background
    [RotationCurve, CMB] = meshgrid(rotationCurveRange, cmbRange);

    % Initialise an output matrix for the Dark Matter Mass results
    Mass = zeros(size(RotationCurve));

    % Evaluate the FIS for each combination of RotationCurve and CMB, with fixed gravitational lensing width
    for i = 1:size(RotationCurve, 1)
        for j = 1:size(RotationCurve, 2)
            input = [RotationCurve(i,j), CMB(i,j), fixedWidthValue]; % ensure these are normalised
            output = evalfis(fis, input);
            if isempty(output) || isnan(output)
                disp('No output for input: ');
                disp(input);
                Mass(i,j) = NaN; % Assign NaN to indicate no output
            else
                Mass(i,j) = output;
            end
        end
    end

    % Check the size of the Mass matrix
    disp(['Size of Mass matrix: ', num2str(size(Mass))]);
    
    % If Mass is not a matrix, display an error message
    if isvector(Mass) || isscalar(Mass)
        error('Mass must be a matrix. Check your FIS configuration and inputs.');
    end

    % Create the surface plot
    figure; % Create a new figure
    surf(RotationCurve, CMB, Mass); % Generate the surface plot
    xlabel('Galaxy Rotation Curve');
    ylabel('Cosmic Microwave Background');
    zlabel('Estimated Dark Matter Mass (eV)');
    title('Fuzzy Inference System Output: Estimated Dark Matter Mass');
    shading interp; % Interpolate colors across surfaces and patches
    colorbar; % Display colorbar
end