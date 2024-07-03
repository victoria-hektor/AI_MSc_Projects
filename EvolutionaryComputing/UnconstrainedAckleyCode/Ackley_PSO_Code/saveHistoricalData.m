function saveHistoricalData(particleHistory, filename)
    % Determine the size of particleHistory
    [numParticles, numDimensions, numIterations] = size(particleHistory);
    
    % Flatten the 3D matrix to a 2D cell array for saving
    flatData = reshape(particleHistory, numParticles, numDimensions * numIterations);
    
    % Create a header for the spreadsheet
    header = cell(1, numDimensions * numIterations);
    for it = 1:numIterations
        for dim = 1:numDimensions
            header{(it - 1) * numDimensions + dim} = sprintf('Var%d_Iter%d', dim, it);
        end
    end
    
    % Combine header and data
    outputCell = [header; num2cell(flatData)];
    
    % Write header and data to file
    writecell(outputCell, filename);
end