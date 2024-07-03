function [y1, y2] = HUXCrossover(x1, x2)
    % HUXCrossover performs half uniform crossover on binary strings
    % Inputs: x1, x2 are parent binary strings (vectors)
    % Outputs: y1, y2 are offspring binary strings (vectors)
    
    % Identifying indices where the parents differ
    diffIndices = find(x1 ~= x2);
    
    % Randomly selecting half of the differing indices to swap
    indicesToSwap = randsample(diffIndices, floor(length(diffIndices)/2));
    
    % Creating offspring by copying parents
    y1 = x1;
    y2 = x2;
    
    % Swapping the selected bits
    y1(indicesToSwap) = x2(indicesToSwap);
    y2(indicesToSwap) = x1(indicesToSwap);
end
