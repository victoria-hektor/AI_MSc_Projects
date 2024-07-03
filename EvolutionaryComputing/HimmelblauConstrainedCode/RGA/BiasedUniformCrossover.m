function [y1, y2] = BiasedUniformCrossover(x1, x2, bias)
    % BiasedUniformCrossover performs uniform crossover with a bias towards one parent
    % Inputs: x1, x2 are parent binary strings (vectors), bias is the probability of choosing a gene from x1
    % Outputs: y1, y2 are offspring binary strings (vectors)
    
    % Initialise offspring
    y1 = zeros(1, length(x1));
    y2 = zeros(1, length(x2));
    
    for i = 1:length(x1)
        if rand <= bias
            y1(i) = x1(i);
            y2(i) = x2(i);
        else
            y1(i) = x2(i);
            y2(i) = x1(i);
        end
    end
end
