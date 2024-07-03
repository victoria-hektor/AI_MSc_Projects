function [new_beta_min, new_beta_max] = adaptMutationRates(beta_min, beta_max, params, currentBestCost, previousBestCost)
    if currentBestCost < previousBestCost % improvement is seen as a decrease in cost
        % Increase beta_min within bounds
        new_beta_min = min(beta_min + params.rateIncrease, params.maxBetaMin);
        % Decrease beta_max but ensure it doesn't go below its minimum allowed value
        new_beta_max = max(beta_max - params.rateIncrease, params.minBetaMax);
    else
        % If no improvement, keep the rates the same
        new_beta_min = beta_min;
        new_beta_max = beta_max;
    end
end
