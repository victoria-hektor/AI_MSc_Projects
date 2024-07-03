function rate = adaptiveRate(iteration, maxIter, initialRate, finalRate)
    % Linearly adapt the rate from initial to final over the iterations
    rate = initialRate - (iteration / maxIter) * (initialRate - finalRate);
    rate = max(rate, finalRate); % Ensure rate does not go below finalRate
end
