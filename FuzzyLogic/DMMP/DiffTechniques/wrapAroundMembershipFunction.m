function mfValues = wrapAroundMembershipFunction(x)
    % Parameters for the membership function
    a = 0;
    b = 0.3;
    c = 0.6;
    d = 1;

    % Compute membership function values
    mfValues = zeros(size(x));
    mfValues(x < a) = 0;
    mfValues(x >= a & x <= b) = (x(x >= a & x <= b) - a) / (b - a);
    mfValues(x > b & x < c) = 1;
    mfValues(x >= c & x <= d) = (d - x(x >= c & x <= d)) / (d - c);
    mfValues(x > d) = 0;
end