% A Separate File for the data split Function
function [trainData, valData, testData] = split_data(data, train_ratio, val_ratio, test_ratio)
    total_samples = size(data, 1);
    train_samples = round(train_ratio * total_samples);
    val_samples = round(val_ratio * total_samples);

    trainData = data(1:train_samples, :);
    valData = data(train_samples + 1:train_samples + val_samples, :);
    testData = data(train_samples + val_samples + 1:end, :);
end
