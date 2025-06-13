% Load CSV file
data = readmatrix('log2.csv');  

% Extract columns
timestamp = data(:, 1);
Q = data(:, 2);
I = data(:, 3);
freq = data(:, 4);

% Get unique frequencies
unique_freqs = unique(freq);

% Create one figure
figure;
hold on;

% Generate distinct colors
colors = lines(length(unique_freqs));

% Plot Q and I samples for each frequency
for i = 1:length(unique_freqs)
    f = unique_freqs(i);
    idx = freq == f;

    % Plot Q samples
    plot(timestamp(idx), Q(idx), 'o', 'Color', colors(i, :), ...
        'DisplayName', ['Q @ ' num2str(f) ' Hz']);

    % Plot I samples
    plot(timestamp(idx), I(idx), 'x', 'Color', colors(i, :), ...
        'DisplayName', ['I @ ' num2str(f) ' Hz']);
end

% Final touches
title('Q and I Samples vs Time (All Frequencies)');
xlabel('Timestamp');
ylabel('Amplitude');
legend('show');
grid on;
