% Load CSV file
data = readmatrix('log2.csv');

% Extract columns
timestamp = data(:, 1);
Q = data(:, 2);
I = data(:, 3);
freq = data(:, 4);

% Get unique frequencies
unique_freqs = unique(freq);

% Define fixed colors
q_color = [0 0.4470 0.7410];      % Blue for Q
i_color = [0.8500 0.3250 0.0980]; % Red for I

% Compute global min/max for normalization
Q_min = min(Q);
Q_max = max(Q);
I_min = min(I);
I_max = max(I);

Q_range = max(Q_max - Q_min, 1e-9);
I_range = max(I_max - I_min, 1e-9);

for i = 1:length(unique_freqs)
    f = unique_freqs(i);
    idx = freq == f;

    % Relative time in seconds
    t = (timestamp(idx) - timestamp(1)) / 1e3;
    Q_raw = Q(idx);
    I_raw = I(idx);
    Q_norm = (Q_raw - Q_min) / Q_range;
    I_norm = (I_raw - I_min) / I_range;

    % Compute padded x-limits
    full_range = t(end) - t(1);
    padding = 0.05 * full_range;
    x_min = t(1);
    x_max = t(end) + padding;

    % === FIGURE 1: Raw Q and I ===
    figure;
    hold on;
    plot(t, Q_raw, '-o', 'Color', q_color, 'DisplayName', 'Q (raw)');
    plot(t, I_raw, '-x', 'Color', i_color, 'DisplayName', 'I (raw)');
    title(['Raw Q and I vs Time @ ' num2str(f) ' Hz']);
    xlabel('Time (seconds)');
    ylabel('Amplitude (Ohms)');
    legend('show');
    grid on;

    if t(end) > 3
        xlim([x_min, x_min + 3]);
    end
    pan on;

    % === FIGURE 2: Normalized Q and I ===
    figure;
    hold on;
    plot(t, Q_norm, '-o', 'Color', q_color, 'DisplayName', 'Q (norm)');
    plot(t, I_norm, '-x', 'Color', i_color, 'DisplayName', 'I (norm)');
    title(['Normalized Q and I vs Time @ ' num2str(f) ' Hz']);
    xlabel('Time (seconds)');
    ylabel('Normalized Amplitude [0–1]');
    legend('show');
    grid on;

    if t(end) > 3
        xlim([x_min, x_min + 3]);
    end
    pan on;
end
