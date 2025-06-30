% Load CSV file
data = readmatrix('log1.csv');

% Extract columns
timestamp = data(:, 1);
Q = data(:, 2);
I = data(:, 3);
freq = data(:, 4);
phase = data(:, 5); % in degrees

% Get unique frequencies
unique_freqs = unique(freq);

% Define fixed colors
q_color = [0 0.4470 0.7410];      % Blue for Q
i_color = [0.8500 0.3250 0.0980]; % Red for I
p_color = [0.4660 0.6740 0.1880]; % Green for Phase

for i = 1:length(unique_freqs)
    f = unique_freqs(i);
    idx = freq == f;

    % Extract and normalize signals
    t = timestamp(idx);
    Q_raw = Q(idx);
    I_raw = I(idx);
    phase_f = phase(idx);
    Q_norm = normalize(Q_raw, 'range');
    I_norm = normalize(I_raw, 'range');

    % === FIGURE 1: Raw Q and I ===
    figure;
    hold on;
    plot(t, Q_raw, '-o', 'Color', q_color, 'DisplayName', 'Q (raw)');
    plot(t, I_raw, '-x', 'Color', i_color, 'DisplayName', 'I (raw)');
    title(['Raw Q and I vs Time @ ' num2str(f) ' Hz']);
    xlabel('Timestamp');
    ylabel('Amplitude (Ohms)');
    legend('show');
    grid on;

    % === FIGURE 2: Normalized Q and I ===
    figure;
    hold on;
    plot(t, Q_norm, '-o', 'Color', q_color, 'DisplayName', 'Q (norm)');
    plot(t, I_norm, '-x', 'Color', i_color, 'DisplayName', 'I (norm)');
    title(['Normalized Q and I vs Time @ ' num2str(f) ' Hz']);
    xlabel('Timestamp');
    ylabel('Normalized Amplitude [0–1]');
    legend('show');
    grid on;
end

% === FIGURE 3: Phase for both frequencies ===
figure;
hold on;

for i = 1:length(unique_freqs)
    f = unique_freqs(i);
    idx = freq == f;
    t = timestamp(idx);
    phase_f = phase(idx);
    
    if f > 100000
        plot(t, phase_f, '-o', 'DisplayName', ['Phase @ ' num2str(f/1000) ' kHz']);
    else
        plot(t, phase_f, '-x', 'DisplayName', ['Phase @ ' num2str(f/1000) ' kHz']);
    end
end

title('Phase vs Time');
xlabel('Timestamp');
ylabel('Phase (degrees)');
legend('show');
grid on;
