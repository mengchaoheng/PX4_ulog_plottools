function draw_fft_analysis(t, data, legends, title_str, params_struct, marker_configs)
% DRAW_FFT_ANALYSIS Generic FFT analysis and plotting function
%
% Inputs:
%   t:           Timestamp vector (seconds)
%   data:           Data matrix (Nx3), each column is data for one axis (Roll/Pitch/Yaw)
%   legends:        Legend cell array, e.g. {'Roll', 'Pitch', 'Yaw'}
%   title_str:      Chart title
%   params_struct:  Parameter structure (log.params), for finding cutoff frequencies
%   marker_configs: Parameter configurations to mark Nx2 cell, {ParamName, LabelName}

    % 1. Basic checks
    if isempty(data) || isempty(t), return; end
    L = size(data, 1);
    if L < 2, return; end
    
    % 2. Calculate sampling frequency Fs
    % Calculate average sampling time (seconds)
    T_sec = (double(t(end)) - double(t(1)))  / (L - 1);
    Fs = 1 / T_sec;
    
    % 3. Calculate FFT
    % Use Next Power of 2 to optimize computation speed
    NFFT = 2^nextpow2(L); 
    Y = fft(data, NFFT);
    
    % 4. Calculate Single-Sided Spectrum P1
    P2 = abs(Y / L);
    P1 = P2(1:NFFT/2+1, :);
    P1(2:end-1, :) = 2 * P1(2:end-1, :);
    
    f = Fs * (0:(NFFT/2)) / NFFT;
    
    % 5. Plotting
    % Automatically create new figure (if not created externally)
    % figure('Color', 'w', 'Name', title_str); 
    
    colors = {'r', 'k', 'b'}; % Roll=red, Pitch=black, Yaw=blue
    hold on;
    for i = 1:min(3, size(data, 2))
        % Using semilogarithmic coordinates (semilogy) better shows low-amplitude noise floor
        semilogy(f, P1(:, i), 'Color', colors{i}, 'LineWidth', 0.8);
    end
    
    % 6. Automatically mark parameter cutoff frequencies (Cutoff Frequencies)
    % Get current axis range to draw lines
    y_lim = ylim; 
    
    if ~isempty(params_struct) && ~isempty(marker_configs)
        for k = 1:size(marker_configs, 1)
            p_name = marker_configs{k, 1};
            label_text = marker_configs{k, 2};
            
            % Check if parameter exists
            if isfield(params_struct, p_name)
                freq_val = double(params_struct.(p_name));
                
                % Only draw if greater than 0 and within Nyquist frequency
                if freq_val > 0 && freq_val < (Fs/2)
                    xline(freq_val, '--', [label_text ' (' num2str(freq_val) ' Hz)'], ...
                        'Color', [0.4 0.4 0.4], ...
                        'LabelVerticalAlignment', 'top', ...
                        'LabelHorizontalAlignment', 'left', ...
                        'FontSize', 8, 'Interpreter', 'none');
                end
            end
        end
    end
    
    % 7. Decorate chart
    title(sprintf('%s (Fs \\approx %.1f Hz)', title_str, Fs), 'Interpreter', 'none');
    xlabel('Frequency (Hz)');
    ylabel('|P1(f)| (Amplitude)');
    grid on;
    legend(legends, 'Location', 'northeast');
    xlim([0, Fs/2]); % Only display up to Nyquist frequency
end