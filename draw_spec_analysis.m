function draw_spec_analysis(t_sec, data, title_str)
% DRAW_SPEC_ANALYSIS Draw three-axis combined power spectral density time-frequency diagram (Spectrogram)
%
% Inputs:
%   t_sec: Timestamp vector (seconds)
%   data:  Data matrix (Nx3), [X, Y, Z]
%   title_str: Title

    if isempty(data) || isempty(t_sec), return; end

    % 1. Calculate sampling rate
    L = length(t_sec);
    if L < 100, return; end
    T_span = t_sec(end) - t_sec(1);
    Fs = round(L / T_span); % Estimate sampling rate
    
    % 2. Spectrogram parameters (reference your px4_fft.m)
    window = hamming(256);
    noverlap = 128;
    nfft = 256; % or 512 for higher frequency resolution
    
    % 3. Calculate spectrum for each of the three axes separately
    % Note: spectrogram output P is Power Spectral Density (PSD)
    [~, F, T, P1] = spectrogram(data(:,1), window, noverlap, nfft, Fs);
    [~, ~, ~, P2] = spectrogram(data(:,2), window, noverlap, nfft, Fs);
    [~, ~, ~, P3] = spectrogram(data(:,3), window, noverlap, nfft, Fs);
    
    % 4. Combine total power (Sum X+Y+Z)
    P_total = P1 + P2 + P3;
    
    % 5. Plotting (imagesc)
    % T is time relative to window start, need to add data start time
    T_plot = T + t_sec(1); 
    
    % Use 10*log10 to convert to dB units, can see more details
    imagesc(T_plot, F, 10*log10(P_total));
    
    axis xy; % Make frequency axis (Y-axis) increase from bottom to top
    colormap('parula'); % Matlab default colormap, or use 'jet'
    h = colorbar;
    h.Label.String = 'Power Spectral Density (dB/Hz)';
    
    ylabel('Frequency (Hz)');
    xlabel('Time (s)');
    title(title_str);
    
    % Limit frequency display range (usually only show below Nyquist frequency)
    ylim([0, Fs/2]);
end