function draw_spec_analysis(t_sec, data, title_str)
% DRAW_SPEC_ANALYSIS 绘制三轴合成的功率谱密度时频图 (Spectrogram)
%
% 输入:
%   t_sec: 时间戳向量 (秒)
%   data:  数据矩阵 (Nx3), [X, Y, Z]
%   title_str: 标题

    if isempty(data) || isempty(t_sec), return; end

    % 1. 计算采样率
    L = length(t_sec);
    if L < 100, return; end
    T_span = t_sec(end) - t_sec(1);
    Fs = round(L / T_span); % 估算采样率
    
    % 2. Spectrogram 参数 (参考您的 px4_fft.m)
    window = hamming(256);
    noverlap = 128;
    nfft = 256; % 或者 512 以获得更高的频率分辨率
    
    % 3. 分别计算三轴的频谱
    % 注意：spectrogram 的输出 P 是功率谱密度 (PSD)
    [~, F, T, P1] = spectrogram(data(:,1), window, noverlap, nfft, Fs);
    [~, ~, ~, P2] = spectrogram(data(:,2), window, noverlap, nfft, Fs);
    [~, ~, ~, P3] = spectrogram(data(:,3), window, noverlap, nfft, Fs);
    
    % 4. 合成总功率 (Sum X+Y+Z)
    P_total = P1 + P2 + P3;
    
    % 5. 绘图 (imagesc)
    % T 是相对于窗口开始的时间，需要加上数据的起始时间
    T_plot = T + t_sec(1); 
    
    % 使用 10*log10 转换为 dB 单位，能看清更多细节
    imagesc(T_plot, F, 10*log10(P_total));
    
    axis xy; % 让频率轴（Y轴）从下往上增长
    colormap('parula'); % Matlab 默认色谱，或者用 'jet'
    h = colorbar;
    h.Label.String = 'Power Spectral Density (dB/Hz)';
    
    ylabel('Frequency (Hz)');
    xlabel('Time (s)');
    title(title_str);
    
    % 限制频率显示范围 (通常只看奈奎斯特频率以下)
    ylim([0, Fs/2]);
end