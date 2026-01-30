function draw_fft_analysis(t_us, data, legends, title_str, params_struct, marker_configs)
% DRAW_FFT_ANALYSIS 通用 FFT 分析与绘图函数
%
% 输入:
%   t_us:           时间戳向量 (微秒)
%   data:           数据矩阵 (Nx3), 每一列是一个轴的数据 (Roll/Pitch/Yaw)
%   legends:        图例 cell array, 例如 {'Roll', 'Pitch', 'Yaw'}
%   title_str:      图表标题
%   params_struct:  参数结构体 (log.params)，用于查找截止频率
%   marker_configs: 需要标记的参数配置 Nx2 cell, {ParamName, LabelName}

    % 1. 基础检查
    if isempty(data) || isempty(t_us), return; end
    L = size(data, 1);
    if L < 2, return; end
    
    % 2. 计算采样频率 Fs
    % 计算平均采样时间 (秒)
    T_sec = (double(t_us(end)) - double(t_us(1))) * 1e-6 / (L - 1);
    Fs = 1 / T_sec;
    
    % 3. 计算 FFT
    % 使用 Next Power of 2 优化计算速度
    NFFT = 2^nextpow2(L); 
    Y = fft(data, NFFT);
    
    % 4. 计算单边谱 (Single-Sided Spectrum) P1
    P2 = abs(Y / L);
    P1 = P2(1:NFFT/2+1, :);
    P1(2:end-1, :) = 2 * P1(2:end-1, :);
    
    f = Fs * (0:(NFFT/2)) / NFFT;
    
    % 5. 绘图
    % 自动创建新图窗 (如果外部没创建)
    % figure('Color', 'w', 'Name', title_str); 
    
    colors = {'r', 'k', 'b'}; % Roll=红, Pitch=黑, Yaw=蓝
    hold on;
    for i = 1:min(3, size(data, 2))
        % 使用半对数坐标 (semilogy) 能更好地看到低幅值的噪声底噪
        semilogy(f, P1(:, i), 'Color', colors{i}, 'LineWidth', 0.8);
    end
    
    % 6. 自动标记参数截止频率 (Cutoff Frequencies)
    % 获取当前坐标轴范围，以便画线
    y_lim = ylim; 
    
    if ~isempty(params_struct) && ~isempty(marker_configs)
        for k = 1:size(marker_configs, 1)
            p_name = marker_configs{k, 1};
            label_text = marker_configs{k, 2};
            
            % 检查参数是否存在
            if isfield(params_struct, p_name)
                freq_val = double(params_struct.(p_name));
                
                % 只有大于0且在奈奎斯特频率内的才画
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
    
    % 7. 装饰图表
    title(sprintf('%s (Fs \\approx %.1f Hz)', title_str, Fs), 'Interpreter', 'none');
    xlabel('Frequency (Hz)');
    ylabel('|P1(f)| (Amplitude)');
    grid on;
    legend(legends, 'Location', 'northeast');
    xlim([0, Fs/2]); % 只显示到奈奎斯特频率
end