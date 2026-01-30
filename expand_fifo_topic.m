function [t_out, data_out] = expand_fifo_topic(fifo_table)
% EXPAND_FIFO_TOPIC 将 PX4 FIFO 话题数据展开为连续时间序列
% 输入: fifo_table (table类型, 如 log.data.sensor_accel_fifo_0)
% 输出: 
%   t_out:    时间戳向量 (微秒)
%   data_out: 数据矩阵 [x, y, z]

    if isempty(fifo_table), t_out=[]; data_out=[]; return; end

    % 获取列名以判断格式
    cols = fifo_table.Properties.VariableNames;
    
    % 预估数据总长度 (假设平均样本数为 N)
    est_N = sum(fifo_table.samples);
    t_out = zeros(est_N, 1);
    data_out = zeros(est_N, 3);
    
    current_idx = 1;
    
    % 遍历每一个 FIFO 数据包
    for i = 1:height(fifo_table)
        N = fifo_table.samples(i);
        dt = fifo_table.dt(i);
        scale = fifo_table.scale(i);
        
        % 确定基准时间
        if ismember('timestamp_sample', cols)
            t_end = fifo_table.timestamp_sample(i);
        else
            t_end = fifo_table.timestamp(i);
        end
        
        % 重建该包内的 N 个采样点
        % 时间戳是从 t_end 往前推的: t[k] = t_end - (N-1-k)*dt
        idx_range = current_idx : (current_idx + N - 1);
        
        % 生成时间序列
        % 0 to N-1
        k = (0:N-1)';
        t_batch = t_end - (N - 1 - k) * dt;
        
        % 提取 X, Y, Z 数据
        x_batch = zeros(N, 1);
        y_batch = zeros(N, 1);
        z_batch = zeros(N, 1);
        
        for s = 0:N-1
            % ulog2csv 通常将数组展开为 x_0_, x_1_ ...
            % 我们需要动态构建列名
            x_col = sprintf('x_%d_', s);
            y_col = sprintf('y_%d_', s);
            z_col = sprintf('z_%d_', s);
            
            if ismember(x_col, cols)
                x_batch(s+1) = fifo_table.(x_col)(i);
                y_batch(s+1) = fifo_table.(y_col)(i);
                z_batch(s+1) = fifo_table.(z_col)(i);
            end
        end
        
        % 存入大数组并应用缩放
        t_out(idx_range) = t_batch;
        data_out(idx_range, :) = [x_batch, y_batch, z_batch] * scale;
        
        current_idx = current_idx + N;
    end
    
    % 截断多余的预分配空间 (如果有)
    t_out = t_out(1:current_idx-1);
    data_out = data_out(1:current_idx-1, :);
end