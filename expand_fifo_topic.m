function [t_out, data_out] = expand_fifo_topic(fifo_table)
% EXPAND_FIFO_TOPIC Expand PX4 FIFO topic data to continuous time series
% Input: fifo_table (table type, e.g. log.data.sensor_accel_fifo_0)
% Output: 
%   t_out:    Timestamp vector (microseconds)
%   data_out: Data matrix [x, y, z]

    if isempty(fifo_table), t_out=[]; data_out=[]; return; end

    % Get column names to determine format
    cols = fifo_table.Properties.VariableNames;
    
    % Estimate total data length (assuming average sample count is N)
    est_N = sum(fifo_table.samples);
    t_out = zeros(est_N, 1);
    data_out = zeros(est_N, 3);
    
    current_idx = 1;
    
    % Iterate through each FIFO data packet
    for i = 1:height(fifo_table)
        N = fifo_table.samples(i);
        dt = fifo_table.dt(i);
        scale = fifo_table.scale(i);
        
        % Determine baseline time
        if ismember('timestamp_sample', cols)
            t_end = fifo_table.timestamp_sample(i);
        else
            t_end = fifo_table.timestamp(i);
        end
        
        % Reconstruct N sampling points within this packet
        % Timestamp goes backwards from t_end: t[k] = t_end - (N-1-k)*dt
        idx_range = current_idx : (current_idx + N - 1);
        
        % Generate time series
        % 0 to N-1
        k = (0:N-1)';
        t_batch = t_end - (N - 1 - k) * dt;
        
        % Extract X, Y, Z data
        x_batch = zeros(N, 1);
        y_batch = zeros(N, 1);
        z_batch = zeros(N, 1);
        
        for s = 0:N-1
            % ulog2csv usually expands arrays to x_0_, x_1_ ...
            % We need to dynamically construct column names
            x_col = sprintf('x_%d_', s);
            y_col = sprintf('y_%d_', s);
            z_col = sprintf('z_%d_', s);
            
            if ismember(x_col, cols)
                x_batch(s+1) = fifo_table.(x_col)(i);
                y_batch(s+1) = fifo_table.(y_col)(i);
                z_batch(s+1) = fifo_table.(z_col)(i);
            end
        end
        
        % Store into large array and apply scaling
        t_out(idx_range) = t_batch;
        data_out(idx_range, :) = [x_batch, y_batch, z_batch] * scale;
        
        current_idx = current_idx + N;
    end
    
    % Truncate excess preallocated space (if any)
    t_out = t_out(1:current_idx-1);
    data_out = data_out(1:current_idx-1, :);
end