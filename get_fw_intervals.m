    function fw_intervals = get_fw_intervals(vehicle_status)
    % fw_intervals: Nx2 [t_start, t_end] in microseconds
    
    t = vehicle_status(:,1);          % timestamp (us)
    vehicle_type = vehicle_status(:,16);
    
    is_fw = (vehicle_type == 2);
    
    % 找到连续区间
    d = diff([0; is_fw; 0]);
    start_idx = find(d == 1);
    end_idx   = find(d == -1) - 1;
    
    fw_intervals = [t(start_idx), t(end_idx)];
    end