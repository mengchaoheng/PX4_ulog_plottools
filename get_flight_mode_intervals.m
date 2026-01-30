function [intervals, mode_names] = get_flight_mode_intervals(status_topic)
    if isempty(status_topic), intervals=[]; mode_names={}; return; end

    t = double(status_topic.timestamp) * 1e-6; 
    nav_state = status_topic.nav_state;
    
    % 映射定义 (保持不变)
    map = containers.Map('KeyType', 'double', 'ValueType', 'char');
    map(0) = 'Manual'; map(1) = 'Altitude'; map(2) = 'Position';
    map(3) = 'Mission'; map(4) = 'Loiter'; map(5) = 'RTL';
    map(10) = 'Acro'; map(14) = 'Offboard'; map(15) = 'Stabilized';
    map(17) = 'Takeoff'; map(18) = 'Land'; map(20) = 'Precland';
    map(22) = 'VTOL Takeoff';
    
    changes = find([1; diff(nav_state) ~= 0]);
    
    intervals = [];
    mode_names = {};
    
    for i = 1:length(changes)
        idx_s = changes(i);
        
        % === 修改开始 ===
        if i < length(changes)
            % 关键修改：结束索引直接取下一段的开始索引
            % 这样绘图时，当前色块的右边缘会和下一个色块的左边缘重合
            idx_e = changes(i+1); 
        else
            % 最后一段，取数据总长度
            idx_e = length(nav_state); 
        end
        % === 修改结束 ===
        
        val = double(nav_state(idx_s));
        intervals = [intervals; t(idx_s), t(idx_e), val];
        
        if isKey(map, val), mode_names{end+1} = map(val);
        else, mode_names{end+1} = sprintf('Mode %d', val); end
    end
end