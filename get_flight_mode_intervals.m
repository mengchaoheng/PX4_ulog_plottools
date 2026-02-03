function [intervals, mode_names] = get_flight_mode_intervals(status_topic)
    if isempty(status_topic), intervals=[]; mode_names={}; return; end

    t = double(status_topic.timestamp) * 1e-6; 
    nav_state = status_topic.nav_state;
    
    % Mapping definition (keep unchanged)
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
        
        % === Modification Start ===
        if i < length(changes)
            % Key modification: end index directly takes the start index of next segment
            % This way when plotting, the right edge of current block will overlap with left edge of next block
            idx_e = changes(i+1); 
        else
            % Last segment, take total data length
            idx_e = length(nav_state); 
        end
        % === Modification End ===
        
        val = double(nav_state(idx_s));
        intervals = [intervals; t(idx_s), t(idx_e), val];
        
        if isKey(map, val), mode_names{end+1} = map(val);
        else, mode_names{end+1} = sprintf('Mode %d', val); end
    end
end