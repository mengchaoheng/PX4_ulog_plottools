function [intervals, state_names] = get_vtol_state_intervals(status_topic)
    if isempty(status_topic), intervals=[]; state_names={}; return; end
    
    t = double(status_topic.timestamp) * 1e-6;
    
    if ismember('vehicle_type', status_topic.Properties.VariableNames)
        v_type = double(status_topic.vehicle_type);
    else
        if ismember('is_rotary_wing', status_topic.Properties.VariableNames)
             v_type = ones(size(t)); 
             v_type(status_topic.is_rotary_wing == 0) = 2; 
        else
             intervals=[]; state_names={}; return; 
        end
    end
    
    combined_state = v_type;
    if ismember('in_transition_mode', status_topic.Properties.VariableNames)
        in_trans = double(status_topic.in_transition_mode) > 0;
        combined_state(in_trans) = 3; 
    end
    
    changes = find([1; diff(combined_state) ~= 0]);
    
    intervals = [];
    state_names = {};
    
    map = containers.Map('KeyType', 'double', 'ValueType', 'char');
    map(1) = 'MC'; map(2) = 'FW'; map(3) = 'Trans';
    
    for i = 1:length(changes)
        idx_s = changes(i);
        
        % === 修改开始 ===
        if i < length(changes)
            % 关键修改：为了消除缝隙，使用下一段的起始点作为本段的终点
            idx_e = changes(i+1); 
        else
            idx_e = length(combined_state);
        end
        % === 修改结束 ===
        
        val = combined_state(idx_s);
        intervals = [intervals; t(idx_s), t(idx_e), val];
        
        if isKey(map, val), state_names{end+1} = map(val);
        else, state_names{end+1} = 'Unk'; end
    end
end