%% =========================================================================
%  辅助函数 3：通用状态带绘制 (支持指定 Y 轴范围)
% =========================================================================
function draw_status_band(intervals, labels, y_range, color_mode)
    % y_range: [y_bottom, y_top] 指定绘制区域
    % color_mode: 'flight_mode' 或 'vtol_state' 用于选择配色方案
    
    if isempty(intervals), return; end
    
    % --- 配色方案 ---
    colors = containers.Map('KeyType', 'double', 'ValueType', 'any');
    default_c = [0.95, 0.95, 0.95];
    
    if strcmp(color_mode, 'flight_mode')
        % 飞行模式颜色 (柔和背景色)
        colors(0) = [0.90, 0.90, 0.90]; % Manual (Gray)
        colors(1) = [0.85, 0.95, 1.0];  % Altitude
        colors(2) = [0.80, 0.90, 1.0];  % Position (Light Blue)
        colors(3) = [0.85, 1.0, 0.85];  % Mission (Light Green)
        colors(4) = [0.90, 0.90, 0.80]; % Loiter
        colors(5) = [1.0, 0.90, 0.90];  % RTL (Reddish)
        colors(14)= [1.0, 0.95, 0.80];  % Offboard (Yellowish)
        colors(15)= [0.80, 0.95, 1.0];  % Stabilized
    elseif strcmp(color_mode, 'vtol_state')
        % VTOL 状态颜色 (鲜艳一点，用于底部条带)
        colors(1) = [0.6, 0.8, 1.0];    % MC (Blue)
        colors(2) = [0.6, 1.0, 0.6];    % FW (Green)
        colors(3) = [1.0, 0.7, 0.4];    % Transition (Orange)
    end
    
    hold on;
    y_b = y_range(1); % bottom
    y_t = y_range(2); % top
    
    % === [修改开始] 文字位置计算 ===
    % 根据模式决定文字的垂直位置和对齐方式
    if strcmp(color_mode, 'flight_mode')
        % 飞行模式：放在顶部 (顶部向下偏移 2% 的高度，防止压线)
        text_y = y_t - (y_t - y_b) * 0.05;
        text_valign = 'top';   % 垂直对齐：顶对齐
    else
        % VTOL 或其他模式：垂直居中 (因为 VTOL 条带本身就在底部)
        text_y = y_b + (y_t - y_b) / 2;
        text_valign = 'middle'; % 垂直对齐：居中
    end
    % === [修改结束] ===

    for i = 1:size(intervals, 1)
        t_s = intervals(i, 1);
        t_e = intervals(i, 2);
        val = intervals(i, 3);
        
        if isKey(colors, val), c = colors(val); else, c = default_c; end
        
        % 绘制矩形
        p = patch([t_s t_e t_e t_s], [y_b y_b y_t y_t], c);
        set(p, 'EdgeColor', 'none', 'FaceAlpha', 0.5, 'HandleVisibility', 'off');
        
        % 文字标签 
        % 只有当持续时间足够长时才显示文字
        if (t_e - t_s) > 2.0
            text(t_s + (t_e-t_s)/2, text_y, labels{i}, ... % 使用计算好的 text_y
                'HorizontalAlignment', 'center', ...
                'VerticalAlignment', text_valign, ...      % 使用计算好的对齐方式
                'FontSize', 8, 'Color', [0.2 0.2 0.2], 'Interpreter', 'none', 'Clipping', 'on');
        end
    end
    % 确保网格和曲线在最上层
    set(gca, 'Layer', 'top');
end