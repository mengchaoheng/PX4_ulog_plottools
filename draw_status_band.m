%% =========================================================================
%  Helper Function 3: Generic Status Band Drawing (Support specifying Y axis range)
% =========================================================================
function draw_status_band(intervals, labels, y_range, color_mode)
    % y_range: [y_bottom, y_top] specify drawing area
    % color_mode: 'flight_mode' or 'vtol_state' to select color scheme
    
    if isempty(intervals), return; end
    
    % --- Color Scheme ---
    colors = containers.Map('KeyType', 'double', 'ValueType', 'any');
    default_c = [0.95, 0.95, 0.95];
    
    if strcmp(color_mode, 'flight_mode')
        % Flight mode colors (soft background colors)
        colors(0) = [0.90, 0.90, 0.90]; % Manual (Gray)
        colors(1) = [0.85, 0.95, 1.0];  % Altitude
        colors(2) = [0.80, 0.90, 1.0];  % Position (Light Blue)
        colors(3) = [0.85, 1.0, 0.85];  % Mission (Light Green)
        colors(4) = [0.90, 0.90, 0.80]; % Loiter
        colors(5) = [1.0, 0.90, 0.90];  % RTL (Reddish)
        colors(14)= [1.0, 0.95, 0.80];  % Offboard (Yellowish)
        colors(15)= [0.80, 0.95, 1.0];  % Stabilized
    elseif strcmp(color_mode, 'vtol_state')
        % VTOL state colors (more vivid, for bottom band)
        colors(1) = [0.6, 0.8, 1.0];    % MC (Blue)
        colors(2) = [0.6, 1.0, 0.6];    % FW (Green)
        colors(3) = [1.0, 0.7, 0.4];    % Transition (Orange)
    end
    
    hold on;
    y_b = y_range(1); % bottom
    y_t = y_range(2); % top
    
    % === [Modification Start] Text Position Calculation ===
    % Determine text vertical position and alignment based on mode
    if strcmp(color_mode, 'flight_mode')
        % Flight mode: place at top (offset down 2% of height from top to avoid line overlap)
        text_y = y_t - (y_t - y_b) * 0.05;
        text_valign = 'top';   % Vertical alignment: top
    else
        % VTOL or other modes: vertically centered (since VTOL band is already at bottom)
        text_y = y_b + (y_t - y_b) / 2;
        text_valign = 'middle'; % Vertical alignment: middle
    end
    % === [Modification End] ===

    for i = 1:size(intervals, 1)
        t_s = intervals(i, 1);
        t_e = intervals(i, 2);
        val = intervals(i, 3);
        
        if isKey(colors, val), c = colors(val); else, c = default_c; end
        
        % Draw rectangle
        p = patch([t_s t_e t_e t_s], [y_b y_b y_t y_t], c);
        set(p, 'EdgeColor', 'none', 'FaceAlpha', 0.5, 'HandleVisibility', 'off');
        
        % Text label 
        % Only display text when duration is long enough
        if (t_e - t_s) > 2.0
            text(t_s + (t_e-t_s)/2, text_y, labels{i}, ... % Use calculated text_y
                'HorizontalAlignment', 'center', ...
                'VerticalAlignment', text_valign, ...      % Use calculated alignment
                'FontSize', 8, 'Color', [0.2 0.2 0.2], 'Interpreter', 'none', 'Clipping', 'on');
        end
    end
    % Ensure grid and curves are on top layer
    set(gca, 'Layer', 'top');
end