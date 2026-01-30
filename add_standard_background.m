%% =========================================================================
%  辅助函数 4: 标准背景绘制封装 (一行代码搞定背景)
% =========================================================================
function add_standard_background(vis_flight_intervals, vis_flight_names, vis_is_vtol, vis_vtol_intervals, vis_vtol_names)
    % 1. 获取当前 Y 轴范围
    yl = ylim;                
    y_h = yl(2) - yl(1);      

    % 2. 绘制飞行模式 (主背景)
    draw_status_band(vis_flight_intervals, vis_flight_names, yl, 'flight_mode');

    % 3. 绘制 VTOL 状态 (底部 8% 条带)
    if vis_is_vtol
        y_band_top = yl(1) + 0.08 * y_h; 
        draw_status_band(vis_vtol_intervals, vis_vtol_names, [yl(1), y_band_top], 'vtol_state');
    end
end