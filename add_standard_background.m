%% =========================================================================
%  Helper Function 4: Standard Background Drawing Wrapper (One line to handle background)
% =========================================================================
function add_standard_background(vis_flight_intervals, vis_flight_names, vis_is_vtol, vis_vtol_intervals, vis_vtol_names)
    % 1. Get current Y axis range
    yl = ylim;                
    y_h = yl(2) - yl(1);      

    % 2. Draw flight mode (main background)
    draw_status_band(vis_flight_intervals, vis_flight_names, yl, 'flight_mode');

    % 3. Draw VTOL state (bottom 8% band)
    if vis_is_vtol
        y_band_top = yl(1) + 0.1 * y_h; 
        draw_status_band(vis_vtol_intervals, vis_vtol_names, [yl(1), y_band_top], 'vtol_state');
    end
end