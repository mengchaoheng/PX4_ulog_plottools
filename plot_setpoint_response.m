clear all;
close all;
clc;
addpath(genpath(pwd));

run('load_data_main.m'); % load('flight_data.mat');

% Parameter settings
MAX_MOTORS = 12; 
MAX_SERVOS = 8;  
plot_together = 0; % 1: Servos and motors combined display, 0: Servos and motors independent display
verbose = 1;  % 1: Display more, such as fft, time-frequency diagram, 0: Only display main states
control_fig =1; % 1: Display control quantities, 0: Only display main states
n_raw_plot = 8; % For versions before 1.13, plot first 8 channels of pwm
%% =========================================================================
%  Figure 1 2, 3, 4, 5: (vehicle_angular_velocity, Attitude, Vel, Pos, Control)
% =========================================================================
% --- Figure 1: vehicle_angular_velocity ---
if(exist('vehicle_angular_velocity', 'var') && exist('vehicle_rates_setpoint', 'var'))
    figure('Name', 'Rates', 'Color', 'w');
    titles = {'Roll Rate', 'Pitch Rate', 'Yaw Rate'};
    ylabels = {'p (deg/s)', 'q (deg/s)', 'r (deg/s)'};
    ax = [];
    for i = 1:3
        ax(i) = subplot(3,1,i); hold on;
        plot(vehicle_rates_setpoint_t, vehicle_rates_setpoint(:,i)*r2d, 'k-', 'LineWidth', 1);
        plot(vehicle_angular_velocity_t, vehicle_angular_velocity(:,i)*r2d, '--', 'LineWidth', 1, 'Color', [0.8, 0.3, 0]);
        grid on; ylabel(ylabels{i}); title(titles{i});
        if i==1, legend('Setpoint', 'Response', 'Location', 'best'); end
        add_standard_background(vis_flight_intervals, vis_flight_names, vis_is_vtol, vis_vtol_intervals, vis_vtol_names);
    end
    linkaxes(ax, 'x'); xlabel('Time (s)');
    % PlotToFile(gcf, 'results/rates.png', 20, 20);
end

% --- Figure 2: Attitude ---
if(exist('Roll', 'var') && exist('Roll_setpoint', 'var'))
    figure('Name', 'Attitude', 'Color', 'w');
    d_sp = {Roll_setpoint, Pitch_setpoint, Yaw_setpoint};
    d_res = {Roll, Pitch, Yaw};
    titles = {'Roll', 'Pitch', 'Yaw'};

    % \varphi corresponds to Roll, \theta corresponds to Pitch, \phi corresponds to Yaw
    ylabels = {'$\varphi$ (deg)', '$\theta$ (deg)', '$\phi$ (deg)'};

    ax = [];
    for i = 1:3
        ax(i) = subplot(3,1,i); hold on;
        plot(vehicle_attitude_setpoint_t, d_sp{i}*r2d, 'k-', 'LineWidth', 1);
        plot(vehicle_attitude_t, d_res{i}*r2d, '--', 'LineWidth', 1, 'Color', [0.6, 0.2, 0]);
        grid on; ylabel(ylabels{i}, 'Interpreter', 'latex', 'FontSize', 12); title(titles{i}); 
        if i==1, legend('Setpoint','Response'); end
        add_standard_background(vis_flight_intervals, vis_flight_names, vis_is_vtol, vis_vtol_intervals, vis_vtol_names);
    end
    linkaxes(ax, 'x'); xlabel('Time (s)');
    % PlotToFile(gcf, 'results/att.png', 20, 20);
end

% --- Figure 3: Velocity ---
% if(exist('V_XYZ', 'var'))
%     figure('Name', 'Velocity', 'Color', 'w'); 
%     ylabels = {'V_X (m/s)', 'V_Y (m/s)', 'V_Z (m/s)'};
%     has_sp = exist('V_XYZ_setpoint', 'var');
%     ax = [];
%     for i = 1:3
%         ax(i) = subplot(3,1,i); hold on;
%         if has_sp, plot(vehicle_local_position_setpoint_t, V_XYZ_setpoint(:,i), 'k-', 'LineWidth', 1); end
%         plot(vehicle_local_position_t, V_XYZ(:,i), '--', 'LineWidth', 1, 'Color', [0.6, 0.2, 0]);
%         grid on; ylabel(ylabels{i}); if i==1, title('Velocity'); legend('Setpoint','Response'); end
%         add_standard_background(vis_flight_intervals, vis_flight_names, vis_is_vtol, vis_vtol_intervals, vis_vtol_names);
%     end
%     linkaxes(ax, 'x'); xlabel('Time (s)');
% end

% --- Figure 3: Velocity & TECS (Dynamic subplot count: 3 or 4) ---
if exist('V_XYZ', 'var')
    figure('Name', 'Velocity', 'Color', 'w');  

    % 1. Determine if 4th subplot is needed
    has_tecs = exist('tecs_h_rate', 'var');
    if has_tecs
        n_rows = 4; % Has TECS -> 4 rows
    else
        n_rows = 3; % No TECS -> 3 rows
    end

    ax = [];

    % --- Subplot 1: Velocity X ---
    ax(1) = subplot(n_rows, 1, 1); hold on;
    if exist('V_XYZ_setpoint', 'var'), plot(vehicle_local_position_setpoint_t, V_XYZ_setpoint(:,1), 'k-', 'LineWidth', 1); end
    plot(vehicle_local_position_t, V_XYZ(:,1), '--', 'LineWidth', 1, 'Color', [0.6, 0.2, 0]);
    grid on; ylabel('V_X (m/s)'); title('Velocity X');
    if exist('V_XYZ_setpoint', 'var'), legend('Setpoint', 'Response', 'Location', 'best'); end
    add_standard_background(vis_flight_intervals, vis_flight_names, vis_is_vtol, vis_vtol_intervals, vis_vtol_names);

    % --- Subplot 2: Velocity Y ---
    ax(2) = subplot(n_rows, 1, 2); hold on;
    if exist('V_XYZ_setpoint', 'var'), plot(vehicle_local_position_setpoint_t, V_XYZ_setpoint(:,2), 'k-', 'LineWidth', 1); end
    plot(vehicle_local_position_t, V_XYZ(:,2), '--', 'LineWidth', 1, 'Color', [0.6, 0.2, 0]);
    grid on; ylabel('V_Y (m/s)'); title('Velocity Y');
    add_standard_background(vis_flight_intervals, vis_flight_names, vis_is_vtol, vis_vtol_intervals, vis_vtol_names);

    % --- Subplot 3: Velocity Z ---
    ax(3) = subplot(n_rows, 1, 3); hold on;
    if exist('V_XYZ_setpoint', 'var'), plot(vehicle_local_position_setpoint_t, V_XYZ_setpoint(:,3), 'k-', 'LineWidth', 1); end
    plot(vehicle_local_position_t, V_XYZ(:,3), '--', 'LineWidth', 1, 'Color', [0.6, 0.2, 0]);
    grid on; ylabel('V_Z (m/s)'); title('Velocity Z');
    add_standard_background(vis_flight_intervals, vis_flight_names, vis_is_vtol, vis_vtol_intervals, vis_vtol_names);

    % --- Subplot 4: TECS Height Rate (Only plotted if exists) ---
    if has_tecs
        ax(4) = subplot(n_rows, 1, 4); hold on;
        plot(log.data.tecs_status_0.timestamp*1e-6, tecs_h_rate_sp, 'k-', 'LineWidth', 1);
        plot(log.data.tecs_status_0.timestamp*1e-6, tecs_h_rate, '--', 'LineWidth', 1, 'Color', [0.6, 0.2, 0]);
        grid on; ylabel('Rate (m/s)'); title('TECS Height Rate');
        legend('H Rate Sp', 'H Rate', 'Location', 'best');
        add_standard_background(vis_flight_intervals, vis_flight_names, vis_is_vtol, vis_vtol_intervals, vis_vtol_names);
    end

    linkaxes(ax, 'x'); xlabel('Time (s)');
    % PlotToFile(gcf, 'results/vel.png', 20, 20);
end

% --- Figure 4: Position ---
if(exist('XYZ', 'var'))
    figure('Name', 'Position', 'Color', 'w'); 
    ylabels = {'X (m)', 'Y (m)', 'Z (m)'};
    has_sp = exist('XYZ_setpoint', 'var');
    ax = [];
    for i = 1:3
        ax(i) = subplot(3,1,i); hold on;
        if has_sp, plot(vehicle_local_position_setpoint_t, XYZ_setpoint(:,i), 'k-', 'LineWidth', 1); end
        plot(vehicle_local_position_t, XYZ(:,i), '--', 'LineWidth', 1, 'Color', [0.6, 0.2, 0]);
        grid on; ylabel(ylabels{i}); if i==1, title('Position'); legend('Setpoint','Response'); end
        add_standard_background(vis_flight_intervals, vis_flight_names, vis_is_vtol, vis_vtol_intervals, vis_vtol_names);
    end
    linkaxes(ax, 'x'); xlabel('Time (s)');
    % PlotToFile(gcf, 'results/pos.png', 20, 20);
end

%% =========================================================================
%  Trajectory  
% =========================================================================
if(exist('XYZ', 'var') && exist('XYZ_setpoint', 'var'))
    figure('Name', 'Trajectory', 'Color', 'w');
    plot3(XYZ_setpoint(:,1), XYZ_setpoint(:,2), -XYZ_setpoint(:,3), '-', 'LineWidth', 1); hold on;
    plot3(XYZ(:,1), XYZ(:,2), -XYZ(:,3), ':', 'LineWidth', 1);
    title('Trajectory'); xlabel('X'); ylabel('Y'); zlabel('Z'); grid on; view(45, 30);
    % PlotToFile(gcf, 'results/traj.png', 15, 15);
end


if control_fig
    %% =========================================================================
    %  Actuator Controls (Two-column layout + independent time axes)
    % =========================================================================
    if ~isempty(actuator_controls_0.time)
        figure('Name', 'Actuator Controls', 'Color', 'w');  
    
        % --- Layout strategy ---
        has_group1 = ~isempty(actuator_controls_1.time);
        if has_group1
            n_cols = 2; layout_title = 'Actuator Controls (Left: Group 0, Right: Group 1)';
        else
            n_cols = 1; layout_title = 'Actuator Controls (Group 0)';
        end
    
        line_cols = {'r-', 'g-', 'b-'}; % Roll, Pitch, Yaw
        titles = {'All', 'Roll', 'Pitch', 'Yaw', 'Thrust'};
        ax_all = []; 
    
        for row = 1:5
            % ======================= Left Side: Group 0 (Main) =======================
            idx_left = (row - 1) * n_cols + 1;
            ax = subplot(5, n_cols, idx_left); 
            ax_all = [ax_all, ax]; hold on;
    
            if row == 1 % Summary plot
                plot(actuator_controls_0.time, actuator_controls_0.roll, line_cols{1}, 'DisplayName', 'Roll');
                plot(actuator_controls_0.time, actuator_controls_0.pitch, line_cols{2}, 'DisplayName', 'Pitch');
                plot(actuator_controls_0.time, actuator_controls_0.yaw, line_cols{3}, 'DisplayName', 'Yaw');
    
                % Thrust plotted using actuator_controls_0.time_thrust
                if ~isempty(actuator_controls_0.thrust_z_neg)
                    plot(actuator_controls_0.time_thrust, actuator_controls_0.thrust_z_neg, 'k-', 'DisplayName', 'Thrust (up)');
                end
                if ~isempty(actuator_controls_0.thrust_x) && any(actuator_controls_0.thrust_x ~= 0)
                    plot(actuator_controls_0.time_thrust, actuator_controls_0.thrust_x, 'k--', 'DisplayName', 'Thrust (fwd)');
                end
                title('Group 0 (Main)'); 
                if n_cols == 1, legend('Location', 'best', 'NumColumns', 5); end
    
            elseif row >= 2 && row <= 4 % R, P, Y breakdown (using torque time actuator_controls_0.time)
                data_map = {actuator_controls_0.roll, actuator_controls_0.pitch, actuator_controls_0.yaw};
                plot(actuator_controls_0.time, data_map{row-1}, line_cols{row-1});
                ylabel(titles{row});
    
            elseif row == 5 % Thrust breakdown (using thrust time actuator_controls_0.time_thrust)
                if ~isempty(actuator_controls_0.thrust_z_neg)
                    plot(actuator_controls_0.time_thrust, actuator_controls_0.thrust_z_neg, 'k-', 'DisplayName', 'Up');
                end
                if ~isempty(actuator_controls_0.thrust_x)
                    plot(actuator_controls_0.time_thrust, actuator_controls_0.thrust_x, 'k--', 'DisplayName', 'Fwd');
                end
                ylabel('Thrust'); legend('show', 'Location', 'best');
            end
            grid on;
            add_standard_background(vis_flight_intervals, vis_flight_names, vis_is_vtol, vis_vtol_intervals, vis_vtol_names);
    
            % ======================= Right Side: Group 1 (Aux/FW) =======================
            if has_group1
                idx_right = (row - 1) * n_cols + 2;
                ax = subplot(5, n_cols, idx_right); 
                ax_all = [ax_all, ax]; hold on;
    
                if row == 1 % Summary plot
                    plot(actuator_controls_1.time, actuator_controls_1.roll, line_cols{1}, 'DisplayName', 'Roll');
                    plot(actuator_controls_1.time, actuator_controls_1.pitch, line_cols{2}, 'DisplayName', 'Pitch');
                    plot(actuator_controls_1.time, actuator_controls_1.yaw, line_cols{3}, 'DisplayName', 'Yaw');
                    if ~isempty(actuator_controls_1.thrust_x)
                        plot(actuator_controls_1.time, actuator_controls_1.thrust_x, 'k--', 'DisplayName', 'Thrust (fwd)');
                    end
                    title('Group 1 (Aux/FW)');
    
                elseif row >= 2 && row <= 4
                    data_map = {actuator_controls_1.roll, actuator_controls_1.pitch, actuator_controls_1.yaw};
                    plot(actuator_controls_1.time, data_map{row-1}, line_cols{row-1});
    
                elseif row == 5
                    if ~isempty(actuator_controls_1.thrust_x)
                        plot(actuator_controls_1.time, actuator_controls_1.thrust_x, 'k--', 'DisplayName', 'Fwd');
                    end
                end
                grid on;
                add_standard_background(vis_flight_intervals, vis_flight_names, vis_is_vtol, vis_vtol_intervals, vis_vtol_names);
            end
        end
    
        linkaxes(ax_all, 'x');
        xlabel(ax_all(end), 'Time (s)');
        sgtitle(layout_title);
        % PlotToFile(gcf, 'results/control.png', 20, 20);
    end
    
    %% =========================================================================
    %  Figure 6/7/8: Actuators & PWM 
    % =========================================================================
    if dynamic_control_alloc
    
        if (plot_together)
            %% Dynamically plot motors and servos (Merged display version - optimized colors)
            % Check if relevant data tables exist
            if (isfield(log.data, 'actuator_motors_0') || isfield(log.data, 'actuator_servos_0'))
    
                % 1. Get motor and servo counts
                n_motors = 0;
                if isfield(log, 'params') && isfield(log.params, 'CA_ROTOR_COUNT')
                    n_motors = double(log.params.CA_ROTOR_COUNT);
                else
                    if exist('motors', 'var'), n_motors = size(motors, 2); end
                end
    
                n_servos = 0;
                if isfield(log, 'params') && isfield(log.params, 'CA_SV_CS_COUNT')
                    n_servos = double(log.params.CA_SV_CS_COUNT);
                else
                    if exist('servos', 'var'), n_servos = size(servos, 2); end
                end
    
                n_motors = min(n_motors, MAX_MOTORS);
                n_servos = min(n_servos, MAX_SERVOS);
    
                has_motor_data = (n_motors > 0) && isfield(log.data, 'actuator_motors_0') && exist('motors', 'var');
                has_servo_data = (n_servos > 0) && isfield(log.data, 'actuator_servos_0') && exist('servos', 'var');
    
                total_subplots = double(has_motor_data) + double(has_servo_data);
    
                if total_subplots > 0
                    figure('Name', 'Actuator Outputs (Merged)', 'Color', 'w');
                    current_plot_idx = 1;
    
                    % Part 1: Plot motors (all motors in one figure)
                    if has_motor_data
                        ax_m = subplot(total_subplots, 1, current_plot_idx);
                        hold on;
                        colors_motor = hsv(n_motors); 
                        t_m = log.data.actuator_motors_0.timestamp*1e-6;
                        for i = 1:n_motors
                            if i <= size(motors, 2)
                                plot(t_m, motors(:, i), 'Color', colors_motor(i,:), 'LineWidth', 1, 'DisplayName', sprintf('Motor %d', i));
                            end
                        end
                        grid on; ylabel('Motors Output'); title(sprintf('Actuator: Motors (Total %d)', n_motors));
                        legend('show'); 
                        add_standard_background(vis_flight_intervals, vis_flight_names, vis_is_vtol, vis_vtol_intervals, vis_vtol_names);
    
                        if has_servo_data, set(gca, 'XTickLabel', []); else, xlabel('Time (s)'); end
                        current_plot_idx = current_plot_idx + 1;
                    end
    
                    % Part 2: Plot servos (all servos in one figure)
                    if has_servo_data
                        ax_s = subplot(total_subplots, 1, current_plot_idx);
                        hold on;
                        if n_servos <= 7, colors_servo = lines(n_servos); else, colors_servo = hsv(n_servos); end
                        t_s = log.data.actuator_servos_0.timestamp*1e-6;
                        for i = 1:n_servos
                            if i <= size(servos, 2)
                                plot(t_s, servos(:, i), 'Color', colors_servo(i,:), 'LineWidth', 1, 'DisplayName', sprintf('Servo %d', i));
                            end
                        end
                        grid on; ylabel('Servos Output'); title(sprintf('Actuator: Servos (Total %d)', n_servos));
                        legend('show'); xlabel('Time (s)');
                        add_standard_background(vis_flight_intervals, vis_flight_names, vis_is_vtol, vis_vtol_intervals, vis_vtol_names);
                    end
    
                    % Link axes if both exist
                    if has_motor_data && has_servo_data
                        linkaxes([ax_m, ax_s], 'x');
                        % PlotToFile(gcf, 'results/motors_servos.png', 20, 20);
                    end
                end
            end
        else
            % Standalone display mode  
            %% === Figure 6: Motor Output (Actuator Motors) ===
            if (isfield(log.data, 'actuator_motors_0') && exist('motors', 'var'))
    
                % 1. Get motor count
                n_motors = 0;
                if isfield(log, 'params') && isfield(log.params, 'CA_ROTOR_COUNT')
                    n_motors = double(log.params.CA_ROTOR_COUNT);
                else
                    n_motors = size(motors, 2); 
                end
    
                % Limit max count to prevent errors
                n_motors = min(n_motors, MAX_MOTORS);
    
                % 2. Plotting
                if n_motors > 0
                    figure;
                    set(gcf, 'Name', 'Actuator Motors', 'Color', 'w');
    
                    % === Key point: Generate n_motors distinct colors ===
                    % Use hsv colorspace for uniform sampling between 0-1, ensuring distinct colors
                    % Can also try 'turbo' or 'jet'
                    motor_colors = hsv(n_motors); 
    
                    for i = 1:n_motors
                        subplot(n_motors, 1, i);
                        hold on;
    
                        if i <= size(motors, 2)
                            % Extract color i
                            this_color = motor_colors(i, :);
    
                            plot(log.data.actuator_motors_0.timestamp*1e-6, motors(:, i), ...
                                 'Color', this_color, 'LineWidth', 1);
                        end
    
                        grid on;
                        ylabel(sprintf('Motor %d', i));
                        add_standard_background(vis_flight_intervals, vis_flight_names, vis_is_vtol, vis_vtol_intervals, vis_vtol_names);
                        % Only first plot shows title
                        if i == 1
                            title(sprintf('Actuator Motors (Total: %d)', n_motors));
                        end
    
                        % Only last plot shows time axis
                        if i == n_motors
                            xlabel('Time (s)');
                        else
                            set(gca, 'XTickLabel', []);
                        end
    
                        % Adjust axis range for clearer waveform (optional)
                        % axis tight; 
                    end
    
                    % Auto-adjust layout (if Matlab version supports it)
                    % sgtitle('Motors Output');
                    % PlotToFile(gcf, 'results/motors.png', 20, 20);
                end
            end
    
            %% === Figure 7: Servo Output (Actuator Servos) ===
            if (isfield(log.data, 'actuator_servos_0') && exist('servos', 'var'))
    
                % 1. Get servo count
                n_servos = 0;
                if isfield(log, 'params') && isfield(log.params, 'CA_SV_CS_COUNT')
                    n_servos = double(log.params.CA_SV_CS_COUNT);
                else
                    n_servos = size(servos, 2);
                end
    
                % Limit max count
                n_servos = min(n_servos, MAX_SERVOS);
    
                % 2. Plotting
                if n_servos > 0
                    figure;
                    set(gcf, 'Name', 'Actuator Servos', 'Color', 'w');
    
                    % === Key point: Generate n_servos distinct colors ===
                    % To differentiate from motor colors, we can offset HSV phase or reverse
                    % Here use 'lines' colormap which has high contrast, suitable for <=8 items
                    if n_servos <= 7
                        servo_colors = lines(n_servos);
                    else
                        servo_colors = hsv(n_servos); % Use hsv for >7 to avoid duplicates
                    end
    
                    for i = 1:n_servos
                        subplot(n_servos, 1, i);
                        hold on;
    
                        if i <= size(servos, 2)
                            this_color = servo_colors(i, :);
    
                            plot(log.data.actuator_servos_0.timestamp*1e-6, servos(:, i), ...
                                 'Color', this_color, 'LineWidth', 1);
                        end
    
                        grid on;
                        ylabel(sprintf('Servo %d', i));
                        add_standard_background(vis_flight_intervals, vis_flight_names, vis_is_vtol, vis_vtol_intervals, vis_vtol_names);
                        if i == 1
                            title(sprintf('Actuator Servos (Total: %d)', n_servos));
                        end
    
                        if i == n_servos
                            xlabel('Time (s)');
                        else
                            set(gca, 'XTickLabel', []);
                        end
                    end
                    % PlotToFile(gcf, 'results/servos.png', 20, 20);
                end
            end
        end
        % -------------------------------------------------------------------------
        % 2. PWM Outputs (Figure 8) - Based on active_channels
        % -------------------------------------------------------------------------
        if exist('active_channels', 'var') && ~isempty(active_channels)
            figure; set(gcf, 'Name', 'Actuator Outputs (PWM)', 'Color', 'w');
    
            % Split Motor vs Servo
            is_motor = strcmp({active_channels.type}, 'Motor');
            idx_m = find(is_motor);
            idx_s = find(~is_motor);
    
            n_plots = double(~isempty(idx_m)) + double(~isempty(idx_s));
            cur = 1; ax_list = [];
    
            % Subplot 1: Motors
            if ~isempty(idx_m)
                ax = subplot(n_plots, 1, cur); hold on;
                cols = hsv(length(idx_m));
                for k = 1:length(idx_m)
                    info = active_channels(idx_m(k));
                    data = log.data.actuator_outputs_0.(info.col_name);
                    plot(outputs_t, data, 'Color', cols(k,:), 'LineWidth', 1, 'DisplayName', sprintf('%s (Ch%d)', info.name, info.idx));
                end
                grid on; ylabel('PWM (us)'); title('Motors PWM'); legend('show');
                add_standard_background(vis_flight_intervals, vis_flight_names, vis_is_vtol, vis_vtol_intervals, vis_vtol_names);
                ax_list = [ax_list, ax]; cur = cur + 1;
            end
    
            % Subplot 2: Others
            if ~isempty(idx_s)
                ax = subplot(n_plots, 1, cur); hold on;
                cols = lines(length(idx_s));
                for k = 1:length(idx_s)
                    info = active_channels(idx_s(k));
                    data = log.data.actuator_outputs_0.(info.col_name);
                    plot(outputs_t, data, 'Color', cols(k,:), 'LineWidth', 1, 'DisplayName', sprintf('%s (Ch%d)', info.name, info.idx));
                end
                grid on; ylabel('PWM (us)'); title('Servos/Other PWM'); legend('show');
                add_standard_background(vis_flight_intervals, vis_flight_names, vis_is_vtol, vis_vtol_intervals, vis_vtol_names);
                ax_list = [ax_list, ax];
            end
            if ~isempty(ax_list), linkaxes(ax_list, 'x'); end; xlabel('Time (s)');
        else
            fprintf('Figure 8 Skipped: No active PWM channels identified.\n');
        end
        % PlotToFile(gcf, 'results/PWM.png', 20, 20);
    else
        %% === Mode B: Original plotting (Fallback) ===
        % Logic: When active_channels parsing fails, directly plot first N columns of legacy_pwm
    
        figure;
        set(gcf, 'Name', 'Actuator Outputs (Raw)', 'Color', 'w');
    
        ax_raw = [];
        raw_colors = lines(n_raw_plot);
        for i = 1:n_raw_plot
            ax_raw(i) = subplot(n_raw_plot, 1, i); hold on;
    
            y_data = outputs(:, i);
    
            % Simple filtering: Remove all-NaN or all-zero data to avoid empty lines
            if ~all(isnan(y_data)) && any(y_data ~= 0)
                plot(outputs_t, y_data, 'Color', raw_colors(i, :), 'LineWidth', 1);
            end
    
            ylabel(sprintf('Out %d', i-1)); 
            grid on;
    
            if i == 1, title(sprintf('Actuator Outputs (First %d Raw)', n_raw_plot)); end
            if i < n_raw_plot, set(gca, 'XTickLabel', []); else, xlabel('Time (s)'); end
    
            axis tight;
            add_standard_background(vis_flight_intervals, vis_flight_names, vis_is_vtol, vis_vtol_intervals, vis_vtol_names);
        end
    
        linkaxes(ax_raw, 'x');
        % PlotToFile(gcf, 'results/pwm.png', 20, 20);
    end

end



%%
if verbose

    
    %% =========================================================================
    %   Angular Accel
    % =========================================================================
    if exist('vehicle_angular_acceleration', 'var') || isfield(log.data, 'vehicle_angular_velocity_0') && ...
           ismember('xyz_derivative_0_', log.data.vehicle_angular_velocity_0.Properties.VariableNames)
        figure; set(gcf, 'Name', 'Ang Acc vs rates', 'Color', 'w');
        titles = {'Roll Ang Acc', 'Pitch Ang Acc', 'Yaw Ang Acc'};
        ax = [];
        for i = 1:3
            ax(i) = subplot(3,1,i); hold on;
            plot(vehicle_angular_acceleration_t, vehicle_angular_acceleration(:,i), '--', 'LineWidth', 0.5);
            plot(vehicle_angular_velocity_t, vehicle_angular_velocity(:,i), '-', 'LineWidth', 1, 'Color', [0.6, 0.2, 0, 0.5]);
            grid on; ylabel('rad/s^2'); title(titles{i});
            add_standard_background(vis_flight_intervals, vis_flight_names, vis_is_vtol, vis_vtol_intervals, vis_vtol_names);
        end
        linkaxes(ax, 'x'); xlabel('Time (s)');
        % PlotToFile(gcf, 'results/Angular_Accel.png', 20, 10);
    end
    %% =========================================================================
    %  Manual Control Inputs 
    %  Corresponds to Python: Manual Control Inputs (Radio or Joystick)
    % =========================================================================
    if isfield(log.data, 'manual_control_setpoint_0')
        figure; 
        set(gcf, 'Name', 'Manual Control Inputs', 'Color', 'w');
        hold on;
        plot(rc_t, rc_roll,  'LineWidth', 1.5, 'DisplayName', 'Roll');
        plot(rc_t, rc_pitch, 'LineWidth', 1.5, 'DisplayName', 'Pitch');
        plot(rc_t, rc_yaw,   'LineWidth', 1.5, 'DisplayName', 'Yaw');
        plot(rc_t, rc_throttle,   'LineWidth', 1.5, 'DisplayName', 'Throttle');
    
        grid on; 
        legend('show', 'Location', 'best', 'NumColumns', 4);
    
        ylabel('Norm Input [-1, 1]'); 
        title('Manual Control Inputs (Sticks)');
        xlabel('Time (s)');
        ylim([-1.1, 1.1]); % Fix Y-axis range for clearer view
    
        % Add background
        add_standard_background(vis_flight_intervals, vis_flight_names, vis_is_vtol, vis_vtol_intervals, vis_vtol_names);
        % PlotToFile(gcf, 'results/Manual_input.png', 20, 10);
    end

    
    %% =========================================================================
    %  Frequency Analysis (FFT) - Figure 12/13/14
    %  Function: Perform frequency domain analysis on control, angular velocity, angular acceleration
    %  and automatically mark filter parameters
    % =========================================================================
    % --- 1. Actuator Controls FFT (corresponds to vehicle_torque_setpoint) ---
    if exist('actuator_controls_0', 'var')
        figure; 
        set(gcf, 'Color', 'w', 'Name', 'Actuator Controls FFT');
        % Prepare data: [Roll, Pitch, Yaw]
        ctrl_data = [actuator_controls_0.roll, actuator_controls_0.pitch, actuator_controls_0.yaw];
        % Define parameters to mark (parameter name, display label)
        markers = {
            'MC_DTERM_CUTOFF',  'D-Term Cutoff';
            'IMU_DGYRO_CUTOFF', 'D-Gyro Cutoff';
            'IMU_GYRO_CUTOFF',  'Gyro Cutoff'
        };
    
        draw_fft_analysis(actuator_controls_0.time, ctrl_data, {'Roll', 'Pitch', 'Yaw'}, ...
                          'Actuator Controls FFT (Torque Setpoint)', log.params, markers);
        % PlotToFile(gcf, 'results/control_fft.png', 20, 10);
    end
    
    % --- 2. Angular Velocity FFT (corresponds to vehicle_angular_velocity) ---
    if exist('vehicle_angular_velocity', 'var')
        figure; 
        set(gcf, 'Color', 'w', 'Name', 'Angular Velocity FFT');
        markers = {
            'IMU_GYRO_CUTOFF',   'Gyro Cutoff';
            'IMU_GYRO_NF_FREQ',  'Notch Freq'
        };
    
        draw_fft_analysis(vehicle_angular_velocity_t, vehicle_angular_velocity, {'Rollrate', 'Pitchrate', 'Yawrate'}, ...
                          'Angular Velocity FFT', log.params, markers);
        % PlotToFile(gcf, 'results/rate_fft.png', 20, 10);
    end
    
    % --- 3. Angular Acceleration FFT (corresponds to vehicle_angular_acceleration) ---
    if exist('vehicle_angular_acceleration', 'var')
        figure; 
        set(gcf, 'Color', 'w', 'Name', 'Angular Acceleration FFT');
        markers = {
            'IMU_DGYRO_CUTOFF',  'D-Gyro Cutoff';
            'IMU_GYRO_NF_FREQ',  'Notch Freq'
        };
    
        draw_fft_analysis(vehicle_angular_acceleration_t, vehicle_angular_acceleration, {'Roll Acc', 'Pitch Acc', 'Yaw Acc'}, ...
                          'Angular Acceleration FFT', log.params, markers);
        % PlotToFile(gcf, 'results/acc_fft.png', 20, 10);
    end
    
    %% =========================================================================
    %  Raw Acceleration
    %  Corresponds to Python: Raw Acceleration (sensor_combined)
    % =========================================================================
    if exist('raw_acc', 'var')
        figure;
        set(gcf, 'Name', 'Raw Acceleration', 'Color', 'w');
    
        hold on;
        % Use a thinner line width (0.5) since raw sensor data is typically noisy and dense
        plot(raw_acc_t, raw_acc(:,1), 'r-', 'LineWidth', 0.5, 'DisplayName', 'Acc X');
        plot(raw_acc_t, raw_acc(:,2), 'k-', 'LineWidth', 0.5, 'DisplayName', 'Acc Y');
        plot(raw_acc_t, raw_acc(:,3), 'b-', 'LineWidth', 0.5, 'DisplayName', 'Acc Z');
    
        grid on; 
        legend('show', 'Location', 'best');
        ylabel('Acceleration [m/s^2]'); 
        title('Raw Acceleration');
        xlabel('Time (s)');
    
        % Add background
        add_standard_background(vis_flight_intervals, vis_flight_names, vis_is_vtol, vis_vtol_intervals, vis_vtol_names);
        % PlotToFile(gcf, 'results/raw_acc.png', 20, 10);
    end
    
    %% =========================================================================
    %  Vibration Metrics (Curve-only version)
    %  Logic: Only plot vibration curves, no threshold background shading
    % =========================================================================
    if exist('vib_data', 'var') && ~isempty(vib_data)
        figure;
        set(gcf, 'Name', 'Vibration Metrics', 'Color', 'w');
        hold on;
    
        % --- Plot data curves ---
        line_colors = {[0.8, 0.4, 0], [0, 0.4, 0.8], [0.8, 0, 0.8], [0, 0.5, 0]}; 
        for k = 1:length(vib_data)
            id = vib_data(k).id;
            c_idx = mod(k-1, length(line_colors)) + 1;
            plot(vib_data(k).t, vib_data(k).val, ...
                 'Color', line_colors{c_idx}, 'LineWidth', 1.5, ...
                 'DisplayName', sprintf('Accel %d Vib [m/s^2]', id));
        end
    
        % --- Basic decoration ---
        grid on;
        ylabel('Vibration Level [m/s^2]');
        title('Vibration Metrics');
        xlabel('Time (s)');
        legend('show', 'Location', 'best');
    
        % Overlay unified flight mode background (usually kept for seeing which phase has high vibration)
        add_standard_background(vis_flight_intervals, vis_flight_names, vis_is_vtol, vis_vtol_intervals, vis_vtol_names);
    
        set(gca, 'Layer', 'top');
        % PlotToFile(gcf, 'results/Vibration_Metrics.png', 20, 10);
    end
    
    %% =========================================================================
    %  Spectrogram (PSD over Time)
    %  Corresponds to Python: Acceleration / Gyro / AngAcc Spectrogram
    %  Principle: Calculate total power spectral density of X+Y+Z (dB)
    % =========================================================================
    
    % --- 1. Acceleration Spectrogram (corresponds to sensor_combined) ---
    if exist('raw_acc', 'var') && ~isempty(raw_acc)
        figure;
        set(gcf, 'Name', 'Accel PSD', 'Color', 'w');
        draw_spec_analysis(raw_acc_t, raw_acc, 'Acceleration Power Spectral Density (Sum X+Y+Z)');
        % PlotToFile(gcf, 'results/Acceleration_Spectrogram.png', 20, 10);
    end
    
    % --- 2. Filtered Gyro Spectrogram (corresponds to vehicle_angular_velocity) ---
    if exist('vehicle_angular_velocity', 'var')
        figure;
        set(gcf, 'Name', 'Gyro PSD', 'Color', 'w');
        draw_spec_analysis(vehicle_angular_velocity_t, vehicle_angular_velocity, 'Angular Velocity PSD (Sum X+Y+Z)');
        % PlotToFile(gcf, 'results/Gyro_Spectrogram.png', 20, 10);
    end
    
    % --- 3. Filtered Angular Acceleration Spectrogram (corresponds to vehicle_angular_acceleration) ---
    if exist('vehicle_angular_acceleration', 'var')
        figure;
        set(gcf, 'Name', 'AngAcc PSD', 'Color', 'w');
        draw_spec_analysis(vehicle_angular_acceleration_t, vehicle_angular_acceleration, 'Angular Acceleration PSD (Sum X+Y+Z)');
        % PlotToFile(gcf, 'results/AngAcc_Spectrogram.png', 20, 10);
    end
    %% =========================================================================
    %  Raw Angular Speed (Gyroscope)
    %  Corresponds to Python: Raw Angular Speed (sensor_combined)
    % =========================================================================
    if exist('raw_gyro', 'var')
        figure;
        set(gcf, 'Name', 'Raw Angular Speed', 'Color', 'w');
        hold on;
        plot(raw_gyro_t, raw_gyro(:,1), 'r-', 'LineWidth', 0.5, 'DisplayName', 'X');
        plot(raw_gyro_t, raw_gyro(:,2), 'k-', 'LineWidth', 0.5, 'DisplayName', 'Y');
        plot(raw_gyro_t, raw_gyro(:,3), 'b-', 'LineWidth', 0.5, 'DisplayName', 'Z');
    
        grid on; legend('show');
        ylabel('Angular Speed [deg/s]'); title('Raw Angular Speed (Gyroscope)');
        xlabel('Time (s)');
        add_standard_background(vis_flight_intervals, vis_flight_names, vis_is_vtol, vis_vtol_intervals, vis_vtol_names);
        % PlotToFile(gcf, 'results/raw_gyro.png', 20, 10);
    end

    % ToDO:
    % %% =========================================================================
    % %  FIFO Acceleration Analysis
    % %  Corresponds to Python: FIFO accel (Raw, PSD, Sampling Regularity)
    % % =========================================================================
    % if exist('fifo_acc', 'var')
    %     for k = 1:length(fifo_acc)
    %         id = fifo_acc(k).id;
    %         t = fifo_acc(k).t;
    %         d = fifo_acc(k).d;
    % 
    %         % 1. Raw Acceleration (FIFO)
    %         f_num = 20 + k*3 - 2; % Dynamic figure numbering: 21, 24, 27...
    %         figure(f_num); set(gcf, 'Color', 'w', 'Name', sprintf('FIFO Accel %d Raw', id));
    %         hold on;
    %         plot(t, d(:,1), 'r-', 'LineWidth', 0.1);
    %         plot(t, d(:,2), 'k-', 'LineWidth', 0.1);
    %         plot(t, d(:,3), 'b-', 'LineWidth', 0.1);
    %         title(sprintf('Raw Acceleration (FIFO, IMU%d)', id));
    %         ylabel('[m/s^2]'); grid on; legend('X','Y','Z');
    %         add_standard_background(vis_flight_intervals, vis_flight_names, vis_is_vtol, vis_vtol_intervals, vis_vtol_names);
    % 
    %         % 2. PSD (Spectrogram)
    %         % Call previous draw_spec_analysis
    %         % Note: pass microsecond timestamps or let the function handle seconds
    %         figure(f_num+1); set(gcf, 'Color', 'w', 'Name', sprintf('FIFO Accel %d PSD', id));
    %         draw_spec_analysis(t, d, sprintf('Acceleration PSD (FIFO, IMU%d)', id));
    % 
    %         % 3. Sampling Regularity
    %         % Python logic: diff(raw_message_timestamps)
    %         % Check packet arrival intervals rather than sample intervals
    %         figure(f_num+2); set(gcf, 'Color', 'w', 'Name', sprintf('FIFO Accel %d Sampling', id));
    %         if length(fifo_acc(k).raw_t) > 1
    %             dt_diff = diff(fifo_acc(k).raw_t) * 1e6; % Convert to microseconds
    %             plot(fifo_acc(k).raw_t(2:end), dt_diff, 'b.-');
    %             ylabel('Delta t [us]'); title(sprintf('Sampling Regularity (FIFO, IMU%d)', id));
    %             xlabel('Time (s)'); grid on;
    % 
    %             % Mark dropouts (Python logic: plot_dropouts)
    %             % Draw a reference line, e.g., 2000us (500Hz) or 1000us (1kHz)
    %             avg_dt = median(dt_diff);
    %             yline(avg_dt, 'r--', sprintf('Median: %.0f us', avg_dt));
    %         end
    %         add_standard_background(vis_flight_intervals, vis_flight_names, vis_is_vtol, vis_vtol_intervals, vis_vtol_names);
    %     end
    % end
    % 
    % %% =========================================================================
    % %  FIFO Gyro Analysis
    % %  Corresponds to Python: FIFO gyro (Raw, PSD)
    % % =========================================================================
    % if exist('fifo_gyro', 'var')
    %     % To avoid figure number conflicts above, start from 40
    %     start_fig = 40;
    %     for k = 1:length(fifo_gyro)
    %         id = fifo_gyro(k).id;
    %         t = fifo_gyro(k).t;
    %         d = fifo_gyro(k).d; % Already in deg/s
    % 
    %         % 1. Raw Gyro (FIFO)
    %         f_num = start_fig + (k-1)*2;
    %         figure(f_num); set(gcf, 'Color', 'w', 'Name', sprintf('FIFO Gyro %d Raw', id));
    %         hold on;
    %         plot(t, d(:,1), 'r-', 'LineWidth', 0.1);
    %         plot(t, d(:,2), 'k-', 'LineWidth', 0.1);
    %         plot(t, d(:,3), 'b-', 'LineWidth', 0.1);
    %         title(sprintf('Raw Gyro (FIFO, IMU%d)', id));
    %         ylabel('[deg/s]'); grid on; legend('X','Y','Z');
    %         add_standard_background(vis_flight_intervals, vis_flight_names, vis_is_vtol, vis_vtol_intervals, vis_vtol_names);
    % 
    %         % 2. PSD
    %         figure(f_num+1); set(gcf, 'Color', 'w', 'Name', sprintf('FIFO Gyro %d PSD', id));
    %         % PSD: use original units (rad/s) or deg/s? Python computes PSD without unit conversion,
    %         % and vibration metrics usually care about frequency distribution, so consistent units are fine.
    %         draw_spec_analysis(t, d, sprintf('Gyro PSD (FIFO, IMU%d)', id));
    %     end
    % end
    
    
    
    %% =========================================================================
    %  Raw Magnetic Field Strength
    %  Corresponds to Python: magnetometer_ga_topic
    % =========================================================================
    if exist('mag_data', 'var')
        figure; 
        set(gcf, 'Name', 'Raw Magnetic Field', 'Color', 'w');
        hold on;
        plot(mag_t, mag_data(:,1), 'r-', 'LineWidth', 1, 'DisplayName', 'X');
        plot(mag_t, mag_data(:,2), 'k-', 'LineWidth', 1, 'DisplayName', 'Y');
        plot(mag_t, mag_data(:,3), 'b-', 'LineWidth', 1, 'DisplayName', 'Z');
    
        grid on; legend('show');
        ylabel('Magnetic Field [Gauss]'); title('Raw Magnetic Field Strength');
        xlabel('Time (s)');
        add_standard_background(vis_flight_intervals, vis_flight_names, vis_is_vtol, vis_vtol_intervals, vis_vtol_names);
        % PlotToFile(gcf, 'results/Raw_Magnetic.png', 20, 10);
    end
    
    %% =========================================================================
    %  Distance Sensor
    %  Corresponds to Python: distance_sensor
    % =========================================================================
    if exist('dist_val', 'var') || exist('dist_bottom', 'var')
        figure; 
        set(gcf, 'Name', 'Distance Sensor', 'Color', 'w');
        hold on;
    
        % Sensor measured values
        if exist('dist_val', 'var')
            plot(dist_sensor_t, dist_val, 'b-', 'LineWidth', 1, 'DisplayName', 'Distance');
            % Variance is usually small or may need dual-axis display; plot together for reference
            % plot(dist_sensor_t, dist_var, 'r:', 'DisplayName', 'Variance'); 
        end
    
        % Estimated value (Dist Bottom)
        if exist('dist_bottom', 'var')
            plot(dist_bottom_t, dist_bottom, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Est. Dist Bottom');
            % Optionally plot the valid flag, or keep as reference only
        end
    
        grid on; legend('show');
        ylabel('Distance [m]'); title('Distance Sensor');
        xlabel('Time (s)');
        add_standard_background(vis_flight_intervals, vis_flight_names, vis_is_vtol, vis_vtol_intervals, vis_vtol_names);
        % PlotToFile(gcf, 'results/distance_sensor.png', 20, 10);
    end
    
    %% =========================================================================
    %  GPS Uncertainty
    %  Corresponds to Python: GPS Uncertainty
    % =========================================================================
    if exist('gps_info', 'var')
        figure; 
        set(gcf, 'Name', 'GPS Uncertainty', 'Color', 'w');
        hold on;
    
        % Plot each accuracy metric
        plot(gps_t, gps_info.eph, 'r-', 'LineWidth', 1, 'DisplayName', 'H Pos Accuracy (EPH) [m]');
        plot(gps_t, gps_info.epv, 'b-', 'LineWidth', 1, 'DisplayName', 'V Pos Accuracy (EPV) [m]');
    
        if ismember('hdop', gps_info.Properties.VariableNames)
            plot(gps_t, gps_info.hdop, 'r--', 'LineWidth', 0.5, 'DisplayName', 'HDOP [m]');
            plot(gps_t, gps_info.vdop, 'b--', 'LineWidth', 0.5, 'DisplayName', 'VDOP [m]');
        end
    
        plot(gps_t, gps_info.s_variance, 'k:', 'LineWidth', 1, 'DisplayName', 'Speed Accuracy [m/s]');
    
        % Satellites and Fix Type usually have different scales; could use dual-axis, or plot only above.
        % Here scale satellites by 2 or plot separately.
        % Following Python logic, plot in the same figure and limit Y range to [0, 40]
        plot(gps_t, gps_info.satellites, 'm-', 'LineWidth', 1.5, 'DisplayName', 'Satellites Used');
        plot(gps_t, gps_info.fix_type, 'g-', 'LineWidth', 1.5, 'DisplayName', 'Fix Type (3=3D, 4=DGPS)');
    
        grid on; legend('show', 'Location', 'best', 'NumColumns', 2);
        ylabel('Value'); title('GPS Uncertainty & Status');
        xlabel('Time (s)');
        ylim([0, 40]); % Limit range to avoid huge variance before fix ruining the view
        add_standard_background(vis_flight_intervals, vis_flight_names, vis_is_vtol, vis_vtol_intervals, vis_vtol_names);
        % PlotToFile(gcf, 'results/GPS_Uncertainty.png', 20, 10);
    end
    
    %% =========================================================================
    %  GPS Noise & Jamming
    %  Corresponds to Python: GPS Noise & Jamming
    % =========================================================================
    if exist('gps_info', 'var')
        figure; 
        set(gcf, 'Name', 'GPS Noise & Jamming', 'Color', 'w');
        hold on;
    
        plot(gps_t, gps_info.noise, 'b-', 'LineWidth', 1, 'DisplayName', 'Noise per ms');
        plot(gps_t, gps_info.jamming, 'r-', 'LineWidth', 1, 'DisplayName', 'Jamming Indicator');
    
        grid on; legend('show');
        ylabel('Value'); title('GPS Noise & Jamming');
        xlabel('Time (s)');
        add_standard_background(vis_flight_intervals, vis_flight_names, vis_is_vtol, vis_vtol_intervals, vis_vtol_names);
        % PlotToFile(gcf, 'results/GPS_Noise_Jamming.png', 20, 10);
    end
    %% =========================================================================
    %  Thrust and Magnetic Field
    %  Corresponds to Python: Thrust and Magnetic Field
    %  Purpose: Check if magnetic field is disturbed under high thrust (Mag Norm should remain constant)
    % =========================================================================
    figure;
    set(gcf, 'Name', 'Thrust and Magnetic Field', 'Color', 'w');
    
    % Plot the magnetic field norm (magnitude)
    mag_mag = sqrt(mag_data(:,1).^2 + mag_data(:,2).^2 + mag_data(:,3).^2);
    plot(mag_t,mag_mag, 'r'); % Example: red color for the magnetic field
    
    hold on;
    
    % Plot thrust if available
    if ~isempty(actuator_controls_0.thrust)
        plot(actuator_controls_0.time_thrust, actuator_controls_0.thrust, 'g'); % Example: green color for thrust
    end
    
    % If VTOL and not dynamic control allocation, plot fixed-wing thrust
    if log.data.vehicle_status_0.is_vtol(1) && ~dynamic_control_alloc && ~isempty(thrust_sp_1)
        plot(actuator_controls_1.time, actuator_controls_1.thrust_x, 'b'); % Example: blue color for fixed-wing thrust
    end
    
    % Plot background for flight modes if available
    add_standard_background(vis_flight_intervals, vis_flight_names, vis_is_vtol, vis_vtol_intervals, vis_vtol_names);
    
    % Finalize the plot
    hold off;
    grid on;
    xlabel('Time (s)');
    ylabel('Magnitude');title('Thrust and Magnetic Field');
    legend({'Magnetic Field Norm', 'Thrust', 'Thrust (Fixed-wing)'}, 'Location', 'best');
    % PlotToFile(gcf, 'results/Thrust_Magnetic_Field.png', 20, 10);
    
    %% =========================================================================
    %  Power (Battery & System)
    %  Corresponds to Python: Power
    % =========================================================================
    if exist('bat_v', 'var')
        figure; 
        set(gcf, 'Name', 'Power', 'Color', 'w');
    
        % --- Subplot 1: Voltage/Current ---
        ax1 = subplot(2,1,1); hold on;
        yyaxis left
        plot(bat_t, bat_v, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Voltage [V]');
        ylabel('Voltage [V]');
    
        yyaxis right
        plot(bat_t, bat_i, 'r-', 'LineWidth', 1, 'DisplayName', 'Current [A]');
        ylabel('Current [A]');
    
        title('Battery Voltage & Current'); grid on; legend('show', 'Location', 'best');
        add_standard_background(vis_flight_intervals, vis_flight_names, vis_is_vtol, vis_vtol_intervals, vis_vtol_names);
    
        % --- Subplot 2: Capacity consumed & Remaining ---
        ax2 = subplot(2,1,2); hold on;
        yyaxis left
        plot(bat_t, bat_discharged/100, 'k-', 'DisplayName', 'Discharged [mAh/100]');
        ylabel('Discharged [mAh/100]');
    
        yyaxis right
        plot(bat_t, bat_remaining*100, 'g-', 'LineWidth', 1.5, 'DisplayName', 'Remaining [%]');
        ylabel('Remaining [%]');
        ylim([0, 105]);
    
        xlabel('Time (s)'); grid on; legend('show', 'Location', 'best');
        linkaxes([ax1, ax2], 'x');
        add_standard_background(vis_flight_intervals, vis_flight_names, vis_is_vtol, vis_vtol_intervals, vis_vtol_names);
        % PlotToFile(gcf, 'results/Battery_System.png', 20, 20);
    end
    
    %% =========================================================================
    %  Temperature
    %  Corresponds to Python: Temperature
    % =========================================================================
    if exist('temp_data', 'var') && ~isempty(temp_data)
        figure('Name', 'Temperature', 'Color', 'w');
        hold on;
    
        % Auto-generate colors to avoid hardcoding
        colors = lines(length(temp_data));
    
        for i = 1:length(temp_data)
            plot(temp_data(i).t, temp_data(i).val, 'LineWidth', 1.5, ...
                 'Color', colors(i,:), 'DisplayName', temp_data(i).name);
        end
    
        grid on; legend('show', 'Location', 'best');
        ylabel('Temperature [°C]'); title('System Temperatures');
        xlabel('Time (s)');
        add_standard_background(vis_flight_intervals, vis_flight_names, vis_is_vtol, vis_vtol_intervals, vis_vtol_names);
        % PlotToFile(gcf, 'results/Temperature.png', 20, 10);
    end
    
    %% =========================================================================
    %  Estimator Flags (Python logic replication)
    %  Function: Dynamically extract Health, Timeout and Innovation Flags, only plot non-zero data
    % =========================================================================
    if isfield(log.data, 'estimator_status_0')
        figure('Name', 'Estimator Flags (Dynamic)', 'Color', 'w'); 
        hold on;
    
        plot_count = 0;
        max_plots = 8; % Python limits to max 8 lines
        colors = lines(max_plots); % Generate color table
        active_legends = {};
    
        % Iterate through all candidate signals
        for i = 1:size(candidates, 1)
            lbl = candidates{i, 1};
            data = candidates{i, 2};
    
            % Filtering logic: if np.amax(cur_data) > 0.1
            if max(data) > 0.1
                plot_count = plot_count + 1;
    
                % Plot
                % To avoid overlap, we can add offset like a logic analyzer
                % Or plot original values directly (as Python code shows), here choose direct plotting
                plot(est_t, data, 'Color', colors(plot_count, :), 'LineWidth', 1.5);
    
                active_legends{end+1} = lbl;
    
                % Exit when reaching limit
                if plot_count >= max_plots
                    break;
                end
            end
        end
    
        % 3. Post-processing
        if plot_count == 0
            % If no error flags, plot Health Flags as placeholder (following Python logic)
            plot(est_t, candidates{1,2}, 'k');
            active_legends{end+1} = candidates{1,1};
            title('Estimator Flags (All Good)');
        else
            title(sprintf('Estimator Flags (Top %d Active)', plot_count));
        end
    
        grid on;
        legend(active_legends, 'Location', 'best');
        ylabel('Flag Value'); 
        xlabel('Time (s)');
        add_standard_background(vis_flight_intervals, vis_flight_names, vis_is_vtol, vis_vtol_intervals, vis_vtol_names);
        % Limit Y-axis range (some flags are 0-7)
        % ylim([-0.5, 7.5]);
        % PlotToFile(gcf, 'results/Estimator_Flags.png', 20, 10);
    end
    %% =========================================================================
    %  Failsafe Flags
    %  Corresponds to Python: Failsafe Flags
    %  Logic: Automatically search failsafe_flags_0 table for all non-zero columns and plot
    % =========================================================================
    if isfield(log.data, 'failsafe_flags_0')
        figure; 
        set(gcf, 'Name', 'Failsafe Flags', 'Color', 'w');
        hold on;
    
        % 1. Plot total Failsafe state
        if exist('vs_failsafe', 'var')
            area(vs_t, double(vs_failsafe), 'FaceColor', [1 0.8 0.8], 'EdgeColor', 'none', 'DisplayName', 'In Failsafe Mode');
        end
    
        % 2. Iterate through all flags
        plot_idx = 0;
        colors = lines(10); % Take 10 colors for cycling
    
        for i = 1:length(fs_cols)
            col_name = fs_cols{i};
            % Skip timestamp and mode_req non-flag fields
            if strcmp(col_name, 'timestamp') || startsWith(col_name, 'mode_req'), continue; end
    
            data = fs_table.(col_name);
            % Only plot if this flag was triggered during flight (max>0)
            if max(data) > 0
                plot_idx = plot_idx + 1;
                % Stacked offset display: 1, 2, 3...
                plot(fs_t, double(data) * 0.8 + plot_idx, 'LineWidth', 1.5, ...
                     'Color', colors(mod(plot_idx-1,10)+1, :), ...
                     'DisplayName', strrep(col_name, '_', ' ')); % Remove underscores for better appearance
            end
        end
    
        if plot_idx == 0
            text(mean(fs_t), 0.5, 'No Failsafe Flags Triggered', 'HorizontalAlignment', 'center');
        end
    
        grid on; legend('show');
        title('Failsafe Flags Triggered');
        ylabel('Flags (Stacked)'); xlabel('Time (s)');
        ylim([0, plot_idx + 2]);
        add_standard_background(vis_flight_intervals, vis_flight_names, vis_is_vtol, vis_vtol_intervals, vis_vtol_names);
        % PlotToFile(gcf, 'results/Failsafe_Flags.png', 20, 10);
    end
    
    
    %% =========================================================================
    %  CPU & RAM
    %  Corresponds to Python: CPU & RAM
    % =========================================================================
    if isfield(log.data, 'cpuload_0')
        figure('Name', 'CPU & RAM', 'Color', 'w'); 
        hold on;
        % 1. Plot RAM Usage (corresponds to colors3[1])
        plot(cpu_t, ram_usage, 'LineWidth', 1.5, 'DisplayName', 'RAM Usage');
        
        % 2. Plot CPU Load (corresponds to colors3[2])
        plot(cpu_t, cpu_load, 'LineWidth', 1.5, 'DisplayName', 'CPU Load');
        
        % Style tweaks
        grid on; 
        legend('show', 'Location', 'best');
        ylabel('Load / Usage [0-1]'); 
        title('CPU & RAM');
        xlabel('Time (s)');
        
        % Lock Y-axis range to 0..1 (consistent with Python y_range)
        ylim([0, 1]);
        
        % Add background
        add_standard_background(vis_flight_intervals, vis_flight_names, vis_is_vtol, vis_vtol_intervals, vis_vtol_names);
        % PlotToFile(gcf, 'results/CPU.png', 20, 10);
    end
    
    %% =========================================================================
    %  Sampling Regularity of Sensor Data
    %  Corresponds to Python: Sampling Regularity (sensor_combined & estimator_status)
    % =========================================================================
    if isfield(log.data, 'sensor_combined_0')
        figure('Name', 'Sampling Regularity', 'Color', 'w');
        hold on;
    
        % --- 1. Delta T (sampling interval) ---
        % Python: np.diff(sensor_combined['timestamp'])
        sc = log.data.sensor_combined_0;
        
        % Original timestamp is usually uint64 (us), convert to double for diff
        t_raw = double(sc.timestamp); 
        dt_seq = diff(t_raw); % Result unit: us
        
        % X-axis time: corresponds to 2nd point onward after diff, convert to seconds
        t_plot_sc = t_raw(2:end) * 1e-6; 
    
        % Plot curve (green)
        plot(t_plot_sc, dt_seq, 'Color', [0, 0.6, 0], 'LineWidth', 0.5, ...
             'DisplayName', 'delta t (between 2 logged samples)');
    
        % --- 2. Estimator Time Slip (time slip) ---
        % Python: data['time_slip']*1e6
        if isfield(log.data, 'estimator_status_0')
            es = log.data.estimator_status_0;
            t_es = double(es.timestamp) * 1e-6;
            
            % Convert seconds to microseconds
            slip_us = double(es.time_slip) * 1e6;
            
            % Plot curve (orange)
            plot(t_es, slip_us, 'Color', [0.9, 0.5, 0], 'LineWidth', 1.5, ...
                 'DisplayName', 'Estimator time slip (cumulative)');
        end
    
        % --- Style settings ---
        grid on; legend('show', 'Location', 'best');
        ylabel('[us]'); 
        title('Sampling Regularity of Sensor Data');
        xlabel('Time (s)');
        
        % Python limits Y axis range to [0, 25000] us (i.e. 0-25ms)
        % This is very effective for observing 250Hz (4000us) or 1kHz (1000us) data
        ylim([0, 25000]); 
        
        % Add background
        add_standard_background(vis_flight_intervals, vis_flight_names, vis_is_vtol, vis_vtol_intervals, vis_vtol_names);
        % PlotToFile(gcf, 'results/Sampling_Regularity.png', 20, 10);
    end

end