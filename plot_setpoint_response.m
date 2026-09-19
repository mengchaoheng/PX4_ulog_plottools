clear all;
close all;
clc;

run('load_data_main.m'); 
addpath(genpath(pwd));

%% =========================================================================
%  Global figure / font style
% =========================================================================
set(groot, ...
    'defaultAxesFontSize', 9, ...
    'defaultAxesFontName', 'Times New Roman', ...
    'defaultAxesLineWidth', 0.5, ...
    'defaultAxesLabelFontSizeMultiplier', 1, ...
    'defaultAxesTitleFontSizeMultiplier', 1);

set(0, 'DefaultTextInterpreter', 'latex');
set(0, 'DefaultLegendInterpreter', 'latex');
set(0, 'DefaultAxesTickLabelInterpreter', 'latex');

%% =========================================================================
%  Unified named-color manager
% =========================================================================
sty = plot_style_manager();

% ---- Local line-width definition ----
LW_XS = 0.5;
LW_S  = 0.5;
LW_M  = 0.7;
LW_L  = 1.1;
LW_XL = 1.4;

LW_SP = LW_M;
LW_RES = LW_S;
LW_MAIN = LW_M;
LW_MAIN_BOLD = LW_L;
LW_MULTI = LW_M;
LW_MULTI_BOLD = LW_XL;

STYLE_SP  = plot_style(sty, 'setpoint1', '--', LW_SP);
STYLE_RES = plot_style(sty, 'response1', '-',  LW_RES);
STYLE_AXIS1 = plot_style(sty, 'setpoint2', '-',  LW_MAIN);
STYLE_AXIS2 = plot_style(sty, 'response2', '--', LW_MAIN);
STYLE_AXIS3 = plot_style(sty, 'setpoint3', '-.', LW_MAIN);
STYLE_AXIS4 = plot_style(sty, 'response3', ':',  LW_MAIN);
STYLE_AXIS5 = plot_style(sty, 'setpoint4', '-',  LW_MAIN);
STYLE_AXIS1_BOLD = plot_style(sty, 'setpoint2', '-',  LW_MAIN_BOLD);
STYLE_AXIS2_BOLD = plot_style(sty, 'response2', '--', LW_MAIN_BOLD);
STYLE_AXIS3_BOLD = plot_style(sty, 'setpoint3', '-.', LW_MAIN_BOLD);
STYLE_AXIS4_BOLD = plot_style(sty, 'response3', ':',  LW_MAIN_BOLD);
STYLE_AXIS5_BOLD = plot_style(sty, 'setpoint4', '-',  LW_MAIN_BOLD);
STYLE_AXIS1_THIN = plot_style(sty, 'setpoint2', '-',  LW_XS);
STYLE_AXIS2_THIN = plot_style(sty, 'response2', '--', LW_XS);
STYLE_AXIS3_THIN = plot_style(sty, 'setpoint3', '-.', LW_XS);
STYLE_AXIS1_DASH = plot_style(sty, 'setpoint2', '--', LW_XS);
STYLE_AXIS2_DASH = plot_style(sty, 'response2', '-.', LW_XS);
STYLE_AXIS4_DOT  = plot_style(sty, 'response3', ':',  LW_MAIN);
STYLE_THRUST     = plot_style(sty, 'neutral1', '-',  LW_MAIN_BOLD);
STYLE_THRUST_ALT = plot_style(sty, 'neutral1', '--', LW_MAIN);
STYLE_THRUST_BOLD = plot_style(sty, 'neutral1', '-', LW_XL);

C_AXIS1 = sty.color.setpoint2;
C_AXIS2 = sty.color.response2;
C_AXIS3 = sty.color.setpoint3;
C_AXIS4 = sty.color.response3;
C_AXIS5 = sty.color.setpoint4;
C_AXIS6 = sty.color.response4;
C_AXIS7 = sty.color.setpoint5;
COLOR_WARN_FILL = sty.color.warn_fill;

%% =========================================================================
%  Parameter settings
% =========================================================================
MAX_MOTORS = 12; 
MAX_SERVOS = 8;  
plot_together = 1;   % 1: Servos and motors combined display, 0: independent display
verbose = 0;         % 1: Display more diagnostics
control_fig = 1;     % 1: Display control quantities
n_raw_plot = 8;      % For versions before 1.13, plot first 8 channels of pwm

%% =========================================================================
%  Figure 1 2, 3, 4, 5: (vehicle_angular_velocity, Attitude, Vel, Pos, Control)
% =========================================================================
% --- Figure 1: vehicle_angular_velocity ---
if(exist('vehicle_angular_velocity', 'var') && exist('vehicle_rates_setpoint', 'var'))
    figure('Name', 'Rates', 'Color', 'w');
    % titles = {'Roll Rate', 'Pitch Rate', 'Yaw Rate'};
    ylabels = {'p (deg/s)', 'q (deg/s)', 'r (deg/s)'};
    ax = [];
    for i = 1:3
        ax(i) = subplot(3,1,i); hold on;step = 30;
        plot(vehicle_rates_setpoint_t(1:step:end), vehicle_rates_setpoint(1:step:end,i)*r2d, STYLE_SP{:});step = 30;
        plot(vehicle_angular_velocity_t(1:step:end), vehicle_angular_velocity(1:step:end,i)*r2d, STYLE_RES{:});
        grid on; ylabel(ylabels{i}); xlabel('Time (s)'); %title(titles{i}); 
        if i == 1
            legend('Setpoint', 'Response', 'Location', 'best');
        end
        add_standard_background(vis_flight_intervals, vis_flight_names, vis_is_vtol, vis_vtol_intervals, vis_vtol_names);
    end
    linkaxes(ax, 'x');
    % PlotToFile(gcf, 'results/rates.png', 20, 20);
end

% --- Figure 2: Attitude ---
if(exist('Roll', 'var') && exist('Roll_setpoint', 'var'))
    figure('Name', 'Attitude', 'Color', 'w');step = 10;
    d_sp = {Roll_setpoint(1:step:end), Pitch_setpoint(1:step:end), Yaw_setpoint(1:step:end)};step = 1;
    d_res = {Roll(1:step:end), Pitch(1:step:end), Yaw(1:step:end)};
    % titles = {'Roll', 'Pitch', 'Yaw'};

    % \varphi corresponds to Roll, \theta corresponds to Pitch, \phi corresponds to Yaw
    ylabels = {'$\varphi$ (deg)', '$\theta$ (deg)', '$\phi$ (deg)'};

    ax = [];
    for i = 1:3
        ax(i) = subplot(3,1,i); hold on;step = 10;
        plot(vehicle_attitude_setpoint_t(1:step:end), d_sp{i}*r2d, STYLE_SP{:});step = 1;
        plot(vehicle_attitude_t(1:step:end), d_res{i}*r2d, STYLE_RES{:});
        grid on; ylabel(ylabels{i});xlabel('Time (s)');  %title(titles{i}); 
        if i==1, legend('Setpoint','Response','NumColumns',2, 'Location', 'best'); end
        add_standard_background(vis_flight_intervals, vis_flight_names, vis_is_vtol, vis_vtol_intervals, vis_vtol_names);
    end
    linkaxes(ax, 'x');
    % PlotToFile(gcf, 'results/att.png', 20, 20);
end

% --- Figure 3a: Velocity & TECS (Dynamic subplot count: 3 or 4) ---
if exist('V_XYZ', 'var')
    figure('Name', 'Velocity', 'Color', 'w');  
    n_rows = 3;
    ax = [];

    % --- Subplot 1: Velocity X ---
    ax(1) = subplot(n_rows, 1, 1); hold on;
    if exist('V_XYZ_setpoint', 'var'), plot(vehicle_local_position_setpoint_t, V_XYZ_setpoint(:,1), STYLE_SP{:}); end
    plot(vehicle_local_position_t, V_XYZ(:,1), STYLE_RES{:});
    grid on; ylabel('$v_x$ (m/s)'); xlabel('Time (s)');  %title('Velocity X');
    add_standard_background(vis_flight_intervals, vis_flight_names, vis_is_vtol, vis_vtol_intervals, vis_vtol_names);

    % --- Subplot 2: Velocity Y ---
    ax(2) = subplot(n_rows, 1, 2); hold on;
    if exist('V_XYZ_setpoint', 'var'), plot(vehicle_local_position_setpoint_t, V_XYZ_setpoint(:,2), STYLE_SP{:}); end
    plot(vehicle_local_position_t, V_XYZ(:,2), STYLE_RES{:});
    grid on; ylabel('$v_y$ (m/s)'); xlabel('Time (s)');   %title('Velocity Y');
    add_standard_background(vis_flight_intervals, vis_flight_names, vis_is_vtol, vis_vtol_intervals, vis_vtol_names);

    % --- Subplot 3: Velocity Z ---
    ax(3) = subplot(n_rows, 1, 3); hold on;
    if exist('V_XYZ_setpoint', 'var'), plot(vehicle_local_position_setpoint_t, V_XYZ_setpoint(:,3), STYLE_SP{:}); end
    plot(vehicle_local_position_t, V_XYZ(:,3), STYLE_RES{:});
    if exist('V_XYZ_setpoint', 'var'), legend('Setpoint','Response','NumColumns',2, 'Location', 'best'); end
    grid on; ylabel('$v_z$ (m/s)'); xlabel('Time (s)'); %title('Velocity Z');
    add_standard_background(vis_flight_intervals, vis_flight_names, vis_is_vtol, vis_vtol_intervals, vis_vtol_names);



    linkaxes(ax, 'x');   
    % PlotToFile(gcf, 'results/vel.png', 20, 20);
end

% --- Subplot 3b: TECS Height Rate (Only plotted if exists) ---
if exist('tecs_t', 'var')
    figure('Name', 'tecs', 'Color', 'w');  
    n_rows = 6;
    ax = [];
    
    ax(1) = subplot(n_rows, 1, 1); hold on;
    plot(tecs_t, altitude_sp, STYLE_SP{:});hold on;
    s = plot_style(sty, 'setpoint7', ':', 1.4);
    plot(tecs_t, altitude_reference, s{:});
    plot(vehicle_local_position_t, -XYZ(:,3)+ref_alt, STYLE_RES{:});hold on;
    grid on; ylabel('$h$ (m)');xlabel('Time (s)');title('TECS altitude');
    legend('altitude sp', 'altitude reference','altitude', 'Location', 'best');
    add_standard_background(vis_flight_intervals, vis_flight_names, vis_is_vtol, vis_vtol_intervals, vis_vtol_names);

    ax(2) = subplot(n_rows, 1, 2); hold on;
     
    s = plot_style(sty, 'setpoint4', '--', 1);
    plot(tecs_t, height_rate_reference, s{:});  
    s = plot_style(sty, 'response4', '-', 1.5);
    plot(tecs_t, height_rate_direct, s{:}); 
    plot(tecs_t, height_rate_setpoint, STYLE_SP{:}); 
    plot(tecs_t, height_rate, STYLE_RES{:});
    grid on; ylabel('$\dot{h}$ (m/s)'); xlabel('Time (s)'); title('TECS height rate');
    legend('height rate reference', 'height rate direct','height rate setpoint','height rate', 'Location', 'best');
    add_standard_background(vis_flight_intervals, vis_flight_names, vis_is_vtol, vis_vtol_intervals, vis_vtol_names);

    ax(3) = subplot(n_rows, 1, 3); hold on;
    % s = plot_style(sty, 'setpoint7', '--', 0.8);
    % plot(tecs_t, equivalent_airspeed_sp, s{:});
    plot(tecs_t, true_airspeed_sp, STYLE_SP{:});
    plot(tecs_t, true_airspeed_filtered, STYLE_RES{:});
    grid on; ylabel('$V_a$ (m/s)'); xlabel('Time (s)'); title('TECS airspeed');
    legend('true airspeed sp', 'true airspeed filtered', 'Location', 'best');
    add_standard_background(vis_flight_intervals, vis_flight_names, vis_is_vtol, vis_vtol_intervals, vis_vtol_names);

    ax(4) = subplot(n_rows, 1, 4); hold on;
    plot(tecs_t, true_airspeed_derivative_sp, STYLE_SP{:});
    plot(tecs_t, true_airspeed_derivative, STYLE_RES{:});
    s = plot_style(sty, 'response4', '-.', 1.1);
    % plot(tecs_t, true_airspeed_derivative_raw, s{:});
    grid on; ylabel('$\dot{V}_a$ (m/s)'); title('TECS airspeed derivative');
    legend('true airspeed derivative sp', 'true airspeed derivative', 'Location', 'best');
    add_standard_background(vis_flight_intervals, vis_flight_names, vis_is_vtol, vis_vtol_intervals, vis_vtol_names);

    ax(5) = subplot(n_rows, 1, 5); hold on;
    plot(tecs_t, total_energy_rate_sp, STYLE_SP{:});
    plot(tecs_t, total_energy_rate, STYLE_RES{:});
    grid on; ylabel('$\dot{E}$'); xlabel('Time (s)');title('TECS total energy rate');
    legend('total energy rate sp','total energy rate', 'Location', 'best');
    add_standard_background(vis_flight_intervals, vis_flight_names, vis_is_vtol, vis_vtol_intervals, vis_vtol_names);

    ax(6) = subplot(n_rows, 1, 6); hold on;
    plot(tecs_t, total_energy_balance_rate_sp, STYLE_SP{:});
    plot(tecs_t, total_energy_balance_rate, STYLE_RES{:});
    grid on; ylabel('$\dot{B}$'); xlabel('Time (s)'); title('TECS total energy balance rate');
    legend('total energy balance rate sp','total energy balance rate', 'Location', 'best');
    add_standard_background(vis_flight_intervals, vis_flight_names, vis_is_vtol, vis_vtol_intervals, vis_vtol_names);

    % PlotToFile(gcf, 'results/tecs.pdf', 20, 24);
end

% --- Figure 4: Position ---
if(exist('XYZ', 'var'))
    figure('Name', 'Position', 'Color', 'w'); 
    ylabels = {'x (m)', 'y (m)', 'z (m)'};
    has_sp = exist('XYZ_setpoint', 'var');
    ax = [];
    for i = 1:3
        ax(i) = subplot(3,1,i); hold on;
        if has_sp, plot(vehicle_local_position_setpoint_t, XYZ_setpoint(:,i), STYLE_SP{:}); end
        plot(vehicle_local_position_t, XYZ(:,i), STYLE_RES{:});
        grid on; ylabel(ylabels{i}); 
        if i==3
            % title('Position'); 
            legend('Setpoint','Response','NumColumns',2, 'Location', 'northeast'); 
        end
        add_standard_background(vis_flight_intervals, vis_flight_names, vis_is_vtol, vis_vtol_intervals, vis_vtol_names);
    end
    linkaxes(ax, 'x');   xlabel('Time (s)');
    % PlotToFile(gcf, 'results/pos.png', 20, 20);
end

%% =========================================================================
%  Trajectory  
% =========================================================================
if(exist('XYZ', 'var') && exist('XYZ_setpoint', 'var'))
    figure('Name', 'Trajectory', 'Color', 'w');
    step = 1;
    plot3(XYZ_setpoint(1:step:end,2), XYZ_setpoint(1:step:end,1), -XYZ_setpoint(1:step:end,3), STYLE_SP{:}); hold on;
    plot3(XYZ(:,2), XYZ(:,1), -XYZ(:,3), STYLE_RES{:});
    % title('Trajectory'); 
    xlabel('y (m)'); ylabel('x (m)'); zlabel('z (m)');
    grid on; view(45, 30);
    legend('Setpoint', 'Response', 'Location', 'best');
    % PlotToFile(gcf, 'results/traj.png', 15, 15);
end

%% =========================================================================
%  Control figures
% =========================================================================
if control_fig
    % ---------------------------------------------------------------------
    % Actuator Controls
    % ---------------------------------------------------------------------
    if ~isempty(actuator_controls_0.time)
        figure('Name', 'Actuator Controls', 'Color', 'w');

        has_group1 = ~isempty(actuator_controls_1.time);
        if has_group1
            n_cols = 2;
            layout_title = 'Actuator Controls (Left: Group 0, Right: Group 1)';
        else
            n_cols = 1;
            layout_title = 'Actuator Controls (Group 0)';
        end

        line_cols = {STYLE_AXIS1, STYLE_AXIS2, STYLE_AXIS3};
        titles = {'All', 'Roll', 'Pitch', 'Yaw', 'Thrust'};
        ax_all = [];

        for row = 1:5
            idx_left = (row - 1) * n_cols + 1;
            ax = subplot(5, n_cols, idx_left); hold on;
            ax_all = [ax_all, ax];

            if row == 1
                plot(actuator_controls_0.time, actuator_controls_0.roll,  line_cols{1}{:}, 'DisplayName', 'Roll');
                plot(actuator_controls_0.time, actuator_controls_0.pitch, line_cols{2}{:}, 'DisplayName', 'Pitch');
                plot(actuator_controls_0.time, actuator_controls_0.yaw,   line_cols{3}{:}, 'DisplayName', 'Yaw');

                if ~isempty(actuator_controls_0.thrust_z_neg)
                    plot(actuator_controls_0.time_thrust, actuator_controls_0.thrust_z_neg, ...
                        STYLE_THRUST{:}, 'DisplayName', 'Thrust (up)');
                end
                if ~isempty(actuator_controls_0.thrust_x) && any(actuator_controls_0.thrust_x ~= 0)
                    plot(actuator_controls_0.time_thrust, actuator_controls_0.thrust_x, ...
                        STYLE_THRUST_ALT{:}, 'DisplayName', 'Thrust (fwd)');
                end
                title('Group 0 (Main)');
                if n_cols == 1
                    legend('Location', 'best', 'NumColumns', 5);
                end

            elseif row >= 2 && row <= 4
                data_map = {actuator_controls_0.roll, actuator_controls_0.pitch, actuator_controls_0.yaw};
                plot(actuator_controls_0.time, data_map{row-1}, line_cols{row-1}{:});
                ylabel(titles{row});

            elseif row == 5
                if ~isempty(actuator_controls_0.thrust_z_neg)
                    plot(actuator_controls_0.time_thrust, actuator_controls_0.thrust_z_neg, ...
                        STYLE_THRUST{:}, 'DisplayName', 'Up');
                end
                if ~isempty(actuator_controls_0.thrust_x)
                    plot(actuator_controls_0.time_thrust, actuator_controls_0.thrust_x, ...
                        STYLE_THRUST_ALT{:}, 'DisplayName', 'Fwd');
                end
                ylabel('Thrust');
                legend('show', 'Location', 'best');
            end
            grid on;
            add_standard_background(vis_flight_intervals, vis_flight_names, ...
                vis_is_vtol, vis_vtol_intervals, vis_vtol_names);

            if has_group1
                idx_right = (row - 1) * n_cols + 2;
                ax = subplot(5, n_cols, idx_right); hold on;
                ax_all = [ax_all, ax];

                if row == 1
                    plot(actuator_controls_1.time, actuator_controls_1.roll,  line_cols{1}{:}, 'DisplayName', 'Roll');
                    plot(actuator_controls_1.time, actuator_controls_1.pitch, line_cols{2}{:}, 'DisplayName', 'Pitch');
                    plot(actuator_controls_1.time, actuator_controls_1.yaw,   line_cols{3}{:}, 'DisplayName', 'Yaw');
                    if ~isempty(actuator_controls_1.thrust_x)
                        plot(actuator_controls_1.time, actuator_controls_1.thrust_x, ...
                            STYLE_THRUST_ALT{:}, 'DisplayName', 'Thrust (fwd)');
                    end
                    title('Group 1 (Aux/FW)');

                elseif row >= 2 && row <= 4
                    data_map = {actuator_controls_1.roll, actuator_controls_1.pitch, actuator_controls_1.yaw};
                    plot(actuator_controls_1.time, data_map{row-1}, line_cols{row-1}{:});

                elseif row == 5
                    if ~isempty(actuator_controls_1.thrust_x)
                        plot(actuator_controls_1.time, actuator_controls_1.thrust_x, ...
                            STYLE_THRUST_ALT{:}, 'DisplayName', 'Fwd');
                    end
                end
                grid on;
                add_standard_background(vis_flight_intervals, vis_flight_names, ...
                    vis_is_vtol, vis_vtol_intervals, vis_vtol_names);
            end
        end

        linkaxes(ax_all, 'x');
        xlabel(ax_all(end), 'Time (s)');
        sgtitle(layout_title);
        % PlotToFile(gcf, 'results/control.png', 20, 20);
    end

    % ---------------------------------------------------------------------
    % Allocation INDI feedback: raw and filtered physical wrench
    % ---------------------------------------------------------------------
    if exist('allocation_feedback', 'var')
        figure('Name', 'Allocation INDI Feedback Filter', 'Color', 'w');
        axis_names = {'x', 'y', 'z'};
        ax_filter = [];

        for axis_idx = 1:3
            ax = subplot(3, 2, 2 * axis_idx - 1); hold on;
            ax_filter = [ax_filter, ax];
            plot(allocation_feedback.time, allocation_feedback.torque_raw(:, axis_idx), ...
                STYLE_AXIS2_THIN{:}, 'DisplayName', 'Raw');
            plot(allocation_feedback.time, allocation_feedback.torque_filtered(:, axis_idx), ...
                STYLE_AXIS1_BOLD{:}, 'DisplayName', 'Filtered');
            grid on;
            ylabel(sprintf('$M_%s$ (N m)', axis_names{axis_idx}));
            if axis_idx == 1
                title('Allocated moment feedback');
                legend('Location', 'best');
            end
            add_standard_background(vis_flight_intervals, vis_flight_names, ...
                vis_is_vtol, vis_vtol_intervals, vis_vtol_names);

            ax = subplot(3, 2, 2 * axis_idx); hold on;
            ax_filter = [ax_filter, ax];
            plot(allocation_feedback.time, allocation_feedback.force_raw(:, axis_idx), ...
                STYLE_AXIS2_THIN{:}, 'DisplayName', 'Raw');
            plot(allocation_feedback.time, allocation_feedback.force_filtered(:, axis_idx), ...
                STYLE_AXIS1_BOLD{:}, 'DisplayName', 'Filtered');
            grid on;
            ylabel(sprintf('$F_%s$ (N)', axis_names{axis_idx}));
            if axis_idx == 1
                title('Allocated force feedback');
                legend('Location', 'best');
            end
            add_standard_background(vis_flight_intervals, vis_flight_names, ...
                vis_is_vtol, vis_vtol_intervals, vis_vtol_names);
        end

        xlabel(ax_filter(end - 1), 'Time (s)');
        xlabel(ax_filter(end), 'Time (s)');
        linkaxes(ax_filter, 'x');
    end

    % ---------------------------------------------------------------------
    % Allocation INDI feedback filter: low-rate diagnostic state
    % ---------------------------------------------------------------------
    if exist('allocation_filter_status', 'var')
        figure('Name', 'Allocation INDI Feedback Filter Status', 'Color', 'w');
        ax_status = [];

        ax = subplot(4, 1, 1); hold on;
        ax_status = [ax_status, ax];
        if exist('allocation_feedback', 'var') && ~isempty(allocation_feedback.logged_interval)
            plot(allocation_feedback.time(2:end), ...
                allocation_feedback.logged_interval * 1000, ...
                STYLE_AXIS2_THIN{:}, ...
                'DisplayName', 'Recorded topic spacing (logger)');
        end
        plot(allocation_filter_status.time, ...
            allocation_filter_status.sample_interval * 1000, ...
            STYLE_AXIS1_BOLD{:}, 'DisplayName', 'Filter mean update dt');
        grid on; ylabel('dt (ms)');
        legend('Location', 'best');
        title('Logger record spacing vs actual filter update interval');
        add_standard_background(vis_flight_intervals, vis_flight_names, ...
            vis_is_vtol, vis_vtol_intervals, vis_vtol_names);

        ax = subplot(4, 1, 2); hold on;
        ax_status = [ax_status, ax];
        plot(allocation_filter_status.time, allocation_filter_status.sample_rate_hz, ...
            STYLE_AXIS1_BOLD{:});
        grid on; ylabel('$f_s$ (Hz)');
        title('Filter coefficient sample rate');
        add_standard_background(vis_flight_intervals, vis_flight_names, ...
            vis_is_vtol, vis_vtol_intervals, vis_vtol_names);

        ax = subplot(4, 1, 3); hold on;
        ax_status = [ax_status, ax];
        plot(allocation_filter_status.time, allocation_filter_status.torque_cutoff_hz, ...
            STYLE_AXIS1_BOLD{:}, 'DisplayName', 'Moment');
        plot(allocation_filter_status.time, allocation_filter_status.force_cutoff_hz, ...
            STYLE_AXIS2_BOLD{:}, 'DisplayName', 'Force');
        grid on; ylabel('$f_c$ (Hz)');
        legend('Location', 'best');
        title('Configured cutoff');
        add_standard_background(vis_flight_intervals, vis_flight_names, ...
            vis_is_vtol, vis_vtol_intervals, vis_vtol_names);

        ax = subplot(4, 1, 4); hold on;
        ax_status = [ax_status, ax];
        reset_events = allocation_filter_status.reset_count ...
            - allocation_filter_status.reset_count(1);
        reconfigure_events = allocation_filter_status.reconfigure_count ...
            - allocation_filter_status.reconfigure_count(1);
        stairs(allocation_filter_status.time, reset_events, ...
            'Color', C_AXIS1, 'LineWidth', LW_MAIN_BOLD, ...
            'DisplayName', 'Reset events');
        stairs(allocation_filter_status.time, reconfigure_events, ...
            'Color', C_AXIS2, 'LineWidth', LW_MAIN_BOLD, ...
            'DisplayName', 'Reconfigure events');
        ylabel('Events since log start');
        grid on; xlabel('Time (s)');
        legend('Location', 'best');
        title('Filter state changes (each increment is one event)');
        add_standard_background(vis_flight_intervals, vis_flight_names, ...
            vis_is_vtol, vis_vtol_intervals, vis_vtol_names);

        linkaxes(ax_status, 'x');
    end

    % ---------------------------------------------------------------------
    % Acceleration INDI: applied state, a_0 sources, HTE and force scale
    % ---------------------------------------------------------------------
    if exist('indi_accel_status', 'var')
        figure('Name', 'Acceleration INDI Feedback', 'Color', 'w');
        axis_names = {'x', 'y', 'z'};
        ax_accel_indi = [];

        ax = subplot(6, 1, 1); hold on;
        ax_accel_indi = [ax_accel_indi, ax];
        if exist('acc_indi_active', 'var')
            stairs(acc_indi_active_t, double(acc_indi_active), ...
                STYLE_AXIS1_BOLD{:}, 'DisplayName', 'acc INDI active');
        else
            plot(indi_accel_status.time, nan(size(indi_accel_status.time)), ...
                STYLE_AXIS1_BOLD{:}, 'DisplayName', 'acc INDI active (not logged)');
        end
        ylim([-0.05, 1.05]); yticks([0, 1]);
        grid on; ylabel('Active');
        title('Acceleration INDI actually produced the thrust setpoint');
        legend('Location', 'best');
        add_standard_background(vis_flight_intervals, vis_flight_names, ...
            vis_is_vtol, vis_vtol_intervals, vis_vtol_names);

        for axis_idx = 1:3
            ax = subplot(6, 1, axis_idx + 1); hold on;
            ax_accel_indi = [ax_accel_indi, ax];
            plot(indi_accel_status.time, indi_accel_status.feedback(:, axis_idx), ...
                STYLE_AXIS1_BOLD{:}, 'DisplayName', 'Current $a_0$');
            plot(indi_accel_status.time, indi_accel_status.velocity_derivative(:, axis_idx), ...
                STYLE_AXIS2_THIN{:}, 'DisplayName', 'Velocity derivative');
            plot(indi_accel_status.time, indi_accel_status.ekf(:, axis_idx), ...
                STYLE_AXIS3_THIN{:}, 'DisplayName', 'EKF acceleration');
            plot(indi_accel_status.time, indi_accel_status.imu(:, axis_idx), ...
                STYLE_AXIS4_DOT{:}, 'DisplayName', 'IMU acceleration');
            grid on; ylabel(sprintf('$a_{0,%s}$ (m/s$^2$)', axis_names{axis_idx}));
            title(sprintf('$a_0$ source comparison (%s axis)', axis_names{axis_idx}));
            if axis_idx == 1
                legend('Location', 'best');
            end
            add_standard_background(vis_flight_intervals, vis_flight_names, ...
                vis_is_vtol, vis_vtol_intervals, vis_vtol_names);
        end

        ax = subplot(6, 1, 5); hold on;
        ax_accel_indi = [ax_accel_indi, ax];
        if exist('hte_status', 'var')
            plot(hte_status.time, hte_status.hover_thrust, ...
                STYLE_AXIS1_BOLD{:}, 'DisplayName', 'HTE estimate');
        end
        plot(indi_accel_status.time, indi_accel_status.hover_thrust, ...
            STYLE_AXIS2_BOLD{:}, 'DisplayName', 'Applied hover thrust');
        grid on; ylabel('Normalized thrust');
        title('Hover thrust: estimator output and controller-applied value');
        legend('Location', 'best');
        add_standard_background(vis_flight_intervals, vis_flight_names, ...
            vis_is_vtol, vis_vtol_intervals, vis_vtol_names);

        ax = subplot(6, 1, 6); hold on;
        ax_accel_indi = [ax_accel_indi, ax];
        plot(indi_accel_status.time, indi_accel_status.force_feedback_scale, ...
            STYLE_AXIS1_BOLD{:}, 'DisplayName', 'Force feedback scale');
        grid on; ylabel('Scale'); xlabel('Time (s)');
        title('Allocation-force scale used by acceleration INDI');
        legend('Location', 'best');
        add_standard_background(vis_flight_intervals, vis_flight_names, ...
            vis_is_vtol, vis_vtol_intervals, vis_vtol_names);

        linkaxes(ax_accel_indi, 'x');
    end

    % ---------------------------------------------------------------------
    % Figure 6/7/8: Actuators & PWM
    % ---------------------------------------------------------------------
    if dynamic_control_alloc

        if plot_together
            if isfield(log.data, 'actuator_motors_0') || isfield(log.data, 'actuator_servos_0')

                n_motors = 0;
                if isfield(log, 'params') && isfield(log.params, 'CA_ROTOR_COUNT')
                    n_motors = double(log.params.CA_ROTOR_COUNT);
                else
                    if exist('motors', 'var')
                        n_motors = size(motors, 2);
                    end
                end

                n_servos = 0;
                if isfield(log, 'params') && isfield(log.params, 'CA_SV_CS_COUNT')
                    n_servos = double(log.params.CA_SV_CS_COUNT);
                else
                    if exist('servos', 'var')
                        n_servos = size(servos, 2);
                    end
                end

                n_motors = min(n_motors, MAX_MOTORS);
                n_servos = min(n_servos, MAX_SERVOS);

                has_motor_data = (n_motors > 0) && isfield(log.data, 'actuator_motors_0') && exist('motors', 'var');
                has_servo_data = (n_servos > 0) && isfield(log.data, 'actuator_servos_0') && exist('servos', 'var');

                total_subplots = double(has_motor_data) + double(has_servo_data);

                if total_subplots > 0
                    figure('Name', 'Actuator Outputs (Merged)', 'Color', 'w');
                    current_plot_idx = 1;

                    if has_motor_data
                        ax_m = subplot(total_subplots, 1, current_plot_idx); hold on;
                        colors_motor = make_channel_colors(n_motors, sty);
                        t_m = log.data.actuator_motors_0.timestamp * 1e-6;
                        for i = 1:n_motors
                            if i <= size(motors, 2)
                                plot(t_m, motors(:, i), 'Color', colors_motor(i,:), ...
                                    'LineWidth', LW_MULTI, ...
                                    'DisplayName', sprintf('Motor %d', i));
                            end
                        end
                        grid on;
                        ylabel('Motors Output');
                        title(sprintf('Actuator: Motors (Total %d)', n_motors));
                        legend('show');
                        add_standard_background(vis_flight_intervals, vis_flight_names, ...
                            vis_is_vtol, vis_vtol_intervals, vis_vtol_names);

                        if has_servo_data
                            set(gca, 'XTickLabel', []);
                        else
                            xlabel('Time (s)');
                        end
                        current_plot_idx = current_plot_idx + 1;
                    end

                    if has_servo_data
                        ax_s = subplot(total_subplots, 1, current_plot_idx); hold on;
                        colors_servo = make_channel_colors(n_servos, sty);
                        t_s = log.data.actuator_servos_0.timestamp * 1e-6;
                        for i = 1:n_servos
                            if i <= size(servos, 2)
                                plot(t_s, servos(:, i), 'Color', colors_servo(i,:), ...
                                    'LineWidth', LW_MULTI, ...
                                    'DisplayName', sprintf('Servo %d', i));
                            end
                        end
                        grid on;
                        ylabel('Servos Output');
                        title(sprintf('Actuator: Servos (Total %d)', n_servos));
                        legend('show');
                        xlabel('Time (s)');
                        add_standard_background(vis_flight_intervals, vis_flight_names, ...
                            vis_is_vtol, vis_vtol_intervals, vis_vtol_names);
                    end

                    if has_motor_data && has_servo_data
                        linkaxes([ax_m, ax_s], 'x');
                    end
                    % PlotToFile(gcf, 'results/motors_servos.png', 20, 20);
                end
            end

        else
            % -----------------------------------------------------------------
            % Figure 6: Motors
            % -----------------------------------------------------------------
            if isfield(log.data, 'actuator_motors_0') && exist('motors', 'var')

                if isfield(log, 'params') && isfield(log.params, 'CA_ROTOR_COUNT')
                    n_motors = double(log.params.CA_ROTOR_COUNT);
                else
                    n_motors = size(motors, 2);
                end
                n_motors = min(n_motors, MAX_MOTORS);

                if n_motors > 0
                    figure('Name', 'Actuator Motors', 'Color', 'w');
                    motor_colors = make_channel_colors(n_motors, sty);

                    for i = 1:n_motors
                        subplot(n_motors, 1, i); hold on;

                        if i <= size(motors, 2)
                            plot(log.data.actuator_motors_0.timestamp*1e-6, motors(:, i), ...
                                'Color', motor_colors(i,:), 'LineWidth', LW_MULTI);
                        end

                        grid on;
                        ylabel(sprintf('Motor %d', i));
                        add_standard_background(vis_flight_intervals, vis_flight_names, ...
                            vis_is_vtol, vis_vtol_intervals, vis_vtol_names);

                        if i == 1
                            title(sprintf('Actuator Motors (Total: %d)', n_motors));
                        end
                        if i == n_motors
                            xlabel('Time (s)');
                        else
                            set(gca, 'XTickLabel', []);
                        end
                    end
                    % PlotToFile(gcf, 'results/motors.png', 20, 20);
                end
            end

            % -----------------------------------------------------------------
            % Figure 7: Servos
            % -----------------------------------------------------------------
            if isfield(log.data, 'actuator_servos_0') && exist('servos', 'var')

                if isfield(log, 'params') && isfield(log.params, 'CA_SV_CS_COUNT')
                    n_servos = double(log.params.CA_SV_CS_COUNT);
                else
                    n_servos = size(servos, 2);
                end
                n_servos = min(n_servos, MAX_SERVOS);

                if n_servos > 0
                    figure('Name', 'Actuator Servos', 'Color', 'w');
                    servo_colors = make_channel_colors(n_servos, sty);

                    for i = 1:n_servos
                        subplot(n_servos, 1, i); hold on;

                        if i <= size(servos, 2)
                            plot(log.data.actuator_servos_0.timestamp*1e-6, servos(:, i), ...
                                'Color', servo_colors(i,:), 'LineWidth', LW_MULTI);
                        end

                        grid on;
                        ylabel(sprintf('Servo %d', i));
                        add_standard_background(vis_flight_intervals, vis_flight_names, ...
                            vis_is_vtol, vis_vtol_intervals, vis_vtol_names);

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

        % -----------------------------------------------------------------
        % Figure 8: PWM Outputs
        % -----------------------------------------------------------------
        if exist('active_channels', 'var') && ~isempty(active_channels)
            figure('Name', 'Actuator Outputs (PWM)', 'Color', 'w');

            is_motor = strcmp({active_channels.type}, 'Motor');
            idx_m = find(is_motor);
            idx_s = find(~is_motor);

            n_plots = double(~isempty(idx_m)) + double(~isempty(idx_s));
            cur = 1;
            ax_list = [];

            if ~isempty(idx_m)
                ax = subplot(n_plots, 1, cur); hold on;
                cols = make_channel_colors(length(idx_m), sty);
                for k = 1:length(idx_m)
                    info = active_channels(idx_m(k));
                    data = log.data.actuator_outputs_0.(info.col_name);
                    plot(outputs_t, data, 'Color', cols(k,:), 'LineWidth', LW_MULTI, ...
                        'DisplayName', sprintf('%s (Ch%d)', info.name, info.idx));
                end
                grid on; ylabel('PWM (us)'); title('Motors PWM'); legend('show');
                add_standard_background(vis_flight_intervals, vis_flight_names, ...
                    vis_is_vtol, vis_vtol_intervals, vis_vtol_names);
                ax_list = [ax_list, ax];
                cur = cur + 1;
            end

            if ~isempty(idx_s)
                ax = subplot(n_plots, 1, cur); hold on;
                cols = make_channel_colors(length(idx_s), sty);
                for k = 1:length(idx_s)
                    info = active_channels(idx_s(k));
                    data = log.data.actuator_outputs_0.(info.col_name);
                    plot(outputs_t, data, 'Color', cols(k,:), 'LineWidth', LW_MULTI, ...
                        'DisplayName', sprintf('%s (Ch%d)', info.name, info.idx));
                end
                grid on; ylabel('PWM (us)'); title('Servos/Other PWM'); legend('show');
                add_standard_background(vis_flight_intervals, vis_flight_names, ...
                    vis_is_vtol, vis_vtol_intervals, vis_vtol_names);
                ax_list = [ax_list, ax];
            end

            if ~isempty(ax_list)
                linkaxes(ax_list, 'x');
            end
            xlabel('Time (s)');
            % PlotToFile(gcf, 'results/PWM.png', 20, 20);
        else
            fprintf('Figure 8 Skipped: No active PWM channels identified.\n');
        end

    else
        % -----------------------------------------------------------------
        % Original plotting fallback
        % -----------------------------------------------------------------
        figure('Name', 'Actuator Outputs (Raw)', 'Color', 'w');

        ax_raw = gobjects(1, n_raw_plot);
        raw_colors = make_channel_colors(n_raw_plot, sty);

        for i = 1:n_raw_plot
            ax_raw(i) = subplot(n_raw_plot, 1, i); hold on;

            y_data = outputs(:, i);

            if ~all(isnan(y_data)) && any(y_data ~= 0)
                plot(outputs_t, y_data, 'Color', raw_colors(i,:), 'LineWidth', LW_MULTI);
            end

            ylabel(sprintf('Out %d', i-1));
            grid on;

            if i == 1
                title(sprintf('Actuator Outputs (First %d Raw)', n_raw_plot));
            end
            if i < n_raw_plot
                set(gca, 'XTickLabel', []);
            else
                xlabel('Time (s)');
            end

            axis tight;
            add_standard_background(vis_flight_intervals, vis_flight_names, ...
                vis_is_vtol, vis_vtol_intervals, vis_vtol_names);
        end

        linkaxes(ax_raw, 'x');
        % PlotToFile(gcf, 'results/pwm.png', 20, 20);
    end
end

%% =========================================================================
%  Verbose figures
% =========================================================================
if verbose

    % ---------------------------------------------------------------------
    % Angular Accel
    % ---------------------------------------------------------------------
    if exist('vehicle_angular_acceleration', 'var') || ...
       (isfield(log.data, 'vehicle_angular_velocity_0') && ...
        ismember('xyz_derivative_0_', log.data.vehicle_angular_velocity_0.Properties.VariableNames))

        figure('Name', 'Ang Acc vs rates', 'Color', 'w');
        titles = {'Roll Ang Acc', 'Pitch Ang Acc', 'Yaw Ang Acc'};
        ax = gobjects(1,3);

        for i = 1:3
            ax(i) = subplot(3,1,i); hold on;
            plot(vehicle_angular_acceleration_t, vehicle_angular_acceleration(:,i), STYLE_AXIS1{:});
            plot(vehicle_angular_velocity_t, vehicle_angular_velocity(:,i), STYLE_AXIS2{:});
            grid on; ylabel('rad/s$^2$'); title(titles{i});
            add_standard_background(vis_flight_intervals, vis_flight_names, ...
                vis_is_vtol, vis_vtol_intervals, vis_vtol_names);
        end
        linkaxes(ax, 'x'); xlabel('Time (s)');
        % PlotToFile(gcf, 'results/Angular_Accel.png', 20, 10);
    end

    % ---------------------------------------------------------------------
    % Manual Control Inputs
    % ---------------------------------------------------------------------
    if isfield(log.data, 'manual_control_setpoint_0')
        figure('Name', 'Manual Control Inputs', 'Color', 'w'); hold on;
        plot(rc_t, rc_roll,     STYLE_AXIS1_BOLD{:}, 'DisplayName', 'Roll');
        plot(rc_t, rc_pitch,    STYLE_AXIS2_BOLD{:}, 'DisplayName', 'Pitch');
        plot(rc_t, rc_yaw,      STYLE_AXIS3_BOLD{:}, 'DisplayName', 'Yaw');
        plot(rc_t, rc_throttle, STYLE_THRUST_BOLD{:}, 'DisplayName', 'Throttle');

        grid on;
        legend('show', 'Location', 'best', 'NumColumns', 4);
        ylabel('Norm Input [-1, 1]');
        title('Manual Control Inputs (Sticks)');
        xlabel('Time (s)');
        ylim([-1.1, 1.1]);
        add_standard_background(vis_flight_intervals, vis_flight_names, ...
            vis_is_vtol, vis_vtol_intervals, vis_vtol_names);
        % PlotToFile(gcf, 'results/Manual_input.png', 20, 10);
    end

    % ---------------------------------------------------------------------
    % FFT
    % ---------------------------------------------------------------------
    if exist('actuator_controls_0', 'var')
        figure('Color', 'w', 'Name', 'Actuator Controls FFT');
        ctrl_data = [actuator_controls_0.roll, actuator_controls_0.pitch, actuator_controls_0.yaw];
        markers = {
            'MC_DTERM_CUTOFF',  'D-Term Cutoff';
            'IMU_DGYRO_CUTOFF', 'D-Gyro Cutoff';
            'IMU_GYRO_CUTOFF',  'Gyro Cutoff'
        };
        draw_fft_analysis(actuator_controls_0.time, ctrl_data, {'Roll', 'Pitch', 'Yaw'}, ...
            'Actuator Controls FFT (Torque Setpoint)', log.params, markers);
        % PlotToFile(gcf, 'results/control_fft.png', 20, 10);
    end

    if exist('vehicle_angular_velocity', 'var')
        figure('Color', 'w', 'Name', 'Angular Velocity FFT');
        markers = {
            'IMU_GYRO_CUTOFF',  'Gyro Cutoff';
            'IMU_GYRO_NF_FREQ', 'Notch Freq'
        };
        draw_fft_analysis(vehicle_angular_velocity_t, vehicle_angular_velocity, ...
            {'Rollrate', 'Pitchrate', 'Yawrate'}, 'Angular Velocity FFT', log.params, markers);
        % PlotToFile(gcf, 'results/rate_fft.png', 20, 10);
    end

    if exist('vehicle_angular_acceleration', 'var')
        figure('Color', 'w', 'Name', 'Angular Acceleration FFT');
        markers = {
            'IMU_DGYRO_CUTOFF', 'D-Gyro Cutoff';
            'IMU_GYRO_NF_FREQ', 'Notch Freq'
        };
        draw_fft_analysis(vehicle_angular_acceleration_t, vehicle_angular_acceleration, ...
            {'Roll Acc', 'Pitch Acc', 'Yaw Acc'}, 'Angular Acceleration FFT', log.params, markers);
        % PlotToFile(gcf, 'results/acc_fft.png', 20, 10);
    end

    % ---------------------------------------------------------------------
    % Raw Acceleration
    % ---------------------------------------------------------------------
    if exist('raw_acc', 'var')
        figure('Name', 'Raw Acceleration', 'Color', 'w'); hold on;
        plot(raw_acc_t, raw_acc(:,1), STYLE_AXIS1_THIN{:}, 'DisplayName', 'Acc X');
        plot(raw_acc_t, raw_acc(:,2), STYLE_AXIS2_THIN{:}, 'DisplayName', 'Acc Y');
        plot(raw_acc_t, raw_acc(:,3), STYLE_AXIS3_THIN{:}, 'DisplayName', 'Acc Z');

        grid on;
        legend('show', 'Location', 'best');
        ylabel('Acceleration [m/s$^2$]');
        title('Raw Acceleration');
        xlabel('Time (s)');
        add_standard_background(vis_flight_intervals, vis_flight_names, ...
            vis_is_vtol, vis_vtol_intervals, vis_vtol_names);
        % PlotToFile(gcf, 'results/raw_acc.png', 20, 10);
    end

    % ---------------------------------------------------------------------
    % Vibration Metrics
    % ---------------------------------------------------------------------
    if exist('vib_data', 'var') && ~isempty(vib_data)
        figure('Name', 'Vibration Metrics', 'Color', 'w'); hold on;
        line_colors = make_channel_colors(length(vib_data), sty);
        for k = 1:length(vib_data)
            id = vib_data(k).id;
            plot(vib_data(k).t, vib_data(k).val, ...
                'Color', line_colors(k,:), ...
                'LineWidth', LW_MULTI_BOLD, ...
                'DisplayName', sprintf('Accel %d Vib [m/s$^2$]', id));
        end
        grid on;
        ylabel('Vibration Level [m/s$^2$]');
        title('Vibration Metrics');
        xlabel('Time (s)');
        legend('show', 'Location', 'best');
        add_standard_background(vis_flight_intervals, vis_flight_names, ...
            vis_is_vtol, vis_vtol_intervals, vis_vtol_names);
        set(gca, 'Layer', 'top');
        % PlotToFile(gcf, 'results/Vibration_Metrics.png', 20, 10);
    end

    % ---------------------------------------------------------------------
    % Spectrogram
    % ---------------------------------------------------------------------
    if exist('raw_acc', 'var') && ~isempty(raw_acc)
        figure('Name', 'Accel PSD', 'Color', 'w');
        draw_spec_analysis(raw_acc_t, raw_acc, 'Acceleration Power Spectral Density (Sum X+Y+Z)');
        % PlotToFile(gcf, 'results/Acceleration_Spectrogram.png', 20, 10);
    end

    if exist('vehicle_angular_velocity', 'var')
        figure('Name', 'Gyro PSD', 'Color', 'w');
        draw_spec_analysis(vehicle_angular_velocity_t, vehicle_angular_velocity, ...
            'Angular Velocity PSD (Sum X+Y+Z)');
        % PlotToFile(gcf, 'results/Gyro_Spectrogram.png', 20, 10);
    end

    if exist('vehicle_angular_acceleration', 'var')
        figure('Name', 'AngAcc PSD', 'Color', 'w');
        draw_spec_analysis(vehicle_angular_acceleration_t, vehicle_angular_acceleration, ...
            'Angular Acceleration PSD (Sum X+Y+Z)');
        % PlotToFile(gcf, 'results/AngAcc_Spectrogram.png', 20, 10);
    end

    % ---------------------------------------------------------------------
    % Raw Angular Speed
    % ---------------------------------------------------------------------
    if exist('raw_gyro', 'var')
        figure('Name', 'Raw Angular Speed', 'Color', 'w'); hold on;
        plot(raw_gyro_t, raw_gyro(:,1), STYLE_AXIS1_THIN{:}, 'DisplayName', 'X');
        plot(raw_gyro_t, raw_gyro(:,2), STYLE_AXIS2_THIN{:}, 'DisplayName', 'Y');
        plot(raw_gyro_t, raw_gyro(:,3), STYLE_AXIS3_THIN{:}, 'DisplayName', 'Z');

        grid on; legend('show');
        ylabel('Angular Speed [deg/s]');
        title('Raw Angular Speed (Gyroscope)');
        xlabel('Time (s)');
        add_standard_background(vis_flight_intervals, vis_flight_names, ...
            vis_is_vtol, vis_vtol_intervals, vis_vtol_names);
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
        figure('Name', 'Raw Magnetic Field', 'Color', 'w'); hold on;
        plot(mag_t, mag_data(:,1), STYLE_AXIS1{:}, 'DisplayName', 'X');
        plot(mag_t, mag_data(:,2), STYLE_AXIS2{:}, 'DisplayName', 'Y');
        plot(mag_t, mag_data(:,3), STYLE_AXIS3{:}, 'DisplayName', 'Z');

        grid on; legend('show');
        ylabel('Magnetic Field [Gauss]');
        title('Raw Magnetic Field Strength');
        xlabel('Time (s)');
        add_standard_background(vis_flight_intervals, vis_flight_names, ...
            vis_is_vtol, vis_vtol_intervals, vis_vtol_names);
        % PlotToFile(gcf, 'results/Raw_Magnetic.png', 20, 10);
    end

    % ---------------------------------------------------------------------
    % Distance Sensor
    % ---------------------------------------------------------------------
    if exist('dist_val', 'var') || exist('dist_bottom', 'var')
        figure('Name', 'Distance Sensor', 'Color', 'w'); hold on;

        if exist('dist_val', 'var')
            plot(dist_sensor_t, dist_val, STYLE_RES{:}, 'DisplayName', 'Distance');
        end
        if exist('dist_bottom', 'var')
            plot(dist_bottom_t, dist_bottom, STYLE_SP{:}, 'DisplayName', 'Est. Dist Bottom');
        end

        grid on; legend('show');
        ylabel('Distance [m]');
        title('Distance Sensor');
        xlabel('Time (s)');
        add_standard_background(vis_flight_intervals, vis_flight_names, ...
            vis_is_vtol, vis_vtol_intervals, vis_vtol_names);
        % PlotToFile(gcf, 'results/distance_sensor.png', 20, 10);
    end

    % ---------------------------------------------------------------------
    % GPS Uncertainty
    % ---------------------------------------------------------------------
    if exist('gps_info', 'var')
        figure('Name', 'GPS Uncertainty', 'Color', 'w'); hold on;

        plot(gps_t, gps_info.eph, STYLE_AXIS1{:}, 'DisplayName', 'H Pos Accuracy (EPH) [m]');
        plot(gps_t, gps_info.epv, STYLE_AXIS2{:}, 'DisplayName', 'V Pos Accuracy (EPV) [m]');

        if ismember('hdop', gps_info.Properties.VariableNames)
            plot(gps_t, gps_info.hdop, STYLE_AXIS1_DASH{:}, 'DisplayName', 'HDOP [m]');
            plot(gps_t, gps_info.vdop, STYLE_AXIS2_DASH{:}, 'DisplayName', 'VDOP [m]');
        end

        plot(gps_t, gps_info.s_variance, STYLE_AXIS4_DOT{:}, 'DisplayName', 'Speed Accuracy [m/s]');
        plot(gps_t, gps_info.satellites, STYLE_AXIS5_BOLD{:}, 'DisplayName', 'Satellites Used');
        plot(gps_t, gps_info.fix_type, STYLE_AXIS3_BOLD{:}, 'DisplayName', 'Fix Type');

        grid on;
        legend('show', 'Location', 'best', 'NumColumns', 2);
        ylabel('Value');
        title('GPS Uncertainty & Status');
        xlabel('Time (s)');
        ylim([0, 40]);
        add_standard_background(vis_flight_intervals, vis_flight_names, ...
            vis_is_vtol, vis_vtol_intervals, vis_vtol_names);
        % PlotToFile(gcf, 'results/GPS_Uncertainty.png', 20, 10);
    end

    % ---------------------------------------------------------------------
    % GPS Noise & Jamming
    % ---------------------------------------------------------------------
    if exist('gps_info', 'var')
        figure('Name', 'GPS Noise & Jamming', 'Color', 'w'); hold on;
        plot(gps_t, gps_info.noise,   STYLE_AXIS2{:}, 'DisplayName', 'Noise per ms');
        plot(gps_t, gps_info.jamming, STYLE_AXIS1{:}, 'DisplayName', 'Jamming Indicator');

        grid on; legend('show');
        ylabel('Value');
        title('GPS Noise & Jamming');
        xlabel('Time (s)');
        add_standard_background(vis_flight_intervals, vis_flight_names, ...
            vis_is_vtol, vis_vtol_intervals, vis_vtol_names);
        % PlotToFile(gcf, 'results/GPS_Noise_Jamming.png', 20, 10);
    end

    % ---------------------------------------------------------------------
    % Thrust and Magnetic Field
    % ---------------------------------------------------------------------
    figure('Name', 'Thrust and Magnetic Field', 'Color', 'w');
    mag_mag = sqrt(mag_data(:,1).^2 + mag_data(:,2).^2 + mag_data(:,3).^2);

    hold on;
    plot(mag_t, mag_mag, STYLE_AXIS1{:}, 'DisplayName', 'Magnetic Field Norm');

    if ~isempty(actuator_controls_0.thrust)
        plot(actuator_controls_0.time_thrust, actuator_controls_0.thrust, ...
            STYLE_THRUST{:}, 'DisplayName', 'Thrust');
    end

    if log.data.vehicle_status_0.is_vtol(1) && ~dynamic_control_alloc && ~isempty(thrust_sp_1)
        plot(actuator_controls_1.time, actuator_controls_1.thrust_x, ...
            STYLE_THRUST_ALT{:}, 'DisplayName', 'Thrust (Fixed-wing)');
    end

    add_standard_background(vis_flight_intervals, vis_flight_names, ...
        vis_is_vtol, vis_vtol_intervals, vis_vtol_names);

    grid on;
    xlabel('Time (s)');
    ylabel('Magnitude');
    title('Thrust and Magnetic Field');
    legend('show', 'Location', 'best');
    % PlotToFile(gcf, 'results/Thrust_Magnetic_Field.png', 20, 10);

    % ---------------------------------------------------------------------
    % Power (Battery & System)
    % ---------------------------------------------------------------------
    if exist('bat_v', 'var')
        figure('Name', 'Power', 'Color', 'w');

        ax1 = subplot(2,1,1); hold on;
        yyaxis left
        plot(bat_t, bat_v, STYLE_AXIS2_BOLD{:}, 'DisplayName', 'Voltage [V]');
        ylabel('Voltage [V]');

        yyaxis right
        plot(bat_t, bat_i, STYLE_AXIS1{:}, 'DisplayName', 'Current [A]');
        ylabel('Current [A]');

        title('Battery Voltage & Current');
        grid on;
        legend('show', 'Location', 'best');
        add_standard_background(vis_flight_intervals, vis_flight_names, ...
            vis_is_vtol, vis_vtol_intervals, vis_vtol_names);

        ax2 = subplot(2,1,2); hold on;
        yyaxis left
        plot(bat_t, bat_discharged/100, STYLE_AXIS4{:}, 'DisplayName', 'Discharged [mAh/100]');
        ylabel('Discharged [mAh/100]');

        yyaxis right
        plot(bat_t, bat_remaining*100, STYLE_AXIS3_BOLD{:}, 'DisplayName', 'Remaining [\%]');
        ylabel('Remaining [\%]');
        ylim([0, 105]);

        xlabel('Time (s)');
        grid on;
        legend('show', 'Location', 'best');
        linkaxes([ax1, ax2], 'x');
        add_standard_background(vis_flight_intervals, vis_flight_names, ...
            vis_is_vtol, vis_vtol_intervals, vis_vtol_names);
        % PlotToFile(gcf, 'results/Battery_System.png', 20, 20);
    end

    % ---------------------------------------------------------------------
    % Temperature
    % ---------------------------------------------------------------------
    if exist('temp_data', 'var') && ~isempty(temp_data)
        figure('Name', 'Temperature', 'Color', 'w'); hold on;
        colors = make_channel_colors(length(temp_data), sty);

        for i = 1:length(temp_data)
            plot(temp_data(i).t, temp_data(i).val, ...
                'LineWidth', LW_MULTI_BOLD, ...
                'Color', colors(i,:), ...
                'DisplayName', temp_data(i).name);
        end

        grid on; legend('show', 'Location', 'best');
        ylabel('Temperature [$^\circ$C]');
        title('System Temperatures');
        xlabel('Time (s)');
        add_standard_background(vis_flight_intervals, vis_flight_names, ...
            vis_is_vtol, vis_vtol_intervals, vis_vtol_names);
        % PlotToFile(gcf, 'results/Temperature.png', 20, 10);
    end

    % ---------------------------------------------------------------------
    % Estimator Flags
    % ---------------------------------------------------------------------
    if isfield(log.data, 'estimator_status_0')
        figure('Name', 'Estimator Flags (Dynamic)', 'Color', 'w'); hold on;

        plot_count = 0;
        max_plots = 8;
        colors = make_channel_colors(max_plots, sty);
        active_legends = {};

        for i = 1:size(candidates, 1)
            lbl = candidates{i, 1};
            data = candidates{i, 2};

            if max(data) > 0.1
                plot_count = plot_count + 1;
                plot(est_t, data, 'Color', colors(plot_count,:), 'LineWidth', LW_MULTI_BOLD);
                active_legends{end+1} = lbl;

                if plot_count >= max_plots
                    break;
                end
            end
        end

        if plot_count == 0
            plot(est_t, candidates{1,2}, STYLE_AXIS4{:});
            active_legends{end+1} = candidates{1,1};
            title('Estimator Flags (All Good)');
        else
            title(sprintf('Estimator Flags (Top %d Active)', plot_count));
        end

        grid on;
        legend(active_legends, 'Location', 'best');
        ylabel('Flag Value');
        xlabel('Time (s)');
        add_standard_background(vis_flight_intervals, vis_flight_names, ...
            vis_is_vtol, vis_vtol_intervals, vis_vtol_names);
        % PlotToFile(gcf, 'results/Estimator_Flags.png', 20, 10);
    end

    % ---------------------------------------------------------------------
    % Failsafe Flags
    % ---------------------------------------------------------------------
    if isfield(log.data, 'failsafe_flags_0')
        figure('Name', 'Failsafe Flags', 'Color', 'w'); hold on;

        if exist('vs_failsafe', 'var')
            area(vs_t, double(vs_failsafe), 'FaceColor', COLOR_WARN_FILL, ...
                'EdgeColor', 'none', 'DisplayName', 'In Failsafe Mode');
        end

        plot_idx = 0;
        colors = make_channel_colors(10, sty);

        for i = 1:length(fs_cols)
            col_name = fs_cols{i};
            if strcmp(col_name, 'timestamp') || startsWith(col_name, 'mode_req')
                continue;
            end

            data = fs_table.(col_name);
            if max(data) > 0
                plot_idx = plot_idx + 1;
                plot(fs_t, double(data) * 0.8 + plot_idx, ...
                    'LineWidth', LW_MULTI_BOLD, ...
                    'Color', colors(mod(plot_idx-1,10)+1,:), ...
                    'DisplayName', strrep(col_name, '_', ' '));
            end
        end

        if plot_idx == 0
            text(mean(fs_t), 0.5, 'No Failsafe Flags Triggered', 'HorizontalAlignment', 'center');
        end

        grid on; legend('show');
        title('Failsafe Flags Triggered');
        ylabel('Flags (Stacked)');
        xlabel('Time (s)');
        ylim([0, plot_idx + 2]);
        add_standard_background(vis_flight_intervals, vis_flight_names, ...
            vis_is_vtol, vis_vtol_intervals, vis_vtol_names);
        % PlotToFile(gcf, 'results/Failsafe_Flags.png', 20, 10);
    end

    % ---------------------------------------------------------------------
    % CPU & RAM
    % ---------------------------------------------------------------------
    if isfield(log.data, 'cpuload_0')
        figure('Name', 'CPU & RAM', 'Color', 'w'); hold on;
        plot(cpu_t, ram_usage, STYLE_AXIS2_BOLD{:}, 'DisplayName', 'RAM Usage');
        plot(cpu_t, cpu_load,  STYLE_AXIS1_BOLD{:}, 'DisplayName', 'CPU Load');

        grid on;
        legend('show', 'Location', 'best');
        ylabel('Load / Usage [0-1]');
        title('CPU & RAM');
        xlabel('Time (s)');
        ylim([0, 1]);
        add_standard_background(vis_flight_intervals, vis_flight_names, ...
            vis_is_vtol, vis_vtol_intervals, vis_vtol_names);
        % PlotToFile(gcf, 'results/CPU.png', 20, 10);
    end

    % ---------------------------------------------------------------------
    % Sampling Regularity
    % ---------------------------------------------------------------------
    if isfield(log.data, 'sensor_combined_0')
        figure('Name', 'Sampling Regularity', 'Color', 'w'); hold on;

        sc = log.data.sensor_combined_0;
        t_raw = double(sc.timestamp);
        dt_seq = diff(t_raw);
        t_plot_sc = t_raw(2:end) * 1e-6;

        plot(t_plot_sc, dt_seq, STYLE_AXIS3_THIN{:}, ...
            'DisplayName', 'delta t (between 2 logged samples)');

        if isfield(log.data, 'estimator_status_0')
            es = log.data.estimator_status_0;
            t_es = double(es.timestamp) * 1e-6;
            slip_us = double(es.time_slip) * 1e6;

            plot(t_es, slip_us, STYLE_AXIS4_BOLD{:}, ...
                'DisplayName', 'Estimator time slip (cumulative)');
        end

        grid on;
        legend('show', 'Location', 'best');
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

%% =========================================================================
%  Local functions
% =========================================================================
function cols = make_channel_colors(n, sty)
    base = [sty.color.setpoint2;
        sty.color.response2;
        sty.color.setpoint3;
        sty.color.response3;
        sty.color.setpoint4;
        sty.color.response4;
        sty.color.setpoint5;
        sty.color.response5;
        sty.color.setpoint6;
        sty.color.response6];

    if n <= size(base,1)
        cols = base(1:n, :);
    else
        cols = lines(n);
    end
end


function idx_out = subsample_idx(idx_in, step)
    if isempty(idx_in)
        idx_out = idx_in;
        return;
    end
    idx_out = idx_in(1:step:end);
    if idx_out(end) ~= idx_in(end)
        idx_out = [idx_out; idx_in(end)];
    end
end
