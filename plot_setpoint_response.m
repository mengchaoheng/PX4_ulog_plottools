clear all;
close all;
clc;
addpath(genpath(pwd));

run('load_data_main.m'); % load('flight_data.mat');

%% =========================================================================
%  Step 4: 绘图 - Figure 1 2, 3, 4, 5: (Rates Attitude, Vel, Pos, Control)
% =========================================================================
% --- Figure 1: Rates ---
if(exist('vehicle_angular_velocity', 'var') && exist('vehicle_rates_setpoint', 'var'))
    fig1 = figure(1); set(gcf, 'Color', 'w');
    titles = {'Roll Rate', 'Pitch Rate', 'Yaw Rate'};
    ylabels = {'p (deg/s)', 'q (deg/s)', 'r (deg/s)'};
    ax = [];
    for i = 1:3
        ax(i) = subplot(3,1,i); hold on;
        plot(log.data.vehicle_rates_setpoint_0.timestamp*1e-6, vehicle_rates_setpoint(:,i)*r2d, 'k-', 'LineWidth', 1);
        plot(log.data.vehicle_angular_velocity_0.timestamp*1e-6, vehicle_angular_velocity(:,i)*r2d, '--', 'LineWidth', 1, 'Color', [0.8, 0.3, 0]);
        grid on; ylabel(ylabels{i}); title(titles{i});
        if i==1, legend('Setpoint', 'Response', 'Location', 'best'); end
        add_standard_background(vis_flight_intervals, vis_flight_names, vis_is_vtol, vis_vtol_intervals, vis_vtol_names);
    end
    linkaxes(ax, 'x'); xlabel('Time (s)');
end

% --- Figure 2: Attitude ---
if(exist('Roll', 'var') && exist('Roll_setpoint', 'var'))
    fig2 = figure(2); set(gcf, 'Color', 'w');
    d_sp = {Roll_setpoint, Pitch_setpoint, Yaw_setpoint};
    d_res = {Roll, Pitch, Yaw};
    titles = {'Roll', 'Pitch', 'Yaw'};
    
    % 【修改点1】使用 LaTeX 语法定义标签
    % \varphi 对应 Roll, \theta 对应 Pitch, \phi 对应 Yaw
    ylabels = {'$\varphi$ (deg)', '$\theta$ (deg)', '$\phi$ (deg)'};
    
    ax = [];
    for i = 1:3
        ax(i) = subplot(3,1,i); hold on;
        plot(log.data.vehicle_attitude_setpoint_0.timestamp*1e-6, d_sp{i}*r2d, 'k-', 'LineWidth', 1);
        plot(log.data.vehicle_attitude_0.timestamp*1e-6, d_res{i}*r2d, '--', 'LineWidth', 1, 'Color', [0.6, 0.2, 0]);
        grid on; ylabel(ylabels{i}, 'Interpreter', 'latex', 'FontSize', 12); title(titles{i}); 
        if i==1, legend('Setpoint','Response'); end
        add_standard_background(vis_flight_intervals, vis_flight_names, vis_is_vtol, vis_vtol_intervals, vis_vtol_names);
    end
    linkaxes(ax, 'x'); xlabel('Time (s)');
end

% --- Figure 3: Velocity ---
if(exist('V_XYZ', 'var'))
    fig3 = figure(3); set(gcf, 'Color', 'w');
    ylabels = {'V_X (m/s)', 'V_Y (m/s)', 'V_Z (m/s)'};
    has_sp = exist('V_XYZ_setpoint', 'var');
    ax = [];
    for i = 1:3
        ax(i) = subplot(3,1,i); hold on;
        if has_sp, plot(log.data.vehicle_local_position_setpoint_0.timestamp*1e-6, V_XYZ_setpoint(:,i), 'k-', 'LineWidth', 1); end
        plot(log.data.vehicle_local_position_0.timestamp*1e-6, V_XYZ(:,i), '--', 'LineWidth', 1, 'Color', [0.6, 0.2, 0]);
        grid on; ylabel(ylabels{i}); if i==1, title('Velocity'); legend('Setpoint','Response'); end
        add_standard_background(vis_flight_intervals, vis_flight_names, vis_is_vtol, vis_vtol_intervals, vis_vtol_names);
    end
    linkaxes(ax, 'x'); xlabel('Time (s)');
end

% --- Figure 4: Position ---
if(exist('XYZ', 'var'))
    fig4 = figure(4); set(gcf, 'Color', 'w');
    ylabels = {'X (m)', 'Y (m)', 'Z (m)'};
    has_sp = exist('XYZ_setpoint', 'var');
    ax = [];
    for i = 1:3
        ax(i) = subplot(3,1,i); hold on;
        if has_sp, plot(log.data.vehicle_local_position_setpoint_0.timestamp*1e-6, XYZ_setpoint(:,i), 'k-', 'LineWidth', 1); end
        plot(log.data.vehicle_local_position_0.timestamp*1e-6, XYZ(:,i), '--', 'LineWidth', 1, 'Color', [0.6, 0.2, 0]);
        grid on; ylabel(ylabels{i}); if i==1, title('Position'); legend('Setpoint','Response'); end
        add_standard_background(vis_flight_intervals, vis_flight_names, vis_is_vtol, vis_vtol_intervals, vis_vtol_names);
    end
    linkaxes(ax, 'x'); xlabel('Time (s)');
end

% --- Figure 5: Controls ---
if(exist('Roll_control', 'var'))
    fig5 = figure(5); set(gcf, 'Color', 'w');
    x_t = log.data.vehicle_torque_setpoint_0.timestamp*1e-6;
    x_th = log.data.vehicle_thrust_setpoint_0.timestamp*1e-6;
    
    ax1 = subplot(5,1,1); hold on;
    plot(x_t, Roll_control(:,1),'r-', 'LineWidth', 1); plot(x_t, Pitch_control(:,1),'k--', 'LineWidth', 1);
    plot(x_t, Yaw_control(:,1),'b-.', 'LineWidth', 1); plot(x_th, thrust_sp(:,1),'k-', 'LineWidth', 1);
    grid on; title('Motors Output [0,1] or [-1,1]'); legend('Roll','Pitch','Yaw','Thrust');
    add_standard_background(vis_flight_intervals, vis_flight_names, vis_is_vtol, vis_vtol_intervals, vis_vtol_names);
    
    d_list = {Roll_control, Pitch_control, Yaw_control, thrust_sp};
    t_list = {x_t, x_t, x_t, x_th};
    ylabs = {'Roll', 'Pitch', 'Yaw', 'Thrust'};
    cols = {'r-', 'k--', 'b-.', 'k-'};
    ax_oth = [];
    for i = 1:4
        ax_oth(i) = subplot(5,1,i+1); hold on;
        plot(t_list{i}, d_list{i}(:,1), cols{i}, 'LineWidth', 1); grid on; ylabel(ylabs{i});
        add_standard_background(vis_flight_intervals, vis_flight_names, vis_is_vtol, vis_vtol_intervals, vis_vtol_names);
    end
    linkaxes([ax1, ax_oth], 'x'); xlabel('Time (s)');
end

%% =========================================================================
%  Step 5: 绘图 - Figure 6/7/8: Actuators & PWM 
% =========================================================================
% 参数设置
MAX_MOTORS = 12; 
MAX_SERVOS = 8;  
plot_together = 0; % 1:合并显示, 0:独立显示

if (plot_together)
    %% 动态绘制电机和舵机 (合并显示版 - 优化颜色)
    % 检查是否存在相关数据表
    if (isfield(log.data, 'actuator_motors_0') || isfield(log.data, 'actuator_servos_0'))
        
        % 1. 获取电机和舵机数量
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
            fig6 = figure(6);
            set(fig6, 'Name', 'Actuator Outputs (Merged)', 'Color', 'w');
            current_plot_idx = 1;
            
            % Part 1: 绘制电机 (所有电机在一张图)
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
            
            % Part 2: 绘制舵机 (所有舵机在一张图)
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
            end
        end
    end
else
    % 独立显示模式  
    %% === 图 6: 电机输出 (Actuator Motors) ===
    if (isfield(log.data, 'actuator_motors_0') && exist('motors', 'var'))
    
        % 1. 获取电机数量
        n_motors = 0;
        if isfield(log, 'params') && isfield(log.params, 'CA_ROTOR_COUNT')
            n_motors = double(log.params.CA_ROTOR_COUNT);
        else
            n_motors = size(motors, 2); 
        end
    
        % 限制最大数量，防止出错
        n_motors = min(n_motors, MAX_MOTORS);
    
        % 2. 绘图
        if n_motors > 0
            fig6 = figure(6);
            set(fig6, 'Name', 'Actuator Motors', 'Color', 'w');
    
            % === 关键点：生成 n_motors 个特定颜色 ===
            % 使用 hsv 颜色空间，可以在 0-1 之间均匀取色，保证颜色互不相同
            % 也可以尝试 'turbo' 或 'jet'
            motor_colors = hsv(n_motors); 
    
            for i = 1:n_motors
                subplot(n_motors, 1, i);
                hold on;
    
                if i <= size(motors, 2)
                    % 取出第 i 种颜色
                    this_color = motor_colors(i, :);
    
                    plot(log.data.actuator_motors_0.timestamp*1e-6, motors(:, i), ...
                         'Color', this_color, 'LineWidth', 1);
                end
    
                grid on;
                ylabel(sprintf('Motor %d', i));
                add_standard_background(vis_flight_intervals, vis_flight_names, vis_is_vtol, vis_vtol_intervals, vis_vtol_names);
                % 仅第一张图显示标题
                if i == 1
                    title(sprintf('Actuator Motors (Total: %d)', n_motors));
                end
    
                % 仅最后一张图显示时间轴
                if i == n_motors
                    xlabel('Time (s)');
                else
                    set(gca, 'XTickLabel', []);
                end
    
                % 调整坐标轴范围让波形更清楚 (可选)
                % axis tight; 
            end
    
            % 自动调整布局（如果Matlab版本支持）
            % sgtitle('Motors Output');
        end
    end
    
    %% === 图 7: 舵机输出 (Actuator Servos) ===
    if (isfield(log.data, 'actuator_servos_0') && exist('servos', 'var'))
    
        % 1. 获取舵机数量
        n_servos = 0;
        if isfield(log, 'params') && isfield(log.params, 'CA_SV_CS_COUNT')
            n_servos = double(log.params.CA_SV_CS_COUNT);
        else
            n_servos = size(servos, 2);
        end
    
        % 限制最大数量
        n_servos = min(n_servos, MAX_SERVOS);
    
        % 2. 绘图
        if n_servos > 0
            fig7 = figure(7);
            set(fig7, 'Name', 'Actuator Servos', 'Color', 'w');
    
            % === 关键点：生成 n_servos 个特定颜色 ===
            % 为了和电机颜色区分开，我们可以把 HSV 的起始相位偏移一下，或者倒序使用
            % 这里使用 'lines' 色图，它对比度较高，适合数量较少(<=8)的情况
            if n_servos <= 7
                servo_colors = lines(n_servos);
            else
                servo_colors = hsv(n_servos); % 超过7个用hsv保证不重复
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
        end
    end
end

%% --------------------PWM (Figure 8)----------------------------
n_active = length(active_channels);
if n_active > 0
    fig8 = figure(8);
    set(fig8, 'Name', 'Actuator Outputs (PWM)', 'Color', 'w');
    
    t_pwm = log.data.actuator_outputs_0.timestamp * 1e-6;
    
    motors_list = active_channels(strcmp({active_channels.type}, 'Motor'));
    others_list = active_channels(strcmp({active_channels.type}, 'Servo/Other'));
    
    has_motors = ~isempty(motors_list);
    n_others = length(others_list);
    total_subplots = double(has_motors) + n_others;
    
    current_plot = 1;
    ax_pwm = [];
    
    % --- 子图 1: 所有电机 ---
    if has_motors
        ax_pwm(current_plot) = subplot(total_subplots, 1, current_plot);
        hold on;
        colors_motor = hsv(length(motors_list));
        
        for k = 1:length(motors_list)
            chn = motors_list(k);
            y_data = log.data.actuator_outputs_0.(chn.col_name);
            plot(t_pwm, y_data, 'Color', colors_motor(k,:), 'LineWidth', 1, 'DisplayName', sprintf('%s (Ch%d)', chn.name, chn.idx));
        end
        grid on; ylabel('PWM (us)'); title('Motors PWM Output'); legend('show');
        add_standard_background(vis_flight_intervals, vis_flight_names, vis_is_vtol, vis_vtol_intervals, vis_vtol_names);
        
        if current_plot < total_subplots, set(gca, 'XTickLabel', []); else, xlabel('Time (s)'); end
        current_plot = current_plot + 1;
    end
    
    % --- 子图 2~N: 其他通道 ---
    if n_others > 0
        if n_others <= 7, colors_others = lines(n_others); else, colors_others = hsv(n_others); end
    end
    for k = 1:n_others
        chn = others_list(k);
        ax_pwm(current_plot) = subplot(total_subplots, 1, current_plot);
        hold on;
        y_data = log.data.actuator_outputs_0.(chn.col_name);
        this_color = colors_others(k, :);
        
        plot(t_pwm, y_data, 'Color', this_color, 'LineWidth', 1);
        grid on; ylabel('PWM (us)');
        title(sprintf('%s (Output Ch%d)', chn.name, chn.idx));
        add_standard_background(vis_flight_intervals, vis_flight_names, vis_is_vtol, vis_vtol_intervals, vis_vtol_names);
        
        if current_plot < total_subplots, set(gca, 'XTickLabel', []); else, xlabel('Time (s)'); end
        current_plot = current_plot + 1;
    end
    linkaxes(ax_pwm, 'x');
else
    fprintf('没有找到启用的 PWM 输出通道。\n');
end

%% =========================================================================
%  Step 6: Trajectory & Angular Accel
% =========================================================================
if(exist('XYZ', 'var') && exist('XYZ_setpoint', 'var'))
    fig9=figure(9); set(gcf, 'Color', 'w');
    plot3(XYZ_setpoint(:,1), XYZ_setpoint(:,2), -XYZ_setpoint(:,3), '-', 'LineWidth', 1); hold on;
    plot3(XYZ(:,1), XYZ(:,2), -XYZ(:,3), ':', 'LineWidth', 1);
    title('Trajectory'); xlabel('X'); ylabel('Y'); zlabel('Z'); grid on; view(45, 30);
end

if exist('vehicle_angular_acceleration', 'var') || isfield(log.data, 'vehicle_angular_velocity_0') && ...
       ismember('xyz_derivative_0_', log.data.vehicle_angular_velocity_0.Properties.VariableNames)
    fig10 = figure(10); set(gcf, 'Color', 'w');
    titles = {'Roll Acc', 'Pitch Acc', 'Yaw Acc'};
    ax = [];
    for i = 1:3
        ax(i) = subplot(3,1,i); hold on;
        plot(log.data.vehicle_angular_velocity_0.timestamp*1e-6, vehicle_angular_acceleration(:,i), 'k-', 'LineWidth', 1);
        plot(log.data.vehicle_angular_velocity_0.timestamp*1e-6, vehicle_angular_velocity(:,i), '--', 'LineWidth', 1, 'Color', [0.6, 0.2, 0, 0.5]);
        grid on; ylabel('rad/s^2'); title(titles{i});
        add_standard_background(vis_flight_intervals, vis_flight_names, vis_is_vtol, vis_vtol_intervals, vis_vtol_names);
    end
    linkaxes(ax, 'x'); xlabel('Time (s)');
end
