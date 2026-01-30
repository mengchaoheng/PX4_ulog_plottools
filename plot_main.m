clear all;
close all;
clc;
addpath(genpath(pwd));

%% =========================================================================
%  Step 0: 文件选择 & 基础设置
% =========================================================================
d2r = pi/180;
r2d = 180/pi;

% --- 用户配置区域 ---------------------------------------------------------
% 在这里指定文件名（可以是相对路径 'data/09_49_18' 或绝对路径）
% 【关键】：如果这里留空 (即 specifiedFileName = '';)，脚本运行时会自动弹窗让您选择。
specifiedFileName = 'data/09_49_18'; 
% -------------------------------------------------------------------------

if isempty(specifiedFileName)
    % 情况 A: 变量为空 -> 弹窗选择
    [fileName, pathName] = uigetfile('*.ulg', '请选择要分析的 ULog 文件');
    if isequal(fileName, 0)
        disp('用户取消了文件选择');
        return;
    end
    fullPath = fullfile(pathName, fileName);
else
    % 情况 B: 变量不为空 -> 使用指定文件
    % 1. 自动补充 .ulg 后缀（如果用户没写）
    if ~endsWith(specifiedFileName, '.ulg')
        fullPath = [specifiedFileName '.ulg'];
    else
        fullPath = specifiedFileName;
    end
    
    % 2. 检查文件是否存在
    if ~exist(fullPath, 'file')
        error('❌ 找不到指定文件: %s \n请检查路径是否正确，或将 specifiedFileName 设为空以使用弹窗选择。', fullPath);
    end
    
    % 3. 获取绝对路径（为了后续工具调用稳定）
    if java.io.File(fullPath).isAbsolute()
        % 已经是绝对路径
    else
        % 相对路径转绝对路径
        fullPath = fullfile(pwd, fullPath);
    end
end

% --- 统一处理路径信息 (供后续步骤使用) ---
[pathName, nameNoExt, ~] = fileparts(fullPath);

% ulgFileName: 包含路径但不带后缀的文件名 (后续生成 .mat 和读取 .csv 需要这个格式)
ulgFileName = fullfile(pathName, nameNoExt);

% 定义缓存 .mat 文件名
tmp = [ulgFileName '.mat'];
rootDir = fileparts(mfilename('fullpath'));

disp(['分析目标: ' nameNoExt '.ulg']);

%% =========================================================================
%  Step 1: 检查或转换数据
% =========================================================================
if exist(tmp, "file")
    disp(['Found MAT file: ' tmp]);
    load(tmp, 'log');
else
    disp('No MAT file found, start parsing ULog...');
    
    % 定义工具路径 (请根据实际情况调整)
    if ismac
        base_path = '~/Library/Python/3.9/bin/'; 
        cmd_info = [base_path 'ulog_info'];
        cmd_msgs = [base_path 'ulog_messages'];
        cmd_params = [base_path 'ulog_params'];
        cmd_ulog2csv = [base_path 'ulog2csv'];
    else
        cmd_info = 'ulog_info';
        cmd_msgs = 'ulog_messages';
        cmd_params = 'ulog_params';
        cmd_ulog2csv = 'ulog2csv';
    end

    ulgAbs = fullfile(pathName, fileName);
    
    % 1. 运行 ulog2csv
    command = [cmd_ulog2csv ' ' '"' ulgAbs '"'];
    disp(['Running: ' command]);
    [status, cmdout] = system(command);
    if status ~= 0
        disp(cmdout);
        error('ulog2csv conversion failed.');
    end
    
    % 2. 读取 CSV 数据
    log.data = csv_topics_to_d(ulgFileName); 
    log.FileName = ulgFileName;
    log.version = 1.0;
    
    % 3. 解析 Info/Messages/Params
    [~, log.info] = system([cmd_info ' "' ulgAbs '"']);
    [~, log.messages] = system([cmd_msgs ' "' ulgAbs '"']);
    
    % 解析 Params 为结构体
    [s_p, out_p] = system([cmd_params ' "' ulgAbs '"']);
    log.params = struct();
    if s_p == 0
        lines = splitlines(out_p);
        for i = 1:length(lines)
            parts = strsplit(strtrim(lines{i}), ',');
            if length(parts) >= 2
                p_name = strtrim(parts{1});
                p_val = str2double(strtrim(parts{2}));
                if ~isnan(p_val), log.params.(p_name) = p_val; end
            end
        end
    end

    % 4. 保存 MAT 并清理 CSV
    save(tmp, 'log');
    delete([ulgFileName '_*.csv']);
    disp('Conversion done & Temp CSV deleted.');
end

disp(log.info);
disp(log.messages);
%% =========================================================================
%  Step 2: 数据预处理 (计算物理量 & PWM解析)
% =========================================================================
% 2.1 角速度
if(isfield(log.data, 'vehicle_angular_velocity_0'))
    vehicle_angular_velocity = [log.data.vehicle_angular_velocity_0.xyz_0_, ...
                                log.data.vehicle_angular_velocity_0.xyz_1_, ...
                                log.data.vehicle_angular_velocity_0.xyz_2_];
end

% 2.2 角加速度
if(isfield(log.data, 'vehicle_angular_acceleration_0'))
    vehicle_angular_acceleration = [log.data.vehicle_angular_acceleration_0.xyz_0_, ...
                                    log.data.vehicle_angular_acceleration_0.xyz_1_, ...
                                    log.data.vehicle_angular_acceleration_0.xyz_2_];
elseif exist('vehicle_angular_velocity', 'var') && isfield(log.data.vehicle_angular_velocity_0, 'xyz_derivative_0_')
    vehicle_angular_acceleration = [log.data.vehicle_angular_velocity_0.xyz_derivative_0_, ...
                                    log.data.vehicle_angular_velocity_0.xyz_derivative_1_, ...
                                    log.data.vehicle_angular_velocity_0.xyz_derivative_2_];
end 

% 2.3 Setpoints
if(isfield(log.data, 'vehicle_rates_setpoint_0'))
    vehicle_rates_setpoint = [log.data.vehicle_rates_setpoint_0.roll, ...
                              log.data.vehicle_rates_setpoint_0.pitch, ...
                              log.data.vehicle_rates_setpoint_0.yaw];
end 

% 2.4 Attitude
if(isfield(log.data, 'vehicle_attitude_0'))
    q = [log.data.vehicle_attitude_0.q_0_, log.data.vehicle_attitude_0.q_1_, log.data.vehicle_attitude_0.q_2_, log.data.vehicle_attitude_0.q_3_];
    eul = quat2eul(q);
    Roll = eul(:,3); Pitch = eul(:,2); Yaw = eul(:,1);
end
if(isfield(log.data, 'vehicle_attitude_setpoint_0'))
    q_d = [log.data.vehicle_attitude_setpoint_0.q_d_0_, log.data.vehicle_attitude_setpoint_0.q_d_1_, log.data.vehicle_attitude_setpoint_0.q_d_2_, log.data.vehicle_attitude_setpoint_0.q_d_3_];
    eul_d = quat2eul(q_d);
    Roll_setpoint = eul_d(:,3); Pitch_setpoint = eul_d(:,2); Yaw_setpoint = eul_d(:,1);
end

% 2.5 Position / Velocity
if(isfield(log.data, 'vehicle_local_position_0'))
    XYZ = [log.data.vehicle_local_position_0.x, log.data.vehicle_local_position_0.y, log.data.vehicle_local_position_0.z];
    V_XYZ = [log.data.vehicle_local_position_0.vx, log.data.vehicle_local_position_0.vy, log.data.vehicle_local_position_0.vz];
end
if(isfield(log.data, 'vehicle_local_position_setpoint_0'))
    XYZ_setpoint = [log.data.vehicle_local_position_setpoint_0.x, log.data.vehicle_local_position_setpoint_0.y, log.data.vehicle_local_position_setpoint_0.z];
    V_XYZ_setpoint = [log.data.vehicle_local_position_setpoint_0.vx, log.data.vehicle_local_position_setpoint_0.vy, log.data.vehicle_local_position_setpoint_0.vz];
end

% 2.6 Control Output
if(isfield(log.data, 'vehicle_torque_setpoint_0'))
    Roll_control = log.data.vehicle_torque_setpoint_0.xyz_0_;
    Pitch_control = log.data.vehicle_torque_setpoint_0.xyz_1_;
    Yaw_control = log.data.vehicle_torque_setpoint_0.xyz_2_;
end
if(isfield(log.data, 'vehicle_thrust_setpoint_0'))
    thrust_sp = log.data.vehicle_thrust_setpoint_0.xyz_2_;
end

% 2.7 Actuator Output Extraction
motors = []; servos = [];
if isfield(log.data, 'actuator_motors_0')
    % 动态寻找 control_x_ 列
    mc_cols = startsWith(log.data.actuator_motors_0.Properties.VariableNames, 'control_');
    motors = log.data.actuator_motors_0{:, mc_cols};
end
if isfield(log.data, 'actuator_servos_0')
    sc_cols = startsWith(log.data.actuator_servos_0.Properties.VariableNames, 'control_');
    servos = log.data.actuator_servos_0{:, sc_cols};
end

% 2.8 PWM 通道解析 (为 Figure 8 做准备)
active_channels = struct('idx', {}, 'code', {}, 'name', {}, 'col_name', {}, 'type', {});
if isfield(log.data, 'actuator_outputs_0') && isfield(log, 'params')
    for i = 1:16
        param_name = sprintf('PWM_MAIN_FUNC%d', i);
        if isfield(log.params, param_name)
            code = double(log.params.(param_name));
            if code ~= 0
                col_name = sprintf('output_%d_', i-1);
                if ismember(col_name, log.data.actuator_outputs_0.Properties.VariableNames)
                    new_idx = length(active_channels) + 1;
                    active_channels(new_idx).idx = i;
                    active_channels(new_idx).code = code;
                    active_channels(new_idx).name = lookup_pwm_func(code);
                    active_channels(new_idx).col_name = col_name;
                    if code >= 101 && code <= 199
                        active_channels(new_idx).type = 'Motor';
                    else
                        active_channels(new_idx).type = 'Servo/Other';
                    end
                end
            end
        end
    end
end

%% =========================================================================
%  Step 3: 准备可视化状态数据 (飞行模式 & VTOL状态)
% =========================================================================
vis_flight_intervals = [];
vis_flight_names = {};
vis_vtol_intervals = [];
vis_vtol_names = {};
vis_is_vtol = false;

if isfield(log.data, 'vehicle_status_0')
    % 1. 解析飞行模式 (无缝隙版)
    [vis_flight_intervals, vis_flight_names] = get_flight_mode_intervals(log.data.vehicle_status_0);
    
    % 2. 解析 VTOL 状态 (无缝隙版)
    if ismember('is_vtol', log.data.vehicle_status_0.Properties.VariableNames) && ...
       max(log.data.vehicle_status_0.is_vtol) == 1
        vis_is_vtol = true;
        [vis_vtol_intervals, vis_vtol_names] = get_vtol_state_intervals(log.data.vehicle_status_0);
    end
end

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
plot_together = 1; % 1:合并显示, 0:独立显示

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

if exist('vehicle_angular_acceleration', 'var')
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


%% =========================================================================
%  Helper Functions (放在脚本末尾)
% =========================================================================
function add_standard_background(vis_flight_intervals, vis_flight_names, vis_is_vtol, vis_vtol_intervals, vis_vtol_names)
    yl = ylim; y_h = yl(2) - yl(1);
    draw_status_band(vis_flight_intervals, vis_flight_names, yl, 'flight_mode');
    if vis_is_vtol
        draw_status_band(vis_vtol_intervals, vis_vtol_names, [yl(1), yl(1)+0.08*y_h], 'vtol_state');
    end
end

function draw_status_band(intervals, labels, y_range, color_mode)
    if isempty(intervals), return; end
    colors = containers.Map('KeyType', 'double', 'ValueType', 'any');
    default_c = [0.95, 0.95, 0.95];
    
    if strcmp(color_mode, 'flight_mode')
        colors(0)=[0.9,0.9,0.9]; colors(1)=[0.85,0.95,1]; colors(2)=[0.8,0.9,1]; 
        colors(3)=[0.85,1,0.85]; colors(4)=[0.9,0.9,0.8]; colors(5)=[1,0.9,0.9];
        colors(14)=[1,0.95,0.8]; colors(15)=[0.8,0.95,1];
    elseif strcmp(color_mode, 'vtol_state')
        colors(1)=[0.6,0.8,1]; colors(2)=[0.6,1,0.6]; colors(3)=[1,0.7,0.4];
    end
    
    hold on; y_b = y_range(1); y_t = y_range(2);
    for i = 1:size(intervals, 1)
        t_s = intervals(i, 1); t_e = intervals(i, 2); val = intervals(i, 3);
        if isKey(colors, val), c = colors(val); else, c = default_c; end
        p = patch([t_s t_e t_e t_s], [y_b y_b y_t y_t], c);
        set(p, 'EdgeColor', 'none', 'FaceAlpha', 0.5, 'HandleVisibility', 'off');
        if (t_e - t_s) > 2.0 && (y_t-y_b) > 0.1*(ylim*([0;-1]+[0;1])) % 简单防遮挡
            text(t_s + (t_e-t_s)/2, y_b + (y_t-y_b)/2, labels{i}, 'HorizontalAlignment', 'center', ...
                'FontSize', 8, 'Color', [0.2 0.2 0.2], 'Interpreter', 'none', 'Clipping', 'on');
        end
    end
    set(gca, 'Layer', 'top');
end

function [intervals, mode_names] = get_flight_mode_intervals(status_topic)
    if isempty(status_topic), intervals=[]; mode_names={}; return; end
    t = double(status_topic.timestamp)*1e-6; nav_state = status_topic.nav_state;
    map = containers.Map('KeyType', 'double', 'ValueType', 'char');
    map(0)='Manual'; map(1)='Altitude'; map(2)='Position'; map(3)='Mission';
    map(4)='Loiter'; map(5)='RTL'; map(14)='Offboard'; map(15)='Stabilized';
    map(17)='Takeoff'; map(18)='Land'; map(22)='VTOL Takeoff';
    
    changes = find([1; diff(nav_state) ~= 0]);
    intervals = []; mode_names = {};
    for i = 1:length(changes)
        idx_s = changes(i);
        if i < length(changes), idx_e = changes(i+1); else, idx_e = length(nav_state); end
        val = double(nav_state(idx_s));
        intervals = [intervals; t(idx_s), t(idx_e), val];
        if isKey(map, val), mode_names{end+1} = map(val); else, mode_names{end+1} = sprintf('Mode %d', val); end
    end
end

function [intervals, state_names] = get_vtol_state_intervals(status_topic)
    if isempty(status_topic), intervals=[]; state_names={}; return; end
    t = double(status_topic.timestamp)*1e-6;
    if ismember('vehicle_type', status_topic.Properties.VariableNames)
        v_type = double(status_topic.vehicle_type);
    elseif ismember('is_rotary_wing', status_topic.Properties.VariableNames)
        v_type = ones(size(t)); v_type(status_topic.is_rotary_wing==0)=2;
    else, intervals=[]; state_names={}; return; end
    
    combined = v_type;
    if ismember('in_transition_mode', status_topic.Properties.VariableNames)
        combined(status_topic.in_transition_mode > 0) = 3; 
    end
    
    changes = find([1; diff(combined) ~= 0]);
    intervals = []; state_names = {};
    map = containers.Map('KeyType', 'double', 'ValueType', 'char');
    map(1)='MC'; map(2)='FW'; map(3)='Trans';
    
    for i = 1:length(changes)
        idx_s = changes(i);
        if i < length(changes), idx_e = changes(i+1); else, idx_e = length(combined); end
        val = combined(idx_s);
        intervals = [intervals; t(idx_s), t(idx_e), val];
        if isKey(map, val), state_names{end+1} = map(val); else, state_names{end+1} = 'Unk'; end
    end
end

function name = lookup_pwm_func(code)
    if code == 0, name = 'Disabled';
    elseif code >= 101 && code <= 112, name = sprintf('Motor %d', code-100);
    elseif code >= 201 && code <= 208, name = sprintf('Servo %d', code-200);
    elseif code == 400, name = 'Landing Gear';
    elseif code == 420, name = 'Gimbal Roll'; elseif code == 421, name = 'Gimbal Pitch'; elseif code == 422, name = 'Gimbal Yaw';
    else, name = sprintf('Func %d', code); end
end