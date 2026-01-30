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
specifiedFileName = 'data/20_00_18'; % 支持带后缀，也支持不带

if isempty(specifiedFileName)
    [fileName, pathName] = uigetfile('*.ulg', '请选择要分析的 ULog 文件');
    if isequal(fileName, 0), disp('取消选择'); return; end
    fullPath = fullfile(pathName, fileName);
else
    % 1. 智能补全后缀
    if ~endsWith(specifiedFileName, '.ulg'), specifiedFileName = [specifiedFileName '.ulg']; end
    % 2. 转为绝对路径 (修复外部工具调用问题)
    if java.io.File(specifiedFileName).isAbsolute()
        fullPath = specifiedFileName;
    else
        fullPath = fullfile(pwd, specifiedFileName);
    end
    % 3. 检查是否存在
    if ~exist(fullPath, 'file'), error(['文件不存在: ' fullPath]); end
end

% --- 统一提取标准变量 ---
[pathName, nameNoExt, ext] = fileparts(fullPath);
fileName = [nameNoExt, ext];
ulgFileName = fullfile(pathName, nameNoExt); 
tmp = [ulgFileName '.mat'];

disp(['分析目标fileName: ' fileName]);
disp(['ulgFileName: ' ulgFileName]);
disp(['tmp: ' tmp]);
disp(['pathName: ' pathName]);
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
    rate_N=size(log.data.vehicle_angular_velocity_0.timestamp,1);
    rate_delta_t=zeros(rate_N-1,1);
    for i=1:rate_N-1
        rate_delta_t(i)=(log.data.vehicle_angular_velocity_0.timestamp(i+1)-log.data.vehicle_angular_velocity_0.timestamp(i))*1e-6;
    end
    fprintf('角速度采样周期: %f (ms)， 频率: %f （Hz） \n', 1000*mean(rate_delta_t), 1/mean(rate_delta_t));
end

% 2.2 角加速度
if(isfield(log.data, 'vehicle_angular_acceleration_0'))
    vehicle_angular_acceleration = [log.data.vehicle_angular_acceleration_0.xyz_0_, ...
                                    log.data.vehicle_angular_acceleration_0.xyz_1_, ...
                                    log.data.vehicle_angular_acceleration_0.xyz_2_];
    rate_acc_N=size(log.data.vehicle_angular_acceleration_0.timestamp,1);
    rate_acc_delta_t=zeros(rate_acc_N-1,1);
    for i=1:rate_acc_N-1
        rate_acc_delta_t(i)=(log.data.vehicle_angular_acceleration_0.timestamp(i+1)-log.data.vehicle_angular_acceleration_0.timestamp(i))*1e-6;
    end
    fprintf('角加速度采样周期: %f (ms)， 频率: %f （Hz） \n', 1000*mean(rate_acc_delta_t), 1/mean(rate_acc_delta_t));
elseif isfield(log.data, 'vehicle_angular_velocity_0') && ...
       ismember('xyz_derivative_0_', log.data.vehicle_angular_velocity_0.Properties.VariableNames)
    vehicle_angular_acceleration = [log.data.vehicle_angular_velocity_0.xyz_derivative_0_, ...
                                    log.data.vehicle_angular_velocity_0.xyz_derivative_1_, ...
                                    log.data.vehicle_angular_velocity_0.xyz_derivative_2_];
    fprintf('角加速度采样周期(与角速度相同): %f (ms)， 频率: %f （Hz） \n', 1000*mean(rate_delta_t), 1/mean(rate_delta_t));
end 


% 2.3 Setpoints
if(isfield(log.data, 'vehicle_rates_setpoint_0'))
    vehicle_rates_setpoint = [log.data.vehicle_rates_setpoint_0.roll, ...
                              log.data.vehicle_rates_setpoint_0.pitch, ...
                              log.data.vehicle_rates_setpoint_0.yaw];
    rate_setpoint_N=size(log.data.vehicle_rates_setpoint_0.timestamp,1);
    rate_setpoint_delta_t=zeros(rate_setpoint_N-1,1);
    for i=1:rate_setpoint_N-1
        rate_setpoint_delta_t(i)=(log.data.vehicle_rates_setpoint_0.timestamp(i+1)-log.data.vehicle_rates_setpoint_0.timestamp(i))*1e-6;
    end
    fprintf('角速度给定采样周期: %f (ms)， 频率: %f （Hz） \n', 1000*mean(rate_setpoint_delta_t), 1/mean(rate_setpoint_delta_t));
end 

% 2.4 Attitude
if(isfield(log.data, 'vehicle_attitude_0'))
    vehicle_attitude_quat = [log.data.vehicle_attitude_0.q_0_, log.data.vehicle_attitude_0.q_1_, log.data.vehicle_attitude_0.q_2_, log.data.vehicle_attitude_0.q_3_];
    eul = quat2eul(vehicle_attitude_quat);
    Roll = eul(:,3); Pitch = eul(:,2); Yaw = eul(:,1);
    attitude_N=size(log.data.vehicle_attitude_0.timestamp,1);
    attitude_delta_t=zeros(attitude_N-1,1);
    for i=1:attitude_N-1
        attitude_delta_t(i)=(log.data.vehicle_attitude_0.timestamp(i+1)-log.data.vehicle_attitude_0.timestamp(i))*1e-6;
    end
    fprintf('姿态采样周期: %f (ms)， 频率: %f （Hz） \n', 1000*mean(attitude_delta_t), 1/mean(attitude_delta_t));
end
if(isfield(log.data, 'vehicle_attitude_setpoint_0'))
    vehicle_attitude_quat_d = [log.data.vehicle_attitude_setpoint_0.q_d_0_, log.data.vehicle_attitude_setpoint_0.q_d_1_, log.data.vehicle_attitude_setpoint_0.q_d_2_, log.data.vehicle_attitude_setpoint_0.q_d_3_];
    eul_setpoint = quat2eul(vehicle_attitude_quat_d);
    Roll_setpoint = eul_setpoint(:,3); Pitch_setpoint = eul_setpoint(:,2); Yaw_setpoint = eul_setpoint(:,1);
    attitude_setpoint_N=size(log.data.vehicle_attitude_setpoint_0.timestamp,1);
    attitude_setpoint_delta_t=zeros(attitude_setpoint_N-1,1);
    for i=1:attitude_setpoint_N-1
        attitude_setpoint_delta_t(i)=(log.data.vehicle_attitude_setpoint_0.timestamp(i+1)-log.data.vehicle_attitude_setpoint_0.timestamp(i))*1e-6;
    end
    fprintf('姿态给定采样周期: %f (ms)， 频率: %f （Hz） \n', 1000*mean(attitude_setpoint_delta_t), 1/mean(attitude_setpoint_delta_t));
end

% 2.5 Position / Velocity
if(isfield(log.data, 'vehicle_local_position_0'))
    XYZ = [log.data.vehicle_local_position_0.x, log.data.vehicle_local_position_0.y, log.data.vehicle_local_position_0.z];
    V_XYZ = [log.data.vehicle_local_position_0.vx, log.data.vehicle_local_position_0.vy, log.data.vehicle_local_position_0.vz];
    pose_N=size(log.data.vehicle_local_position_0.timestamp,1);
    pose_delta_t=zeros(pose_N-1,1);
    for i=1:pose_N-1
        pose_delta_t(i)=(log.data.vehicle_local_position_0.timestamp(i+1)-log.data.vehicle_local_position_0.timestamp(i))*1e-6;
    end
    fprintf('位置采样周期: %f (ms)， 频率: %f （Hz） \n', 1000*mean(pose_delta_t), 1/mean(pose_delta_t));
end
if(isfield(log.data, 'vehicle_local_position_setpoint_0'))
    XYZ_setpoint = [log.data.vehicle_local_position_setpoint_0.x, log.data.vehicle_local_position_setpoint_0.y, log.data.vehicle_local_position_setpoint_0.z];
    V_XYZ_setpoint = [log.data.vehicle_local_position_setpoint_0.vx, log.data.vehicle_local_position_setpoint_0.vy, log.data.vehicle_local_position_setpoint_0.vz];
    pose_setpoint_N=size(log.data.vehicle_local_position_setpoint_0.timestamp,1);
    pose_setpoint_delta_t=zeros(pose_setpoint_N-1,1);
    for i=1:pose_setpoint_N-1
        pose_setpoint_delta_t(i)=(log.data.vehicle_local_position_setpoint_0.timestamp(i+1)-log.data.vehicle_local_position_setpoint_0.timestamp(i))*1e-6;
    end
    fprintf('位置给定采样周期: %f (ms)， 频率: %f （Hz） \n', 1000*mean(pose_setpoint_delta_t), 1/mean(pose_setpoint_delta_t));
end

% 2.6 Control Output
% --- Group 0 (Main) ---
if(isfield(log.data, 'vehicle_torque_setpoint_0'))
    Roll_control = log.data.vehicle_torque_setpoint_0.xyz_0_;
    Pitch_control = log.data.vehicle_torque_setpoint_0.xyz_1_;
    Yaw_control = log.data.vehicle_torque_setpoint_0.xyz_2_;
    
    % 计算频率
    vts_N = size(log.data.vehicle_torque_setpoint_0.timestamp,1);
    vts_dt = zeros(vts_N-1,1);
    for i=1:vts_N-1
        vts_dt(i)=(log.data.vehicle_torque_setpoint_0.timestamp(i+1)-log.data.vehicle_torque_setpoint_0.timestamp(i))*1e-6;
    end
    fprintf('力/力矩(Grp0) 给定采样周期: %f (ms)， 频率: %f （Hz） \n', 1000*mean(vts_dt), 1/mean(vts_dt));
end
if(isfield(log.data, 'vehicle_thrust_setpoint_0'))
    thrust_sp = log.data.vehicle_thrust_setpoint_0.xyz_2_;
end

% --- Group 1 (Auxiliary, e.g. Decoupled or VTOL) ---
if(isfield(log.data, 'vehicle_torque_setpoint_1'))
    Roll_control_1 = log.data.vehicle_torque_setpoint_1.xyz_0_;
    Pitch_control_1 = log.data.vehicle_torque_setpoint_1.xyz_1_;
    Yaw_control_1 = log.data.vehicle_torque_setpoint_1.xyz_2_;
    
    % 计算频率
    vts1_N = size(log.data.vehicle_torque_setpoint_1.timestamp,1);
    vts1_dt = zeros(vts1_N-1,1);
    for i=1:vts1_N-1
        vts1_dt(i)=(log.data.vehicle_torque_setpoint_1.timestamp(i+1)-log.data.vehicle_torque_setpoint_1.timestamp(i))*1e-6;
    end
    fprintf('力/力矩(Grp1) 给定采样周期: %f (ms)， 频率: %f （Hz） \n', 1000*mean(vts1_dt), 1/mean(vts1_dt));
end
if(isfield(log.data, 'vehicle_thrust_setpoint_1'))
    thrust_sp_1 = log.data.vehicle_thrust_setpoint_1.xyz_0_;
end

%% --- 动态控制分配检测 (Dynamic Control Allocation Detection) ---
% 对应 Python: dynamic_control_alloc = any(elem.name in ('actuator_motors', 'actuator_servos'))
dynamic_control_alloc = isfield(log.data, 'actuator_motors_0') || isfield(log.data, 'actuator_servos_0');

if dynamic_control_alloc
    disp('检测到使用动态控制分配 (Dynamic Control Allocation).');
else
    disp('使用旧版控制分配 (Legacy Actuator Controls).');
end

% 2.7 Actuator Output Extraction
motors = []; servos = [];
if isfield(log.data, 'actuator_motors_0')
    % 动态寻找 control_x_ 列
    mc_cols = startsWith(log.data.actuator_motors_0.Properties.VariableNames, 'control_');
    motors = log.data.actuator_motors_0{:, mc_cols};
    motors_N=size(log.data.actuator_motors_0.timestamp,1);
    motors_delta_t=zeros(motors_N-1,1);
    motors_delta=zeros(motors_N-1,1);
    for i=1:motors_N-1
        motors_delta_t(i)=(log.data.actuator_motors_0.timestamp(i+1)-log.data.actuator_motors_0.timestamp(i))*1e-6;
    end
    for i=1:motors_N-1
        motors_delta(i)=log.data.actuator_motors_0.control_0_(i+1)-log.data.actuator_motors_0.control_0_(i);
    end
    fprintf('分配器输出电机采样周期: %f (ms)， 频率: %f （Hz） \n', 1000*mean(motors_delta_t), 1/mean(motors_delta_t));
end
if isfield(log.data, 'actuator_servos_0')
    sc_cols = startsWith(log.data.actuator_servos_0.Properties.VariableNames, 'control_');
    servos = log.data.actuator_servos_0{:, sc_cols};
    servo_N=size(log.data.actuator_servos_0.timestamp,1);
    servo_delta_t=zeros(servo_N-1,1);
    servo_delta=zeros(servo_N-1,1);
    for i=1:servo_N-1
        servo_delta_t(i)=(log.data.actuator_servos_0.timestamp(i+1)-log.data.actuator_servos_0.timestamp(i))*1e-6;
    end
    for i=1:servo_N-1
        servo_delta(i)=log.data.actuator_servos_0.control_0_(i+1)-log.data.actuator_servos_0.control_0_(i);
    end
    fprintf('分配器输出舵机采样周期: %f (ms)， 频率: %f （Hz） \n', 1000*mean(servo_delta_t), 1/mean(servo_delta_t));
end

% 2.8 PWM 通道解析 (为 Figure 8 做准备)
active_channels = struct('idx', {}, 'code', {}, 'name', {}, 'col_name', {}, 'type', {});
if isfield(log.data, 'actuator_outputs_0') && isfield(log, 'params')
    disp('正在解析 PWM 输出通道定义...');
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
    pwm_N=size(log.data.actuator_outputs_0.timestamp,1);
    pwm_delta_t=zeros(pwm_N-1,1);
    pwm_delta=zeros(pwm_N-1,1);
    for i=1:pwm_N-1
        pwm_delta_t(i)=(log.data.actuator_outputs_0.timestamp(i+1)-log.data.actuator_outputs_0.timestamp(i))*1e-6;
    end
    for i=1:pwm_N-1
        pwm_delta(i)=log.data.actuator_outputs_0.output_1_(i+1)-log.data.actuator_outputs_0.output_1_(i);
    end
    fprintf('pwm输出采样周期: %f (ms)， 频率: %f （Hz） \n', 1000*mean(pwm_delta_t), 1/mean(pwm_delta_t));

end
if(isfield(log.data, 'parameter_update_0'))
    flag=log.data.parameter_update_0.timestamp;
    
end 
if(isfield(log.data, 'manual_control_setpoint_0'))
    input_rc_N=size(log.data.manual_control_setpoint_0.timestamp,1);
    input_rc_delta_t=zeros(input_rc_N-1,1);
    for i=1:input_rc_N-1
        input_rc_delta_t(i)=(log.data.manual_control_setpoint_0.timestamp(i+1)-log.data.manual_control_setpoint_0.timestamp(i))*1e-6;
    end
    
end 

%% =========================================================================
%  2.9 准备可视化状态数据 (飞行模式 & VTOL状态)
% =========================================================================
vis_flight_intervals = [];
vis_flight_names = {};
vis_vtol_intervals = [];
vis_vtol_names = {};
vis_is_vtol = false;

if isfield(log.data, 'vehicle_status_0')
    % log.data.vehicle_status_0.vehicle_type; % If the vehicle is a VTOL, then this value will be VEHICLE_TYPE_ROTARY_WING=1 while flying as a multicopter, and VEHICLE_TYPE_FIXED_WING=2 when flying as a fixed-wing
    % log.data.vehicle_status_0.is_vtol; %True if the system is VTOL capable
    % log.data.vehicle_status_0.is_vtol_tailsitter;% True if the system performs a 90° pitch down rotation during transition from MC to FW
    % log.data.vehicle_status_0.in_transition_mode;%True if VTOL is doing a transition

    % 仅当 tailsitter 
    if max(log.data.vehicle_status_0.is_vtol_tailsitter) == 1
    
        fw_intervals = get_fw_intervals(log.data.vehicle_status_0);
    
        % 1) attitude: 覆盖 quaternion
        vehicle_attitude_quat = apply_tailsitter_attitude_fix(log.data.vehicle_attitude_0.timestamp,vehicle_attitude_quat, fw_intervals);
        eul = quat2eul(vehicle_attitude_quat, 'ZYX'); % rad: [yaw pitch roll]
        Yaw   = eul(:,1);
        Pitch = eul(:,2);
        Roll  = eul(:,3);
    
        % 2) rates: 覆盖角速度
        vehicle_angular_velocity = apply_tailsitter_rate_fix(log.data.vehicle_angular_velocity_0.timestamp, vehicle_angular_velocity, fw_intervals);
    
        % 3) rates setpoint: 覆盖设定值
        vehicle_rates_setpoint = apply_tailsitter_rate_sp_fix(log.data.vehicle_rates_setpoint_0.timestamp, vehicle_rates_setpoint, fw_intervals);
    
    end
    % 1. 解析飞行模式 (无缝隙版)
    [vis_flight_intervals, vis_flight_names] = get_flight_mode_intervals(log.data.vehicle_status_0);
    
    % 2. 解析 VTOL 状态 (无缝隙版)
    if ismember('is_vtol', log.data.vehicle_status_0.Properties.VariableNames) && ...
       max(log.data.vehicle_status_0.is_vtol) == 1
        vis_is_vtol = true;
        [vis_vtol_intervals, vis_vtol_names] = get_vtol_state_intervals(log.data.vehicle_status_0);
    end
end

% 2.10 TECS Status (新增：解析 TECS 高度速率)
if isfield(log.data, 'tecs_status_0')
    tecs_h_rate = log.data.tecs_status_0.height_rate;
    tecs_h_rate_sp = log.data.tecs_status_0.height_rate_setpoint;
    
end

%% =========================================================================
%  2.11 Manual Control Inputs  
% =========================================================================
if isfield(log.data, 'manual_control_setpoint_0')
   
    t_rc = log.data.manual_control_setpoint_0.timestamp * 1e-6;
    
    rc_roll  = log.data.manual_control_setpoint_0.roll;
    rc_pitch = log.data.manual_control_setpoint_0.pitch;
    rc_yaw   = log.data.manual_control_setpoint_0.yaw;
    rc_thr   = log.data.manual_control_setpoint_0.throttle;

end

% 2.12 Raw Acceleration (sensor_combined)
if isfield(log.data, 'sensor_combined_0')
    raw_acc_t = log.data.sensor_combined_0.timestamp * 1e-6;
    % 提取 X, Y, Z 加速度 (m/s^2)
    raw_acc = [log.data.sensor_combined_0.accelerometer_m_s2_0_, ...
               log.data.sensor_combined_0.accelerometer_m_s2_1_, ...
               log.data.sensor_combined_0.accelerometer_m_s2_2_];
    
    % 顺便打印一下采样频率，检查传感器数据健康度
    if length(raw_acc_t) > 1
        raw_acc_dt = mean(diff(raw_acc_t));
        fprintf('原始加速度(sensor_combined) 采样周期: %f (ms)， 频率: %f （Hz） \n', 1000*raw_acc_dt, 1/raw_acc_dt);
    end
end

% 2.13 Vibration Metrics (vehicle_imu_status)
% 自动搜索存在的 IMU 实例 (0~3)
vib_data = struct('id', {}, 't', {}, 'val', {});
for i = 0:3
    topic_name = sprintf('vehicle_imu_status_%d', i);
    if isfield(log.data, topic_name)
        idx = length(vib_data) + 1;
        vib_data(idx).id = i;
        vib_data(idx).t = log.data.(topic_name).timestamp * 1e-6;
        % 提取震动指标
        vib_data(idx).val = log.data.(topic_name).accel_vibration_metric;
    end
end
if ~isempty(vib_data)
    % 打印一下采样信息 (取第一个存在的 IMU)
    vib_dt = mean(diff(vib_data(1).t));
    fprintf('振动指标(Vibration) 采样周期: %f (ms)， 频率: %f （Hz） \n', 1000*vib_dt, 1/vib_dt);
end



% 2.14 Raw Angular Speed (sensor_combined)
if isfield(log.data, 'sensor_combined_0')
    % 提取并转换为 deg/s
    if ismember('gyro_rad_0_', log.data.sensor_combined_0.Properties.VariableNames)
        raw_gyro_t = log.data.sensor_combined_0.timestamp * 1e-6;
        raw_gyro = [log.data.sensor_combined_0.gyro_rad_0_, ...
                    log.data.sensor_combined_0.gyro_rad_1_, ...
                    log.data.sensor_combined_0.gyro_rad_2_] * r2d;
    end
end

% 2.15 FIFO Data Parsing (Accel & Gyro)
% 结构体数组存储解析后的数据: fifo_acc(1).t, fifo_acc(1).d
fifo_acc = struct('id', {}, 't', {}, 'd', {}, 'raw_t', {});
fifo_gyro = struct('id', {}, 't', {}, 'd', {});

for i = 0:2
    % --- FIFO Accel ---
    topic_name = sprintf('sensor_accel_fifo_%d', i);
    if isfield(log.data, topic_name)
        idx = length(fifo_acc) + 1;
        fifo_acc(idx).id = i;
        % 保存原始包时间戳用于计算丢包/采样间隔
        fifo_acc(idx).raw_t = log.data.(topic_name).timestamp * 1e-6; 
        % 解析虚拟高频数据
        [t_us, d_val] = expand_fifo_topic(log.data.(topic_name));
        fifo_acc(idx).t = t_us * 1e-6;
        fifo_acc(idx).d = d_val;
        fprintf('Parsed FIFO Accel %d: %d samples\n', i, length(t_us));
    end
    
    % --- FIFO Gyro ---
    topic_name_g = sprintf('sensor_gyro_fifo_%d', i);
    if isfield(log.data, topic_name_g)
        idx = length(fifo_gyro) + 1;
        fifo_gyro(idx).id = i;
        [t_us, d_val] = expand_fifo_topic(log.data.(topic_name_g));
        fifo_gyro(idx).t = t_us * 1e-6;
        fifo_gyro(idx).d = d_val * r2d; % 转换为 deg/s
        fprintf('Parsed FIFO Gyro %d: %d samples\n', i, length(t_us));
    end
end

% 2.16 Magnetic Field (磁场强度)
% 优先读取 vehicle_magnetometer，旧日志可能在 sensor_combined
if isfield(log.data, 'vehicle_magnetometer_0')
    mag_t = log.data.vehicle_magnetometer_0.timestamp * 1e-6;
    mag_data = [log.data.vehicle_magnetometer_0.magnetometer_ga_0_, ...
                log.data.vehicle_magnetometer_0.magnetometer_ga_1_, ...
                log.data.vehicle_magnetometer_0.magnetometer_ga_2_];
elseif isfield(log.data, 'sensor_combined_0') && ismember('magnetometer_ga_0_', log.data.sensor_combined_0.Properties.VariableNames)
    mag_t = log.data.sensor_combined_0.timestamp * 1e-6;
    mag_data = [log.data.sensor_combined_0.magnetometer_ga_0_, ...
                log.data.sensor_combined_0.magnetometer_ga_1_, ...
                log.data.sensor_combined_0.magnetometer_ga_2_];
end

% 2.17 Distance Sensor (距离传感器)
if isfield(log.data, 'distance_sensor_0')
    dist_sensor_t = log.data.distance_sensor_0.timestamp * 1e-6;
    dist_val = log.data.distance_sensor_0.current_distance;
    dist_var = log.data.distance_sensor_0.variance;
end
% 及其对应的估计值 (Dist bottom)
if isfield(log.data, 'vehicle_local_position_0') && ismember('dist_bottom', log.data.vehicle_local_position_0.Properties.VariableNames)
    dist_bottom_t = log.data.vehicle_local_position_0.timestamp * 1e-6;
    dist_bottom = log.data.vehicle_local_position_0.dist_bottom;
    dist_bottom_valid = log.data.vehicle_local_position_0.dist_bottom_valid;
end

% 2.18 GPS Status (精度与干扰)
if isfield(log.data, 'vehicle_gps_position_0')
    gps_t = log.data.vehicle_gps_position_0.timestamp * 1e-6;
    % 使用 table 存储以便灵活处理列是否存在
    gps_info = table();
    gps_info.eph = log.data.vehicle_gps_position_0.eph;
    gps_info.epv = log.data.vehicle_gps_position_0.epv;
    gps_info.s_variance = log.data.vehicle_gps_position_0.s_variance_m_s;
    gps_info.satellites = log.data.vehicle_gps_position_0.satellites_used;
    gps_info.fix_type = log.data.vehicle_gps_position_0.fix_type;
    gps_info.noise = log.data.vehicle_gps_position_0.noise_per_ms;
    gps_info.jamming = log.data.vehicle_gps_position_0.jamming_indicator;
    
    % 兼容旧日志的 HDOP/VDOP
    if ismember('hdop', log.data.vehicle_gps_position_0.Properties.VariableNames)
        gps_info.hdop = log.data.vehicle_gps_position_0.hdop;
        gps_info.vdop = log.data.vehicle_gps_position_0.vdop;
    end
end

% 2.19 Power (Battery & System)
if isfield(log.data, 'battery_status_0')
    bat_t = log.data.battery_status_0.timestamp * 1e-6;
    bat_v = log.data.battery_status_0.voltage_v;
    bat_i = log.data.battery_status_0.current_a;
    bat_discharged = log.data.battery_status_0.discharged_mah;
    bat_remaining = log.data.battery_status_0.remaining; % 0-1
    
    % 可选：内阻估计
    if ismember('internal_resistance_estimate', log.data.battery_status_0.Properties.VariableNames)
        bat_res = log.data.battery_status_0.internal_resistance_estimate;
    end
end
if isfield(log.data, 'system_power_0')
    sys_pwr_t = log.data.system_power_0.timestamp * 1e-6;
    % 兼容不同版本的命名 (5V rail)
    if ismember('voltage5v_v', log.data.system_power_0.Properties.VariableNames)
        sys_5v = log.data.system_power_0.voltage5v_v;
    elseif ismember('voltage5V_v', log.data.system_power_0.Properties.VariableNames)
        sys_5v = log.data.system_power_0.voltage5V_v;
    end
end

% 2.20 Temperature (Multi-source)
temp_data = struct('name', {}, 't', {}, 'val', {});
% Baro Temp
if isfield(log.data, 'sensor_baro_0')
    temp_data(end+1).name = 'Baro';
    temp_data(end).t = log.data.sensor_baro_0.timestamp * 1e-6;
    temp_data(end).val = log.data.sensor_baro_0.temperature;
end
% Accel Temp (IMU)
if isfield(log.data, 'sensor_accel_0')
    temp_data(end+1).name = 'IMU Accel';
    temp_data(end).t = log.data.sensor_accel_0.timestamp * 1e-6;
    temp_data(end).val = log.data.sensor_accel_0.temperature;
end
% Airspeed Temp
if isfield(log.data, 'airspeed_0') && ismember('air_temperature_celsius', log.data.airspeed_0.Properties.VariableNames)
    temp_data(end+1).name = 'Airspeed';
    temp_data(end).t = log.data.airspeed_0.timestamp * 1e-6;
    temp_data(end).val = log.data.airspeed_0.air_temperature_celsius;
end
% Battery Temp
if isfield(log.data, 'battery_status_0')
    temp_data(end+1).name = 'Battery';
    temp_data(end).t = log.data.battery_status_0.timestamp * 1e-6;
    temp_data(end).val = log.data.battery_status_0.temperature;
end
% ESC Temp (如果有)
if isfield(log.data, 'esc_status_0')
    % ESC 数据通常是数组，这里取平均或第一个非零的作为代表
    esc_cols = startsWith(log.data.esc_status_0.Properties.VariableNames, 'esc_');
    % 简单的处理逻辑：尝试寻找 esc_0_esc_temperature 这种展开后的列名
    % 实际 ulog2csv 可能展开为 esc_0__esc_temperature 或类似
    % 这里暂略复杂解析，若有 esc_status 可手动添加
end

% 2.21 Estimator Status (Flags)
if isfield(log.data, 'estimator_status_0')
    est_t = log.data.estimator_status_0.timestamp * 1e-6;
    
    % 改用 struct 存储，比 table 更灵活
    est_flags = struct();
    est_flags.health = log.data.estimator_status_0.health_flags;
    est_flags.timeout = log.data.estimator_status_0.timeout_flags;
    est_flags.time_slip = log.data.estimator_status_0.time_slip;
    
    % 安全读取: 检查 innovation_check_flags 是否存在
    cols = log.data.estimator_status_0.Properties.VariableNames;
    if ismember('innovation_check_flags', cols)
        est_flags.innovation = log.data.estimator_status_0.innovation_check_flags;
    else
        % 如果不存在（旧版或简化版日志），留空并打印警告
        est_flags.innovation = []; 
        disp('注意: 当前日志未包含 innovation_check_flags，将跳过相关绘图。');
    end
end

% 2.22 CPU & RAM
if isfield(log.data, 'cpuload_0')
    cpu_t = log.data.cpuload_0.timestamp * 1e-6;
    cpu_load = log.data.cpuload_0.load;
    ram_usage = log.data.cpuload_0.ram_usage;
end

% 2.23 Failsafe Flags
if isfield(log.data, 'vehicle_status_0')
    vs_t = log.data.vehicle_status_0.timestamp * 1e-6;
    vs_failsafe = log.data.vehicle_status_0.failsafe;
end

save('flight_data.mat')