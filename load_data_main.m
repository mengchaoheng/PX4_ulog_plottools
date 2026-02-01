clear all;
close all;
clc;
addpath(genpath(pwd));

%% =========================================================================
%  Step 0: 文件选择 & 基础设置
% =========================================================================
d2r = pi/180;
r2d = 180/pi;
get_t = @(tbl) tbl.timestamp * 1e-6;
% --- 用户配置区域 ---------------------------------------------------------
% 在这里指定文件名（可以是相对路径 'data/09_49_18' 或绝对路径）
% 【关键】：如果这里留空 (即 specifiedFileName = '';)，脚本运行时会自动弹窗让您选择。
specifiedFileName = 'data/09_49_18'; % 支持带后缀，也支持不带

if isempty(specifiedFileName)
    [fileName, pathName] = uigetfile('*.ulg', '请选择要分析的 ULog 文件');
    if isequal(fileName, 0), disp('取消选择'); return; end
    fullPath = fullfile(pathName, fileName);
else
    % 1. 补全后缀
    if ~endsWith(specifiedFileName, '.ulg'), specifiedFileName = [specifiedFileName '.ulg']; end
    % 2. 转为绝对路径 
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
%  Step 2: 数据预处理 (状态转换 & PWM解析)
% =========================================================================
% 2.1 vehicle_angular_velocity
if(isfield(log.data, 'vehicle_angular_velocity_0'))
    vehicle_angular_velocity = [log.data.vehicle_angular_velocity_0.xyz_0_, ...
                                log.data.vehicle_angular_velocity_0.xyz_1_, ...
                                log.data.vehicle_angular_velocity_0.xyz_2_];
    vehicle_angular_velocity_t = get_t(log.data.vehicle_angular_velocity_0);
    if length(vehicle_angular_velocity_t) > 1
        dt = mean(diff(vehicle_angular_velocity_t));
        fprintf('角速度采样周期: %f (ms)， 频率: %f （Hz） \n', dt*1000, 1/dt);
    end
    if ismember('xyz_derivative_0_', log.data.vehicle_angular_velocity_0.Properties.VariableNames)
        vehicle_angular_acceleration = [log.data.vehicle_angular_velocity_0.xyz_derivative_0_, ...
                                        log.data.vehicle_angular_velocity_0.xyz_derivative_1_, ...
                                        log.data.vehicle_angular_velocity_0.xyz_derivative_2_];
        vehicle_angular_acceleration_t=vehicle_angular_velocity_t;
        fprintf('角加速度采样周期与角速度相同');
    end
end

% 2.2 Angular acceleration
if(isfield(log.data, 'vehicle_angular_acceleration_0'))
    vehicle_angular_acceleration = [log.data.vehicle_angular_acceleration_0.xyz_0_, ...
                                    log.data.vehicle_angular_acceleration_0.xyz_1_, ...
                                    log.data.vehicle_angular_acceleration_0.xyz_2_];    
    vehicle_angular_acceleration_t = get_t(log.data.vehicle_angular_acceleration_0);
    if length(vehicle_angular_acceleration_t) > 1
        dt = mean(diff(vehicle_angular_acceleration_t));
        fprintf('角加速度采样周期: %f (ms)， 频率: %f （Hz） \n', dt*1000, 1/dt);
    end
end 


% 2.3 vehicle_angular_velocity setpoints
if(isfield(log.data, 'vehicle_rates_setpoint_0'))
    vehicle_rates_setpoint = [log.data.vehicle_rates_setpoint_0.roll, ...
                              log.data.vehicle_rates_setpoint_0.pitch, ...
                              log.data.vehicle_rates_setpoint_0.yaw];
    vehicle_rates_setpoint_t = get_t(log.data.vehicle_rates_setpoint_0);
    if length(vehicle_rates_setpoint_t) > 1
        dt = mean(diff(vehicle_rates_setpoint_t));
        fprintf('角速度给定采样周期: %f (ms)， 频率: %f （Hz） \n', dt*1000, 1/dt);
    end
end 

% 2.4 Attitude
if(isfield(log.data, 'vehicle_attitude_0'))
    vehicle_attitude_quat = [log.data.vehicle_attitude_0.q_0_, log.data.vehicle_attitude_0.q_1_, log.data.vehicle_attitude_0.q_2_, log.data.vehicle_attitude_0.q_3_];
    eul = quat2eul(vehicle_attitude_quat);
    Roll = eul(:,3); Pitch = eul(:,2); Yaw = eul(:,1);
    vehicle_attitude_t = get_t(log.data.vehicle_attitude_0);
    if length(vehicle_attitude_t) > 1
        dt = mean(diff(vehicle_attitude_t));
        fprintf('姿态采样周期: %f (ms)， 频率: %f （Hz） \n', dt*1000, 1/dt);
    end
end

if(isfield(log.data, 'vehicle_attitude_setpoint_0'))
    vehicle_attitude_quat_d = [log.data.vehicle_attitude_setpoint_0.q_d_0_, log.data.vehicle_attitude_setpoint_0.q_d_1_, log.data.vehicle_attitude_setpoint_0.q_d_2_, log.data.vehicle_attitude_setpoint_0.q_d_3_];
    eul_setpoint = quat2eul(vehicle_attitude_quat_d);
    Roll_setpoint = eul_setpoint(:,3); Pitch_setpoint = eul_setpoint(:,2); Yaw_setpoint = eul_setpoint(:,1);
    vehicle_attitude_setpoint_t = get_t(log.data.vehicle_attitude_setpoint_0);
    if length(vehicle_attitude_setpoint_t) > 1
        dt = mean(diff(vehicle_attitude_setpoint_t));
        fprintf('姿态给定采样周期: %f (ms)， 频率: %f （Hz） \n', dt*1000, 1/dt);
    end
end

% 2.5 Position / Velocity
if(isfield(log.data, 'vehicle_local_position_0'))
    XYZ = [log.data.vehicle_local_position_0.x, log.data.vehicle_local_position_0.y, log.data.vehicle_local_position_0.z];
    V_XYZ = [log.data.vehicle_local_position_0.vx, log.data.vehicle_local_position_0.vy, log.data.vehicle_local_position_0.vz];
    vehicle_local_position_t = get_t(log.data.vehicle_local_position_0);
    if length(vehicle_local_position_t) > 1
        dt = mean(diff(vehicle_local_position_t));
        fprintf('位置/速度采样周期: %f (ms)， 频率: %f （Hz） \n', dt*1000, 1/dt);
    end
end
if(isfield(log.data, 'vehicle_local_position_setpoint_0'))
    XYZ_setpoint = [log.data.vehicle_local_position_setpoint_0.x, log.data.vehicle_local_position_setpoint_0.y, log.data.vehicle_local_position_setpoint_0.z];
    V_XYZ_setpoint = [log.data.vehicle_local_position_setpoint_0.vx, log.data.vehicle_local_position_setpoint_0.vy, log.data.vehicle_local_position_setpoint_0.vz];
    vehicle_local_position_setpoint_t = get_t(log.data.vehicle_local_position_setpoint_0);
    if length(vehicle_local_position_setpoint_t) > 1
        dt = mean(diff(vehicle_local_position_setpoint_t));
        fprintf('位置/速度给定采样周期: %f (ms)， 频率: %f （Hz） \n', dt*1000, 1/dt);
    end
end

%% =========================================================================
%  Step 2.6: Actuator Controls 数据获取 (修正版：独立时间轴)
% =========================================================================
% 1. 启用控制分配标志 (dynamic_control_alloc)
dynamic_control_alloc = isfield(log.data, 'actuator_motors_0') || isfield(log.data, 'actuator_servos_0');


% 初始化结构体
actuator_controls_0 = struct('time',[], 'time_thrust',[], 'roll',[], 'pitch',[], 'yaw',[], 'thrust_x',[], 'thrust_z_neg',[]);
actuator_controls_1 = struct('time',[], 'roll',[], 'pitch',[], 'yaw',[], 'thrust_x',[], 'thrust_z_neg',[]);

% -------------------------------------------------------------------------
% 实例 0 (Main): 分离时间轴
% -------------------------------------------------------------------------
if dynamic_control_alloc
    % --- Dynamic Instance 0 ---
    % 1. 力矩 (Torque) 
    if isfield(log.data, 'vehicle_torque_setpoint_0')
        actuator_controls_0.time = get_t(log.data.vehicle_torque_setpoint_0);
        actuator_controls_0.roll  = log.data.vehicle_torque_setpoint_0.xyz_0_;
        actuator_controls_0.pitch = log.data.vehicle_torque_setpoint_0.xyz_1_;
        actuator_controls_0.yaw   = log.data.vehicle_torque_setpoint_0.xyz_2_;
    end
    
    % 2. 推力 (Thrust) -> 使用 independent timestamp_thrust
    if isfield(log.data, 'vehicle_thrust_setpoint_0')
        % 独立保存推力时间轴
        t_thrust_0 = get_t(log.data.vehicle_thrust_setpoint_0);
        actuator_controls_0.time_thrust = t_thrust_0;       
        tx = log.data.vehicle_thrust_setpoint_0.xyz_0_;
        ty = log.data.vehicle_thrust_setpoint_0.xyz_1_;
        tz = log.data.vehicle_thrust_setpoint_0.xyz_2_;
        actuator_controls_0.thrust = sqrt(tx.^2 + ty.^2 + tz.^2);          % Python: thrust
        actuator_controls_0.thrust_x = tx;          % Python: thrust_x
        actuator_controls_0.thrust_z_neg = -tz;     % Python: thrust_z_neg
        
        % 保存原始推力数据，供 Instance 1 重采样使用
        raw_thrust_0 = struct('t', t_thrust_0, 'thrust', actuator_controls_0.thrust, 'thrust_x', actuator_controls_0.thrust_x, 'thrust_z_neg', actuator_controls_0.thrust_z_neg );
    end
else
    % --- Legacy Instance 0 ---
    if isfield(log.data, 'actuator_controls_0_0')
        t_legacy = get_t(log.data.actuator_controls_0_0);
        actuator_controls_0.time = t_legacy;
        actuator_controls_0.time_thrust = t_legacy; % 旧版时间轴是同一个
        actuator_controls_0.roll  = log.data.actuator_controls_0_0.control_0_;
        actuator_controls_0.pitch = log.data.actuator_controls_0_0.control_1_;
        actuator_controls_0.yaw   = log.data.actuator_controls_0_0.control_2_;
        actuator_controls_0.thrust_z_neg = log.data.actuator_controls_0_0.control_3_;

    end
end
if length(actuator_controls_0.time) > 1
    dt = mean(diff(actuator_controls_0.time));
    fprintf('力/力矩(Grp0) 给定采样周期: %f (ms)， 频率: %f （Hz） \n', dt*1000, 1/dt);
end
% -------------------------------------------------------------------------
% 实例 1 (Aux/FW): 维持重采样策略 (为了画在同一张图上，通常对齐到 torque)
% -------------------------------------------------------------------------
if dynamic_control_alloc
    % --- Dynamic Instance 1 ---
    if isfield(log.data, 'vehicle_torque_setpoint_1')
        t1 = get_t(log.data.vehicle_torque_setpoint_1);
        actuator_controls_1.time = t1;      
        actuator_controls_1.roll  = log.data.vehicle_torque_setpoint_1.xyz_0_;
        actuator_controls_1.pitch = log.data.vehicle_torque_setpoint_1.xyz_1_;
        actuator_controls_1.yaw   = log.data.vehicle_torque_setpoint_1.xyz_2_;
        
        % 将推力 (来自实例0) 重采样对齐到实例 1 的时间轴
        if exist('raw_thrust_0', 'var')
            actuator_controls_1.thrust = interp1(raw_thrust_0.t, raw_thrust_0.thrust, t1, 'linear', 'extrap');
            actuator_controls_1.thrust_x = interp1(raw_thrust_0.t, raw_thrust_0.thrust_x, t1, 'linear', 'extrap');
            actuator_controls_1.thrust_z_neg = interp1(raw_thrust_0.t, raw_thrust_0.thrust_z_neg, t1, 'linear', 'extrap');
        end
    end
else
    % --- Legacy Instance 1 ---
    if isfield(log.data, 'actuator_controls_1_0')
        actuator_controls_1.time = get_t(log.data.actuator_controls_1_0);
        actuator_controls_1.roll  = log.data.actuator_controls_1_0.control_0_;
        actuator_controls_1.pitch = log.data.actuator_controls_1_0.control_1_;
        actuator_controls_1.yaw   = log.data.actuator_controls_1_0.control_2_;
        actuator_controls_1.thrust_x = log.data.actuator_controls_1_0.control_3_;
    end
end
if length(actuator_controls_1.time) > 1
    dt = mean(diff(actuator_controls_1.time));
    fprintf('力/力矩(Grp1) 给定采样周期: %f (ms)， 频率: %f （Hz） \n', dt*1000, 1/dt);
end
%% =========================================================================
%  Step 2.7 & 2.8: Actuator Data Extraction  
% =========================================================================

% 初始化变量
motors = []; motors_t = [];
servos = []; servos_t = [];
active_channels = []; % 用于存放解析后的 PWM 通道信息
legacy_pwm = [];      % 用于存放旧版原始 PWM 数据
legacy_pwm_t = [];

% -------------------------------------------------------------------------
% 1. 新版动态分配 (Actuator Motors & Servos)
% -------------------------------------------------------------------------
if isfield(log.data, 'actuator_motors_0')
    mc_cols = startsWith(log.data.actuator_motors_0.Properties.VariableNames, 'control_');
    motors = log.data.actuator_motors_0{:, mc_cols};
    motors_t = get_t(log.data.actuator_motors_0);
    
    if length(motors_t) > 1
        dt = mean(diff(motors_t));
        fprintf('New: 电机(Motors) 采样周期: %.3f ms, 频率: %.1f Hz\n', dt*1000, 1/dt);
    end
end

if isfield(log.data, 'actuator_servos_0')
    sc_cols = startsWith(log.data.actuator_servos_0.Properties.VariableNames, 'control_');
    servos = log.data.actuator_servos_0{:, sc_cols};
    servos_t = get_t(log.data.actuator_servos_0);
    
    if length(servos_t) > 1
        dt = mean(diff(servos_t));
        fprintf('New: 舵机(Servos) 采样周期: %.3f ms, 频率: %.1f Hz\n', dt*1000, 1/dt);
    end
end

% -------------------------------------------------------------------------
% 2. PWM 输出解析 (新版：尝试解析参数 / 旧版：直接提取原始数据)
% -------------------------------------------------------------------------

if isfield(log.data, 'actuator_outputs_0')
    output_cols = startsWith(log.data.actuator_outputs_0.Properties.VariableNames, 'output_');
    outputs = log.data.actuator_outputs_0{:, output_cols};
    outputs_t = get_t(log.data.actuator_outputs_0);
    
    if length(outputs_t) > 1
        dt = mean(diff(outputs_t));
        fprintf('PWM 原始输出 采样周期: %.3f ms, 频率: %.1f Hz\n', dt*1000, 1/dt);
    end
    
    % --- 尝试解析通道定义  ---
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
    end
    
    
end


if(isfield(log.data, 'parameter_update_0'))
    flag=log.data.parameter_update_0.timestamp;
    
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
rc_t = []; rc_roll = []; rc_pitch = []; rc_yaw = []; rc_throttle = [];

if isfield(log.data, 'manual_control_setpoint_0')
    tbl = log.data.manual_control_setpoint_0;
    rc_t = get_t(tbl);
    vars = tbl.Properties.VariableNames;
    
    % --- 检测版本并提取 ---
    if ismember('roll', vars)
        % === 新版 (v1.14+) ===
        % 字段名直观：roll, pitch, yaw, throttle
        rc_roll     = tbl.roll;
        rc_pitch    = tbl.pitch;
        rc_yaw      = tbl.yaw;
        rc_throttle = tbl.throttle;
        
    elseif ismember('y', vars)
        % === 旧版 (Legacy) ===
        % 映射关系：
        % y -> Roll  (横滚)
        % x -> Pitch (俯仰)
        % r -> Yaw   (偏航)
        % z -> Throttle (油门)
        rc_roll     = tbl.x;
        rc_pitch    = tbl.y;
        rc_yaw      = tbl.r;
        rc_throttle = tbl.z;
    else
        warning('无法识别 manual_control_setpoint 的字段格式');
    end
    
    % 辅助通道提取 (新旧版通常都有 aux1, aux2...)
    % 这里可以按需提取，例如:
    % if ismember('aux1', vars), rc_aux1 = tbl.aux1; end
end

% 2.12 Raw Acceleration (sensor_combined)
if isfield(log.data, 'sensor_combined_0')
    raw_acc_t = get_t(log.data.sensor_combined_0);
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
        vib_data(idx).t = get_t(log.data.(topic_name));
        % 提取震动指标
        vib_data(idx).val = log.data.(topic_name).accel_vibration_metric;
    end
end
if ~isempty(vib_data)
    % 打印采样信息 (取第一个存在的 IMU)
    vib_dt = mean(diff(vib_data(1).t));
    fprintf('振动指标(Vibration) 采样周期: %f (ms)， 频率: %f （Hz） \n', 1000*vib_dt, 1/vib_dt);
end



% 2.14 Raw Angular Speed (sensor_combined)
if isfield(log.data, 'sensor_combined_0')
    % 提取并转换为 deg/s
    if ismember('gyro_rad_0_', log.data.sensor_combined_0.Properties.VariableNames)
        raw_gyro_t = get_t(log.data.sensor_combined_0);
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
        fifo_acc(idx).raw_t = get_t(log.data.(topic_name)); 
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
    mag_t = get_t(log.data.vehicle_magnetometer_0);
    mag_data = [log.data.vehicle_magnetometer_0.magnetometer_ga_0_, ...
                log.data.vehicle_magnetometer_0.magnetometer_ga_1_, ...
                log.data.vehicle_magnetometer_0.magnetometer_ga_2_];
elseif isfield(log.data, 'sensor_combined_0') ...
        && ismember('magnetometer_ga_0_', log.data.sensor_combined_0.Properties.VariableNames)
    mag_t = get_t(log.data.sensor_combined_0);
    mag_data = [log.data.sensor_combined_0.magnetometer_ga_0_, ...
                log.data.sensor_combined_0.magnetometer_ga_1_, ...
                log.data.sensor_combined_0.magnetometer_ga_2_];
end

% 2.17 Distance Sensor (距离传感器)
if isfield(log.data, 'distance_sensor_0')
    dist_sensor_t = get_t(log.data.distance_sensor_0);
    dist_val = log.data.distance_sensor_0.current_distance;
    dist_var = log.data.distance_sensor_0.variance;
end
% 及其对应的估计值 (Dist bottom)
if isfield(log.data, 'vehicle_local_position_0') && ismember('dist_bottom', log.data.vehicle_local_position_0.Properties.VariableNames)
    dist_bottom_t = get_t(log.data.vehicle_local_position_0);
    dist_bottom = log.data.vehicle_local_position_0.dist_bottom;
    dist_bottom_valid = log.data.vehicle_local_position_0.dist_bottom_valid;
end

% 2.18 GPS Status (精度与干扰)
if isfield(log.data, 'vehicle_gps_position_0')
    gps_t = get_t(log.data.vehicle_gps_position_0);
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
    bat_t = get_t(log.data.battery_status_0);
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
    sys_pwr_t = get_t(log.data.system_power_0);
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
    temp_data(end).t = get_t(log.data.sensor_baro_0);
    temp_data(end).val = log.data.sensor_baro_0.temperature;
end
% Accel Temp (IMU)
if isfield(log.data, 'sensor_accel_0')
    temp_data(end+1).name = 'IMU Accel';
    temp_data(end).t = get_t(log.data.sensor_accel_0);
    temp_data(end).val = log.data.sensor_accel_0.temperature;
end
% Airspeed Temp
if isfield(log.data, 'airspeed_0') && ismember('air_temperature_celsius', log.data.airspeed_0.Properties.VariableNames)
    temp_data(end+1).name = 'Airspeed';
    temp_data(end).t = get_t(log.data.airspeed_0);
    temp_data(end).val = log.data.airspeed_0.air_temperature_celsius;
end
% Battery Temp
if isfield(log.data, 'battery_status_0')
    temp_data(end+1).name = 'Battery';
    temp_data(end).t = get_t(log.data.battery_status_0);
    temp_data(end).val = log.data.battery_status_0.temperature;
end
% ESC Temp (如果有)
if isfield(log.data, 'esc_status_0')
    tbl = log.data.esc_status_0;
    vars = tbl.Properties.VariableNames;
    
    % 确定扫描范围: 有 esc_count 则用最大值，否则默认扫前 8 个
    esc_limit = 8;
    if ismember('esc_count', vars)
        esc_limit = max(tbl.esc_count);
    end
    
    for i = 0:(esc_limit-1)
        % ulog2csv 的列名通常是 'esc_N__esc_temperature' (双下划线) 或 'esc_N_esc_temperature'
        col_name = sprintf('esc_%d__esc_temperature', i);
        if ~ismember(col_name, vars)
            col_name = sprintf('esc_%d_esc_temperature', i); % 备用格式
        end
        
        if ismember(col_name, vars)
            val = tbl.(col_name);
            % Python 逻辑: if np.amax(esc_temp) > 0.001
            if max(val) > 0.001
                temp_data(end+1).name = sprintf('ESC %d', i);
                temp_data(end).t = tbl.timestamp * 1e-6;
                temp_data(end).val = val;
            end
        end
    end
end

%% =========================================================================
%  Estimator Flags (Python 逻辑复刻版)
%  功能: 动态提取 Health, Timeout 和 Innovation Flags，仅绘制非零数据
% =========================================================================
if isfield(log.data, 'estimator_status_0')
    est_status = log.data.estimator_status_0;
    est_t = get_t(est_status);
    
    % 1. 准备数据池 (Label, Data)
    %    使用 cell array 存储所有候选信号
    candidates = {}; 
    
    % --- 基础标志 ---
    candidates{end+1, 1} = 'Health Flags';
    candidates{end, 2}   = double(est_status.health_flags);
    
    candidates{end+1, 1} = 'Timeout Flags';
    candidates{end, 2}   = double(est_status.timeout_flags);
    
    % --- Innovation Check Flags (需要位运算提取) ---
    % 检查是否存在该字段 (防止旧版日志报错)
    if ismember('innovation_check_flags', est_status.Properties.VariableNames)
        inno = est_status.innovation_check_flags;
        
        % Python: (flags) & 0x1 -> MATLAB: bitget(..., 1)
        candidates{end+1, 1} = 'Vel Check Bit';
        candidates{end, 2}   = double(bitget(inno, 1));
        
        % Python: (flags >> 1) & 1 -> MATLAB: bitget(..., 2)
        candidates{end+1, 1} = 'H Pos Check Bit';
        candidates{end, 2}   = double(bitget(inno, 2));
        
        % Python: (flags >> 2) & 1 -> MATLAB: bitget(..., 3)
        candidates{end+1, 1} = 'V Pos Check Bit';
        candidates{end, 2}   = double(bitget(inno, 3));
        
        % Python: (flags >> 3) & 0x7 (Mag X,Y,Z 3 bits)
        % MATLAB: bit 4, 5, 6.  Val = b4 + 2*b5 + 4*b6
        mag_check = double(bitget(inno, 4)) + ...
                    2 * double(bitget(inno, 5)) + ...
                    4 * double(bitget(inno, 6));
        candidates{end+1, 1} = 'Mag Check Bits';
        candidates{end, 2}   = mag_check;
        
        % Python: (flags >> 6) & 1 -> MATLAB: bitget(..., 7)
        candidates{end+1, 1} = 'Yaw Check Bit';
        candidates{end, 2}   = double(bitget(inno, 7));
        
        % Python: (flags >> 7) & 1 -> MATLAB: bitget(..., 8)
        candidates{end+1, 1} = 'Airspeed Check Bit';
        candidates{end, 2}   = double(bitget(inno, 8));
        
        % Python: (flags >> 8) & 1 -> MATLAB: bitget(..., 9)
        candidates{end+1, 1} = 'Sideslip Check Bit';
        candidates{end, 2}   = double(bitget(inno, 9));
        
        % Python: (flags >> 9) & 1 -> MATLAB: bitget(..., 10)
        candidates{end+1, 1} = 'HAGL Check Bit';
        candidates{end, 2}   = double(bitget(inno, 10));
        
        % Python: (flags >> 10) & 0x3 (Optical Flow 2 bits)
        opt_check = double(bitget(inno, 11)) + ...
                    2 * double(bitget(inno, 12));
        candidates{end+1, 1} = 'OptFlow Check Bits';
        candidates{end, 2}   = opt_check;
    else
        fprintf('Warning: innovation_check_flags not found in log data.\n');
    end

end

% 2.22 Failsafe Flags
if isfield(log.data, 'vehicle_status_0')
    vs_t = get_t(log.data.vehicle_status_0);
    vs_failsafe = log.data.vehicle_status_0.failsafe;
end
if isfield(log.data, 'failsafe_flags_0')
    fs_table = log.data.failsafe_flags_0;
    fs_cols = fs_table.Properties.VariableNames;
    fs_t = get_t(fs_table);

end

% 2.23 CPU & RAM
if isfield(log.data, 'cpuload_0')
    cpu_t = get_t(log.data.cpuload_0);
    cpu_load = log.data.cpuload_0.load;
    ram_usage = log.data.cpuload_0.ram_usage;
end


save('flight_data.mat')