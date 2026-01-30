clear all;
close all;
clc;
addpath(genpath(pwd));
% ==============requirements==============
% Install pyulog using pip first.https://github.com/PX4/pyulog.
% in MacOS, it maybe have been installed by the px4-dev

d2r=pi/180;
r2d=180/pi;
%------------------------------------------
% Set ULog relative path
%------------------------------------------
ulgFileName = 'data/09_49_18'; % 1.12.3: , 1.15.4: 13_39_55, main(1.17.0): df 09_38_44 vtol 09_49_18
tmp = [ulgFileName '.mat'];

% Record the current main script path
rootDir = fileparts(mfilename('fullpath'));


% % 弹出文件选择窗口
% [fileName, pathName] = uigetfile('*.ulg', '请选择要分析的 ULog 文件');
% if isequal(fileName, 0)
%     disp('用户取消了文件选择');
%     return;
% end

% % 获取不带后缀的文件名
% [~, nameNoExt, ~] = fileparts(fileName);
% ulgFileName = fullfile(pathName, nameNoExt);
% tmp = [ulgFileName '.mat'];
% rootDir = fileparts(mfilename('fullpath'));
%------------------------------------------
% Step 1: Check if MAT file already exists
%------------------------------------------
if exist(fullfile(rootDir, tmp), "file")
    disp(['Found MAT file: ' tmp]);
    load(fullfile(rootDir, tmp), 'log');
else
    disp('No MAT file found, start parsing ULog...');
    
    %------------------------------------------
    % Step 2: Run ulog2csv (keep full path)
    %------------------------------------------
    if ismac
        base_path = '~/Library/Python/3.9/bin/'; % 你的 Python bin 目录
        cmd_info = [base_path 'ulog_info'];
        cmd_msgs = [base_path 'ulog_messages'];
        cmd_params = [base_path 'ulog_params'];
        cmd_ulog2csv = [base_path 'ulog2csv'];
    else
        % Windows 或已配置好环境变量
        cmd_info = 'ulog_info';
        cmd_msgs = 'ulog_messages';
        cmd_params = 'ulog_params';
        cmd_ulog2csv = 'ulog2csv';
    end

    ulgAbs = fullfile(rootDir, [ulgFileName '.ulg']);
    
    command = [cmd_ulog2csv ' ' '"' ulgAbs '"'];
    
    disp(['Running command: ' command]);
    [status, cmdout] = system(command);
    
    % 检查系统命令执行状态 (0 代表成功)
    if status ~= 0
        disp('Error output:');
        disp(cmdout);
        error('ulog2csv conversion failed. Please check the path and pyulog installation.');
    else
        % 如果你想看成功输出，可以取消下面这行的注释
        % disp(cmdout); 
        disp('ulog2csv conversion successful.');
    end
    %------------------------------------------
    % Step 3: Call parsing function (pass full path)
    %------------------------------------------
    % =========================================================================
    % 处理 data
    % =========================================================================
    % 确保 csv_topics_to_d 能找到生成的文件
    log.data = csv_topics_to_d(fullfile(rootDir, ulgFileName));
    log.FileName = ulgFileName;
    log.version = 1.0;
    % =========================================================================
    % 处理 Info, Messages 和 Params
    % =========================================================================
    % -------------------------------------------------------
    % Part A: 处理 ulog_info (打印 + 存储字符串)
    % -------------------------------------------------------
    disp('----------------- ULog Info -----------------');
    command = [cmd_info ' "' ulgAbs '"'];
    [status, output] = system(command);
    if status == 0
        log.info = output; % 存起来，以后想看还能看
    else
        warning('Failed to run ulog_info');
        log.info = 'Error retrieving info';
    end

    % -------------------------------------------------------
    % Part B: 处理 ulog_messages (打印 + 存储字符串)
    % -------------------------------------------------------
    disp('----------------- ULog Messages -----------------');
    command = [cmd_msgs ' "' ulgAbs '"'];
    [status, output] = system(command);
    if status == 0
        log.messages = output;
    else
        warning('Failed to run ulog_messages');
        log.messages = 'Error retrieving messages';
    end

    % -------------------------------------------------------
    % Part C: 处理 ulog_params (解析为 Struct)
    % -------------------------------------------------------
    disp('----------------- Parsing Params -----------------');
    % 注意：ulog_params 默认输出是用逗号分隔的 (Name,Value)
    command = [cmd_params ' "' ulgAbs '"']; 
    [status, output] = system(command);
    
    if status == 0
        % 初始化参数结构体
        log.params = struct();
        
        % 按行分割输出
        lines = splitlines(output);
        count = 0;
        
        for i = 1:length(lines)
            tline = strtrim(lines{i});
            if isempty(tline), continue; end
            
            % 分割 "PARAM_NAME,VALUE"
            parts = strsplit(tline, ',');
            
            if length(parts) >= 2
                p_name = strtrim(parts{1});
                p_val_str = strtrim(parts{2});
                
                % 尝试将值转换为数字
                p_val = str2double(p_val_str);
                
                % 只有当名字是合法的 MATLAB 变量名时才存入
                % (通常大写字母+下划线都是合法的)
                try
                    % 如果转换结果是 NaN (说明不是纯数字)，则存字符串
                    if isnan(p_val)
                         log.params.(p_name) = p_val_str;
                    else
                         log.params.(p_name) = p_val;
                    end
                    count = count + 1;
                catch
                    % 忽略非法字段
                end
            end
        end
        fprintf('Parsed %d parameters into log.params.\n', count);
    else
        warning('Failed to run ulog_params');
        log.params = [];
    end
    disp('-------------------------------------------------');
    
    %------------------------------------------
    % Step 4: Save MAT file to the same directory
    %------------------------------------------
    save(fullfile(rootDir, tmp), 'log');
    disp(['Saved MAT file: ' tmp]);
    
    %------------------------------------------
    % Step 5: Delete temporary CSV files
    %------------------------------------------
    % 构建删除路径，确保只删除本次生成的 csv
    % 注意：如果 ulgFileName 包含子文件夹（如 'data/name'），这行代码通常也能正常工作
    delete_pattern = fullfile(rootDir, [ulgFileName '_*.csv']);
    delete(delete_pattern);
    disp(['Temporary CSV files deleted: ' delete_pattern]);
end
disp(log.info);
disp(log.messages);
%%
start_time=0;
end_time=55;
%% The version of PX4 after allocator:

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

if(isfield(log.data, 'vehicle_angular_velocity_0'))
    vehicle_angular_velocity=[log.data.vehicle_angular_velocity_0.xyz_0_,log.data.vehicle_angular_velocity_0.xyz_1_,log.data.vehicle_angular_velocity_0.xyz_2_];
    rate_N=size(log.data.vehicle_angular_velocity_0.timestamp,1);
    rate_delta_t=zeros(rate_N-1,1);
    for i=1:rate_N-1
        rate_delta_t(i)=(log.data.vehicle_angular_velocity_0.timestamp(i+1)-log.data.vehicle_angular_velocity_0.timestamp(i))*1e-6;
    end
    fprintf('角速度采样周期: %f (ms)， 频率: %f （Hz） \n', 1000*mean(rate_delta_t), 1/mean(rate_delta_t));
    
end 

if(isfield(log.data, 'vehicle_angular_acceleration_0'))
    vehicle_angular_acceleration=[log.data.vehicle_angular_acceleration_0.xyz_0_, log.data.vehicle_angular_acceleration_0.xyz_1_, log.data.vehicle_angular_acceleration_0.xyz_2_];
    rate_acc_N=size(log.data.vehicle_angular_acceleration_0.timestamp,1);
    rate_acc_delta_t=zeros(rate_acc_N-1,1);
    for i=1:rate_acc_N-1
        rate_acc_delta_t(i)=(log.data.vehicle_angular_acceleration_0.timestamp(i+1)-log.data.vehicle_angular_acceleration_0.timestamp(i))*1e-6;
    end
    fprintf('角加速度采样周期: %f (ms)， 频率: %f （Hz） \n', 1000*mean(rate_acc_delta_t), 1/mean(rate_acc_delta_t));
else
    vehicle_angular_acceleration=[log.data.vehicle_angular_velocity_0.xyz_derivative_0_,log.data.vehicle_angular_velocity_0.xyz_derivative_1_,log.data.vehicle_angular_velocity_0.xyz_derivative_2_];
end 

if(isfield(log.data, 'vehicle_rates_setpoint_0'))
    vehicle_rates_setpoint=[log.data.vehicle_rates_setpoint_0.roll,log.data.vehicle_rates_setpoint_0.pitch,log.data.vehicle_rates_setpoint_0.yaw];
    rate_setpoint_N=size(log.data.vehicle_rates_setpoint_0.timestamp,1);
    rate_setpoint_delta_t=zeros(rate_setpoint_N-1,1);
    for i=1:rate_setpoint_N-1
        rate_setpoint_delta_t(i)=(log.data.vehicle_rates_setpoint_0.timestamp(i+1)-log.data.vehicle_rates_setpoint_0.timestamp(i))*1e-6;
    end
    fprintf('角速度给定采样周期: %f (ms)， 频率: %f （Hz） \n', 1000*mean(rate_setpoint_delta_t), 1/mean(rate_setpoint_delta_t));
end 
if(isfield(log.data, 'vehicle_attitude_0'))
    vehicle_attitude_quat=[log.data.vehicle_attitude_0.q_0_, log.data.vehicle_attitude_0.q_1_, log.data.vehicle_attitude_0.q_2_, log.data.vehicle_attitude_0.q_3_];
    eul = quat2eul(vehicle_attitude_quat);
    Roll=eul(:,3);
    Pitch=eul(:,2);
    Yaw=eul(:,1);
    attitude_N=size(log.data.vehicle_attitude_0.timestamp,1);
    attitude_delta_t=zeros(attitude_N-1,1);
    for i=1:attitude_N-1
        attitude_delta_t(i)=(log.data.vehicle_attitude_0.timestamp(i+1)-log.data.vehicle_attitude_0.timestamp(i))*1e-6;
    end
    fprintf('姿态采样周期: %f (ms)， 频率: %f （Hz） \n', 1000*mean(attitude_delta_t), 1/mean(attitude_delta_t));
end 
if(isfield(log.data, 'vehicle_attitude_setpoint_0'))
    vehicle_attitude_quat_d=[log.data.vehicle_attitude_setpoint_0.q_d_0_, log.data.vehicle_attitude_setpoint_0.q_d_1_, log.data.vehicle_attitude_setpoint_0.q_d_2_, log.data.vehicle_attitude_setpoint_0.q_d_3_];
    eul_setpoint = quat2eul(vehicle_attitude_quat_d, 'ZYX'); % eul(:,3)=Roll eul(:,2)=Pitch eul(:,1)=Yaw
    Roll_setpoint=eul_setpoint(:,3);
    Pitch_setpoint=eul_setpoint(:,2);
    Yaw_setpoint=eul_setpoint(:,1);
    attitude_setpoint_N=size(log.data.vehicle_attitude_setpoint_0.timestamp,1);
    attitude_setpoint_delta_t=zeros(attitude_setpoint_N-1,1);
    for i=1:attitude_setpoint_N-1
        attitude_setpoint_delta_t(i)=(log.data.vehicle_attitude_setpoint_0.timestamp(i+1)-log.data.vehicle_attitude_setpoint_0.timestamp(i))*1e-6;
    end
    fprintf('姿态给定采样周期: %f (ms)， 频率: %f （Hz） \n', 1000*mean(attitude_setpoint_delta_t), 1/mean(attitude_setpoint_delta_t));
end 

% ToDo: VTOL
%% =========================================================================
%  (新增) 准备可视化状态数据
%  说明：此部分仅解析 intervals 用于画图，不修改物理数据，与上方 Tailsitter 修正不冲突
% =========================================================================
vis_flight_intervals = [];
vis_flight_names = {};
vis_vtol_intervals = [];
vis_vtol_names = {};
vis_is_vtol = false;
if(isfield(log.data, 'vehicle_status_0'))
    % log.data.vehicle_status_0.vehicle_type; % If the vehicle is a VTOL, then this value will be VEHICLE_TYPE_ROTARY_WING=1 while flying as a multicopter, and VEHICLE_TYPE_FIXED_WING=2 when flying as a fixed-wing
    % log.data.vehicle_status_0.is_vtol; %True if the system is VTOL capable
    % log.data.vehicle_status_0.is_vtol_tailsitter;% True if the system performs a 90° pitch down rotation during transition from MC to FW
    % log.data.vehicle_status_0.in_transition_mode;%True if VTOL is doing a transition

    % 仅当 tailsitter 才做
    if max(log.data.vehicle_status_0.is_vtol_tailsitter) == 1
    
        fw_intervals = get_fw_intervals(log.data.vehicle_status_0);
    
        % 1) attitude: 覆盖 quaternion
        vehicle_attitude_quat = apply_tailsitter_attitude_fix(log.data.vehicle_attitude_0.timestamp,vehicle_attitude_quat, fw_intervals);
        eulZYX = quat2eul(vehicle_attitude_quat, 'ZYX'); % rad: [yaw pitch roll]
        Yaw   = eulZYX(:,1);
        Pitch = eulZYX(:,2);
        Roll  = eulZYX(:,3);
    
        % 2) rates: 覆盖角速度
        vehicle_angular_velocity = apply_tailsitter_rate_fix(log.data.vehicle_angular_velocity_0.timestamp, vehicle_angular_velocity, fw_intervals);
    
        % 3) rates setpoint: 覆盖设定值
        vehicle_rates_setpoint = apply_tailsitter_rate_sp_fix(log.data.vehicle_rates_setpoint_0.timestamp, vehicle_rates_setpoint, fw_intervals);
    
    end
    % 1. 解析飞行模式 (用于主背景)
    [vis_flight_intervals, vis_flight_names] = get_flight_mode_intervals(log.data.vehicle_status_0);
    
    % 2. 解析 VTOL 状态 (用于底部状态带)
    % 检查是否为 VTOL (依据 VehicleStatus 定义)
    % is_vtol 字段存在且为 true
    if ismember('is_vtol', log.data.vehicle_status_0.Properties.VariableNames) && ...
       max(log.data.vehicle_status_0.is_vtol) == 1
        vis_is_vtol = true;
        [vis_vtol_intervals, vis_vtol_names] = get_vtol_state_intervals(log.data.vehicle_status_0);
    end
end




if(isfield(log.data, 'vehicle_local_position_0'))
    XYZ=[log.data.vehicle_local_position_0.x,log.data.vehicle_local_position_0.y,log.data.vehicle_local_position_0.z];
    V_XYZ=[log.data.vehicle_local_position_0.vx,log.data.vehicle_local_position_0.vy,log.data.vehicle_local_position_0.vz];
    pose_N=size(log.data.vehicle_local_position_0.timestamp,1);
    pose_delta_t=zeros(pose_N-1,1);
    for i=1:pose_N-1
        pose_delta_t(i)=(log.data.vehicle_local_position_0.timestamp(i+1)-log.data.vehicle_local_position_0.timestamp(i))*1e-6;
    end
    fprintf('位置采样周期: %f (ms)， 频率: %f （Hz） \n', 1000*mean(pose_delta_t), 1/mean(pose_delta_t));
end 

if(isfield(log.data, 'vehicle_local_position_setpoint_0'))
    XYZ_setpoint=[log.data.vehicle_local_position_setpoint_0.x,log.data.vehicle_local_position_setpoint_0.y,log.data.vehicle_local_position_setpoint_0.z];
    V_XYZ_setpoint=[log.data.vehicle_local_position_setpoint_0.vx,log.data.vehicle_local_position_setpoint_0.vy,log.data.vehicle_local_position_setpoint_0.vz];
    pose_setpoint_N=size(log.data.vehicle_local_position_setpoint_0.timestamp,1);
    pose_setpoint_delta_t=zeros(pose_setpoint_N-1,1);
    for i=1:pose_setpoint_N-1
        pose_setpoint_delta_t(i)=(log.data.vehicle_local_position_setpoint_0.timestamp(i+1)-log.data.vehicle_local_position_setpoint_0.timestamp(i))*1e-6;
    end
    fprintf('位置给定采样周期: %f (ms)， 频率: %f （Hz） \n', 1000*mean(pose_setpoint_delta_t), 1/mean(pose_setpoint_delta_t));
end

% mixer: for the version before 1.13.0
if(isfield(log.data, 'actuator_controls_0_0'))
    Roll_control_0=log.data.actuator_controls_0_0.control_0_;
    Pitch_control_0=log.data.actuator_controls_0_0.control_1_;
    Yaw_control_0=log.data.actuator_controls_0_0.control_2_;
    thrust_sp_0=log.data.actuator_controls_0_0.control_3_;
    if(ismember('indi_fb_0_', log.data.actuator_controls_0_0.Properties.VariableNames))
        indi_feedback_0=[log.data.actuator_controls_0_0.indi_fb_0_,log.data.actuator_controls_0_0.indi_fb_1_,log.data.actuator_controls_0_0.indi_fb_2_];
        error_feedback_0=[log.data.actuator_controls_0_0.error_fb_0_,log.data.actuator_controls_0_0.error_fb_1_,log.data.actuator_controls_0_0.error_fb_2_];
    end
    actuator_N_0=size(log.data.actuator_controls_0_0.timestamp,1);
    actuator_delta_t_0=zeros(actuator_N_0-1,1);
    actuator_delta_0=zeros(actuator_N_0-1,1);
    for i=1:actuator_N_0-1
        actuator_delta_t_0(i)=(log.data.actuator_controls_0_0.timestamp(i+1)-log.data.actuator_controls_0_0.timestamp(i))*1e-6;
    end
    for i=1:actuator_N_0-1
        actuator_delta_0(i)=log.data.actuator_controls_0_0.control_0_(i+1)-log.data.actuator_controls_0_0.control_0_(i) ;
    end
    fprintf('控制器输出采样周期: %f (ms)， 频率: %f （Hz） \n', 1000*mean(actuator_delta_t_0), 1/mean(actuator_delta_t_0));
end
 
% for new allocator
if(isfield(log.data, 'vehicle_torque_setpoint_0'))
    Roll_control=log.data.vehicle_torque_setpoint_0.xyz_0_;
    Pitch_control=log.data.vehicle_torque_setpoint_0.xyz_1_;
    Yaw_control=log.data.vehicle_torque_setpoint_0.xyz_2_;
    
    vehicle_torque_setpoint_N=size(log.data.vehicle_torque_setpoint_0.timestamp,1);
    vehicle_torque_setpoint_delta_t=zeros(vehicle_torque_setpoint_N-1,1);
    vehicle_torque_setpoint_delta=zeros(vehicle_torque_setpoint_N-1,1);
    for i=1:vehicle_torque_setpoint_N-1
        vehicle_torque_setpoint_delta_t(i)=(log.data.vehicle_torque_setpoint_0.timestamp(i+1)-log.data.vehicle_torque_setpoint_0.timestamp(i))*1e-6;
    end
    for i=1:vehicle_torque_setpoint_N-1
        vehicle_torque_setpoint_delta(i)=log.data.vehicle_torque_setpoint_0.xyz_0_(i+1)-log.data.vehicle_torque_setpoint_0.xyz_0_(i) ;
    end
    fprintf('力/力矩给定采样周期: %f (ms)， 频率: %f （Hz） \n', 1000*mean(vehicle_torque_setpoint_delta_t), 1/mean(vehicle_torque_setpoint_delta_t));
end

if(isfield(log.data, 'vehicle_thrust_setpoint_0'))
    thrust_sp=log.data.vehicle_thrust_setpoint_0.xyz_2_;

end

if(isfield(log.data, 'actuator_outputs_0'))
    actuator_outputs_pwm=[log.data.actuator_outputs_0.output_0_,log.data.actuator_outputs_0.output_1_,...
        log.data.actuator_outputs_0.output_2_,log.data.actuator_outputs_0.output_3_,...
        log.data.actuator_outputs_0.output_4_,log.data.actuator_outputs_0.output_5_,...
        log.data.actuator_outputs_0.output_6_,log.data.actuator_outputs_0.output_7_,...
        log.data.actuator_outputs_0.output_8_,log.data.actuator_outputs_0.output_9_,...
        log.data.actuator_outputs_0.output_10_,log.data.actuator_outputs_0.output_11_,...
        log.data.actuator_outputs_0.output_12_,log.data.actuator_outputs_0.output_13_,...
        log.data.actuator_outputs_0.output_14_,log.data.actuator_outputs_0.output_15_];
    cs_pwm_N=size(log.data.actuator_outputs_0.timestamp,1);
    cs_pwm_delta_t=zeros(cs_pwm_N-1,1);
    cs_pwm_delta=zeros(cs_pwm_N-1,1);
    for i=1:cs_pwm_N-1
        cs_pwm_delta_t(i)=(log.data.actuator_outputs_0.timestamp(i+1)-log.data.actuator_outputs_0.timestamp(i))*1e-6;
    end
    for i=1:cs_pwm_N-1
        cs_pwm_delta(i)=log.data.actuator_outputs_0.output_1_(i+1)-log.data.actuator_outputs_0.output_1_(i);
    end
    fprintf('pwm输出采样周期: %f (ms)， 频率: %f （Hz） \n', 1000*mean(cs_pwm_delta_t), 1/mean(cs_pwm_delta_t));


    disp('正在解析 PWM 输出通道定义...');
    % 1. 扫描 PWM_MAIN_FUNC1 到 PWM_MAIN_FUNC16
    % ------------------------------------------------
    active_channels = struct('idx', {}, 'code', {}, 'name', {}, 'col_name', {}, 'type', {});
    
    % PX4 的 actuator_outputs 通常对应 Main 输出
    % 注意：PWM_MAIN_FUNC1 对应 output_0_, FUNC2 对应 output_1_ ...
    for i = 1:16
        param_name = sprintf('PWM_MAIN_FUNC%d', i);
        
        if isfield(log.params, param_name)
            code = double(log.params.(param_name));
            
            % code = 0 代表 Disabled，跳过
            if code ~= 0
                % 获取列名 (log 中的列名通常是 output_0_, output_1_ ...)
                col_name = sprintf('output_%d_', i-1);
                
                % 检查该列是否在数据表中真实存在
                if ismember(col_name, log.data.actuator_outputs_0.Properties.VariableNames)
                    
                    % 记录这个通道的信息
                    new_idx = length(active_channels) + 1;
                    active_channels(new_idx).idx = i;
                    active_channels(new_idx).code = code;
                    active_channels(new_idx).name = lookup_pwm_func(code);
                    active_channels(new_idx).col_name = col_name;
                    
                    % 简单的分类：100段是电机，其他归为舵机/杂项
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

% for new allocator
if(isfield(log.data, 'actuator_motors_0'))
    motors=[log.data.actuator_motors_0.control_0_,log.data.actuator_motors_0.control_1_,...
        log.data.actuator_motors_0.control_2_,log.data.actuator_motors_0.control_3_,...
        log.data.actuator_motors_0.control_4_,log.data.actuator_motors_0.control_5_,...
        log.data.actuator_motors_0.control_6_,log.data.actuator_motors_0.control_7_,...
        log.data.actuator_motors_0.control_8_,log.data.actuator_motors_0.control_9_,...
        log.data.actuator_motors_0.control_10_,log.data.actuator_motors_0.control_11_];

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

% for new allocator
if(isfield(log.data, 'actuator_servos_0'))
    servos=[log.data.actuator_servos_0.control_0_,log.data.actuator_servos_0.control_1_,...
        log.data.actuator_servos_0.control_2_,log.data.actuator_servos_0.control_3_,...
        log.data.actuator_servos_0.control_4_,log.data.actuator_servos_0.control_5_,...
        log.data.actuator_servos_0.control_6_,log.data.actuator_servos_0.control_7_];
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



% plot
if(isfield(log.data, 'vehicle_angular_velocity_0') && isfield(log.data, 'vehicle_rates_setpoint_0'))
    fig1=figure(1);
    subplot(311)
    plot(log.data.vehicle_rates_setpoint_0.timestamp*1e-6, vehicle_rates_setpoint(:,1)*r2d,'k-','LineWidth',1);hold on;
    plot(log.data.vehicle_angular_velocity_0.timestamp*1e-6, vehicle_angular_velocity(:,1)*r2d,'--','LineWidth',1,'color',[0.6,0.2,0]);hold on;
    grid on;
    add_standard_background(vis_flight_intervals, vis_flight_names, vis_is_vtol, vis_vtol_intervals, vis_vtol_names);
    % axis([start_time end_time -inf inf]);
    xlabel({'Time (s)'});
    ylabel('p (deg/s)')
    title('Angular velocity');
    legend('Setpoint','Response','Location', 'best');
    %% 
    subplot(312)
    plot(log.data.vehicle_rates_setpoint_0.timestamp*1e-6, vehicle_rates_setpoint(:,2)*r2d,'k-','LineWidth',1);hold on;
    plot(log.data.vehicle_angular_velocity_0.timestamp*1e-6, vehicle_angular_velocity(:,2)*r2d,'--','LineWidth',1,'color',[0.6,0.2,0]);hold on;
    grid on;
    add_standard_background(vis_flight_intervals, vis_flight_names, vis_is_vtol, vis_vtol_intervals, vis_vtol_names);
    % axis([start_time end_time -inf inf]);
    xlabel({'Time (s)'});
    ylabel('q (deg/s)')
    legend('Setpoint','Response','Location', 'best');
    %% 
    subplot(313)
    plot(log.data.vehicle_rates_setpoint_0.timestamp*1e-6, vehicle_rates_setpoint(:,3)*r2d,'k-','LineWidth',1);hold on;
    plot(log.data.vehicle_angular_velocity_0.timestamp*1e-6, vehicle_angular_velocity(:,3)*r2d,'--','LineWidth',1,'color',[0.6,0.2,0]);hold on;
    grid on;add_standard_background(vis_flight_intervals, vis_flight_names, vis_is_vtol, vis_vtol_intervals, vis_vtol_names);
    % axis([start_time end_time -inf inf]);
    xlabel({'Time (s)'});
    ylabel('r (deg/s)')
    % title('Yaw angular velocity');
    legend('Setpoint','Response','Location', 'best');
    %% 
    % PlotToFileColorPDF(fig1,'results/pqr.pdf',15,18); 
end

%% plot
% if(isfield(log.data, 'vehicle_angular_velocity_0') && isfield(log.data, 'vehicle_rates_setpoint_0'))
%     fig1=figure(1);
%     ax1 = subplot(311);
%     plot(log.data.vehicle_rates_setpoint_0.timestamp*1e-6, vehicle_rates_setpoint(:,1)*r2d,'k-','LineWidth',1); hold on;
%     plot(log.data.vehicle_angular_velocity_0.timestamp*1e-6, vehicle_angular_velocity(:,1)*r2d,'--','LineWidth',1,'color',[0.6,0.2,0]);
%     grid on;
%     xlabel({'Time (s)'}); ylabel('p (deg/s)'); title('Roll Rate');
%     legend('Setpoint','Response','Location', 'best');
% 
%     % === 一行代码添加背景 ===
%     add_standard_background(vis_flight_intervals, vis_flight_names, vis_is_vtol, vis_vtol_intervals, vis_vtol_names);
% 
%     % --- Subplot 2 ---
%     ax2 = subplot(312);
%     plot(log.data.vehicle_rates_setpoint_0.timestamp*1e-6, vehicle_rates_setpoint(:,2)*r2d,'k-','LineWidth',1); hold on;
%     plot(log.data.vehicle_angular_velocity_0.timestamp*1e-6, vehicle_angular_velocity(:,2)*r2d,'--','LineWidth',1,'color',[0.6,0.2,0]);
%     grid on; ylabel('q (deg/s)');
% 
%     % 重复背景绘制逻辑
%     add_standard_background(vis_flight_intervals, vis_flight_names, vis_is_vtol, vis_vtol_intervals, vis_vtol_names);
% 
% 
%     % --- Subplot 3 ---
%     ax3 = subplot(313);
%     plot(log.data.vehicle_rates_setpoint_0.timestamp*1e-6, vehicle_rates_setpoint(:,3)*r2d,'k-','LineWidth',1); hold on;
%     plot(log.data.vehicle_angular_velocity_0.timestamp*1e-6, vehicle_angular_velocity(:,3)*r2d,'--','LineWidth',1,'color',[0.6,0.2,0]);
%     grid on; ylabel('r (deg/s)');
% 
%     % 重复背景绘制逻辑
%     add_standard_background(vis_flight_intervals, vis_flight_names, vis_is_vtol, vis_vtol_intervals, vis_vtol_names);
% 
% 
%     linkaxes([ax1, ax2, ax3], 'x');
% end


%% 
if(isfield(log.data, 'vehicle_attitude_setpoint_0') && isfield(log.data, 'vehicle_attitude_0'))
    fig2=figure(2);
    subplot(311)
    plot(log.data.vehicle_attitude_setpoint_0.timestamp*1e-6, Roll_setpoint*r2d,'k-','LineWidth',1);hold on;
    plot(log.data.vehicle_attitude_0.timestamp*1e-6, Roll*r2d,'--','LineWidth',1,'color',[0.6,0.2,0]);hold on;
    grid on;
    % axis([start_time end_time -inf inf]);
    xlabel({'Time (s)'});
    ylabel('Roll (deg)')
    title('Euler angle');
    legend('Setpoint','Response','Location', 'best');
    %% 
    subplot(312)
    plot(log.data.vehicle_attitude_setpoint_0.timestamp*1e-6, Pitch_setpoint*r2d,'k-','LineWidth',1);hold on;
    plot(log.data.vehicle_attitude_0.timestamp*1e-6, Pitch*r2d,'--','LineWidth',1,'color',[0.6,0.2,0]);hold on;
    grid on;
    % axis([start_time end_time -inf inf]);
    xlabel({'Time (s)'});
    ylabel('Pitch (deg)')
    legend('Setpoint','Response','Location', 'best');
    %% 
    subplot(313)
    plot(log.data.vehicle_attitude_setpoint_0.timestamp*1e-6, Yaw_setpoint*r2d,'k-','LineWidth',1);hold on;
    plot(log.data.vehicle_attitude_0.timestamp*1e-6, Yaw*r2d,'--','LineWidth',1,'color',[0.6,0.2,0]);hold on;
    grid on;
    % axis([start_time end_time -inf inf]);
    xlabel({'Time (s)'});
    ylabel('Yaw (deg)')
    legend('Setpoint','Response','Location', 'best');
    %% 
    % PlotToFileColorPDF(fig2,'results/RPY.pdf',15,18);  
end





%%
if(isfield(log.data, 'vehicle_local_position_0') && isfield(log.data, 'vehicle_local_position_setpoint_0'))
    fig3=figure(3);
    subplot(311)
    plot(log.data.vehicle_local_position_setpoint_0.timestamp*1e-6, V_XYZ_setpoint(:,1),'k-','LineWidth',1);hold on;
    plot(log.data.vehicle_local_position_0.timestamp*1e-6, V_XYZ(:,1),'--','LineWidth',1,'color',[0.6,0.2,0]);hold on;
    grid on;
    % axis([start_time end_time -inf inf]);
    title('Velocity');
    xlabel({'Time (s)'});
    ylabel('V_X (m/s)')
    legend('Setpoint','Response','Location', 'best');
    %%
    subplot(312)
    plot(log.data.vehicle_local_position_setpoint_0.timestamp*1e-6, V_XYZ_setpoint(:,2),'k-','LineWidth',1);hold on;
    plot(log.data.vehicle_local_position_0.timestamp*1e-6, V_XYZ(:,2),'--','LineWidth',1,'color',[0.6,0.2,0]);hold on;
    grid on;
    % axis([start_time end_time -inf inf]);
    xlabel({'Time (s)'});
    ylabel('V_Y (m/s)')
    legend('Setpoint','Response','Location', 'best');
    %%
    subplot(313)
    plot(log.data.vehicle_local_position_setpoint_0.timestamp*1e-6, V_XYZ_setpoint(:,3),'k-','LineWidth',1);hold on;
    plot(log.data.vehicle_local_position_0.timestamp*1e-6, V_XYZ(:,3),'--','LineWidth',1,'color',[0.6,0.2,0]);hold on;
    grid on;
    % axis([start_time end_time -inf inf]);
    xlabel({'Time (s)'});
    ylabel('V_Z (m/s)')
    legend('Setpoint','Response','Location', 'best');
    % PlotToFileColorPDF(fig3,'results/V_XYZ.pdf',15,18);
elseif(isfield(log.data, 'vehicle_local_position_0'))
    fig3=figure(3);
    subplot(311)
    plot(log.data.vehicle_local_position_0.timestamp*1e-6, V_XYZ(:,1),'--','LineWidth',1,'color',[0.6,0.2,0]);hold on;
    grid on;
    % axis([start_time end_time -inf inf]);
    title('Velocity');
    xlabel({'Time (s)'});
    ylabel('V_X (m/s)')
    %%
    subplot(312)
    plot(log.data.vehicle_local_position_0.timestamp*1e-6, V_XYZ(:,2),'--','LineWidth',1,'color',[0.6,0.2,0]);hold on;
    grid on;
    % axis([start_time end_time -inf inf]);
    xlabel({'Time (s)'});
    ylabel('V_Y (m/s)')
    %%
    subplot(313)
    plot(log.data.vehicle_local_position_0.timestamp*1e-6, V_XYZ(:,3),'--','LineWidth',1,'color',[0.6,0.2,0]);hold on;
    grid on;
    % axis([start_time end_time -inf inf]);
    xlabel({'Time (s)'});
    ylabel('V_Z (m/s)')
    %%
    % PlotToFileColorPDF(fig3,'results/V_XYZ.pdf',15,18);
end



%% 
if(isfield(log.data, 'vehicle_local_position_0') && isfield(log.data, 'vehicle_local_position_setpoint_0'))
    fig4=figure(4);
    subplot(311)
    plot(log.data.vehicle_local_position_setpoint_0.timestamp*1e-6, XYZ_setpoint(:,1),'k-','LineWidth',1);hold on;
    plot(log.data.vehicle_local_position_0.timestamp*1e-6, XYZ(:,1),'--','LineWidth',1,'color',[0.6,0.2,0]);hold on;
    grid on;
    % axis([start_time end_time -inf inf]);
    title('Position');
    xlabel({'Time (s)'});
    ylabel('X (m)')
    legend('Setpoint','Response','Location', 'best');
    %%
    subplot(312)
    plot(log.data.vehicle_local_position_setpoint_0.timestamp*1e-6, XYZ_setpoint(:,2),'k-','LineWidth',1);hold on;
    plot(log.data.vehicle_local_position_0.timestamp*1e-6, XYZ(:,2),'--','LineWidth',1,'color',[0.6,0.2,0]);hold on;
    grid on;
    % axis([start_time end_time -inf inf]);
    xlabel({'Time (s)'});
    ylabel('Y (m)')
    legend('Setpoint','Response','Location', 'best');
    %%
    subplot(313)
    plot(log.data.vehicle_local_position_setpoint_0.timestamp*1e-6, XYZ_setpoint(:,3),'k-','LineWidth',1);hold on;
    plot(log.data.vehicle_local_position_0.timestamp*1e-6, XYZ(:,3),'--','LineWidth',1,'color',[0.6,0.2,0]);hold on;
    grid on;
    % axis([start_time end_time -inf inf]);
    xlabel({'Time (s)'});
    ylabel('Z (m)')
    legend('Setpoint','Response','Location', 'best');
    % PlotToFileColorPDF(fig4,'results/P_XYZ.pdf',15,18);
elseif(isfield(log.data, 'vehicle_local_position_0'))
    fig4=figure(4);
    subplot(311)
    plot(log.data.vehicle_local_position_0.timestamp*1e-6, XYZ(:,1),'--','LineWidth',1,'color',[0.6,0.2,0]);hold on;
    grid on;
    % axis([start_time end_time -inf inf]);
    title('Position');
    xlabel({'Time (s)'});
    ylabel('X (m)')
    %%
    subplot(312)
    plot(log.data.vehicle_local_position_0.timestamp*1e-6, XYZ(:,2),'--','LineWidth',1,'color',[0.6,0.2,0]);hold on;
    grid on;
    % axis([start_time end_time -inf inf]);
    xlabel({'Time (s)'});
    ylabel('Y (m)')
    %%
    subplot(313)
    plot(log.data.vehicle_local_position_0.timestamp*1e-6, XYZ(:,3),'--','LineWidth',1,'color',[0.6,0.2,0]);hold on;
    grid on;
    % axis([start_time end_time -inf inf]);
    xlabel({'Time (s)'});
    ylabel('Z (m)')
    %%
    % PlotToFileColorPDF(fig4,'results/P_XYZ.pdf',15,18);
end



%% 
if(isfield(log.data, 'actuator_controls_0_0') )

    fig5=figure(5);
    subplot(511)
    plot(log.data.actuator_controls_0_0.timestamp*1e-6, Roll_control_0(:,1),'r-','LineWidth',1);hold on;
    plot(log.data.actuator_controls_0_0.timestamp*1e-6, Pitch_control_0(:,1),'k--','LineWidth',1,'color',[0.6,0.2,0]);hold on;
    plot(log.data.actuator_controls_0_0.timestamp*1e-6, Yaw_control_0(:,1),'b-.','LineWidth',1);hold on;
    plot(log.data.actuator_controls_0_0.timestamp*1e-6, thrust_sp_0(:,1),'k-','LineWidth',1);hold on;
    grid on;
    % axis([start_time end_time -1 1]);
    title('Controller output group 0');
    xlabel({'Time (s)'});
    ylabel('Value')
    legend('Roll','Pitch',  'Yaw','Thrust (up)','Location', 'best','NumColumns', 4);
    subplot(512)
    plot(log.data.actuator_controls_0_0.timestamp*1e-6, Roll_control_0(:,1),'r-','LineWidth',1);hold on;
    grid on;
    % axis([start_time end_time -1 1]);
    xlabel({'Time (s)'});
    ylabel('Roll')
    subplot(513)
    plot(log.data.actuator_controls_0_0.timestamp*1e-6, Pitch_control_0(:,1),'k--','LineWidth',1,'color',[0.6,0.2,0]);hold on;
    grid on;
    % axis([start_time end_time -1 1]);
    xlabel({'Time (s)'});
    ylabel('Pitch')
    subplot(514)
    plot(log.data.actuator_controls_0_0.timestamp*1e-6, Yaw_control_0(:,1),'b-.','LineWidth',1);hold on;
    grid on;
    % axis([start_time end_time -1 1]);
    xlabel({'Time (s)'});
    ylabel('Yaw')

    subplot(515)
    plot(log.data.actuator_controls_0_0.timestamp*1e-6, thrust_sp_0(:,1),'k-','LineWidth',1);hold on;
    grid on;
    % axis([start_time end_time -1 1]);
    xlabel({'Time (s)'});
    ylabel('Thrust (up)')
    % PlotToFileColorPDF(fig5,'results/actuator_controls_0.pdf',15,18);

%% 

elseif( (isfield(log.data, 'vehicle_torque_setpoint_0') && isfield(log.data, 'vehicle_thrust_setpoint_0') ) )
    fig5=figure(5);
    subplot(511)
    plot(log.data.vehicle_torque_setpoint_0.timestamp*1e-6, Roll_control(:,1),'r-','LineWidth',1);hold on;
    plot(log.data.vehicle_torque_setpoint_0.timestamp*1e-6, Pitch_control(:,1),'k--','LineWidth',1,'color',[0.6,0.2,0]);hold on;
    plot(log.data.vehicle_torque_setpoint_0.timestamp*1e-6, Yaw_control(:,1),'b-.','LineWidth',1);hold on;
    plot(log.data.vehicle_thrust_setpoint_0.timestamp*1e-6, thrust_sp(:,1),'k-','LineWidth',1);hold on;
    grid on;
    % axis([start_time end_time -1 1]);
    title('Controller output: torque and thrust setpoint');
    xlabel({'Time (s)'});
    ylabel('Value')
    legend('Roll','Pitch', 'Yaw','Thrust (up)','Location', 'best','NumColumns', 4);
    %%
    subplot(512)
    plot(log.data.vehicle_torque_setpoint_0.timestamp*1e-6, Roll_control(:,1),'r-','LineWidth',1);hold on;
    grid on;
    % axis([start_time end_time -1 1]);
    xlabel({'Time (s)'});
    ylabel('Roll')
    %%
    subplot(513)
    plot(log.data.vehicle_torque_setpoint_0.timestamp*1e-6, Pitch_control(:,1),'k--','LineWidth',1,'color',[0.6,0.2,0]);hold on;
    grid on;
    % axis([start_time end_time -1 1]);
    xlabel({'Time (s)'});
    ylabel('Pitch')
    %%
    subplot(514)
    plot(log.data.vehicle_torque_setpoint_0.timestamp*1e-6, Yaw_control(:,1),'b-.','LineWidth',1);hold on;
    grid on;
    % axis([start_time end_time -1 1]);
    xlabel({'Time (s)'});
    ylabel('Yaw')
    %%
    subplot(515)
    plot(log.data.vehicle_torque_setpoint_0.timestamp*1e-6, thrust_sp(:,1),'k-','LineWidth',1);hold on;
    grid on;
    % axis([start_time end_time -1 1]);
    xlabel({'Time (s)'});
    ylabel('Thrust (up)')
    %%
    % PlotToFileColorPDF(fig5,'results/vehicle_torque_thrust_setpoint.pdf',15,18);
end

%% 参数设置
MAX_MOTORS = 12; % 最大 12 
MAX_SERVOS = 8;  % 最大 8 
plot_together=1;
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
        
        % --- 关键：限制数量，防止越界或生成过多颜色 ---
        n_motors = min(n_motors, MAX_MOTORS);
        n_servos = min(n_servos, MAX_SERVOS);
    
        % 2. 准备数据存在标志
        has_motor_data = (n_motors > 0) && isfield(log.data, 'actuator_motors_0') && exist('motors', 'var');
        has_servo_data = (n_servos > 0) && isfield(log.data, 'actuator_servos_0') && exist('servos', 'var');
        
        % 3. 计算子图数量：最多只有2个（电机图 + 舵机图）
        total_subplots = double(has_motor_data) + double(has_servo_data);
        
        if total_subplots > 0
            fig6 = figure(6);
            set(fig6, 'Name', 'Actuator Outputs (Merged)', 'Color', 'w');
            current_plot_idx = 1;
            
            % ==========================================================
            % Part 1: 绘制电机 (所有电机在一张图)
            % ==========================================================
            if has_motor_data
                subplot(total_subplots, 1, current_plot_idx);
                hold on;
                
                % --- 颜色生成逻辑 ---
                % 使用 hsv 颜色空间，确保 12 个电机颜色各不相同
                colors_motor = hsv(n_motors); 
                
                for i = 1:n_motors
                    if i <= size(motors, 2)
                        plot(log.data.actuator_motors_0.timestamp*1e-6, motors(:, i), ...
                            'Color', colors_motor(i,:), 'LineWidth', 1, ...
                            'DisplayName', sprintf('Motor %d', i));
                    end
                end
                
                grid on;
                ylabel('Motors Output');
                title(sprintf('Actuator: Motors (Total %d)', n_motors));
                
                % 图例放在外侧，防止遮挡波形
                legend('show'); 
                
                % 如果下面还有舵机图，这层就不显示X轴标签
                if has_servo_data
                    set(gca, 'XTickLabel', []);
                else
                    xlabel('Time (s)');
                end
                
                current_plot_idx = current_plot_idx + 1;
            end
            
            % ==========================================================
            % Part 2: 绘制舵机 (所有舵机在一张图)
            % ==========================================================
            if has_servo_data
                subplot(total_subplots, 1, current_plot_idx);
                hold on;
                
                % --- 颜色生成逻辑 ---
                % 舵机数量较少时用 lines (对比度高)，多了用 hsv (不重复)
                if n_servos <= 7
                    colors_servo = lines(n_servos);
                else
                    colors_servo = hsv(n_servos);
                end
                
                for i = 1:n_servos
                    if i <= size(servos, 2)
                        plot(log.data.actuator_servos_0.timestamp*1e-6, servos(:, i), ...
                            'Color', colors_servo(i,:), 'LineWidth', 1.2, ... 
                            'DisplayName', sprintf('Servo %d', i));
                    end
                end
                
                grid on;
                ylabel('Servos Output');
                title(sprintf('Actuator: Servos (Total %d)', n_servos));
                
                legend('show');
                xlabel('Time (s)');
            end
            
            % 自动调整标题
            % sgtitle('Actuator Outputs Analysis');
        end
        
        % 保存为 PDF (注意高度调整)
        % PlotToFileColorPDF(fig6, 'results/actuator_motors_servos_merged.pdf', 15, 6 * total_subplots);
    end
else
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
                         'Color', this_color, 'LineWidth', 1.2);
                end
    
                grid on;
                ylabel(sprintf('Motor %d', i), 'FontWeight', 'bold', 'Color', this_color);
    
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
                         'Color', this_color, 'LineWidth', 1.2);
                end
    
                grid on;
                % 把 Y 轴标签也设成对应颜色，方便通过颜色定位
                ylabel(sprintf('Servo %d', i), 'FontWeight', 'bold', 'Color', this_color);
    
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

%% --------------------PWM----------------------------
n_active = length(active_channels);
if n_active > 0
    fig8 = figure(8);
    set(fig8, 'Name', 'Actuator Outputs (PWM)', 'Color', 'w');
    
    t_pwm = log.data.actuator_outputs_0.timestamp * 1e-6;
    
    % 策略：电机画在一张图，其他（舵机、脚架等）每个单独画一张图
    motors_list = active_channels(strcmp({active_channels.type}, 'Motor'));
    others_list = active_channels(strcmp({active_channels.type}, 'Servo/Other'));
    
    % 计算总子图数
    has_motors = ~isempty(motors_list);
    n_others = length(others_list);
    total_subplots = double(has_motors) + n_others;
    
    current_plot = 1;
    
    % --- 子图 1: 所有电机 (如果有) ---
    if has_motors
        subplot(total_subplots, 1, current_plot);
        hold on;
        % 电机通常数量多，使用 hsv 保证颜色最大化区分
        colors_motor = hsv(length(motors_list));
        
        for k = 1:length(motors_list)
            chn = motors_list(k);
            y_data = log.data.actuator_outputs_0.(chn.col_name);
            
            % 使用分配的颜色
            this_color = colors_motor(k, :);
            
            plot(t_pwm, y_data, 'Color', this_color, 'LineWidth', 1, ...
                'DisplayName', sprintf('%s (Ch%d)', chn.name, chn.idx));
        end
        grid on;
        ylabel('PWM (us)');
        title('Motors PWM Output');
        legend('show');
        
        if current_plot < total_subplots
            set(gca, 'XTickLabel', []);
        else
            xlabel('Time (s)');
        end
        current_plot = current_plot + 1;
    end
    
    % --- 子图 2~N: 其他通道 (舵机、起落架、相机触发等) ---
    % 为其他通道生成颜色表
    if n_others > 0
        % 如果数量少于7个，用 lines 对比度更高；否则用 hsv
        if n_others <= 7
            colors_others = lines(n_others);
        else
            colors_others = hsv(n_others);
        end
    end

    for k = 1:n_others
        chn = others_list(k);
        subplot(total_subplots, 1, current_plot);
        hold on;
        
        y_data = log.data.actuator_outputs_0.(chn.col_name);
        
        % 取出对应颜色
        this_color = colors_others(k, :);
        
        % 使用指定颜色画图
        plot(t_pwm, y_data, 'Color', this_color, 'LineWidth', 1.2);
        
        grid on;
        % 将 Y 轴标签颜色也设为相同，增加视觉辨识度
        ylabel('PWM (us)', 'Color', this_color, 'FontWeight', 'bold');
        
        % 标题显示具体功能，例如 "Servo 1 (Ch5)" 或 "Landing Gear (Ch6)"
        title(sprintf('%s (Output Ch%d)', chn.name, chn.idx));
        
        if current_plot < total_subplots
            set(gca, 'XTickLabel', []);
        else
            xlabel('Time (s)');
        end
        current_plot = current_plot + 1;
    end
    
    % PlotToFileColorPDF(fig8, 'results/actuator_outputs_parsed.pdf', 15, 4 * total_subplots);
else
    fprintf('没有找到启用的 PWM 输出通道 (PWM_MAIN_FUNCx 全为 0 或数据缺失)。\n');
end



%%
if(isfield(log.data, 'vehicle_local_position_0') && isfield(log.data, 'vehicle_local_position_setpoint_0'))
    fig9=figure(9);
    plot3(XYZ_setpoint(:,1), XYZ_setpoint(:,2), -XYZ_setpoint(:,3), 'LineStyle', '-', 'LineWidth', 1);
    hold on;
    plot3(XYZ(:,1), XYZ(:,2), -XYZ(:,3), 'LineStyle', ':', 'LineWidth', 1);
    title('Trajectory');
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    legend1=legend('Setpoint', 'Response');
    grid on;
    view(45, 30);
    hold off;
    %%
    % PlotToFileColorPDF(fig9,'results/trj.pdf',15,15);
end


fig10=figure(10);
subplot(311)
plot(log.data.vehicle_angular_velocity_0.timestamp*1e-6, vehicle_angular_acceleration(:,1),'k-','LineWidth',1);hold on;
plot(log.data.vehicle_angular_velocity_0.timestamp*1e-6, vehicle_angular_velocity(:,1),'--','LineWidth',1,'color',[0.6,0.2,0,0.5]);hold on;
grid on;
% axis([-inf inf -20 20]);
title('Angular Acceleration');
xlabel({'Time (s)'});
ylabel('p (rad/s^2)')
legend('angular acc','gyro');

subplot(312)
plot(log.data.vehicle_angular_velocity_0.timestamp*1e-6, vehicle_angular_acceleration(:,2),'k-','LineWidth',1);hold on;
plot(log.data.vehicle_angular_velocity_0.timestamp*1e-6, vehicle_angular_velocity(:,2),'--','LineWidth',1,'color',[0.6,0.2,0,0.5]);hold on;
grid on;
% axis([-inf inf -20 20]);
xlabel({'Time (s)'});
ylabel('q (rad/s^2)')
legend('angular acc','gyro');

subplot(313)
plot(log.data.vehicle_angular_velocity_0.timestamp*1e-6, vehicle_angular_acceleration(:,3),'k-','LineWidth',1);hold on;
plot(log.data.vehicle_angular_velocity_0.timestamp*1e-6, vehicle_angular_velocity(:,3),'--','LineWidth',1,'color',[0.6,0.2,0,0.5]);hold on;
grid on;
% axis([-inf inf -20 20]);
xlabel({'Time (s)'});
ylabel('r (rad/s^2)')
legend('angular acc','gyro');
% PlotToFileColorPDF(fig10,'results/vehicle_angular_acceleration.pdf',15,18);
