clear all;
close all;
clc;
addpath(genpath(pwd));
% you can run on terminal 
% ulog2csv log_.ulg 
% to get csv files
% =====================1==========================
% Install pyulog using pip first.https://github.com/PX4/pyulog.
% in MacOS, it maybe have been installed by the px4-dev
% =====================2==========================
% Make sure it has installed ulog2csv correctly (check the output of which ulog2csv in Linux/MacOS or where ulog2csv in Windows).
% =====================3==========================
% Change the following line in ulogviewver.m:
% command = ['!/usr/local/bin/ulog2csv ' ulgFileName '.ulg'];
% to 
% command = ['!your ulog2csv path' ulgFileName '.ulg'];
% and 
% ulgFileName = '00_41_22'; 
% to 
% ulgFileName = 'your log name (full path)'; 

% ----fig size, you have to change it for your fig

d2r=pi/180;
r2d=180/pi;
%------------------------------------------
% Set ULog relative path
%------------------------------------------
ulgFileName = '/data/13_26_53'; % 1.12.3: 09_19_57, 1.15.4: 13_39_55, main(1.17.0): 13_26_53
tmp = [ulgFileName '.mat'];

% Record the current main script path
rootDir = fileparts(mfilename('fullpath'));

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
        ulog2csv_path = '~/Library/Python/3.9/bin/ulog2csv';
    else
        ulog2csv_path = 'ulog2csv';
    end

    ulgAbs = fullfile(rootDir, [ulgFileName '.ulg']);
    command = ['!' ulog2csv_path ' ' '"' ulgAbs '"'];
    disp(['Running command: ' command]);
    eval(command);

    %------------------------------------------
    % Step 3: Call parsing function (pass full path)
    %------------------------------------------
    log.data = csv_topics_to_d(fullfile(rootDir, ulgFileName));
    log.FileName = ulgFileName;
    log.version = 1.0;
    log.params = '';
    log.messages = '';
    log.info = '';

    %------------------------------------------
    % Step 4: Save MAT file to the same directory
    %------------------------------------------
    save(fullfile(rootDir, tmp), 'log');
    disp(['Saved MAT file: ' tmp]);

    %------------------------------------------
    % Step 5: Delete temporary CSV files
    %------------------------------------------
    delete(fullfile(rootDir, [ulgFileName '_*.csv']));
    disp('Temporary CSV files deleted.');
end
% more data can be atained by pyulog:
% ulog_params sample.ulg
% ulog_messages sample.ulg
% ulog_info sample.ulg
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
if(isfield(log.data, 'vehicle_status_0'))
    % log.data.vehicle_status_0.vehicle_type; % If the vehicle is a VTOL, then this value will be VEHICLE_TYPE_ROTARY_WING=1 while flying as a multicopter, and VEHICLE_TYPE_FIXED_WING=2 when flying as a fixed-wing
    % log.data.vehicle_status_0.is_vtol; %True if the system is VTOL capable
    % log.data.vehicle_status_0.is_vtol_tailsitter;% True if the system performs a 90° pitch down rotation during transition from MC to FW
    % log.data.vehicle_status_0.in_transition_mode;%True if VTOL is doing a transition

    % 仅当 tailsitter 才做
    if max(log.data.vehicle_status_0.is_vtol_tailsitter) == 1
    
        fw_intervals = get_fw_intervals(log.data.vehicle_status_0);
    
        % 1) attitude: 覆盖 quaternion
        vehicle_attitude_quat = apply_tailsitter_attitude_fix(vehicle_attitude_quat, fw_intervals);
        eulZYX = quat2eul(vehicle_attitude_quat, 'ZYX'); % rad: [yaw pitch roll]
        Yaw   = eulZYX(:,1);
        Pitch = eulZYX(:,2);
        Roll  = eulZYX(:,3);
    
        % 2) rates: 覆盖角速度
        vehicle_angular_velocity = apply_tailsitter_rate_fix(vehicle_angular_velocity, fw_intervals);
    
        % 3) rates setpoint: 覆盖设定值
        vehicle_rates_setpoint = apply_tailsitter_rate_sp_fix(vehicle_rates_setpoint, fw_intervals);
    
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



%% plot
if(isfield(log.data, 'vehicle_angular_velocity_0') && isfield(log.data, 'vehicle_rates_setpoint_0'))
    fig1=figure(1);
    subplot(311)
    plot(log.data.vehicle_rates_setpoint_0.timestamp*1e-6, vehicle_rates_setpoint(:,1)*r2d,'k-','LineWidth',1);hold on;
    plot(log.data.vehicle_angular_velocity_0.timestamp*1e-6, vehicle_angular_velocity(:,1)*r2d,'--','LineWidth',1,'color',[0.6,0.2,0]);hold on;
    grid on;
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
    % axis([start_time end_time -inf inf]);
    xlabel({'Time (s)'});
    ylabel('q (deg/s)')
    legend('Setpoint','Response','Location', 'best');
    %% 
    subplot(313)
    plot(log.data.vehicle_rates_setpoint_0.timestamp*1e-6, vehicle_rates_setpoint(:,3)*r2d,'k-','LineWidth',1);hold on;
    plot(log.data.vehicle_angular_velocity_0.timestamp*1e-6, vehicle_angular_velocity(:,3)*r2d,'--','LineWidth',1,'color',[0.6,0.2,0]);hold on;
    grid on;
    % axis([start_time end_time -inf inf]);
    xlabel({'Time (s)'});
    ylabel('r (deg/s)')
    % title('Yaw angular velocity');
    legend('Setpoint','Response','Location', 'best');
    %% 
    % PlotToFileColorPDF(fig1,'results/pqr.pdf',15,18); 
end


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





%% 
if((isfield(log.data, 'actuator_motors_0') && isfield(log.data, 'actuator_servos_0') ) )
    fig6=figure(6);
    subplot(511)
    plot(log.data.actuator_motors_0.timestamp*1e-6, motors(:,1),'k-','LineWidth',1);hold on;
    grid on;
    % axis([start_time end_time -1 1]);
    title('actuator: motors and servos, [0,1] or [-1,1].');
    xlabel({'Time (s)'});
    ylabel('motors')
    %%
    subplot(512)
    plot(log.data.actuator_servos_0.timestamp*1e-6, servos(:,1),'r-','LineWidth',1);hold on;
    grid on;
    % axis([start_time end_time -1 1]);
    xlabel({'Time (s)'});
    ylabel('servo1')
    %%
    subplot(513)
    plot(log.data.actuator_servos_0.timestamp*1e-6, servos(:,2),'k--','LineWidth',1,'color',[0.6,0.2,0]);hold on;
    grid on;
    % axis([start_time end_time -1 1]);
    xlabel({'Time (s)'});
    ylabel('servo2')
    %%
    subplot(514)
    plot(log.data.actuator_servos_0.timestamp*1e-6, servos(:,3),'b-.','LineWidth',1);hold on;
    grid on;
    % axis([start_time end_time -1 1]);
    xlabel({'Time (s)'});
    ylabel('servo3')
    %%
    subplot(515)
    plot(log.data.actuator_servos_0.timestamp*1e-6, servos(:,4),'k-','LineWidth',1);hold on;
    grid on;
    % axis([start_time end_time -1 1]);
    xlabel({'Time (s)'});
    ylabel('servo4')
    %%
    % PlotToFileColorPDF(fig6,'results/actuator_motors_servos.pdf',15,18);
end





%% 
if(isfield(log.data, 'actuator_outputs_0'))
    fig7=figure(7);
    subplot(511)
    plot(log.data.actuator_outputs_0.timestamp*1e-6, actuator_outputs_pwm(:,1),'g-','LineWidth',1);hold on;
    grid on;
    % axis([-inf inf 1000 2000]);
    title('actuator outputs (pwm)');
    xlabel({'Time (s)'});
    ylabel('rotor (pwm)')
    legend('rotor');
    %%
    subplot(512)
    plot(log.data.actuator_outputs_0.timestamp*1e-6, actuator_outputs_pwm(:,2),'r-','LineWidth',1);hold on;
    grid on;
    % axis([-inf inf 1000 2000]);
    xlabel({'Time (s)'});
    ylabel('cs1 (pwm)')
    %%
    subplot(513)
    plot(log.data.actuator_outputs_0.timestamp*1e-6, actuator_outputs_pwm(:,3),'k--','LineWidth',1);hold on;
    grid on;
    % axis([-inf inf 1000 2000]);
    xlabel({'Time (s)'});
    ylabel('cs2 (pwm)')
    %%
    subplot(514)
    plot(log.data.actuator_outputs_0.timestamp*1e-6, actuator_outputs_pwm(:,4),'b-.','LineWidth',1);hold on;
    grid on;
    % axis([-inf inf 1000 2000]);
    xlabel({'Time (s)'});
    ylabel('cs3 (pwm)')
    %%
    subplot(515)
    plot(log.data.actuator_outputs_0.timestamp*1e-6, actuator_outputs_pwm(:,5),'g-','LineWidth',1);hold on;
    grid on;
    % axis([-inf inf 1000 2000]);
    xlabel({'Time (s)'});
    ylabel('cs4 (pwm)')
    %% 
    % PlotToFileColorPDF(fig7,'results/actuator_outputs.pdf',15,18);
end




%%
if(isfield(log.data, 'vehicle_local_position_0') && isfield(log.data, 'vehicle_local_position_setpoint_0'))
    fig8=figure(8);
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
    % PlotToFileColorPDF(fig8,'results/trj.pdf',15,15);
end


fig9=figure(9);
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
% PlotToFileColorPDF(fig9,'results/vehicle_angular_acceleration.pdf',15,18);
