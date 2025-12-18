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
% ulgFileName = 'your log name'; 

% ----fig size, you have to change it for your fig

d2r=pi/180;
r2d=180/pi;
%------------------------------------------
% 设定 ULog 相对路径
%------------------------------------------
ulgFileName = 'data/06_40_01';
tmp = [ulgFileName '.mat'];

% 记录当前主脚本路径
rootDir = fileparts(mfilename('fullpath'));

%------------------------------------------
% Step 1: 检查是否已有 MAT 文件
%------------------------------------------
if exist(fullfile(rootDir, tmp), "file")
    disp(['✅ Found MAT file: ' tmp]);
    load(fullfile(rootDir, tmp), 'log');

else
    disp('⚙️ No MAT file found, start parsing ULog...');

    %------------------------------------------
    % Step 2: 执行 ulog2csv（保持路径完整）
    %------------------------------------------
    if ismac
        ulog2csv_path = '/Users/mch/Library/Python/3.9/bin/ulog2csv';
    else
        ulog2csv_path = 'ulog2csv';
    end

    ulgAbs = fullfile(rootDir, [ulgFileName '.ulg']);
    command = ['!' ulog2csv_path ' ' '"' ulgAbs '"'];
    disp(['📄 Running command: ' command]);
    eval(command);

    %------------------------------------------
    % Step 3: 调用解析函数（传完整路径）
    %------------------------------------------
    log.data = csv_topics_to_d(fullfile(rootDir, ulgFileName));
    log.FileName = ulgFileName;
    log.version = 1.0;
    log.params = '';
    log.messages = '';
    log.info = '';

    %------------------------------------------
    % Step 4: 保存 MAT 文件到同目录
    %------------------------------------------
    save(fullfile(rootDir, tmp), 'log');
    disp(['💾 Saved MAT file: ' tmp]);

    %------------------------------------------
    % Step 5: 删除临时 CSV
    %------------------------------------------
    delete(fullfile(rootDir, [ulgFileName '_*.csv']));
    disp('🧹 Temporary CSV files deleted.');
end
%%

if(isfield(log.data, 'parameter_update_0'))
    parameter_update=log.data.parameter_update_0{:,:};
    flag=parameter_update(:,1);
    
end 
if(isfield(log.data, 'input_rc_0'))
    input_rc=log.data.input_rc_0{:,:};
    [input_rc_N,~]=size(input_rc(:,1));
    input_rc_delta_t=zeros(input_rc_N-1,1);
    for i=1:input_rc_N-1
        input_rc_delta_t(i)=(input_rc(i+1,1))*1e-6-(input_rc(i,1))*1e-6;
    end
    
end 

%% sitl
rate_dowm_simple=1;
att_dowm_simple=1;
att_set_dowm_simple=1;
% k=0.05;
%% fmu
% rate_dowm_simple=3;
% att_dowm_simple=2;
% att_set_dowm_simple=1;
%%
if(isfield(log.data, 'vehicle_angular_velocity_0'))
    vehicle_angular_velocity=log.data.vehicle_angular_velocity_0{:,:}(1:rate_dowm_simple:end, :);
    [rate_N,~]=size(vehicle_angular_velocity(:,1));
    rate_delta_t=zeros(rate_N-1,1);
    for i=1:rate_N-1
        rate_delta_t(i)=(vehicle_angular_velocity(i+1,1))*1e-6-(vehicle_angular_velocity(i,1))*1e-6;
    end
end 
if(isfield(log.data, 'vehicle_angular_acceleration_0'))
    vehicle_angular_acceleration=log.data.vehicle_angular_acceleration_0{:,:}(1:rate_dowm_simple:end, :);
    [rate_acc_N,~]=size(vehicle_angular_acceleration(:,1));
    rate_acc_delta_t=zeros(rate_acc_N-1,1);
    for i=1:rate_acc_N-1
        rate_acc_delta_t(i)=(vehicle_angular_acceleration(i+1,1))*1e-6-(vehicle_angular_acceleration(i,1))*1e-6;
    end
end 
if(isfield(log.data, 'vehicle_rates_setpoint_0'))
    vehicle_rates_setpoint=log.data.vehicle_rates_setpoint_0{:,:}(1:att_dowm_simple:end, :);
    [rate_setpoint_N,~]=size(vehicle_rates_setpoint(:,1));
    rate_setpoint_delta_t=zeros(rate_setpoint_N-1,1);
    for i=1:rate_setpoint_N-1
        rate_setpoint_delta_t(i)=(vehicle_rates_setpoint(i+1,1))*1e-6-(vehicle_rates_setpoint(i,1))*1e-6;
    end
end 
if(isfield(log.data, 'vehicle_attitude_0'))
    vehicle_attitude=log.data.vehicle_attitude_0{:,:}(1:att_dowm_simple:end, :);
    eul = quat2eul(vehicle_attitude(:,3:6));
    q_0=vehicle_attitude(:,3);
    q_1=vehicle_attitude(:,4);
    q_2=vehicle_attitude(:,5);
    q_3=vehicle_attitude(:,6);
    Roll=quat_to_roll([q_0 q_1 q_2 q_3]);
    Pitch=quat_to_pitch([q_0 q_1 q_2 q_3]);
    Yaw=quat_to_yaw([q_0 q_1 q_2 q_3]);
    [attitude_N,~]=size(vehicle_attitude(:,1));
    attitude_delta_t=zeros(attitude_N-1,1);
    for i=1:attitude_N-1
        attitude_delta_t(i)=(vehicle_attitude(i+1,1))*1e-6-(vehicle_attitude(i,1))*1e-6;
    end
end 
if(isfield(log.data, 'vehicle_attitude_setpoint_0'))
    vehicle_attitude_setpoint=log.data.vehicle_attitude_setpoint_0{:,:}(1:att_set_dowm_simple:end, :);
    
    q_0_setpoint=vehicle_attitude_setpoint(:,6);
    q_1_setpoint=vehicle_attitude_setpoint(:,7);
    q_2_setpoint=vehicle_attitude_setpoint(:,8);
    q_3_setpoint=vehicle_attitude_setpoint(:,9);
    Roll_setpoint=quat_to_roll([q_0_setpoint q_1_setpoint q_2_setpoint q_3_setpoint]);
    Pitch_setpoint=quat_to_pitch([q_0_setpoint q_1_setpoint q_2_setpoint q_3_setpoint]);
    Yaw_setpoint=quat_to_yaw([q_0_setpoint q_1_setpoint q_2_setpoint q_3_setpoint]);
    [attitude_setpoint_N,~]=size(vehicle_attitude_setpoint(:,1));
    attitude_setpoint_delta_t=zeros(attitude_setpoint_N-1,1);
    for i=1:attitude_setpoint_N-1
        attitude_setpoint_delta_t(i)=(vehicle_attitude_setpoint(i+1,1))*1e-6-(vehicle_attitude_setpoint(i,1))*1e-6;
    end
end 
% eul(:,3)=Roll
% eul(:,2)=Pitch
% eul(:,1)=Yaw

if(isfield(log.data, 'vehicle_status_0'))
    vehicle_status=log.data.vehicle_status_0{:,:};  
    vehicle_type=vehicle_status(:,16); % If the vehicle is a VTOL, then this value will be VEHICLE_TYPE_ROTARY_WING=1 while flying as a multicopter, and VEHICLE_TYPE_FIXED_WING=2 when flying as a fixed-wing
    is_vtol=vehicle_status(:,17); %True if the system is VTOL capable
    is_vtol_tailsitter=vehicle_status(:,18);% True if the system performs a 90° pitch down rotation during transition from MC to FW
    in_transition_mode=vehicle_status(:,20);%True if VTOL is doing a transition
   

    % 仅当 tailsitter 才做
    if max(is_vtol_tailsitter) == 1
    
        fw_intervals = get_fw_intervals(vehicle_status);
    
        % 1) attitude: 覆盖 quaternion
        vehicle_attitude = apply_tailsitter_attitude_fix(vehicle_attitude, fw_intervals);
        eulZYX = quat2eul(vehicle_attitude(:,3:6), 'ZYX'); % rad: [yaw pitch roll]
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
    vehicle_local_position=log.data.vehicle_local_position_0{:,:};
    XYZ=vehicle_local_position(:,6:8);
    V_XYZ=vehicle_local_position(:,12:14);
    [pose_N,~]=size(vehicle_local_position(:,1));
    pose_delta_t=zeros(pose_N-1,1);
    for i=1:pose_N-1
        pose_delta_t(i)=(vehicle_local_position(i+1,1))*1e-6-(vehicle_local_position(i,1))*1e-6;
    end
end 

if(isfield(log.data, 'vehicle_local_position_setpoint_0'))
    vehicle_local_position_setpoint=log.data.vehicle_local_position_setpoint_0{:,:};
    XYZ_setpoint=vehicle_local_position_setpoint(:,2:4);
    V_XYZ_setpoint=vehicle_local_position_setpoint(:,7:9);
    [pose_setpoint_N,~]=size(vehicle_local_position_setpoint(:,1));
    pose_setpoint_delta_t=zeros(pose_setpoint_N-1,1);
    for i=1:pose_setpoint_N-1
        pose_setpoint_delta_t(i)=(vehicle_local_position_setpoint(i+1,1))*1e-6-(vehicle_local_position_setpoint(i,1))*1e-6;
    end
end 

if(isfield(log.data, 'actuator_controls_0_0'))
    actuator_controls_0=log.data.actuator_controls_0_0{:,:}(1:rate_dowm_simple:end, :);   
    Roll_control_0=actuator_controls_0(:,3);
    Pitch_control_0=actuator_controls_0(:,4);
    Yaw_control_0=actuator_controls_0(:,5);
    thrust_sp_0=actuator_controls_0(:,6);
    if(ismember('indi_fb_0_', log.data.actuator_controls_0_0.Properties.VariableNames))
        indi_feedback_0=actuator_controls_0(:,11:13);
        error_feedback_0=actuator_controls_0(:,14:16);
    end
    [actuator_N_0,~]=size(actuator_controls_0(:,1));
    actuator_delta_t_0=zeros(actuator_N_0-1,1);
    actuator_delta_0=zeros(actuator_N_0-1,1);
    for i=1:actuator_N_0-1
        actuator_delta_t_0(i)=(actuator_controls_0(i+1,1))*1e-6-(actuator_controls_0(i,1))*1e-6;
    end
    for i=1:actuator_N_0-1
        actuator_delta_0(i)=actuator_controls_0(i+1,3)-actuator_controls_0(i,3) ;
    end
end
if(isfield(log.data, 'actuator_controls_1_0'))
    actuator_controls_1=log.data.actuator_controls_1_0{:,:}(1:rate_dowm_simple:end, :);   
    Roll_control_1=actuator_controls_1(:,3);
    Pitch_control_1=actuator_controls_1(:,4);
    Yaw_control_1=actuator_controls_1(:,5);
    thrust_sp_1=actuator_controls_1(:,6);  
    if(ismember('indi_fb_0_', log.data.actuator_controls_1_0.Properties.VariableNames))
        indi_feedback_1=actuator_controls_1(:,11:13);
        error_feedback_1=actuator_controls_1(:,14:16);
    end
    [actuator_N_1,~]=size(actuator_controls_1(:,1));
    actuator_delta_t_1=zeros(actuator_N_1-1,1);
    actuator_delta_1=zeros(actuator_N_1-1,1);
    for i=1:actuator_N_1-1
        actuator_delta_t_1(i)=(actuator_controls_1(i+1,1))*1e-6-(actuator_controls_1(i,1))*1e-6;
    end
    for i=1:actuator_N_1-1
        actuator_delta_1(i)=actuator_controls_1(i+1,3)-actuator_controls_1(i,3) ;
    end
end
if(isfield(log.data, 'actuator_outputs_0'))
    actuator_outputs=log.data.actuator_outputs_0{:,:}(1:rate_dowm_simple:end, :);   
    cs1=actuator_outputs(:,7);
    cs2=actuator_outputs(:,8);
    cs3=actuator_outputs(:,9);
    cs4=actuator_outputs(:,10);
    [cs_N,~]=size(actuator_outputs(:,1));
    cs_delta_t=zeros(cs_N-1,1);
    cs_delta=zeros(cs_N-1,1);
    for i=1:cs_N-1
        cs_delta_t(i)=(actuator_outputs(i+1,1))*1e-6-(actuator_outputs(i,1))*1e-6;
    end
    for i=1:cs_N-1
        cs_delta(i)=actuator_outputs(i+1,7)-actuator_outputs(i,7);
    end
end


if(isfield(log.data, 'allocation_value_0'))
    allocation_value=log.data.allocation_value_0{:,:}(1:rate_dowm_simple:end, :);
    [allocation_value_N,~]=size(allocation_value(:,1));
    allocation_value_delta_t=zeros(allocation_value_N-1,1);
    allocation_value_delta=zeros(allocation_value_N-1,1);
    for i=1:allocation_value_N-1
        allocation_value_delta_t(i)=(allocation_value(i+1,1))*1e-6-(allocation_value(i,1))*1e-6;
    end
    for i=1:allocation_value_N-1
        allocation_value_delta(i)=allocation_value(i+1,12)-allocation_value(i,12);
    end
end 

if(isfield(log.data, 'vehicle_visual_odometry_0'))
    vehicle_visual_odometry=log.data.vehicle_visual_odometry_0{:,:};
    visual_odometry_X=vehicle_visual_odometry(:,3);
    visual_odometry_Y=vehicle_visual_odometry(:,4);
    visual_odometry_Z=vehicle_visual_odometry(:,5);
    visual_odometry_q0=vehicle_visual_odometry(:,6);
    visual_odometry_q1=vehicle_visual_odometry(:,7);
    visual_odometry_q2=vehicle_visual_odometry(:,8);
    visual_odometry_q3=vehicle_visual_odometry(:,9);
    
end

start_time=35;
end_time=55;
%% plot
if(isfield(log.data, 'vehicle_angular_velocity_0') && isfield(log.data, 'vehicle_rates_setpoint_0'))
    fig1=figure(1);
    subplot(311)
    plot((vehicle_rates_setpoint(:,1))*1e-6, vehicle_rates_setpoint(:,2)*r2d,'k-','LineWidth',1);hold on;
    plot((vehicle_angular_velocity(:,1))*1e-6, vehicle_angular_velocity(:,3)*r2d,'--','LineWidth',1,'color',[0.6,0.2,0]);hold on;
    grid on;
    axis([start_time end_time -inf inf]);
    xlabel({'Time (s)'});
    ylabel('p (deg/s)')
    title('Angular velocity');
    legend('Setpoint','Response','Location', 'best');
    %% 
    subplot(312)
    plot((vehicle_rates_setpoint(:,1))*1e-6, vehicle_rates_setpoint(:,3)*r2d,'k-','LineWidth',1);hold on;
    plot((vehicle_angular_velocity(:,1))*1e-6, vehicle_angular_velocity(:,4)*r2d,'--','LineWidth',1,'color',[0.6,0.2,0]);hold on;
    grid on;
    axis([start_time end_time -inf inf]);
    xlabel({'Time (s)'});
    ylabel('q (deg/s)')
    legend('Setpoint','Response','Location', 'best');
    %% 
    subplot(313)
    plot((vehicle_rates_setpoint(:,1))*1e-6, vehicle_rates_setpoint(:,4)*r2d,'k-','LineWidth',1);hold on;
    plot((vehicle_angular_velocity(:,1))*1e-6, vehicle_angular_velocity(:,5)*r2d,'--','LineWidth',1,'color',[0.6,0.2,0]);hold on;
    grid on;
    axis([start_time end_time -inf inf]);
    xlabel({'Time (s)'});
    ylabel('r (deg/s)')
    % title('Yaw angular velocity');
    legend('Setpoint','Response','Location', 'best');
    %% 
    PlotToFileColorPDF(fig1,'results/pqr.pdf',15,18); 
end


%% and maybe more figure, all in the variable "log.data"
if(isfield(log.data, 'vehicle_attitude_setpoint_0') && isfield(log.data, 'vehicle_attitude_0'))
    fig2=figure(2);
    subplot(311)
    plot((vehicle_attitude_setpoint(:,1))*1e-6, Roll_setpoint*r2d,'k-','LineWidth',1);hold on;
    plot((vehicle_attitude(:,1))*1e-6, Roll*r2d,'--','LineWidth',1,'color',[0.6,0.2,0]);hold on;
    grid on;
    axis([start_time end_time -inf inf]);
    xlabel({'Time (s)'});
    ylabel('Roll (deg)')
    title('Euler angle');
    legend('Setpoint','Response','Location', 'best');
    %% and maybe more figure, all in the variable "log.data"
    subplot(312)
    plot((vehicle_attitude_setpoint(:,1))*1e-6, Pitch_setpoint*r2d,'k-','LineWidth',1);hold on;
    plot((vehicle_attitude(:,1))*1e-6, Pitch*r2d,'--','LineWidth',1,'color',[0.6,0.2,0]);hold on;
    grid on;
    axis([start_time end_time -inf inf]);
    xlabel({'Time (s)'});
    ylabel('Pitch (deg)')
    legend('Setpoint','Response','Location', 'best');
    %% and maybe more figure, all in the variable "log.data"
    subplot(313)
    plot((vehicle_attitude_setpoint(:,1))*1e-6, Yaw_setpoint*r2d,'k-','LineWidth',1);hold on;
    plot((vehicle_attitude(:,1))*1e-6, Yaw*r2d,'--','LineWidth',1,'color',[0.6,0.2,0]);hold on;
    grid on;
    axis([start_time end_time -inf inf]);
    xlabel({'Time (s)'});
    ylabel('Yaw (deg)')
    legend('Setpoint','Response','Location', 'best');
    %% 
    PlotToFileColorPDF(fig2,'results/RPY.pdf',15,18);  
end





%% and maybe more figure, all in the variable "log.data"
if(isfield(log.data, 'vehicle_local_position_0') && isfield(log.data, 'vehicle_local_position_setpoint_0'))

    fig3=figure(3);
    subplot(311)
    plot((vehicle_local_position_setpoint(:,1))*1e-6, V_XYZ_setpoint(:,1),'k-','LineWidth',1);hold on;
    plot((vehicle_local_position(:,1))*1e-6, V_XYZ(:,1),'--','LineWidth',1,'color',[0.6,0.2,0]);hold on;
    grid on;
    axis([start_time end_time -inf inf]);
    title('Velocity');
    xlabel({'Time (s)'});
    ylabel('V_X (m/s)')
    legend('Setpoint','Response','Location', 'best');
    %% and maybe more figure, all in the variable "log.data"
    subplot(312)
    plot((vehicle_local_position_setpoint(:,1))*1e-6, V_XYZ_setpoint(:,2),'k-','LineWidth',1);hold on;
    plot((vehicle_local_position(:,1))*1e-6, V_XYZ(:,2),'--','LineWidth',1,'color',[0.6,0.2,0]);hold on;
    grid on;
    axis([start_time end_time -inf inf]);
    xlabel({'Time (s)'});
    ylabel('V_Y (m/s)')
    legend('Setpoint','Response','Location', 'best');
    %% and maybe more figure, all in the variable "log.data"
    subplot(313)
    plot((vehicle_local_position_setpoint(:,1))*1e-6, V_XYZ_setpoint(:,3),'k-','LineWidth',1);hold on;
    plot((vehicle_local_position(:,1))*1e-6, V_XYZ(:,3),'--','LineWidth',1,'color',[0.6,0.2,0]);hold on;
    grid on;
    axis([start_time end_time -inf inf]);
    xlabel({'Time (s)'});
    ylabel('V_Z (m/s)')
    legend('Setpoint','Response','Location', 'best');
    PlotToFileColorPDF(fig3,'results/V_XYZ.pdf',15,18);
elseif(isfield(log.data, 'vehicle_local_position_0'))
    fig3=figure(3);
    subplot(311)
    plot((vehicle_local_position(:,1))*1e-6, V_XYZ(:,1),'--','LineWidth',1,'color',[0.6,0.2,0]);hold on;
    grid on;
    axis([start_time end_time -inf inf]);
    title('Velocity');
    xlabel({'Time (s)'});
    ylabel('V_X (m/s)')
    %% and maybe more figure, all in the variable "log.data"
    subplot(312)
    plot((vehicle_local_position(:,1))*1e-6, V_XYZ(:,2),'--','LineWidth',1,'color',[0.6,0.2,0]);hold on;
    grid on;
    axis([start_time end_time -inf inf]);
    xlabel({'Time (s)'});
    ylabel('V_Y (m/s)')
    %% and maybe more figure, all in the variable "log.data"
    subplot(313)
    plot((vehicle_local_position(:,1))*1e-6, V_XYZ(:,3),'--','LineWidth',1,'color',[0.6,0.2,0]);hold on;
    grid on;
    axis([start_time end_time -inf inf]);
    xlabel({'Time (s)'});
    ylabel('V_Z (m/s)')
    PlotToFileColorPDF(fig3,'results/V_XYZ.pdf',15,18);
end
%% 


if(isfield(log.data, 'vehicle_local_position_0') && isfield(log.data, 'vehicle_local_position_setpoint_0'))

%% and maybe more figure, all in the variable "log.data"
    fig4=figure(4);
    subplot(311)
    plot((vehicle_local_position_setpoint(:,1))*1e-6, XYZ_setpoint(:,1),'k-','LineWidth',1);hold on;
    plot((vehicle_local_position(:,1))*1e-6, XYZ(:,1),'--','LineWidth',1,'color',[0.6,0.2,0]);hold on;
    grid on;
    axis([start_time end_time -inf inf]);
    title('Position');
    xlabel({'Time (s)'});
    ylabel('X (m)')
    legend('Setpoint','Response','Location', 'best');
    %% and maybe more figure, all in the variable "log.data"
    subplot(312)
    plot((vehicle_local_position_setpoint(:,1))*1e-6, XYZ_setpoint(:,2),'k-','LineWidth',1);hold on;
    plot((vehicle_local_position(:,1))*1e-6, XYZ(:,2),'--','LineWidth',1,'color',[0.6,0.2,0]);hold on;
    grid on;
    axis([start_time end_time -inf inf]);
    xlabel({'Time (s)'});
    ylabel('Y (m)')
    legend('Setpoint','Response','Location', 'best');
    %% and maybe more figure, all in the variable "log.data"
    subplot(313)
    plot((vehicle_local_position_setpoint(:,1))*1e-6, XYZ_setpoint(:,3),'k-','LineWidth',1);hold on;
    plot((vehicle_local_position(:,1))*1e-6, XYZ(:,3),'--','LineWidth',1,'color',[0.6,0.2,0]);hold on;
    grid on;
    axis([start_time end_time -inf inf]);
    xlabel({'Time (s)'});
    ylabel('Z (m)')
    legend('Setpoint','Response','Location', 'best');
    PlotToFileColorPDF(fig4,'results/P_XYZ.pdf',15,18);
elseif(isfield(log.data, 'vehicle_local_position_0'))
    fig4=figure(4);
    subplot(311)
    plot((vehicle_local_position(:,1))*1e-6, XYZ(:,1),'--','LineWidth',1,'color',[0.6,0.2,0]);hold on;
    grid on;
    axis([start_time end_time -inf inf]);
    title('Position');
    xlabel({'Time (s)'});
    ylabel('X (m)')
    %% and maybe more figure, all in the variable "log.data"
    subplot(312)
    plot((vehicle_local_position(:,1))*1e-6, XYZ(:,2),'--','LineWidth',1,'color',[0.6,0.2,0]);hold on;
    grid on;
    axis([start_time end_time -inf inf]);
    xlabel({'Time (s)'});
    ylabel('Y (m)')
    %% and maybe more figure, all in the variable "log.data"
    subplot(313)
    plot((vehicle_local_position(:,1))*1e-6, XYZ(:,3),'--','LineWidth',1,'color',[0.6,0.2,0]);hold on;
    grid on;
    axis([start_time end_time -inf inf]);
    xlabel({'Time (s)'});
    ylabel('Z (m)')
    PlotToFileColorPDF(fig4,'results/P_XYZ.pdf',15,18);
%%
end
%% 
% 



%% 
if(isfield(log.data, 'actuator_controls_0_0'))

    fig5=figure(5);
    subplot(511)
    plot((actuator_controls_0(:,1))*1e-6, Roll_control_0(:,1),'r-','LineWidth',1);hold on;
    plot((actuator_controls_0(:,1))*1e-6, Pitch_control_0(:,1),'k--','LineWidth',1,'color',[0.6,0.2,0]);hold on;
    plot((actuator_controls_0(:,1))*1e-6, Yaw_control_0(:,1),'b-.','LineWidth',1);hold on;
    plot((actuator_controls_0(:,1))*1e-6, thrust_sp_0(:,1),'k-','LineWidth',1);hold on;
    grid on;
    axis([start_time end_time -1 1]);
    title('Controller output group 0');
    xlabel({'Time (s)'});
    ylabel('Value')
    legend('Roll','Pitch',  'Yaw','Thrust (up)','Location', 'best','NumColumns', 4);
    subplot(512)
    plot((actuator_controls_0(:,1))*1e-6, Roll_control_0(:,1),'r-','LineWidth',1);hold on;
    grid on;
    axis([start_time end_time -1 1]);
    xlabel({'Time (s)'});
    ylabel('Roll')
    subplot(513)
    plot((actuator_controls_0(:,1))*1e-6, Pitch_control_0(:,1),'k--','LineWidth',1,'color',[0.6,0.2,0]);hold on;
    grid on;
    axis([start_time end_time -1 1]);
    xlabel({'Time (s)'});
    ylabel('Pitch')
    subplot(514)
    plot((actuator_controls_0(:,1))*1e-6, Yaw_control_0(:,1),'b-.','LineWidth',1);hold on;
    grid on;
    axis([start_time end_time -1 1]);
    xlabel({'Time (s)'});
    ylabel('Yaw')

    subplot(515)
    plot((actuator_controls_0(:,1))*1e-6, thrust_sp_0(:,1),'k-','LineWidth',1);hold on;
    grid on;
    axis([start_time end_time -1 1]);
    xlabel({'Time (s)'});
    ylabel('Thrust (up)')
    PlotToFileColorPDF(fig5,'results/actuator_controls_0.pdf',15,18);

%% 
    if(ismember('indi_fb_0_', log.data.actuator_controls_0_0.Properties.VariableNames))
        fig6=figure(6);
        subplot(411)
        plot((actuator_controls_0(:,1))*1e-6, indi_feedback_0(:,1),'r-','LineWidth',1);hold on;
        plot((actuator_controls_0(:,1))*1e-6, indi_feedback_0(:,2),'k--','LineWidth',1);hold on;
        plot((actuator_controls_0(:,1))*1e-6, indi_feedback_0(:,3),'b-.','LineWidth',1);hold on;
        grid on;
        % axis([-inf inf -0.5 0.5]);
        title('Trim-control term');
        xlabel({'Time (s)'});
        ylabel('Value')
        legend('Roll','Pitch','Yaw');
        subplot(412)
        plot((actuator_controls_0(:,1))*1e-6, indi_feedback_0(:,1),'r-','LineWidth',1);hold on;
        grid on;
        % axis([-inf inf -0.5 0.5]);
        xlabel({'Time (s)'});
        ylabel('Roll')
        subplot(413)
        plot((actuator_controls_0(:,1))*1e-6, indi_feedback_0(:,2),'k--','LineWidth',1);hold on;
        grid on;
        % axis([-inf inf -0.5 0.5]);
        xlabel({'Time (s)'});
        ylabel('Pitch')
        subplot(414)
        plot((actuator_controls_0(:,1))*1e-6, indi_feedback_0(:,3),'b-.','LineWidth',1);hold on;
        grid on;
        % axis([-inf inf -0.5 0.5]);
        xlabel({'Time (s)'});
        ylabel('Yaw')
        
        PlotToFileColorPDF(fig6,'results/indi_feedback_0.pdf',15,18);
    
    %% 
        fig7=figure(7);
        subplot(411)
        plot((actuator_controls_0(:,1))*1e-6, error_feedback_0(:,1),'r-','LineWidth',1);hold on;
        plot((actuator_controls_0(:,1))*1e-6, error_feedback_0(:,2),'k--','LineWidth',1);hold on;
        plot((actuator_controls_0(:,1))*1e-6, error_feedback_0(:,3),'b-.','LineWidth',1);hold on;
    
        grid on;
        % axis([-inf inf -0.5 0.5]);
        title('Error-feedback term');
        xlabel({'Time (s)'});
        ylabel('Value')
        legend('Roll','Pitch','Yaw');
        subplot(412)
        plot((actuator_controls_0(:,1))*1e-6, error_feedback_0(:,1),'r-','LineWidth',1);hold on;
        grid on;
        % axis([-inf inf -0.5 0.5]);
        xlabel({'Time (s)'});
        ylabel('Roll')
        subplot(413)
        plot((actuator_controls_0(:,1))*1e-6, error_feedback_0(:,2),'k--','LineWidth',1);hold on;
        grid on;
        % axis([-inf inf -0.5 0.5]);
        xlabel({'Time (s)'});
        ylabel('Pitch')
        subplot(414)
        plot((actuator_controls_0(:,1))*1e-6, error_feedback_0(:,3),'b-.','LineWidth',1);hold on;
        grid on;
        % axis([-inf inf -0.5 0.5]);
        xlabel({'Time (s)'});
        ylabel('Yaw') 
        PlotToFileColorPDF(fig7,'results/error_control_input.pdf',15,18);
    end

end



%% 
if(isfield(log.data, 'actuator_controls_1_0'))

    fig8=figure(8);
    subplot(511)
    plot((actuator_controls_1(:,1))*1e-6, Roll_control_1(:,1),'r-','LineWidth',1);hold on;
    plot((actuator_controls_1(:,1))*1e-6, Pitch_control_1(:,1),'k--','LineWidth',1,'color',[0.6,0.2,0]);hold on;
    plot((actuator_controls_1(:,1))*1e-6, Yaw_control_1(:,1),'b-.','LineWidth',1);hold on;
    plot((actuator_controls_1(:,1))*1e-6, thrust_sp_1(:,1),'k-','LineWidth',1);hold on;
    grid on;
    axis([start_time end_time -1 1]);
    title('Controller output group 1');
    xlabel({'Time (s)'});
    ylabel('Value')
    legend('Roll','Pitch',  'Yaw', 'Thrust (forward)','Location', 'best','NumColumns', 4);
    subplot(512)
    plot((actuator_controls_1(:,1))*1e-6, Roll_control_1(:,1),'r-','LineWidth',1);hold on;
    grid on;
    axis([start_time end_time -1 1]);
    xlabel({'Time (s)'});
    ylabel('Roll')
    subplot(513)
    plot((actuator_controls_1(:,1))*1e-6, Pitch_control_1(:,1),'k--','LineWidth',1,'color',[0.6,0.2,0]);hold on;
    grid on;
    axis([start_time end_time -1 1]);
    xlabel({'Time (s)'});
    ylabel('Pitch')
    subplot(514)
    plot((actuator_controls_1(:,1))*1e-6, Yaw_control_1(:,1),'b-.','LineWidth',1);hold on;
    grid on;
    axis([start_time end_time -1 1]);
    xlabel({'Time (s)'});
    ylabel('Yaw')

    subplot(515)
    plot((actuator_controls_1(:,1))*1e-6, thrust_sp_1(:,1),'k-','LineWidth',1);hold on;
    grid on;
    axis([start_time end_time -1 1]);
    xlabel({'Time (s)'});
    ylabel('Thrust (forward)')
    PlotToFileColorPDF(fig8,'results/actuator_controls_1.pdf',15,18);

               
end



%% 
if(isfield(log.data, 'actuator_outputs_0'))

    fig9=figure(9);
    subplot(511)
    plot((actuator_outputs(:,1))*1e-6, cs1(:,1),'r-','LineWidth',1);hold on;
    plot((actuator_outputs(:,1))*1e-6, cs2(:,1),'k--','LineWidth',1);hold on;
    plot((actuator_outputs(:,1))*1e-6, cs3(:,1),'b-.','LineWidth',1);hold on;
    plot((actuator_outputs(:,1))*1e-6, cs4(:,1),'g-','LineWidth',1);hold on;
    grid on;
    % axis([-inf inf 1000 2000]);
    title('Control surface');
    xlabel({'Time (s)'});
    ylabel('Value (pwm)')
    legend('cs1','cs2','cs3','cs4');
    subplot(512)
    plot((actuator_outputs(:,1))*1e-6, cs1(:,1),'r-','LineWidth',1);hold on;
    grid on;
    % axis([-inf inf 1000 2000]);
    xlabel({'Time (s)'});
    ylabel('cs1')
    subplot(513)
    plot((actuator_outputs(:,1))*1e-6, cs2(:,1),'k--','LineWidth',1);hold on;
    grid on;
    % axis([-inf inf 1000 2000]);
    xlabel({'Time (s)'});
    ylabel('cs2')
    subplot(514)
    plot((actuator_outputs(:,1))*1e-6, cs3(:,1),'b-.','LineWidth',1);hold on;
    grid on;
    % axis([-inf inf 1000 2000]);
    xlabel({'Time (s)'});
    ylabel('cs3')
    subplot(515)
    plot((actuator_outputs(:,1))*1e-6, cs4(:,1),'g-','LineWidth',1);hold on;
    grid on;
    % axis([-inf inf 1000 2000]);
    xlabel({'Time (s)'});
    ylabel('cs4')
    %% 
    % PlotToFileColorPDF(fig9,'results/cs.pdf',15,18);
end




%%
if(isfield(log.data, 'vehicle_local_position_0') && isfield(log.data, 'vehicle_local_position_setpoint_0'))
    fig10=figure(10);
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
    PlotToFileColorPDF(fig10,'results/trj.pdf',15,15);
end

%%
if(isfield(log.data, 'vehicle_angular_acceleration_0'))
    fig11=figure(11);
    subplot(311)
    plot((vehicle_angular_acceleration(:,1))*1e-6, vehicle_angular_acceleration(:,3),'k-','LineWidth',1);hold on;
    plot((vehicle_angular_velocity(:,1))*1e-6, vehicle_angular_velocity(:,3),'--','LineWidth',1,'color',[0.6,0.2,0,0.5]);hold on;
    grid on;
    % axis([-inf inf -20 20]);
    title('Angular Acceleration');
    xlabel({'Time (s)'});
    ylabel('p (rad/s^2)')
    legend('angular acc','gyro');
    %% and maybe more figure, all in the variable "log.data"
    subplot(312)
    plot((vehicle_angular_acceleration(:,1))*1e-6, vehicle_angular_acceleration(:,4),'k-','LineWidth',1);hold on;
    plot((vehicle_angular_velocity(:,1))*1e-6, vehicle_angular_velocity(:,4),'--','LineWidth',1,'color',[0.6,0.2,0,0.5]);hold on;
    grid on;
    % axis([-inf inf -20 20]);
    xlabel({'Time (s)'});
    ylabel('q (rad/s^2)')
    legend('angular acc','gyro');
    %% and maybe more figure, all in the variable "log.data"
    subplot(313)
    plot((vehicle_angular_acceleration(:,1))*1e-6, vehicle_angular_acceleration(:,5),'k-','LineWidth',1);hold on;
    plot((vehicle_angular_velocity(:,1))*1e-6, vehicle_angular_velocity(:,5),'--','LineWidth',1,'color',[0.6,0.2,0,0.5]);hold on;
    grid on;
    % axis([-inf inf -20 20]);
    xlabel({'Time (s)'});
    ylabel('r (rad/s^2)')
    legend('angular acc','gyro');
    % PlotToFileColorPDF(fig11,'results/vehicle_angular_acceleration.pdf',15,18);

end

if(isfield(log.data, 'vehicle_visual_odometry_0') && isfield(log.data, 'vehicle_attitude_0')) 
    fig12=figure(12);
    plot((vehicle_visual_odometry(:,1))*1e-6, visual_odometry_q0,'k-','LineWidth',1);hold on;
    plot((vehicle_attitude(:,1))*1e-6, q_0,'r-','LineWidth',1);hold on;grid on;
    % The sampling frequencies are different, so only a rough estimation can be made
    % Method 1 for calculating time delay
    % % Use the finddelay function
    % timeDelay = finddelay(q_0, visual_odometry_q0);
    % % Display result
    % disp(['The time delay between the signals is ', num2str(timeDelay), ' samples.']);

    % Method 2 for calculating time delay
    % % Compute cross-correlation
    % [c, lags] = xcorr(visual_odometry_q0, q_0);
    % 
    % % Find the position of maximum correlation
    % [~, I] = max(c);
    % timeDelay = lags(I);
    % 
    % % Display result
    % disp(['The time delay between the signals is ', num2str(timeDelay), ' samples.']);

    % Method 3 for calculating time delay
    % X_q_0 = fft(q_0);
    % Y_visual_odometry_q0 = fft(visual_odometry_q0);
    % % Compute phase difference
    % dPhi = angle(Y_visual_odometry_q0 ./ X_q_0);
    % % Compute time delay
    % frequencies = (0:length(t)-1) * (fs/length(t));
    % timeDelay = mean(dPhi ./ (2 * pi * frequencies));
    % 
    % % Display result
    % disp(['The estimated time delay between the signals is ', num2str(timeDelay), ' seconds.']);

end


if(isfield(log.data, 'allocation_value_0') && isfield(log.data, 'actuator_outputs_0'))
    figure,
    subplot(211)
    plot((allocation_value(:,1))*1e-6, allocation_value(:,12),'r-','LineWidth',1);hold on;
    plot((allocation_value(:,1))*1e-6, allocation_value(:,13),'k--','LineWidth',1);hold on;
    plot((allocation_value(:,1))*1e-6, allocation_value(:,14),'b-.','LineWidth',1);hold on;
    plot((allocation_value(:,1))*1e-6, allocation_value(:,15),'g-','LineWidth',1);hold on;
if(isfield(log.data, 'input_rc_0'))
    plot((input_rc(:,1))*1e-6, (input_rc(:,15)-1094)/420,'k-','LineWidth',1);hold on;
end
    
    title('Allocator output');
    xlabel({'Time (s)'});
    ylabel('u')
    legend('cs1','cs2','cs3','cs4');

    subplot(212)
    plot((actuator_outputs(:,1))*1e-6, cs1(:,1),'r-','LineWidth',1);hold on;
    plot((actuator_outputs(:,1))*1e-6, cs2(:,1),'k--','LineWidth',1);hold on;
    plot((actuator_outputs(:,1))*1e-6, cs3(:,1),'b-.','LineWidth',1);hold on;
    plot((actuator_outputs(:,1))*1e-6, cs4(:,1),'g-','LineWidth',1);hold on;
    grid on;
    % axis([-inf inf 1000 2000]);
    title('cs command');
    xlabel({'Time (s)'});
    ylabel('偏转指令 (pwm)')
    legend('cs1','cs2','cs3','cs4');
    
end


if ismember('rate_control_running_time', log.data.actuator_controls_0_0.Properties.VariableNames)
    disp('INDI running time (us)');
    mean(actuator_controls_0(actuator_controls_0(:,19)==1,18))
    disp('PID running time (us)');
    mean(actuator_controls_0(actuator_controls_0(:,19)==0,18))
end


%%
if(isfield(log.data, 'vehicle_angular_velocity_0'))
    figure,
    plot(1:rate_N-1, rate_delta_t,'k-','LineWidth',1);hold on;
    ylabel('time (s)');
    title('vehicle angular velocity');
    disp('mean(rate_delta_t)');
    mean(rate_delta_t)/rate_dowm_simple
end
if(isfield(log.data, 'vehicle_rates_setpoint_0'))
    figure,
    plot(1:rate_setpoint_N-1, rate_setpoint_delta_t,'k-','LineWidth',1);hold on;
    ylabel('time (s)');
    title('vehicle rate setpoint');
    disp('mean(rate_setpoint_delta_t)');
    mean(rate_setpoint_delta_t)/att_dowm_simple
end
if(isfield(log.data, 'vehicle_angular_acceleration_0'))
    figure,
    plot(1:rate_acc_N-1, rate_acc_delta_t,'k-','LineWidth',1);hold on;
    ylabel('time (s)');
    title('vehicle angular acceleration');
    disp('mean(rate_acc_delta_t)');
    mean(rate_acc_delta_t)
end
if(isfield(log.data, 'vehicle_attitude_0'))
    figure,
    plot(1:attitude_N-1, attitude_delta_t,'k-','LineWidth',1);hold on;
    ylabel('time (s)');
    title('vehicle attitude');
    disp('mean(attitude_delta_t)');
    mean(attitude_delta_t)/att_dowm_simple

end
if(isfield(log.data, 'vehicle_attitude_setpoint_0'))
    figure,
    plot(1:attitude_setpoint_N-1, attitude_setpoint_delta_t,'k-','LineWidth',1);hold on;
    ylabel('time (s)');
    title('vehicle attitude setpoint');
    disp('mean(attitude_setpoint_delta_t)');
    mean(attitude_setpoint_delta_t)/att_set_dowm_simple

end
if(isfield(log.data, 'vehicle_local_position_0'))
    figure,
    plot(1:pose_N-1, pose_delta_t,'k-','LineWidth',1);hold on;
    ylabel('time (s)');
    title('vehicle local position');
    disp('mean(pose_delta_t)');
    mean(pose_delta_t)
end
if(isfield(log.data, 'vehicle_local_position_setpoint_0'))
    figure,
    plot(1:pose_setpoint_N-1, pose_setpoint_delta_t,'k-','LineWidth',1);hold on;
    ylabel('time (s)');
    title('vehicle local position setpoint');
    disp('mean(pose_setpoint_delta_t)');
    mean(pose_setpoint_delta_t)
end
if(isfield(log.data, 'actuator_controls_0_0'))
    figure,
    plot(1:actuator_N_0-1, actuator_delta_t_0,'k-','LineWidth',1);hold on;
    ylabel('time (s)');
    title('actuator controls');
    disp('mean(actuator_delta_t_0)');
    mean(actuator_delta_t_0)/rate_dowm_simple
end 
if(isfield(log.data, 'actuator_outputs_0'))
    figure,
    plot(1:cs_N-1, cs_delta_t,'k-','LineWidth',1);hold on;
    ylabel('time (s)');
    title('actuator outputs');
    disp('mean(cs_delta_t)');
    mean(cs_delta_t)/rate_dowm_simple
end