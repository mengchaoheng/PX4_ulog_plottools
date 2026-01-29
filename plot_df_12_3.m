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
ulgFileName = 'data/13_39_55';
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
start_time=0;
end_time=55;

%%
B=[-0.50000  0        0.50000  0;       
    0       -0.50000  0        0.50000; 
    0.25000  0.25000  0.25000  0.25000];
rotor_min=0;rotor_max=1;
cs_min=-1;cs_max=1;


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
    Roll=eul(:,3);
    Pitch=eul(:,2);
    Yaw=eul(:,1);
    [attitude_N,~]=size(vehicle_attitude(:,1));
    attitude_delta_t=zeros(attitude_N-1,1);
    for i=1:attitude_N-1
        attitude_delta_t(i)=(vehicle_attitude(i+1,1))*1e-6-(vehicle_attitude(i,1))*1e-6;
    end
end 
if(isfield(log.data, 'vehicle_attitude_setpoint_0'))
    vehicle_attitude_setpoint=log.data.vehicle_attitude_setpoint_0{:,:}(1:att_set_dowm_simple:end, :);
    eul_setpoint = quat2eul(vehicle_attitude_setpoint(:,6:9), 'ZYX');
    Roll_setpoint=eul_setpoint(:,3);
    Pitch_setpoint=eul_setpoint(:,2);
    Yaw_setpoint=eul_setpoint(:,1);
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
% for new allocator
if(isfield(log.data, 'vehicle_torque_setpoint_0'))
    actuator_controls_0=log.data.vehicle_torque_setpoint_0{:,:}(1:rate_dowm_simple:end, :);   
    Roll_control_0=actuator_controls_0(:,3);
    Pitch_control_0=actuator_controls_0(:,4);
    Yaw_control_0=actuator_controls_0(:,5);
    
   
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
if(isfield(log.data, 'vehicle_thrust_setpoint_0'))
    actuator_controls_0=log.data.vehicle_thrust_setpoint_0{:,:}(1:rate_dowm_simple:end, :);   

    thrust_sp_0=actuator_controls_0(:,5);

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

% for new allocator
if(isfield(log.data, 'actuator_outputs_0'))
    actuator_outputs=log.data.actuator_outputs_0{:,:}(1:rate_dowm_simple:end, :);   
    rotor=(actuator_outputs(:,3)-1000)/1000;
    cs1=(actuator_outputs(:,4)-1500)/500;
    cs2=(actuator_outputs(:,5)-1500)/500;
    cs3=(actuator_outputs(:,6)-1500)/500;
    cs4=(actuator_outputs(:,7)-1500)/500;
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

%% plot
if(isfield(log.data, 'vehicle_angular_velocity_0') && isfield(log.data, 'vehicle_rates_setpoint_0'))
    fig1=figure(1);
    subplot(311)
    plot((vehicle_rates_setpoint(:,1))*1e-6, vehicle_rates_setpoint(:,2)*r2d,'k-','LineWidth',1);hold on;
    plot((vehicle_angular_velocity(:,1))*1e-6, vehicle_angular_velocity(:,3)*r2d,'--','LineWidth',1,'color',[0.6,0.2,0]);hold on;
    grid on;
    % axis([start_time end_time -inf inf]);
    xlabel({'Time (s)'});
    ylabel('p (deg/s)')
    title('Angular velocity');
    legend('Setpoint','Response','Location', 'best');
    %% 
    subplot(312)
    plot((vehicle_rates_setpoint(:,1))*1e-6, vehicle_rates_setpoint(:,3)*r2d,'k-','LineWidth',1);hold on;
    plot((vehicle_angular_velocity(:,1))*1e-6, vehicle_angular_velocity(:,4)*r2d,'--','LineWidth',1,'color',[0.6,0.2,0]);hold on;
    grid on;
    % axis([start_time end_time -inf inf]);
    xlabel({'Time (s)'});
    ylabel('q (deg/s)')
    legend('Setpoint','Response','Location', 'best');
    %% 
    subplot(313)
    plot((vehicle_rates_setpoint(:,1))*1e-6, vehicle_rates_setpoint(:,4)*r2d,'k-','LineWidth',1);hold on;
    plot((vehicle_angular_velocity(:,1))*1e-6, vehicle_angular_velocity(:,5)*r2d,'--','LineWidth',1,'color',[0.6,0.2,0]);hold on;
    grid on;
    % axis([start_time end_time -inf inf]);
    xlabel({'Time (s)'});
    ylabel('r (deg/s)')
    % title('Yaw angular velocity');
    legend('Setpoint','Response','Location', 'best');
    %% 
    % PlotToFileColorPDF(fig1,'results/pqr.pdf',15,18); 
end



if(isfield(log.data, 'vehicle_attitude_setpoint_0') && isfield(log.data, 'vehicle_attitude_0'))
    fig2=figure(2);
    subplot(311)
    plot((vehicle_attitude_setpoint(:,1))*1e-6, Roll_setpoint*r2d,'k-','LineWidth',1);hold on;
    plot((vehicle_attitude(:,1))*1e-6, Roll*r2d,'--','LineWidth',1,'color',[0.6,0.2,0]);hold on;
    grid on;
    % axis([start_time end_time -inf inf]);
    xlabel({'Time (s)'});
    ylabel('Roll (deg)')
    title('Euler angle');
    legend('Setpoint','Response','Location', 'best');

    subplot(312)
    plot((vehicle_attitude_setpoint(:,1))*1e-6, Pitch_setpoint*r2d,'k-','LineWidth',1);hold on;
    plot((vehicle_attitude(:,1))*1e-6, Pitch*r2d,'--','LineWidth',1,'color',[0.6,0.2,0]);hold on;
    grid on;
    % axis([start_time end_time -inf inf]);
    xlabel({'Time (s)'});
    ylabel('Pitch (deg)')
    legend('Setpoint','Response','Location', 'best');

    subplot(313)
    plot((vehicle_attitude_setpoint(:,1))*1e-6, Yaw_setpoint*r2d,'k-','LineWidth',1);hold on;
    plot((vehicle_attitude(:,1))*1e-6, Yaw*r2d,'--','LineWidth',1,'color',[0.6,0.2,0]);hold on;
    grid on;
    % axis([start_time end_time -inf inf]);
    xlabel({'Time (s)'});
    ylabel('Yaw (deg)')
    legend('Setpoint','Response','Location', 'best');
    %% 
    % PlotToFileColorPDF(fig2,'results/RPY.pdf',15,18);  
end






if(isfield(log.data, 'vehicle_local_position_0') && isfield(log.data, 'vehicle_local_position_setpoint_0'))

    fig3=figure(3);
    subplot(311)
    plot((vehicle_local_position_setpoint(:,1))*1e-6, V_XYZ_setpoint(:,1),'k-','LineWidth',1);hold on;
    plot((vehicle_local_position(:,1))*1e-6, V_XYZ(:,1),'--','LineWidth',1,'color',[0.6,0.2,0]);hold on;
    grid on;
    % axis([start_time end_time -inf inf]);
    title('Velocity');
    xlabel({'Time (s)'});
    ylabel('V_X (m/s)')
    legend('Setpoint','Response','Location', 'best');
    
    subplot(312)
    plot((vehicle_local_position_setpoint(:,1))*1e-6, V_XYZ_setpoint(:,2),'k-','LineWidth',1);hold on;
    plot((vehicle_local_position(:,1))*1e-6, V_XYZ(:,2),'--','LineWidth',1,'color',[0.6,0.2,0]);hold on;
    grid on;
    % axis([start_time end_time -inf inf]);
    xlabel({'Time (s)'});
    ylabel('V_Y (m/s)')
    legend('Setpoint','Response','Location', 'best');
    
    subplot(313)
    plot((vehicle_local_position_setpoint(:,1))*1e-6, V_XYZ_setpoint(:,3),'k-','LineWidth',1);hold on;
    plot((vehicle_local_position(:,1))*1e-6, V_XYZ(:,3),'--','LineWidth',1,'color',[0.6,0.2,0]);hold on;
    grid on;
    % axis([start_time end_time -inf inf]);
    xlabel({'Time (s)'});
    ylabel('V_Z (m/s)')
    legend('Setpoint','Response','Location', 'best');
    % PlotToFileColorPDF(fig3,'results/V_XYZ.pdf',15,18);
elseif(isfield(log.data, 'vehicle_local_position_0'))
    fig3=figure(3);
    subplot(311)
    plot((vehicle_local_position(:,1))*1e-6, V_XYZ(:,1),'--','LineWidth',1,'color',[0.6,0.2,0]);hold on;
    grid on;
    % axis([start_time end_time -inf inf]);
    title('Velocity');
    xlabel({'Time (s)'});
    ylabel('V_X (m/s)')
    
    subplot(312)
    plot((vehicle_local_position(:,1))*1e-6, V_XYZ(:,2),'--','LineWidth',1,'color',[0.6,0.2,0]);hold on;
    grid on;
    % axis([start_time end_time -inf inf]);
    xlabel({'Time (s)'});
    ylabel('V_Y (m/s)')
    
    subplot(313)
    plot((vehicle_local_position(:,1))*1e-6, V_XYZ(:,3),'--','LineWidth',1,'color',[0.6,0.2,0]);hold on;
    grid on;
    % axis([start_time end_time -inf inf]);
    xlabel({'Time (s)'});
    ylabel('V_Z (m/s)')
    % PlotToFileColorPDF(fig3,'results/V_XYZ.pdf',15,18);
end
%% 


if(isfield(log.data, 'vehicle_local_position_0') && isfield(log.data, 'vehicle_local_position_setpoint_0'))


    fig4=figure(4);
    subplot(311)
    plot((vehicle_local_position_setpoint(:,1))*1e-6, XYZ_setpoint(:,1),'k-','LineWidth',1);hold on;
    plot((vehicle_local_position(:,1))*1e-6, XYZ(:,1),'--','LineWidth',1,'color',[0.6,0.2,0]);hold on;
    grid on;
    % axis([start_time end_time -inf inf]);
    title('Position');
    xlabel({'Time (s)'});
    ylabel('X (m)')
    legend('Setpoint','Response','Location', 'best');
    
    subplot(312)
    plot((vehicle_local_position_setpoint(:,1))*1e-6, XYZ_setpoint(:,2),'k-','LineWidth',1);hold on;
    plot((vehicle_local_position(:,1))*1e-6, XYZ(:,2),'--','LineWidth',1,'color',[0.6,0.2,0]);hold on;
    grid on;
    % axis([start_time end_time -inf inf]);
    xlabel({'Time (s)'});
    ylabel('Y (m)')
    legend('Setpoint','Response','Location', 'best');
    
    subplot(313)
    plot((vehicle_local_position_setpoint(:,1))*1e-6, XYZ_setpoint(:,3),'k-','LineWidth',1);hold on;
    plot((vehicle_local_position(:,1))*1e-6, XYZ(:,3),'--','LineWidth',1,'color',[0.6,0.2,0]);hold on;
    grid on;
    % axis([start_time end_time -inf inf]);
    xlabel({'Time (s)'});
    ylabel('Z (m)')
    legend('Setpoint','Response','Location', 'best');
    % PlotToFileColorPDF(fig4,'results/P_XYZ.pdf',15,18);
elseif(isfield(log.data, 'vehicle_local_position_0'))
    fig4=figure(4);
    subplot(311)
    plot((vehicle_local_position(:,1))*1e-6, XYZ(:,1),'--','LineWidth',1,'color',[0.6,0.2,0]);hold on;
    grid on;
    % axis([start_time end_time -inf inf]);
    title('Position');
    xlabel({'Time (s)'});
    ylabel('X (m)')
    
    subplot(312)
    plot((vehicle_local_position(:,1))*1e-6, XYZ(:,2),'--','LineWidth',1,'color',[0.6,0.2,0]);hold on;
    grid on;
    % axis([start_time end_time -inf inf]);
    xlabel({'Time (s)'});
    ylabel('Y (m)')
    
    subplot(313)
    plot((vehicle_local_position(:,1))*1e-6, XYZ(:,3),'--','LineWidth',1,'color',[0.6,0.2,0]);hold on;
    grid on;
    % axis([start_time end_time -inf inf]);
    xlabel({'Time (s)'});
    ylabel('Z (m)')
    % PlotToFileColorPDF(fig4,'results/P_XYZ.pdf',15,18);
%%
end
%% 
% 



%% 
if(isfield(log.data, 'actuator_controls_0_0') || (isfield(log.data, 'vehicle_torque_setpoint_0') && isfield(log.data, 'vehicle_thrust_setpoint_0') ) )

    fig5=figure(5);
    subplot(511)
    plot((actuator_controls_0(:,1))*1e-6, Roll_control_0(:,1),'r-','LineWidth',1);hold on;
    plot((actuator_controls_0(:,1))*1e-6, Pitch_control_0(:,1),'k--','LineWidth',1,'color',[0.6,0.2,0]);hold on;
    plot((actuator_controls_0(:,1))*1e-6, Yaw_control_0(:,1),'b-.','LineWidth',1);hold on;
    plot((actuator_controls_0(:,1))*1e-6, thrust_sp_0(:,1),'k-','LineWidth',1);hold on;
    grid on;
    % axis([start_time end_time -1 1]);
    title('Controller output group 0');
    xlabel({'Time (s)'});
    ylabel('Value')
    legend('Roll','Pitch',  'Yaw','Thrust (up)','Location', 'best','NumColumns', 4);
    subplot(512)
    plot((actuator_controls_0(:,1))*1e-6, Roll_control_0(:,1),'r-','LineWidth',1);hold on;
    grid on;
    % axis([start_time end_time -1 1]);
    xlabel({'Time (s)'});
    ylabel('Roll')
    subplot(513)
    plot((actuator_controls_0(:,1))*1e-6, Pitch_control_0(:,1),'k--','LineWidth',1,'color',[0.6,0.2,0]);hold on;
    grid on;
    % axis([start_time end_time -1 1]);
    xlabel({'Time (s)'});
    ylabel('Pitch')
    subplot(514)
    plot((actuator_controls_0(:,1))*1e-6, Yaw_control_0(:,1),'b-.','LineWidth',1);hold on;
    grid on;
    % axis([start_time end_time -1 1]);
    xlabel({'Time (s)'});
    ylabel('Yaw')

    subplot(515)
    plot((actuator_controls_0(:,1))*1e-6, thrust_sp_0(:,1),'k-','LineWidth',1);hold on;
    grid on;
    % axis([start_time end_time -1 1]);
    xlabel({'Time (s)'});
    ylabel('Thrust (up)')
    % PlotToFileColorPDF(fig5,'results/actuator_controls_0.pdf',15,18);

%% 

end



%% 
if(isfield(log.data, 'actuator_outputs_0'))

    fig9=figure(9);
    subplot(511)
    plot((actuator_outputs(:,1))*1e-6, rotor(:,1),'g-','LineWidth',1);hold on;
    grid on;
    % axis([-inf inf 1000 2000]);
    title('Control surface');
    xlabel({'Time (s)'});
    ylabel('rotor (pwm)')
    legend('rotor');
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
    % PlotToFileColorPDF(fig10,'results/trj.pdf',15,15);
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
    
    subplot(312)
    plot((vehicle_angular_acceleration(:,1))*1e-6, vehicle_angular_acceleration(:,4),'k-','LineWidth',1);hold on;
    plot((vehicle_angular_velocity(:,1))*1e-6, vehicle_angular_velocity(:,4),'--','LineWidth',1,'color',[0.6,0.2,0,0.5]);hold on;
    grid on;
    % axis([-inf inf -20 20]);
    xlabel({'Time (s)'});
    ylabel('q (rad/s^2)')
    legend('angular acc','gyro');
    
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




%%
if(isfield(log.data, 'vehicle_angular_velocity_0'))
    % figure,
    % plot(1:rate_N-1, rate_delta_t,'k-','LineWidth',1);hold on;
    % ylabel('time (s)');
    % title('Sampling time of vehicle angular velocity');
    disp('mean(rate_delta_t)');
    mean(rate_delta_t)/rate_dowm_simple
end
if(isfield(log.data, 'vehicle_rates_setpoint_0'))
    % figure,
    % plot(1:rate_setpoint_N-1, rate_setpoint_delta_t,'k-','LineWidth',1);hold on;
    % ylabel('time (s)');
    % title('Sampling time of vehicle rate setpoint');
    disp('mean(rate_setpoint_delta_t)');
    mean(rate_setpoint_delta_t)/att_dowm_simple
end
if(isfield(log.data, 'vehicle_angular_acceleration_0'))
    % figure,
    % plot(1:rate_acc_N-1, rate_acc_delta_t,'k-','LineWidth',1);hold on;
    % ylabel('time (s)');
    % title('Sampling time of vehicle angular acceleration');
    disp('mean(rate_acc_delta_t)');
    mean(rate_acc_delta_t)
end
if(isfield(log.data, 'vehicle_attitude_0'))
    % figure,
    % plot(1:attitude_N-1, attitude_delta_t,'k-','LineWidth',1);hold on;
    % ylabel('time (s)');
    % title('Sampling time of vehicle attitude');
    disp('mean(attitude_delta_t)');
    mean(attitude_delta_t)/att_dowm_simple

end
if(isfield(log.data, 'vehicle_attitude_setpoint_0'))
    % figure,
    % plot(1:attitude_setpoint_N-1, attitude_setpoint_delta_t,'k-','LineWidth',1);hold on;
    % ylabel('time (s)');
    % title('Sampling time of vehicle attitude setpoint');
    disp('mean(attitude_setpoint_delta_t)');
    mean(attitude_setpoint_delta_t)/att_set_dowm_simple

end
if(isfield(log.data, 'vehicle_local_position_0'))
    % figure,
    % plot(1:pose_N-1, pose_delta_t,'k-','LineWidth',1);hold on;
    % ylabel('time (s)');
    % title('Sampling time of vehicle local position');
    disp('mean(pose_delta_t)');
    mean(pose_delta_t)
end
if(isfield(log.data, 'vehicle_local_position_setpoint_0'))
    % figure,
    % plot(1:pose_setpoint_N-1, pose_setpoint_delta_t,'k-','LineWidth',1);hold on;
    % ylabel('time (s)');
    % title('Sampling time of vehicle local position setpoint');
    disp('mean(pose_setpoint_delta_t)');
    mean(pose_setpoint_delta_t)
end
if(isfield(log.data, 'actuator_controls_0_0') || (isfield(log.data, 'vehicle_torque_setpoint_0') && isfield(log.data, 'vehicle_thrust_setpoint_0') ) )
    % figure,
    % plot(1:actuator_N_0-1, actuator_delta_t_0,'k-','LineWidth',1);hold on;
    % ylabel('time (s)');
    % title('Sampling time of controls');
    disp('mean(actuator_delta_t_0)');
    mean(actuator_delta_t_0)/rate_dowm_simple
end 
if(isfield(log.data, 'actuator_outputs_0'))
    % figure,
    % plot(1:cs_N-1, cs_delta_t,'k-','LineWidth',1);hold on;
    % ylabel('time (s)');
    % title('Sampling time of actuator outputs');
    disp('mean(cs_delta_t)');
    mean(cs_delta_t)/rate_dowm_simple
end