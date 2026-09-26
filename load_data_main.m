% clear all;
% close all;
% clc;
addpath(genpath(pwd));
%% This file is mainly used to load data and perform conversion processing, such as compatibility between old and new versions, VTOL coordinate system conversion, etc.

%% =========================================================================
%  Step 0: File Selection & Basic Settings
% =========================================================================
d2r = pi/180;
r2d = 180/pi;
get_t = @(tbl) tbl.timestamp * 1e-6;
% --- User Configuration Area ---------------------------------------------------------
% Specify filename here (can be relative path 'data/09_49_18' or absolute path)
% [KEY]: If left empty (i.e. specifiedFileName = '';), a dialog will pop up for selection when the script runs.
specifiedFileName = 'data/log_40_2025-6-24-14-58-42'; % Supports with or without extension

if isempty(specifiedFileName)
    [fileName, pathName] = uigetfile('*.ulg', 'Please select the ULog file to analyze');
    if isequal(fileName, 0), disp('Selection cancelled'); return; end
    fullPath = fullfile(pathName, fileName);
else
    % 1. Add extension if missing
    if ~endsWith(specifiedFileName, '.ulg'), specifiedFileName = [specifiedFileName '.ulg']; end
    % 2. Convert to absolute path 
    if java.io.File(specifiedFileName).isAbsolute()
        fullPath = specifiedFileName;
    else
        fullPath = fullfile(pwd, specifiedFileName);
    end
    % 3. Check if file exists
    if ~exist(fullPath, 'file'), error(['File does not exist: ' fullPath]); end
end

% --- Unified extraction of standard variables ---
[pathName, nameNoExt, ext] = fileparts(fullPath);
fileName = [nameNoExt, ext];
ulgFileName = fullfile(pathName, nameNoExt); 
tmp = [ulgFileName '.mat'];

disp(['Analysis target fileName: ' fileName]);
disp(['ulgFileName: ' ulgFileName]);
disp(['tmp: ' tmp]);
disp(['pathName: ' pathName]);
%% =========================================================================
%  Step 1: Check or Convert Data
% =========================================================================
if exist(tmp, "file")
    disp(['Found MAT file: ' tmp]);
    load(tmp, 'log');
else
    disp('No MAT file found, start parsing ULog...');
    
    % --- Locate the project-local Python environment (created by setup_env.ps1) ---
    % All pyulog CLI tools are called from ./.venv, therefore no Python installation,
    % no globally installed pyulog and no PATH configuration is required.
    if ispc
        venvBinName = 'Scripts';
        exeSuffix   = '.exe';
    else
        venvBinName = 'bin';
        exeSuffix   = '';
    end

    % Candidate project roots: folder of this script first (works both when the script
    % is run directly and via run('load_data_main.m')), then the MATLAB working directory.
    candRoots  = {};
    thisScript = mfilename('fullpath');
    if ~isempty(thisScript)
        candRoots{end+1} = fileparts(thisScript);
    end
    scriptOnPath = which('load_data_main');
    if ~isempty(scriptOnPath)
        candRoots{end+1} = fileparts(scriptOnPath);
    end
    candRoots = unique([candRoots, pwd], 'stable');

    cmd_ulog2csv = '';
    cmd_info     = '';
    cmd_msgs     = '';
    cmd_params   = '';
    for iRoot = 1:numel(candRoots)
        binDir = fullfile(candRoots{iRoot}, '.venv', venvBinName);
        if exist(fullfile(binDir, ['ulog2csv' exeSuffix]), 'file')
            cmd_ulog2csv = fullfile(binDir, ['ulog2csv' exeSuffix]);
            cmd_info     = fullfile(binDir, ['ulog_info' exeSuffix]);
            cmd_msgs     = fullfile(binDir, ['ulog_messages' exeSuffix]);
            cmd_params   = fullfile(binDir, ['ulog_params' exeSuffix]);
            break;
        end
    end

    if isempty(cmd_ulog2csv)
        error(['Python environment not found. Please run setup_env.ps1 first.' newline ...
               'Expected pyulog tools in: ' fullfile(candRoots{end}, '.venv', venvBinName) newline ...
               'Windows: scripts\setup_env.ps1    Linux/macOS: scripts/setup_env.sh' newline ...
               'See README.md for details.']);
    end
    disp(['Using pyulog from: ' cmd_ulog2csv]);

    ulgAbs = fullfile(pathName, fileName);

    % 1. Run ulog2csv
    command = ['"' cmd_ulog2csv '" "' ulgAbs '"'];
    disp(['Running: ' command]);
    [status, cmdout] = system(command);
    if status ~= 0
        disp(cmdout);
        error('ulog2csv conversion failed.');
    end
    
    % 2. Read CSV data
    log.data = csv_topics_to_d(ulgFileName); 
    log.FileName = ulgFileName;
    log.version = 1.0;
    
    % 3. Parse Info/Messages/Params
    [~, log.info] = system(['"' cmd_info '" "' ulgAbs '"']);
    [~, log.messages] = system(['"' cmd_msgs '" "' ulgAbs '"']);

    % Parse Params as struct, save all parameters
    [s_p, out_p] = system(['"' cmd_params '" "' ulgAbs '"']);
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

    % 4. Save MAT and cleanup CSV
    save(tmp, 'log');
    delete([ulgFileName '_*.csv']);
    disp('Conversion done & Temp CSV deleted.');
end
%% Print system info and flight messages
disp(log.info);
disp(log.messages);
%% =========================================================================
%  Step 2: Data Preprocessing (State Conversion & PWM Parsing)
% =========================================================================
% 2.1 vehicle_angular_velocity
if(isfield(log.data, 'vehicle_angular_velocity_0'))
    vehicle_angular_velocity = [log.data.vehicle_angular_velocity_0.xyz_0_, ...
                                log.data.vehicle_angular_velocity_0.xyz_1_, ...
                                log.data.vehicle_angular_velocity_0.xyz_2_];
    vehicle_angular_velocity_t = get_t(log.data.vehicle_angular_velocity_0);
    if length(vehicle_angular_velocity_t) > 1
        dt = mean(diff(vehicle_angular_velocity_t));
        fprintf('Angular velocity sampling period: %f (ms), frequency: %f (Hz) \n', dt*1000, 1/dt);
    end
    if ismember('xyz_derivative_0_', log.data.vehicle_angular_velocity_0.Properties.VariableNames)
        vehicle_angular_acceleration = [log.data.vehicle_angular_velocity_0.xyz_derivative_0_, ...
                                        log.data.vehicle_angular_velocity_0.xyz_derivative_1_, ...
                                        log.data.vehicle_angular_velocity_0.xyz_derivative_2_];
        vehicle_angular_acceleration_t=vehicle_angular_velocity_t;
        fprintf('Angular acceleration sampling period is same as angular velocity');
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
        fprintf('Angular acceleration sampling period: %f (ms), frequency: %f (Hz) \n', dt*1000, 1/dt);
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
        fprintf('Angular velocity setpoint sampling period: %f (ms), frequency: %f (Hz) \n', dt*1000, 1/dt);
    end
end 

% 2.4 Attitude
if(isfield(log.data, 'vehicle_attitude_0'))
    vehicle_attitude_quat = [log.data.vehicle_attitude_0.q_0_, log.data.vehicle_attitude_0.q_1_, log.data.vehicle_attitude_0.q_2_, log.data.vehicle_attitude_0.q_3_];
    eul_ZYX = quat2eul(vehicle_attitude_quat);
    % eul_ZYX is equivalent to eulerAnglesRandians: q_b2n=quaternion(vehicle_attitude_quat); eulerAnglesRandians = euler(q_b2n, "ZYX" , "frame" );
    Roll = eul_ZYX(:,3); Pitch = eul_ZYX(:,2); Yaw = eul_ZYX(:,1);
    vehicle_attitude_t = get_t(log.data.vehicle_attitude_0);
    if length(vehicle_attitude_t) > 1
        dt = mean(diff(vehicle_attitude_t));
        fprintf('Attitude sampling period: %f (ms), frequency: %f (Hz) \n', dt*1000, 1/dt);
    end
end

if(isfield(log.data, 'vehicle_attitude_setpoint_0'))
    vehicle_attitude_quat_d = [log.data.vehicle_attitude_setpoint_0.q_d_0_, log.data.vehicle_attitude_setpoint_0.q_d_1_, log.data.vehicle_attitude_setpoint_0.q_d_2_, log.data.vehicle_attitude_setpoint_0.q_d_3_];
    eul_ZYX_setpoint = quat2eul(vehicle_attitude_quat_d);
    Roll_setpoint = eul_ZYX_setpoint(:,3); Pitch_setpoint = eul_ZYX_setpoint(:,2); Yaw_setpoint = eul_ZYX_setpoint(:,1);
    vehicle_attitude_setpoint_t = get_t(log.data.vehicle_attitude_setpoint_0);
    if length(vehicle_attitude_setpoint_t) > 1
        dt = mean(diff(vehicle_attitude_setpoint_t));
        fprintf('Attitude setpoint sampling period: %f (ms), frequency: %f (Hz) \n', dt*1000, 1/dt);
    end
end

% 2.5 Position / Velocity
clear acc_indi_active acc_indi_active_t hte_status
if(isfield(log.data, 'vehicle_local_position_0'))
    XYZ = [log.data.vehicle_local_position_0.x, log.data.vehicle_local_position_0.y, log.data.vehicle_local_position_0.z];
    V_XYZ = [log.data.vehicle_local_position_0.vx, log.data.vehicle_local_position_0.vy, log.data.vehicle_local_position_0.vz];
    vehicle_local_position_t = get_t(log.data.vehicle_local_position_0);
    ref_alt=log.data.vehicle_local_position_0.ref_alt;
    if length(vehicle_local_position_t) > 1
        dt = mean(diff(vehicle_local_position_t));
        fprintf('Position/velocity sampling period: %f (ms), frequency: %f (Hz) \n', dt*1000, 1/dt);
    end
end
if(isfield(log.data, 'vehicle_local_position_setpoint_0'))
    XYZ_setpoint = [log.data.vehicle_local_position_setpoint_0.x, log.data.vehicle_local_position_setpoint_0.y, log.data.vehicle_local_position_setpoint_0.z];
    V_XYZ_setpoint = [log.data.vehicle_local_position_setpoint_0.vx, log.data.vehicle_local_position_setpoint_0.vy, log.data.vehicle_local_position_setpoint_0.vz];
    vehicle_local_position_setpoint_t = get_t(log.data.vehicle_local_position_setpoint_0);
    if length(vehicle_local_position_setpoint_t) > 1
        dt = mean(diff(vehicle_local_position_setpoint_t));
        fprintf('Position/velocity setpoint sampling period: %f (ms), frequency: %f (Hz) \n', dt*1000, 1/dt);
    end

    if ismember('acc_indi_active', log.data.vehicle_local_position_setpoint_0.Properties.VariableNames)
        acc_indi_active_t = vehicle_local_position_setpoint_t;
        acc_indi_active = logical(log.data.vehicle_local_position_setpoint_0.acc_indi_active);
    end
end

% HoverThrustEstimator output. The controller-applied hover thrust is in
% acceleration_indi_status; this topic is the estimator's own output.
if isfield(log.data, 'hover_thrust_estimate_0')
    tbl = log.data.hover_thrust_estimate_0;

    if all(ismember({'timestamp', 'hover_thrust', 'valid'}, tbl.Properties.VariableNames))
        hte_status = struct();
        hte_status.time = get_t(tbl);
        hte_status.hover_thrust = double(tbl.hover_thrust);
        hte_status.valid = logical(tbl.valid);
        hte_status.hover_thrust(~hte_status.valid) = NaN;
    end
end

%% =========================================================================
%  Step 2.6: Actuator Controls Data Acquisition (Revised: Independent Time Axis)
% =========================================================================
% 1. Enable control allocation flag (dynamic_control_alloc)
dynamic_control_alloc = isfield(log.data, 'actuator_motors_0') || isfield(log.data, 'actuator_servos_0');


% Initialize structures
actuator_controls_0 = struct('time',[], 'time_thrust',[], 'roll',[], 'pitch',[], 'yaw',[], 'thrust_x',[], 'thrust_z_neg',[]);
actuator_controls_1 = struct('time',[], 'roll',[], 'pitch',[], 'yaw',[], 'thrust_x',[], 'thrust_z_neg',[]);

% -------------------------------------------------------------------------
% Instance 0 (Main): Separated Time Axis
% -------------------------------------------------------------------------
if dynamic_control_alloc
    % --- Dynamic Instance 0 ---
    % 1. Torque 
    if isfield(log.data, 'vehicle_torque_setpoint_0')
        actuator_controls_0.time = get_t(log.data.vehicle_torque_setpoint_0);
        actuator_controls_0.roll  = log.data.vehicle_torque_setpoint_0.xyz_0_;
        actuator_controls_0.pitch = log.data.vehicle_torque_setpoint_0.xyz_1_;
        actuator_controls_0.yaw   = log.data.vehicle_torque_setpoint_0.xyz_2_;
    end
    
    % 2. Thrust -> using independent timestamp_thrust
    if isfield(log.data, 'vehicle_thrust_setpoint_0')
        % Save thrust time axis independently
        t_thrust_0 = get_t(log.data.vehicle_thrust_setpoint_0);
        actuator_controls_0.time_thrust = t_thrust_0;       
        tx = log.data.vehicle_thrust_setpoint_0.xyz_0_;
        ty = log.data.vehicle_thrust_setpoint_0.xyz_1_;
        tz = log.data.vehicle_thrust_setpoint_0.xyz_2_;
        actuator_controls_0.thrust = sqrt(tx.^2 + ty.^2 + tz.^2);          % Python: thrust
        actuator_controls_0.thrust_x = tx;          % Python: thrust_x
        actuator_controls_0.thrust_z_neg = -tz;     % Python: thrust_z_neg
        
        % Save original thrust data for Instance 1 resampling
        raw_thrust_0 = struct('t', t_thrust_0, 'thrust', actuator_controls_0.thrust, 'thrust_x', actuator_controls_0.thrust_x, 'thrust_z_neg', actuator_controls_0.thrust_z_neg );
    end
else
    % --- Legacy Instance 0 ---
    if isfield(log.data, 'actuator_controls_0_0')
        t_legacy = get_t(log.data.actuator_controls_0_0);
        actuator_controls_0.time = t_legacy;
        actuator_controls_0.time_thrust = t_legacy; % Legacy version uses same time axis
        actuator_controls_0.roll  = log.data.actuator_controls_0_0.control_0_;
        actuator_controls_0.pitch = log.data.actuator_controls_0_0.control_1_;
        actuator_controls_0.yaw   = log.data.actuator_controls_0_0.control_2_;
        actuator_controls_0.thrust_z_neg = log.data.actuator_controls_0_0.control_3_;

    end
end
if length(actuator_controls_0.time) > 1
    dt = mean(diff(actuator_controls_0.time));
    fprintf('Force/Torque (Grp0) setpoint sampling period: %f (ms), frequency: %f (Hz) \n', dt*1000, 1/dt);
end
% -------------------------------------------------------------------------
% Instance 1 (Aux/FW): Maintain resampling strategy (usually aligned to torque for plotting on same graph)
% -------------------------------------------------------------------------
if dynamic_control_alloc
    % --- Dynamic Instance 1 ---
    if isfield(log.data, 'vehicle_torque_setpoint_1')
        t1 = get_t(log.data.vehicle_torque_setpoint_1);
        actuator_controls_1.time = t1;      
        actuator_controls_1.roll  = log.data.vehicle_torque_setpoint_1.xyz_0_;
        actuator_controls_1.pitch = log.data.vehicle_torque_setpoint_1.xyz_1_;
        actuator_controls_1.yaw   = log.data.vehicle_torque_setpoint_1.xyz_2_;
        
        % Resample thrust (from Instance 0) to align with Instance 1 time axis
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
    fprintf('Force/Torque (Grp1) setpoint sampling period: %f (ms), frequency: %f (Hz) \n', dt*1000, 1/dt);
end

%% =========================================================================
%  Step 2.6a: Allocation INDI feedback filter
% =========================================================================
% These variables may still exist in the caller workspace when this script is
% run repeatedly with different logs. Clear them so that an old log without
% the new fields cannot accidentally reuse data from the previous run.
clear allocation_feedback allocation_filter_status indi_accel_status

% High-rate input/output values used by the INDI feedback path.
if isfield(log.data, 'allocation_value_0')
    tbl = log.data.allocation_value_0;
    feedback_fields = { ...
        'timestamp', 'timestamp_sample', ...
        'raw_allocated_torque_0_', 'raw_allocated_torque_1_', 'raw_allocated_torque_2_', ...
        'allocated_torque_0_', 'allocated_torque_1_', 'allocated_torque_2_', ...
        'raw_allocated_force_0_', 'raw_allocated_force_1_', 'raw_allocated_force_2_', ...
        'allocated_force_0_', 'allocated_force_1_', 'allocated_force_2_'};

    % Older firmware also publishes allocation_value_0, but with a different
    % schema (error_*, u_*, flag, ...). Only decode the new INDI feedback
    % layout when all of its fields are present.
    if all(ismember(feedback_fields, tbl.Properties.VariableNames))
        allocation_feedback = struct();
        allocation_feedback.time = get_t(tbl);
        allocation_feedback.time_sample = double(tbl.timestamp_sample) * 1e-6;
        allocation_feedback.torque_raw = [tbl.raw_allocated_torque_0_, ...
                                          tbl.raw_allocated_torque_1_, ...
                                          tbl.raw_allocated_torque_2_];
        allocation_feedback.torque_filtered = [tbl.allocated_torque_0_, ...
                                               tbl.allocated_torque_1_, ...
                                               tbl.allocated_torque_2_];
        allocation_feedback.force_raw = [tbl.raw_allocated_force_0_, ...
                                         tbl.raw_allocated_force_1_, ...
                                         tbl.raw_allocated_force_2_];
        allocation_feedback.force_filtered = [tbl.allocated_force_0_, ...
                                              tbl.allocated_force_1_, ...
                                              tbl.allocated_force_2_];

        if height(tbl) > 1
            allocation_feedback.logged_interval = diff(allocation_feedback.time);
            dt = mean(allocation_feedback.logged_interval);
            fprintf(['Allocation feedback recorded-topic period (logger spacing, not the ' ...
                     'filter update dt): %.3f ms, frequency: %.1f Hz\n'], ...
                dt * 1000, 1 / dt);
        else
            allocation_feedback.logged_interval = [];
        end
    else
        fprintf(['allocation_value_0 uses the legacy schema; skipping the ' ...
                 'INDI feedback-filter data.\n']);
    end
end

% Low-rate diagnostic state. The publication interval is intentionally
% independent from the high-rate filter update interval above.
if isfield(log.data, 'allocation_feedback_filter_status_0')
    tbl = log.data.allocation_feedback_filter_status_0;
    allocation_filter_status = struct();
    allocation_filter_status.time = get_t(tbl);
    allocation_filter_status.event_time = double(tbl.filter_event_timestamp) * 1e-6;
    allocation_filter_status.sample_interval = double(tbl.sample_interval);
    allocation_filter_status.sample_rate_hz = double(tbl.filter_sample_rate_hz);
    allocation_filter_status.torque_cutoff_hz = double(tbl.torque_filter_cutoff_hz);
    allocation_filter_status.force_cutoff_hz = double(tbl.force_filter_cutoff_hz);
    allocation_filter_status.reset_count = double(tbl.filter_reset_count);
    allocation_filter_status.reconfigure_count = double(tbl.filter_reconfigure_count);
    allocation_filter_status.event_reason = double(tbl.filter_event_reason);
    allocation_filter_status.configured = logical(tbl.filters_configured);

    if height(tbl) > 1
        status_dt = mean(diff(allocation_filter_status.time));
        fprintf('Allocation filter status logged period: %.3f ms, frequency: %.1f Hz\n', ...
            status_dt * 1000, 1 / status_dt);
    end

    event_reason = allocation_filter_status.event_reason(end);

    switch event_reason
        case 1
            event_reason_text = 'initialization';
        case 2
            event_reason_text = 'cutoff parameter load/change';
        case 4
            event_reason_text = 'sample-rate reconfiguration';
        case 8
            event_reason_text = 'effectiveness update (legacy firmware behavior)';
        case 16
            event_reason_text = 'feedback gap';
        case 32
            event_reason_text = 'axis recovery';
        otherwise
            event_reason_text = 'none/unknown';
    end

    fprintf(['Allocation filter state: mean update dt=%.3f ms, coefficient rate=%.1f Hz, ' ...
             'torque cutoff=%.2f Hz, force cutoff=%.2f Hz, ' ...
             'reset=%u, reconfigure=%u, last event=%s (%u), configured=%u\n'], ...
        allocation_filter_status.sample_interval(end) * 1000, ...
        allocation_filter_status.sample_rate_hz(end), ...
        allocation_filter_status.torque_cutoff_hz(end), ...
        allocation_filter_status.force_cutoff_hz(end), ...
        allocation_filter_status.reset_count(end), ...
        allocation_filter_status.reconfigure_count(end), ...
        event_reason_text, event_reason, ...
        allocation_filter_status.configured(end));
end

% -------------------------------------------------------------------------
% Acceleration INDI feedback status (source/validity/alignment diagnostics)
% -------------------------------------------------------------------------
% AccelerationIndiStatus is already logged by the flight controller. Decode it
% here so the plotting script can compare all available a_0 sources and the
% delayed/undelayed allocation-force acceleration without adding another ULog
% topic.
if isfield(log.data, 'acceleration_indi_status_0')
    tbl = log.data.acceleration_indi_status_0;
    indi_fields = { ...
        'timestamp', 'timestamp_sample', 'force_timestamp', ...
        'acceleration_feedback_0_', 'acceleration_feedback_1_', 'acceleration_feedback_2_', ...
        'acceleration_velocity_derivative_0_', 'acceleration_velocity_derivative_1_', 'acceleration_velocity_derivative_2_', ...
        'acceleration_ekf_0_', 'acceleration_ekf_1_', 'acceleration_ekf_2_', ...
        'acceleration_imu_0_', 'acceleration_imu_1_', 'acceleration_imu_2_', ...
        'allocated_thrust_acceleration_0_', 'allocated_thrust_acceleration_1_', 'allocated_thrust_acceleration_2_', ...
        'allocated_thrust_acceleration_undelayed_0_', 'allocated_thrust_acceleration_undelayed_1_', 'allocated_thrust_acceleration_undelayed_2_', ...
        'hover_thrust', 'force_feedback_scale', ...
        'controller_transition_progress', 'hover_thrust_transition_progress', ...
        'acceleration_source', 'acceleration_source_valid', 'feedback_valid', ...
        'controller_transition_active', 'hover_thrust_transition_active'};

    if all(ismember(indi_fields, tbl.Properties.VariableNames))
        indi_accel_status = struct();
        indi_accel_status.time = get_t(tbl);
        indi_accel_status.time_sample = double(tbl.timestamp_sample) * 1e-6;
        indi_accel_status.force_time = double(tbl.force_timestamp) * 1e-6;
        indi_accel_status.feedback = [tbl.acceleration_feedback_0_, ...
                                      tbl.acceleration_feedback_1_, ...
                                      tbl.acceleration_feedback_2_];
        indi_accel_status.velocity_derivative = [tbl.acceleration_velocity_derivative_0_, ...
                                                 tbl.acceleration_velocity_derivative_1_, ...
                                                 tbl.acceleration_velocity_derivative_2_];
        indi_accel_status.ekf = [tbl.acceleration_ekf_0_, ...
                                 tbl.acceleration_ekf_1_, ...
                                 tbl.acceleration_ekf_2_];
        indi_accel_status.imu = [tbl.acceleration_imu_0_, ...
                                 tbl.acceleration_imu_1_, ...
                                 tbl.acceleration_imu_2_];
        indi_accel_status.force_accel = [tbl.allocated_thrust_acceleration_0_, ...
                                         tbl.allocated_thrust_acceleration_1_, ...
                                         tbl.allocated_thrust_acceleration_2_];
        indi_accel_status.force_accel_undelayed = [tbl.allocated_thrust_acceleration_undelayed_0_, ...
                                                   tbl.allocated_thrust_acceleration_undelayed_1_, ...
                                                   tbl.allocated_thrust_acceleration_undelayed_2_];
        indi_accel_status.hover_thrust = double(tbl.hover_thrust);
        indi_accel_status.force_feedback_scale = double(tbl.force_feedback_scale);
        indi_accel_status.controller_progress = double(tbl.controller_transition_progress);
        indi_accel_status.hover_progress = double(tbl.hover_thrust_transition_progress);
        indi_accel_status.source = double(tbl.acceleration_source);
        indi_accel_status.source_valid = double(tbl.acceleration_source_valid);
        indi_accel_status.feedback_valid = logical(tbl.feedback_valid);
        indi_accel_status.controller_active = logical(tbl.controller_transition_active);
        indi_accel_status.hover_active = logical(tbl.hover_thrust_transition_active);
        indi_accel_status.force_delay = (indi_accel_status.time_sample - indi_accel_status.force_time) * 1000;
        % Before the delayed-force history is filled, force_timestamp may be
        % zero/old and the resulting number is not a physical delay. Keep the
        % raw validity flag, but expose only meaningful delay samples.
        invalid_delay = ~indi_accel_status.feedback_valid ...
                        | ~isfinite(indi_accel_status.force_delay) ...
                        | indi_accel_status.force_time <= 0;
        indi_accel_status.force_delay(invalid_delay) = NaN;

        finite_feedback = all(isfinite(indi_accel_status.feedback), 2);
        fprintf(['Acceleration INDI status: %.1f Hz, valid feedback %d/%d, ' ...
                 'force delay median %.1f ms, source=%d\n'], ...
            1 / mean(diff(indi_accel_status.time)), ...
            sum(indi_accel_status.feedback_valid & finite_feedback), ...
            length(indi_accel_status.time), ...
            median(indi_accel_status.force_delay(isfinite(indi_accel_status.force_delay))), ...
            indi_accel_status.source(end));
    else
        fprintf(['acceleration_indi_status_0 is present but has an older schema; ' ...
                 'skipping Acceleration INDI diagnostics.\n']);
    end
end

%% =========================================================================
%  Step 2.7 & 2.8: Actuator Data Extraction  
% =========================================================================

% Initialize variables
motors = []; motors_t = [];
servos = []; servos_t = [];
active_channels = []; % For storing parsed PWM channel info
legacy_pwm = [];      % For storing legacy raw PWM data
legacy_pwm_t = [];

% -------------------------------------------------------------------------
% 1. New dynamic allocation (Actuator Motors & Servos)
% -------------------------------------------------------------------------
if isfield(log.data, 'actuator_motors_0')
    mc_cols = startsWith(log.data.actuator_motors_0.Properties.VariableNames, 'control_');
    motors = log.data.actuator_motors_0{:, mc_cols};
    motors_t = get_t(log.data.actuator_motors_0);
    
    if length(motors_t) > 1
        dt = mean(diff(motors_t));
        fprintf('New: Motors sampling period: %.3f ms, frequency: %.1f Hz\n', dt*1000, 1/dt);
    end
end

if isfield(log.data, 'actuator_servos_0')
    sc_cols = startsWith(log.data.actuator_servos_0.Properties.VariableNames, 'control_');
    servos = log.data.actuator_servos_0{:, sc_cols};
    servos_t = get_t(log.data.actuator_servos_0);
    
    if length(servos_t) > 1
        dt = mean(diff(servos_t));
        fprintf('New: Servos sampling period: %.3f ms, frequency: %.1f Hz\n', dt*1000, 1/dt);
    end
end

% -------------------------------------------------------------------------
% 2. PWM Output Parsing (New: attempt to parse parameters / Legacy: directly extract raw data)
% -------------------------------------------------------------------------

if isfield(log.data, 'actuator_outputs_0')
    output_cols = startsWith(log.data.actuator_outputs_0.Properties.VariableNames, 'output_');
    outputs = log.data.actuator_outputs_0{:, output_cols};
    outputs_t = get_t(log.data.actuator_outputs_0);
    
    if length(outputs_t) > 1
        dt = mean(diff(outputs_t));
        fprintf('PWM raw output sampling period: %.3f ms, frequency: %.1f Hz\n', dt*1000, 1/dt);
    end
    
    % --- Attempt to parse channel definitions  ---
    active_channels = struct('idx', {}, 'code', {}, 'name', {}, 'col_name', {}, 'type', {});
    if isfield(log.data, 'actuator_outputs_0') && isfield(log, 'params')
        disp('Parsing PWM output channel definitions...');
        % Some boards publish AUX outputs as actuator_outputs_0. Use AUX
        % when the log has no MAIN function parameters (instance is not a bank ID).
        pwm_param_prefix = 'PWM_MAIN_FUNC';
        pwm_param_names = fieldnames(log.params);
        if ~any(startsWith(pwm_param_names, 'PWM_MAIN_FUNC')) && ...
                any(startsWith(pwm_param_names, 'PWM_AUX_FUNC'))
            pwm_param_prefix = 'PWM_AUX_FUNC';
        end
        fprintf('PWM channel parameters: %s\n', pwm_param_prefix);
        for i = 1:16
            param_name = sprintf('%s%d', pwm_param_prefix, i);
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
%  2.9 Prepare visualization state data (Flight mode & VTOL state)
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

    % Only for tailsitter 
    if max(log.data.vehicle_status_0.is_vtol_tailsitter) == 1
    
        fw_intervals = get_fw_intervals(log.data.vehicle_status_0);
    
        % 1) attitude: override quaternion
        vehicle_attitude_quat = apply_tailsitter_attitude_fix(log.data.vehicle_attitude_0.timestamp,vehicle_attitude_quat, fw_intervals);
        eul_ZYX = quat2eul(vehicle_attitude_quat, 'ZYX'); % rad: [yaw pitch roll]
        Yaw   = eul_ZYX(:,1);
        Pitch = eul_ZYX(:,2);
        Roll  = eul_ZYX(:,3);
    
        % 2) rates: override angular velocity
        vehicle_angular_velocity = apply_tailsitter_rate_fix(log.data.vehicle_angular_velocity_0.timestamp, vehicle_angular_velocity, fw_intervals);
    
        % 3) rates setpoint: override setpoints
        vehicle_rates_setpoint = apply_tailsitter_rate_sp_fix(log.data.vehicle_rates_setpoint_0.timestamp, vehicle_rates_setpoint, fw_intervals);
    
    end
    % 1. Parse flight mode (seamless version)
    [vis_flight_intervals, vis_flight_names] = get_flight_mode_intervals(log.data.vehicle_status_0);
    
    % 2. Parse VTOL state (seamless version)
    if ismember('is_vtol', log.data.vehicle_status_0.Properties.VariableNames) && ...
       max(log.data.vehicle_status_0.is_vtol) == 1
        vis_is_vtol = true;
        [vis_vtol_intervals, vis_vtol_names] = get_vtol_state_intervals(log.data.vehicle_status_0);
    end
end

% 2.10 TECS Status (New: Parse TECS altitude rate)
if isfield(log.data, 'tecs_status_0')
    tecs_t = get_t(log.data.tecs_status_0);
    altitude_sp= log.data.tecs_status_0.altitude_sp;
    altitude_reference= log.data.tecs_status_0.altitude_reference;
    height_rate_reference= log.data.tecs_status_0.height_rate_reference;
    height_rate_direct= log.data.tecs_status_0.height_rate_direct;
    height_rate_setpoint= log.data.tecs_status_0.height_rate_setpoint;
    height_rate= log.data.tecs_status_0.height_rate;
    equivalent_airspeed_sp= log.data.tecs_status_0.equivalent_airspeed_sp;
    true_airspeed_sp= log.data.tecs_status_0.true_airspeed_sp;
    true_airspeed_filtered= log.data.tecs_status_0.true_airspeed_filtered;
    true_airspeed_derivative_sp= log.data.tecs_status_0.true_airspeed_derivative_sp;
    true_airspeed_derivative= log.data.tecs_status_0.true_airspeed_derivative;
    true_airspeed_derivative_raw= log.data.tecs_status_0.true_airspeed_derivative_raw;
    total_energy_rate_sp= log.data.tecs_status_0.total_energy_rate_sp;
    total_energy_rate= log.data.tecs_status_0.total_energy_rate;
    total_energy_balance_rate_sp= log.data.tecs_status_0.total_energy_balance_rate_sp;
    total_energy_balance_rate= log.data.tecs_status_0.total_energy_balance_rate;
end

%% =========================================================================
%  2.11 Manual Control Inputs  
% =========================================================================
rc_t = []; rc_roll = []; rc_pitch = []; rc_yaw = []; rc_throttle = [];

if isfield(log.data, 'manual_control_setpoint_0')
    tbl = log.data.manual_control_setpoint_0;
    rc_t = get_t(tbl);
    vars = tbl.Properties.VariableNames;
    
    % --- Detect version and extract ---
    if ismember('roll', vars)
        % === New version (v1.14+) ===
        % Field names are intuitive: roll, pitch, yaw, throttle
        rc_roll     = tbl.roll;
        rc_pitch    = tbl.pitch;
        rc_yaw      = tbl.yaw;
        rc_throttle = tbl.throttle;
        
    elseif ismember('y', vars)
        % === Legacy version ===
        % Mapping relationship:
        % y -> stick position in y direction -1..1
        % x -> stick position in x direction -1..1
        % r -> yaw stick/twist position, -1..1
        % z -> throttle stick position 0..1
        rc_roll     = tbl.y;
        rc_pitch    = tbl.x;
        rc_yaw      = tbl.r;
        rc_throttle = tbl.z;
    else
        warning('Unable to recognize manual_control_setpoint field format');
    end
    
    % Auxiliary channel extraction (usually both new and legacy have aux1, aux2...)
    % Can be extracted as needed, for example:
    % if ismember('aux1', vars), rc_aux1 = tbl.aux1; end
end

% 2.12 Raw Acceleration (sensor_combined)
if isfield(log.data, 'sensor_combined_0')
    raw_acc_t = get_t(log.data.sensor_combined_0);
    % Extract X, Y, Z acceleration (m/s^2)
    raw_acc = [log.data.sensor_combined_0.accelerometer_m_s2_0_, ...
               log.data.sensor_combined_0.accelerometer_m_s2_1_, ...
               log.data.sensor_combined_0.accelerometer_m_s2_2_];
    
    % Also print sampling frequency to check sensor data health
    if length(raw_acc_t) > 1
        raw_acc_dt = mean(diff(raw_acc_t));
        fprintf('Raw acceleration (sensor_combined) sampling period: %f (ms), frequency: %f (Hz) \n', 1000*raw_acc_dt, 1/raw_acc_dt);
    end
end

% 2.13 Vibration Metrics (vehicle_imu_status)
% Automatically search for existing IMU instances (0~3)
vib_data = struct('id', {}, 't', {}, 'val', {});
for i = 0:3
    topic_name = sprintf('vehicle_imu_status_%d', i);
    if isfield(log.data, topic_name)
        idx = length(vib_data) + 1;
        vib_data(idx).id = i;
        vib_data(idx).t = get_t(log.data.(topic_name));
        % Extract vibration metric
        vib_data(idx).val = log.data.(topic_name).accel_vibration_metric;
    end
end
if ~isempty(vib_data)
    % Print sampling info (use first existing IMU)
    vib_dt = mean(diff(vib_data(1).t));
    fprintf('Vibration metric sampling period: %f (ms), frequency: %f (Hz) \n', 1000*vib_dt, 1/vib_dt);
end



% 2.14 Raw Angular Speed (sensor_combined)
if isfield(log.data, 'sensor_combined_0')
    % Extract and convert to deg/s
    if ismember('gyro_rad_0_', log.data.sensor_combined_0.Properties.VariableNames)
        raw_gyro_t = get_t(log.data.sensor_combined_0);
        raw_gyro = [log.data.sensor_combined_0.gyro_rad_0_, ...
                    log.data.sensor_combined_0.gyro_rad_1_, ...
                    log.data.sensor_combined_0.gyro_rad_2_] * r2d;
    end
end

% 2.15 FIFO Data Parsing (Accel & Gyro)
% Struct array to store parsed data: fifo_acc(1).t, fifo_acc(1).d
fifo_acc = struct('id', {}, 't', {}, 'd', {}, 'raw_t', {});
fifo_gyro = struct('id', {}, 't', {}, 'd', {});

for i = 0:2
    % --- FIFO Accel ---
    topic_name = sprintf('sensor_accel_fifo_%d', i);
    if isfield(log.data, topic_name)
        idx = length(fifo_acc) + 1;
        fifo_acc(idx).id = i;
        % Save original packet timestamp for calculating packet loss/sampling interval
        fifo_acc(idx).raw_t = get_t(log.data.(topic_name)); 
        % Parse virtual high-frequency data
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
        fifo_gyro(idx).d = d_val * r2d; % Convert to deg/s
        fprintf('Parsed FIFO Gyro %d: %d samples\n', i, length(t_us));
    end
end

% 2.16 Magnetic Field (magnetic field strength)
% Prefer vehicle_magnetometer, legacy logs may use sensor_combined
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

% 2.17 Distance Sensor (distance sensor)
if isfield(log.data, 'distance_sensor_0')
    dist_sensor_t = get_t(log.data.distance_sensor_0);
    dist_val = log.data.distance_sensor_0.current_distance;
    dist_var = log.data.distance_sensor_0.variance;
end
% And corresponding estimate (Dist bottom)
if isfield(log.data, 'vehicle_local_position_0') && ismember('dist_bottom', log.data.vehicle_local_position_0.Properties.VariableNames)
    dist_bottom_t = get_t(log.data.vehicle_local_position_0);
    dist_bottom = log.data.vehicle_local_position_0.dist_bottom;
    dist_bottom_valid = log.data.vehicle_local_position_0.dist_bottom_valid;
end

% 2.18 GPS Status (precision & interference)
if isfield(log.data, 'vehicle_gps_position_0')
    gps_t = get_t(log.data.vehicle_gps_position_0);
    % Use table to flexibly handle column existence
    gps_info = table();
    gps_info.eph = log.data.vehicle_gps_position_0.eph;
    gps_info.epv = log.data.vehicle_gps_position_0.epv;
    gps_info.s_variance = log.data.vehicle_gps_position_0.s_variance_m_s;
    gps_info.satellites = log.data.vehicle_gps_position_0.satellites_used;
    gps_info.fix_type = log.data.vehicle_gps_position_0.fix_type;
    gps_info.noise = log.data.vehicle_gps_position_0.noise_per_ms;
    gps_info.jamming = log.data.vehicle_gps_position_0.jamming_indicator;
    
    % Compatibility with HDOP/VDOP in legacy logs
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
    
    % Optional: internal resistance estimate
    if ismember('internal_resistance_estimate', log.data.battery_status_0.Properties.VariableNames)
        bat_res = log.data.battery_status_0.internal_resistance_estimate;
    end
end
if isfield(log.data, 'system_power_0')
    sys_pwr_t = get_t(log.data.system_power_0);
    % Compatibility with different naming conventions (5V rail)
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
% ESC Temp (if available)
if isfield(log.data, 'esc_status_0')
    tbl = log.data.esc_status_0;
    vars = tbl.Properties.VariableNames;
    
    % Determine scan range: use max value if esc_count exists, otherwise default to first 8
    esc_limit = 8;
    if ismember('esc_count', vars)
        esc_limit = max(tbl.esc_count);
    end
    
    for i = 0:(esc_limit-1)
        % ulog2csv column name is usually 'esc_N__esc_temperature' (double underscore) or 'esc_N_esc_temperature'
        col_name = sprintf('esc_%d__esc_temperature', i);
        if ~ismember(col_name, vars)
            col_name = sprintf('esc_%d_esc_temperature', i); % Fallback format
        end
        
        if ismember(col_name, vars)
            val = tbl.(col_name);
            % Python logic: if np.amax(esc_temp) > 0.001
            if max(val) > 0.001
                temp_data(end+1).name = sprintf('ESC %d', i);
                temp_data(end).t = tbl.timestamp * 1e-6;
                temp_data(end).val = val;
            end
        end
    end
end

%% =========================================================================
%  Estimator Flags (Python logic replication)
%  Function: Dynamically extract Health, Timeout and Innovation Flags, only plot non-zero data
% =========================================================================
if isfield(log.data, 'estimator_status_0')
    est_status = log.data.estimator_status_0;
    est_t = get_t(est_status);
    
    % 1. Prepare data pool (Label, Data)
    %    Use cell array to store all candidate signals
    candidates = {}; 
    
    % --- Basic flags ---
    candidates{end+1, 1} = 'Health Flags';
    candidates{end, 2}   = double(est_status.health_flags);
    
    candidates{end+1, 1} = 'Timeout Flags';
    candidates{end, 2}   = double(est_status.timeout_flags);
    
    % --- Innovation Check Flags (require bit manipulation to extract) ---
    % Check if field exists (prevent error in legacy logs)
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

% 2.24 airspeed
if isfield(log.data, 'airspeed_validated_0')
    airspeed_t = get_t(log.data.airspeed_validated_0);
    cairspeed_m_s = log.data.airspeed_validated_0.true_airspeed_m_s;
 end
