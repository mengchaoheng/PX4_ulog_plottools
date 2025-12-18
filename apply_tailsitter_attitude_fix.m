function vehicle_attitude = apply_tailsitter_attitude_fix(vehicle_attitude, fw_intervals)
% vehicle_attitude: [t, ..., q0,q1,q2,q3,...] 
t_att = vehicle_attitude(:,1);
Q = vehicle_attitude(:,3:6); % [w x y z]

R_all = quat2rotm(Q); % Nx3x3

for k = 1:size(fw_intervals,1)
    t0 = fw_intervals(k,1);
    t1 = fw_intervals(k,2);
    mask = (t_att >= t0) & (t_att <= t1);

    idx = find(mask);
    for ii = idx'
        R = squeeze(R_all(:,:,ii));
        R = tailsitter_R_adapt(R);
        R_all(:,:,ii) = R;
    end
end

Q_new = rotm2quat(R_all); % Nx4 [w x y z]
vehicle_attitude(:,3:6) = Q_new;
end