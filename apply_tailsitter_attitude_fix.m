function vehicle_attitude = apply_tailsitter_attitude_fix(t_att, vehicle_attitude, fw_intervals)
% vehicle_attitude: [q0,q1,q2,q3] 
Q = vehicle_attitude; % [w x y z]

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
vehicle_attitude = Q_new;
end