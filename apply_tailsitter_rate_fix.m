function vehicle_angular_velocity = apply_tailsitter_rate_fix(vehicle_angular_velocity, fw_intervals)
t = vehicle_angular_velocity(:,1);

for k = 1:size(fw_intervals,1)
    t0 = fw_intervals(k,1);
    t1 = fw_intervals(k,2);
    mask = (t >= t0) & (t <= t1);

    rollspeed = vehicle_angular_velocity(mask,3);
    yawspeed  = vehicle_angular_velocity(mask,5);

    helper = rollspeed;
    vehicle_angular_velocity(mask,3) = -yawspeed;
    vehicle_angular_velocity(mask,5) = helper;
end
end