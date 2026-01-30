function vehicle_angular_velocity = apply_tailsitter_rate_fix(t, vehicle_angular_velocity, fw_intervals)
 

for k = 1:size(fw_intervals,1)
    t0 = fw_intervals(k,1);
    t1 = fw_intervals(k,2);
    mask = (t >= t0) & (t <= t1);

    rollspeed = vehicle_angular_velocity(mask,1);
    yawspeed  = vehicle_angular_velocity(mask,3);

    helper = rollspeed;
    vehicle_angular_velocity(mask,1) = -yawspeed;
    vehicle_angular_velocity(mask,3) = helper;
end
end