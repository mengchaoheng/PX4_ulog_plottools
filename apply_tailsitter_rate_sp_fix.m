function vehicle_rates_setpoint = apply_tailsitter_rate_sp_fix(t, vehicle_rates_setpoint, fw_intervals)


for k = 1:size(fw_intervals,1)
    t0 = fw_intervals(k,1);
    t1 = fw_intervals(k,2);
    mask = (t >= t0) & (t <= t1);

    rollsp = vehicle_rates_setpoint(mask,1);
    yawsp  = vehicle_rates_setpoint(mask,3);

    helper = rollsp;
    vehicle_rates_setpoint(mask,1) = -yawsp;
    vehicle_rates_setpoint(mask,3) = helper;
end
end