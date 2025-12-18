function vehicle_rates_setpoint = apply_tailsitter_rate_sp_fix(vehicle_rates_setpoint, fw_intervals)
t = vehicle_rates_setpoint(:,1);

for k = 1:size(fw_intervals,1)
    t0 = fw_intervals(k,1);
    t1 = fw_intervals(k,2);
    mask = (t >= t0) & (t <= t1);

    rollsp = vehicle_rates_setpoint(mask,2);
    yawsp  = vehicle_rates_setpoint(mask,4);

    helper = rollsp;
    vehicle_rates_setpoint(mask,2) = -yawsp;
    vehicle_rates_setpoint(mask,4) = helper;
end
end