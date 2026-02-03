function name = lookup_pwm_func(code)
    % Translate based on XML definitions
    if code == 0
        name = 'Disabled';
    elseif code == 1
        name = 'Constant Min';
    elseif code == 2
        name = 'Constant Max';
    elseif code >= 101 && code <= 112
        name = sprintf('Motor %d', code - 100);
    elseif code >= 201 && code <= 208
        name = sprintf('Servo %d', code - 200);
    elseif code >= 301 && code <= 306
        name = sprintf('Actuator Set %d', code - 300);
    elseif code == 400
        name = 'Landing Gear';
    elseif code == 401
        name = 'Parachute';
    elseif code == 402
        name = 'RC Roll';
    elseif code == 403
        name = 'RC Pitch';
    elseif code == 404
        name = 'RC Throttle';
    elseif code == 405
        name = 'RC Yaw';
    elseif code >= 406 && code <= 412
        name = sprintf('RC AUX %d', code - 406);
    elseif code == 420
        name = 'Gimbal Roll';
    elseif code == 421
        name = 'Gimbal Pitch';
    elseif code == 422
        name = 'Gimbal Yaw';
    elseif code == 430
        name = 'Gripper';
    elseif code == 440
        name = 'Landing Gear Wheel';
    elseif code == 450
        name = 'IC Engine Ignition';
    elseif code == 451
        name = 'IC Engine Throttle';
    else
        name = sprintf('Unknown Func (%d)', code);
    end
end