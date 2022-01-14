function rpm = thrust_to_rpm_lookup(thrust,thruster_param)
% Inputs a desired thrust in newton.
% Thrister param: Struct generated from running sim_init;
% Outputs a corresponding motor RPM. 

if(thrust == 0)
    rpm = 0;
    return;
end




l_brakepoint = length(thruster_param.Thrust_brakepoints);
% Saturate thrust
if(thrust > thruster_param.Thrust_brakepoints(1,end))
    thrust = thruster_param.Thrust_brakepoints(1,end);
    %warning('Thrust is above limit in "thrust_to_throttle"');
elseif(thrust < thruster_param.Thrust_brakepoints(1,1))
    thrust = thruster_param.Thrust_brakepoints(1,1);
    %warning('Thrust is below limit in "thrust_to_throttle"');
end


% Find lower RPM index
i = sum(thruster_param.Thrust_brakepoints <= thrust);
if i<1
    i = 1;
elseif i>(l_brakepoint-1)
    i=(l_brakepoint-1);
end

rpm = thruster_param.RPM_brakepoints(1,i);

% Make linear interpolation between i and i+1;

delta = thrust - thruster_param.Thrust_brakepoints(1,i);

delta = delta*(thruster_param.RPM_brakepoints(1,i+1) - thruster_param.RPM_brakepoints(1,i))/(thruster_param.Thrust_brakepoints(1,i+1) - thruster_param.Thrust_brakepoints(1,i));

rpm = rpm + delta;

rpm = saturate(rpm, thruster_param.min_rpm, thruster_param.max_rpm);


end

