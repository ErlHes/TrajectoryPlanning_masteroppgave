function thrust = rpm_to_thrust_lookup(rpm,thruster_param)
% Inputs a desired thrust in newton.
% Thrister param: Struct generated from running sim_init;
% Outputs a corresponding motor throttle for the thruster system. 

if(rpm == 0)
    thrust = 0;
    return;
end

max_thrust = thruster_param.max_thrust;
min_thrust = thruster_param.min_thrust;
l_lookup = length(thruster_param.RPM_brakepoints);
% Saturate the input RPM to within the brakepoint values.
if(rpm > thruster_param.RPM_brakepoints(1,l_lookup))
    rpm = thruster_param.RPM_brakepoints(1,l_lookup);
    %warning('Thrust is above limit in "thrust_to_throttle"');
elseif(rpm < thruster_param.RPM_brakepoints(1,1))
    rpm = thruster_param.RPM_brakepoints(1,1);
    %warning('Thrust is below limit in "thrust_to_throttle"');
end


% Find lower RPM index
i = sum(thruster_param.RPM_brakepoints <= rpm);
if i<1
    i = 1;
elseif i>(l_lookup-1)
    i=(l_lookup-1);
end

thrust = thruster_param.Thrust_brakepoints(1,i);

% Make linear interpolation between i and i+1;
delta_rpm = rpm - thruster_param.RPM_brakepoints(1,i); 

delta_thrust = delta_rpm*(thruster_param.Thrust_brakepoints(1,i+1) - thruster_param.Thrust_brakepoints(1,i))/(thruster_param.RPM_brakepoints(1,i+1) - thruster_param.RPM_brakepoints(1,i));

thrust = thrust + delta_thrust;

% Saturate
thrust = saturate(thrust, min_thrust, max_thrust);

end

