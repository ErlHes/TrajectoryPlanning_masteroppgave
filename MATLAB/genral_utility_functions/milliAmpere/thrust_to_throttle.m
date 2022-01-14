function throttle = thrust_to_throttle(thrust,thruster_param)
% Inputs a desired thrust in newton.
% Thrister param: Struct generated from running sim_init;
% Outputs a corresponding motor throttle for the thruster system. 

if(thrust == 0)
    throttle = 0;
    return;
end

max_RPM = 1300;
min_RPM = -1300;
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

RPM = thruster_param.RPM_brakepoints(1,i);

% Make linear interpolation between i and i+1;

delta = thrust - thruster_param.Thrust_brakepoints(1,i);

delta = delta*(thruster_param.RPM_brakepoints(1,i+1) - thruster_param.RPM_brakepoints(1,i))/(thruster_param.Thrust_brakepoints(1,i+1) - thruster_param.Thrust_brakepoints(1,i));

RPM = RPM + delta;

if(RPM > max_RPM)
    RPM = 1300;
    %warning('RPM is above max RPM "thrust_to_throttle"');
elseif(RPM < min_RPM)   
    RPM = min_RPM;
    %warning('RPM is below min RPM "thrust_to_throttle"');
end

throttle = RPM/thruster_param.throttle_to_rpm;
end

