
run('lyapynov_control_system_parameters.m');
run('timing_parameters.m');
run('system_parameters.m');
run('pid_controller_parameters.m')
run('guidance_parameters.m');


% run('init_path_following.m')

%% Assign parameters to combined struct to reduce number of inputs.

parameters.guidance = guidance;
parameters.pid_controller = parameters_pid_controller;
parameters.thruster = thruster_param;
% parameters.lyapunov_controller = controller_param;
parameters.timing = timing;
parameters.system = sys_parameters;


% Clear reallocated struct;
clear 'guidance' 'parameters_pid_controller'  'thruster_param';
clear 'controller_param' 'timing'  'sys_parameters'