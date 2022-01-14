

yaml_struct = ReadYaml('C:\Users\erlen\Documents\GitHub\Fordypnings-rapport\multiAgent_simulator-master\parameters\dynamic_positioning.yaml');

parameters_pid_controller.controller = yaml_struct.controller;
parameters_pid_controller.model = yaml_struct.model;
parameters_pid_controller.dt = 0.1;


parameters_pid_controller.controller.Kp = [parameters_pid_controller.controller.Kp{:}];
parameters_pid_controller.controller.Ki = [parameters_pid_controller.controller.Ki{:}];
parameters_pid_controller.controller.Kd = [parameters_pid_controller.controller.Kd{:}];
parameters_pid_controller.controller.tau_i_windup = [parameters_pid_controller.controller.tau_i_windup{:}];

parameters_pid_controller.model.M = [parameters_pid_controller.model.M{:}];
parameters_pid_controller.model.D_lin = [parameters_pid_controller.model.D_lin{:}];
parameters_pid_controller.model.D_quad = [parameters_pid_controller.model.D_quad{:}];
parameters_pid_controller.model.D_cub = [parameters_pid_controller.model.D_cub{:}];
