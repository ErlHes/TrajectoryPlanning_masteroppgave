function vessel = pidController(vessel, parameters_pid_controller)

dt = parameters_pid_controller.dt; 
% Parameters
Kp = diag(parameters_pid_controller.controller.Kp);
Ki = diag(parameters_pid_controller.controller.Ki);
Kd = diag(parameters_pid_controller.controller.Kd);
integrator_windup_limits = parameters_pid_controller.controller.tau_i_windup';
k_ff_acc = parameters_pid_controller.controller.k_ff_acc;
k_ff_vel = parameters_pid_controller.controller.k_ff_vel;


D_lin = diag(parameters_pid_controller.model.D_lin);
D_quad = diag(parameters_pid_controller.model.D_quad);
D_cub = diag(parameters_pid_controller.model.D_cub);

% Rotation
R = rotZ(-vessel.eta(3));
M = diag(parameters_pid_controller.model.M);
% Trackign objective
eta_tilde = vessel.eta-vessel.eta_d;
eta_tilde(3) = shortest_angle_path(vessel.eta_d(3),vessel.eta(3));
eta_tilde_d = vessel.eta_dot-vessel.eta_dot_d;

% PID terms
tau_i_ned = vessel.pid_I - dt*Ki*eta_tilde;
tau_i_ned = max(-integrator_windup_limits,min(tau_i_ned,integrator_windup_limits));

tau_p_ned = -Kp*eta_tilde;
tau_d_ned = -Kd*eta_tilde_d;

tau_pid_ned = tau_p_ned + tau_i_ned + tau_d_ned;

% Feed forward terms
nu_ref = R*vessel.eta_dot_d;
nu_dot_ref = R*vessel.eta_ddot_d;
D_ff = D_lin + D_quad*diag(abs(nu_ref)) + D_cub*diag(nu_ref)*diag(nu_ref);
tau_ff_acc = k_ff_acc*M*nu_dot_ref;
tau_ff_vel = k_ff_vel*D_ff*nu_ref;

tau_d = R*tau_pid_ned +tau_ff_acc+tau_ff_vel;
tau_N_max = 1000;
tau_d(3) = max(-tau_N_max, min(tau_N_max, tau_d(3)));

vessel.tau_d = tau_d;
vessel.pid_P = tau_p_ned;
vessel.pid_I = tau_i_ned;
vessel.pid_D = tau_d_ned;
vessel.pid_ff_acc = tau_ff_acc;
vessel.pid_ff_vel = tau_ff_vel;
end