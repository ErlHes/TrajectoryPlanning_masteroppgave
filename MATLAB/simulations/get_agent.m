function agent = get_agent(id, eta_0, nu_0 ,size, model, gnc,speed, wp,visible)
agent = struct;

agent.id = id;
% Vessel states
agent.eta = eta_0;
agent.nu = nu_0;
agent.nu_dot = zeros(3,1);
agent.eta_dot = rotZ(eta_0(3))*nu_0;
agent.vel = norm(nu_0(1:2));
agent.ts = [0,0,0,0]'; % Thruster states [thrust1, thrust2, angle1, angle2]
agent.ts_d = [0,0,0,0]';


agent.size = size;
agent.model = model;
agent.gnc = gnc;
agent.speed = speed;
agent.wp = wp;
agent.visible = visible;
agent.current_wp = 1;
agent.wp_est = nan(2,20);

% States from controller
agent.tau = [0,0,0]';
agent.tau_d = [0,0,0]';
agent.pid_P = [0,0,0]';
agent.pid_I = [0,0,0]';
agent.pid_D = [0,0,0]';
agent.pid_ff_acc = [0,0,0]';
agent.pid_ff_vel = [0,0,0]';

% States from guidance
agent.eta_ref = eta_0;
agent.eta_dot_ref = [0,0,0]';

% States from reference filter
agent.eta_d = eta_0;
agent.eta_dot_d = rotZ(eta_0(3))*nu_0;
agent.eta_ddot_d = [0,0,0]';
agent.end_of_path = 0;

agent.classification = nan(2,20);
agent.rel_vel_hysteresis = zeros(2,20);



end