%% Initialize control parameters
Ts_ctrl = 0.1;
% Ts_ctrl = 0.1;
Ts_thrust = 0.1;
% Ts_thrust = 0.1;


b0 = zeros(16,1);

A = [0 1; 0 0];
B = [0;1];
p = [-2 -1];
K_obs = place(A,B,p);
    
% U_c = 0.0;
U_c = 0;
% U_c = 0.7;
% U_c = 1.7;
% U_c = 0.4;
beta_c = -45*pi/180;
% beta_c = -30*pi/180;
velc = U_c*[cos(beta_c) sin(beta_c)]';        % current in the inertial frame

%Thruster system
load('thruster.mat');
thruster_param = thruster;
thruster_param.dt = Ts_thrust;
%Controller parameters
controller_param = struct;
controller_param.delta_t = Ts_ctrl;
adaptive = 0;
adaptive_h = 0;



% gamma_bias = .1;
% gamma_bias_psi = 1.2;
% gamma_d = .1;
% gamma_d_psi = 10;
% gamma_nrr = .1;
% gamma_c = 10;
% gamma_bias2 = 0.03;
% gamma_bias_psi2 = 0.03;

gamma_bias = .5;
gamma_bias_psi = 2;
gamma_d = .5;
gamma_d_psi = 10;
gamma_nrr = .5;
gamma_c = 10;
gamma_bias2 = 1;
gamma_bias_psi2 = 1.4;

% controller_param.Gamma = 10e3*adaptive*diag([gamma_bias gamma_bias gamma_bias_psi gamma_d gamma_d gamma_d gamma_d_psi gamma_d_psi gamma_d_psi gamma_nrr ]);
% controller_param.Gamma = 25*10e3*adaptive*diag([gamma_bias gamma_bias gamma_bias_psi gamma_d gamma_d gamma_d gamma_d_psi gamma_d_psi gamma_d_psi gamma_nrr gamma_c gamma_c gamma_c gamma_c]);
controller_param.Gamma = 10e3*adaptive*diag([gamma_bias gamma_bias gamma_bias_psi gamma_d gamma_d gamma_d gamma_d_psi gamma_d_psi gamma_d_psi gamma_nrr gamma_c gamma_c gamma_c gamma_c gamma_bias2 gamma_bias_psi2]);
controller_param.Gamma_h = 10e3*adaptive_h*diag([gamma_bias gamma_bias gamma_bias_psi gamma_d gamma_d gamma_d gamma_d_psi gamma_d_psi gamma_d_psi gamma_nrr gamma_c gamma_c gamma_c gamma_c gamma_bias2 gamma_bias_psi2]);

integral_action = 0;
controller_param.integral_action = integral_action;
if(integral_action)
    m_1 = 2;
    F_1 = [zeros(m_1,m_1) eye(m_1), zeros(m_1,m_1);
           zeros(m_1,m_1) zeros(m_1,m_1) eye(m_1);
           zeros(m_1,m_1) zeros(m_1,m_1) zeros(m_1,m_1)];
    F_2 = [0 1 0;0 0 1;0 0 0];
    G_1 = [zeros(m_1,m_1); zeros(m_1,m_1); eye(m_1)];
    G_2 = [0;0;1];
    G1 = -G_1*G_1';
    Q1 = eye(6);
    Q2 = eye(3); 
    G2 = -G_2*G_2';
    F = [zeros(3,3) eye(3) zeros(3,3);
        zeros(3,3) zeros(3,3) eye(3);
        zeros(3,3) zeros(3,3) zeros(3,3)];
    G = [zeros(3,3); zeros(3,3); eye(3)];
    controller_param.G = G;
    G = -G*G';
    Q = blkdiag(Q1,Q2);

    [P_1,~,~] = icare(F_1,[],Q1,[],[],[],G1);
    [P_2,~,~] = icare(F_2,[],Q2,[],[],[],G2);
    [P,~,~] = icare(F,[],Q,[],[],[],G);
    % eps1 = 6;
    % eps2 = 2;
%     eps1 = .85;
%     eps2 = .05;
    eps1 = 1;
    eps2 = .15;
    % Integral "gains"
    eps1_int = 0.1;
    eps2_int = 0.05;
    M_1 = blkdiag(eps1_int*eye(2),1/eps1*eye(2),eye(2));
    M_2 = blkdiag(eps2_int*1,1/eps2,1);
    M = blkdiag(eye(3),1/eps1*eye(2),1/eps2,eye(3));
else
    m_1 = 2;
    F_1 = [zeros(m_1,m_1) eye(m_1);
           zeros(m_1,m_1) zeros(m_1,m_1)];
    F_2 = [0 1; 0 0];
    G_1 = [zeros(m_1,m_1); eye(m_1)];
    G_2 = [0;1];
    G1 = -G_1*G_1';
    Q1 = eye(4);                                                       % Changed from 0 to *10   
    Q2 = eye(2); %\0.03
    % Q1 = 0.08*eye(4);
    % Q2 = 0.08*eye(2); %\0.03
    G2 = -G_2*G_2';
    F = [zeros(3,3) eye(3);
        zeros(3,3) zeros(3,3)];
    G = [zeros(3,3); eye(3)];
    controller_param.G = G;
    G = -G*G';
    Q = blkdiag(Q1,Q2);
    % [controller_param.P_1,~,~] = icare(F_1,[],Q1,[],[],[],G1);
    % [controller_param.P_2,~,~] = icare(F_2,[],Q2,[],[],[],G2);
    % [controller_param.P,~,~] = icare(F,[],Q,[],[],[],G);
    % controller_param.gamma_1 = min(eig(Q1))/max(eig(controller_param.P_1));
    % controller_param.gamma_2 = min(eig(Q2))/max(eig(controller_param.P_2));

    [P_1,~,~] = icare(F_1,[],Q1,[],[],[],G1);
    [P_2,~,~] = icare(F_2,[],Q2,[],[],[],G2);
    [P,~,~] = icare(F,[],Q,[],[],[],G);
    % eps1 = 6;
    % eps2 = 2;
    eps1 = .85;
    eps2 = .05;    
%     eps1 = .05;
%     eps2 = .85;
    M_1 = blkdiag(1/eps1*eye(2),eye(2));
    M_2 = blkdiag(1/eps2,1);
    M = blkdiag(1/eps1*eye(2),1/eps2,eye(3));
end
controller_param.P_1 = M_1*P_1*M_1;
controller_param.P_2 = M_2*P_2*M_2;
controller_param.P = M*P*M;
controller_param.eps1 = eps1;
controller_param.eps2 = eps2;

controller_param.gamma_1 = min(eig(Q1))/max(eig(controller_param.P_1));
controller_param.gamma_2 = min(eig(Q2))/max(eig(controller_param.P_2));



controller_param.p1 = 500.0;
controller_param.p2 = 200.0;
controller_param.heading_error_angle_saturation = 0.20;
controller_param.yaw_rate_saturation = 1.0;
controller_param.c_y1_dot = 2;
controller_param.c_y2_dot = 1;
controller_param.cbf_slack_variable_penalty = 10000;
controller_param.actuator_rate_constraints_factor = 2.0;

controller_param.F_1 = F_1;
controller_param.F_2 = F_2;
controller_param.G_1 = G_1;
controller_param.G_2 = G_2;

controller_param.alpha_accel = 0.01;

% Scaling for damping matrix
controller_param.d11_scale = 0.5;
controller_param.d22_scale = 0.5;
controller_param.d23_scale = 0.5;
controller_param.d32_scale = 0.5;
controller_param.d33_scale = 0;
% ROS message periods
navigation_message_period = 0.01;
thruster_control_msgs_period = Ts_ctrl;
simulator_thruster_state_msgs = Ts_thrust;
object_data_frequency = 0.01;
trj_gen_and_ref_filter_period = 0.01;
trajectory_reference_period = trj_gen_and_ref_filter_period;

clear('F','F_1','F_2','A','adaptive','adaptive_h','B','beta_c','eps1','eps2','G','G1','G2','G_1','G_2')
clear('gamma_bias','gamma_bias2','gamma_bias_psi','gamma_bias_psi2','gamma_c','gamma_d','gamma_d_psi','gamma_nrr');
clear('integral_action','K_obs','M,','m_1','M_1','M_2','P','P_1','P_2','Q','Q1','Q2')




controller_param_bus = controller_param;
simulink_running = 1;
if(simulink_running)
Simulink_obj = Simulink.Parameter;
Simulink_obj.Value = controller_param_bus;
Simulink_obj.CoderInfo.StorageClass = 'ExportedGlobal';
controller_param_bus = Simulink_obj;
clear Simulink_el;

busInfo=Simulink.Bus.createObject(controller_param_bus.Value);
ControllerParam = eval(busInfo.busName);
controller_param_bus.DataType='Bus: ControllerParam';
clear(busInfo.busName);
clear busInfo;
end