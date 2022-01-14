home_dir = '/Users/erlen/Documents/GitHub/Fordypnings-rapport/multiAgent_simulator-master';
load(strcat(home_dir,'simulations/', simulation, '/sim_output.mat'));

% load(strcat(home_dir,'simulations/', simulation, '/batch_run_14/data/','/sim_output_5105_140.mat'));

settings = sim_output.settings;
agent_data = sim_output.agent_data;
time = sim_output.time;


clear_figures = true;
agent_number = 1;

% %%
% figure(201),title('Thruster states');
% clf(201);
l =  size(time,2); % Length of data-array (number of logged timesteps)
% ts = zeros(4,l);
% ts_d = zeros(4,l);
% tau = zeros(3,l);
% tau_d = zeros(3,l);
% for i=1:l
%     ts(:,i) = agent_data(i,agent_number).ts;  
%     
%     ts(3:4,i) = wrap_plus_minus_pi(ts(3:4,i));
%     ts_d(:,i) = agent_data(i,agent_number).ts_d;    
%     ts_d(3:4,i) = wrap_plus_minus_pi(ts_d(3:4,i));
% 
%     poly = parameters.path_following_model_params.thruster_force_polynomial;
%     tau(:,i) = [cos(ts(3,i)), cos(ts(4,i));...
%               sin(ts(3,i)), sin(ts(4,i));...
%               1.2*sin(ts(3,i)), -1.2*sin(ts(4,i))]*[thruster_force(ts(1,i), poly);thruster_force(ts(2,i), poly)];
%     tau_d(:,i) = [cos(ts_d(3,i)), cos(ts_d(4,i));...
%           sin(ts_d(3,i)), sin(ts_d(4,i));...
%           1.2*sin(ts_d(3,i)), -1.2*sin(ts_d(4,i))]*[thruster_force(ts_d(1,i), poly);thruster_force(ts_d(2,i), poly)];
% end
% 
% subplot(3,1,1), title('Thrust');
% hold on, grid on;
% plot(time,  ts(1:2,:));
% plot(time,  ts_d(1:2,:));
% legend('$\omega_1$','$\omega_2$','$\omega_1d$','$\omega_2d$');
% 
% subplot(3,1,2), title('Azimuth Angle');
% hold on, grid on;
% plot(time,  (rad2deg((ts(3:4,:)))));
% plot(time,  (rad2deg((ts_d(3:4,:)))));
% legend('$\alpha_1$','$\alpha_2$','$\alpha_1d$','$\alpha_2d$');
% % plot(time, exitflag(1:end-1))
% 
% 
% subplot(3,1,3), title('Tau and tau d');
% hold on, grid on;
% plot(time,  tau);
% plot(time,  tau_d);
% legend('X','Y','N', 'Xd','Yd', 'Nd');





figure(2002),title('Vessel states');
if(clear_figures), clf(2002); end
nu_dot = zeros(3,l);
for i=1:l
    nu_dot(:,i) = agent_data(i,agent_number).nu_dot;   
end

acc_abs = sqrt(nu_dot(1,:).^2 + nu_dot(2,:).^2);

subplot(2,1,1), title('surge and sway acceleration');
hold on, grid on;
plot(time,  nu_dot(1,:));
plot(time,  nu_dot(2,:));

subplot(2,1,2), title('Absolute acceleration');
hold on, grid on;
plot(time,  acc_abs(1,:));



%%
figure(202),title('Vessel states');
if(clear_figures), clf(202); end
eta = zeros(3,l);
eta_d = zeros(3,l);
eta_ref = zeros(3,l);
for i=1:l
    eta(:,i) = agent_data(i,agent_number).eta;   
    eta_d(:,i) = agent_data(i,agent_number).eta_d;
    eta_ref(:,i) = agent_data(i,agent_number).eta_ref;
end

subplot(2,1,1), title('North East');
hold on, grid on;
plot(time,  eta(1:2,:));
plot(time,  eta_d(1:2,:));
subplot(2,1,2), title('Heading');
hold on, grid on;
plot(time,  rad2deg(eta(3,:)),'b');
plot(time,  rad2deg(eta_d(3,:)),'r');
plot(time,  rad2deg(eta_ref(3,:)),'g');
legend('\psi', '\psi_d', '\psi_{ref}');

%%
figure(203),title('Vessel states');
if(clear_figures),clf(203); end
eta_dot = zeros(3,l);
eta_dot_d = zeros(3,l);
eta_dot_ref = zeros(3,l);
for i=1:l
    eta_dot(:,i) = agent_data(i,agent_number).eta_dot;   
    eta_dot_d(:,i) = agent_data(i,agent_number).eta_dot_d;
    eta_dot_ref(:,i) = agent_data(i,agent_number).eta_dot_ref;
end

subplot(2,1,1), title('eta dot ');
hold on, grid on;
plot(time,  eta_dot(1:2,:));
plot(time,  eta_dot_d(1:2,:));
subplot(2,1,2), title('Yaw-rate');
hold on, grid on;
plot(time,  rad2deg(eta_dot(3,:)),'b');
plot(time,  rad2deg(eta_dot_d(3,:)),'r');
plot(time,  rad2deg(eta_dot_ref(3,:)),'g');

%%
figure(204),
if(clear_figures), clf(204); end
speed = zeros(1,l);
speed_d = zeros(1,l);
speed_ref = zeros(1,l);
for i=1:l
    speed(1,i) = norm(agent_data(i,agent_number).eta_dot(1:2),2);   
    speed_d(1,i) = norm(agent_data(i,agent_number).eta_dot_d(1:2,1),2);
    speed_ref(1,i) = norm(agent_data(i,agent_number).eta_dot_ref(1:2,1),2);
end

% subplot(2,1,1), title('eta dot ');
hold on, grid on;
title('Speed and reference');
plot(time,  speed(1,:),'b');
plot(time,  speed_d(1,:),'y');
plot(time,  speed_ref(1,:),'r');
legend('u', 'u_d', 'u_{ref}')
% subplot(2,1,2), title('Yaw-rate');
% hold on, grid on;
% plot(time,  rad2deg(eta_dot(3,:)),'b');
% plot(time,  rad2deg(eta_dot_d(3,:)),'r');
% plot(time,  rad2deg(eta_dot_ref(3,:)),'g');

%%
figure(205),title('Vessel states');
if(clear_figures), clf(205); end

subplot(3,1,1), title('North East error');
hold on, grid on;
plot(time,  eta(1:2,:)-eta_d(1:2,:));
% plot(time,  eta_d(1:2,:));

subplot(3,1,2), title('Position error');
hold on, grid on;
eta_err = eta(1:2,:)-eta_d(1:2,:);
eta_err_ref = eta(1:2,:)-eta_ref(1:2,:);

pos_err = zeros(1,size(eta_err,2));
pos_err_ref = zeros(1,size(eta_err,2));
for i=1:size(eta_err,2)
    pos_err(i) = norm(eta_err(:,i),2);
    pos_err_ref(i) = norm(eta_err_ref(:,i),2);
end
eta_err = norm(eta_err,2);
plot(time,  pos_err,'b');
plot(time, pos_err_ref,'r');


subplot(3,1,3), title('Velocity error');
hold on, grid on;
tilde_eta_dot = eta_dot(1:3,:)-eta_dot_d(1:3,:);
plot(time, tilde_eta_dot);
% legend('\tilde{p}');


%% PID controller states
% if isfield(agent_data(1,1), 'tau')
%     tau = zeros(3,l);
%     tau_d = zeros(3,l);
%     pid_P = zeros(3,l);
%     pid_I = zeros(3,l);
%     pid_D = zeros(3,l);
%     pid_ff_vel = zeros(3,l);
%     pid_ff_acc = zeros(3,l);
%     
%     for i=1:l
%         R = rotZ(eta(3,i));
%         tau(:,i) = agent_data(i,agent_number).tau;
%         tau_d(:,i) = agent_data(i,agent_number).tau_d;
%         pid_P(:,i) = R*agent_data(i,agent_number).pid_P;
%         pid_I(:,i) = R*agent_data(i,agent_number).pid_I;
%         pid_D(:,i) = R*agent_data(i,agent_number).pid_D;
%         pid_ff_vel(:,i) = agent_data(i,agent_number).pid_ff_vel;
%         pid_ff_acc(:,i) = agent_data(i,agent_number).pid_ff_acc;
%     end
%     
%     
%     
%     
%     figure(206),title('PID tau');
%     clf(206);
% 
%     subplot(3,1,1), title('PID tau X');
%     hold on, grid on;
%     plot(time,  tau(1,:),'b');
%     plot(time,  tau_d(1,:),'r');
% 
%     subplot(3,1,2), title('PID tau Y');
%     hold on, grid on;
%     plot(time,  tau(2,:),'b');
%     plot(time,  tau_d(2,:),'r');
% 
%     subplot(3,1,3), title('PID tau N');
%     hold on, grid on;
%     plot(time,  tau(3,:),'b');
%     plot(time,  tau_d(3,:),'r');
%     
%     figure(207),title('PID tau components');
%     clf(207);
%     
%     subplot(3,1,1), title('PID tau X');
%     hold on, grid on;
%     plot(time,  pid_P(1,:),'b');
%     plot(time,  pid_I(1,:),'r');
%     plot(time,  pid_D(1,:),'g');
%     plot(time,  pid_ff_vel(1,:),'y');
%     plot(time,  pid_ff_acc(1,:),'m');
% 
% 
%     subplot(3,1,2), title('PID tau Y');
%     hold on, grid on;
%     plot(time,  pid_P(2,:),'b');
%     plot(time,  pid_I(2,:),'r');
%     plot(time,  pid_D(2,:),'g');
%     plot(time,  pid_ff_vel(2,:),'y');
%     plot(time,  pid_ff_acc(2,:),'m');
%     
%     subplot(3,1,3), title('PID tau N');
%     hold on, grid on;
%     plot(time,  pid_P(3,:),'b');
%     plot(time,  pid_I(3,:),'r');
%     plot(time,  pid_D(3,:),'g');
%     plot(time,  pid_ff_vel(3,:),'y');
%     plot(time,  pid_ff_acc(3,:),'m');
%     
%     figure(208),title('PID tau');
%     clf(208);
% 
%     subplot(3,1,1), title('PID tau X');
%     hold on, grid on;
%     plot(time,  tau(1,:),'b');
%     plot(time,  tau_d(1,:),'r');
% 
%     subplot(3,1,2), title('PID tau Y');
%     hold on, grid on;
%     plot(time,  tau(2,:),'b');
%     plot(time,  tau_d(2,:),'r');
% 
%     subplot(3,1,3), title('PID tau N');
%     hold on, grid on;
%     plot(time,  tau(3,:),'b');
%     plot(time,  tau_d(3,:),'r');
% end


%%
if isfield(agent_data(1,1), 'dd_nud')
nu = zeros(3,l);
nud = zeros(3,l);
d_nud = zeros(3,l);
dd_nud = zeros(3,l);
speed_ref = zeros(1,l);
course_ref = zeros(3,l);
tilde_x = zeros(3,l);

d_nu = zeros(3,l);
tilde_d_nu = zeros(3,l);
z_1 = zeros(3,l);
d_z_1 = zeros(3,l);
alpha_1 = zeros(3,l);
d_alpha_1 = zeros(3,l);
dd_alpha_1 = zeros(3,l);
alpha2 = zeros(3,l);
d_alpha2 = zeros(3,l);
z_2 = zeros(3,l);
rhod = zeros(3,l);
b_rho = zeros(9,l);

    
for i=1:l
    nu(:,i) = agent_data(i,agent_number).nu;
    nud(:,i) = agent_data(i,agent_number).nud;
    d_nud(:,i) = agent_data(i,agent_number).d_nud;
    dd_nud(:,i) = agent_data(i,agent_number).dd_nud;
    speed_ref(:,i) = agent_data(i,agent_number).speed_ref;
    course_ref(:,i) = agent_data(i,agent_number).course_ref;
    tilde_x(:,i) = agent_data(i,agent_number).tilde_x;
    
    d_nu(:,i) = agent_data(i,agent_number).d_nu;
    tilde_d_nu(:,i) = agent_data(i,agent_number).tilde_d_nu;
    z_1(:,i) = agent_data(i,agent_number).z_1;
    d_z_1(:,i) = agent_data(i,agent_number).d_z_1;
    alpha_1(:,i) = agent_data(i,agent_number).alpha_1;
    d_alpha_1(:,i) = agent_data(i,agent_number).d_alpha_1;
    dd_alpha_1(:,i) = agent_data(i,agent_number).dd_alpha_1;
    alpha2(:,i) = agent_data(i,agent_number).alpha2;
    d_alpha2(:,i) = agent_data(i,agent_number).d_alpha2;
    z_2(:,i) = agent_data(i,agent_number).z_2;
    rhod(:,i) = agent_data(i,agent_number).rhod;
    b_rho_ = agent_data(i,agent_number).b_rho;
    b_rho(:,i) = b_rho_(:);
    
    
    
end
    

figure(301)
if(clear_figures), clf(301),  end
subplot(3,1,1)
title('$\nu$ and $\nu_d$')
hold on;
plot(time, nud)
plot(time, nu)
legend({'$u_d$','$v_d$','$r_d$','$u$','$v$','$r$'});

subplot(3,1,2)
hold on;
title('$\dot{\nu}_d$')
plot(time, d_nud)
plot(time, d_nu);
legend({'$\dot{u}_d$','$\dot{v}_d$','$\dot{r}_d$','$\dot{u}$','$\dot{v}$','$\dot{r}$'});

subplot(3,1,3)
hold on;
title('$\ddot{\nu}_d$')
plot(time, dd_nud)
legend({'$u$','$v$','$r$'});


figure(302)
if(clear_figures), clf(302), end
subplot(3,1,1)
hold on;
plot(time, speed_ref)

subplot(3,1,2)
hold on;
plot(time, course_ref)

subplot(3,1,3)
hold on;
plot(time, atan2(nu(2,:), nu(1,:)))


figure(303)
if(clear_figures), clf(303), end
hold on;
title('$\tilde{x}$');
plot(time, tilde_x);
legend({'$\tilde{u}$','$\tilde{v}$','$\tilde{\psi}$'})



figure(304)
if(clear_figures), clf(304), end
hold on;
title('Transit Path')
plot(eta_ref(2,:), eta_ref(1,:), 'r');
plot(eta(2,:), eta(1,:), 'b');
    

%%
% figure(305)
% clf(305)
% subplot(3,1,1)
% title('$z_1$');
% hold on;
% plot(time, z_1)
% 
% subplot(3,1,2)
% title('$z_2$');
% hold on;
% plot(time, z_2)
% 
% subplot(3,1,3)
% title('$\rho_d$');
% hold on;
% plot(time, rhod)


% figure(306)
% clf(306)
% subplot(3,1,1)
% title('$\alpha_1$');
% hold on;
% plot(time, alpha_1)
% 
% subplot(3,1,2)
% title('$\dot{\alpha}_1$');
% hold on;
% plot(time, alpha_1)
% 
% subplot(3,1,3)
% title('$\ddot{\alpha}_1$');
% hold on;
% plot(time, dd_alpha_1)
% 
% figure(307)
% clf(307)
% subplot(3,1,1)
% title('$\alpha_2$');
% hold on;
% plot(time, alpha2)
% 
% subplot(3,1,2)
% title('$\dot{\alpha}_2$');
% hold on;
% plot(time, d_alpha2)
% 
% subplot(3,1,3)
% title('$B_{\rho}$');
% hold on;
% plot(time, b_rho)


    
end