home_dir = '/home/emilht/MATLAB/simulator_multiAgent/';
load(strcat(home_dir,'simulations/', simulation, '/sim_output_2DOF.mat'));
sim_output_2DOF = sim_output;
agent_data_2DOF = sim_output_2DOF.agent_data;

load(strcat(home_dir,'simulations/', simulation, '/sim_output_PID.mat'));
% load(strcat(home_dir,'simulations/', simulation, '/sim_output_3DOF.mat'));
sim_output_PID = sim_output;
sim_output_PID = sim_output_PID.agent_data;


% load(strcat(home_dir,'simulations/', simulation, '/sim_output_3DOF.mat'));
load(strcat(home_dir,'simulations/', simulation, '/sim_output_3DOF.mat'));


% load(strcat(home_dir,'simulations/', simulation, '/batch_run_14/data/','/sim_output_5105_140.mat'));
% close all;
set(0,'defaultTextInterpreter','latex'); %trying to set the default


settings = sim_output.settings;
agent_data = sim_output.agent_data;
time = sim_output.time;

agent_number = 1;

t_start = 71;
t_end = 400;
dt = settings.dt;
t_array = (t_start/dt):t_end/dt;
time = time(t_array)-time(t_array(1));
%%
figure(201),%title('Thruster states');
clf(201);
l =  size(t_array,2); % Length of data-array (number of logged timesteps)
ts = zeros(4,l);
ts_d = zeros(4,l);
tau = zeros(3,l);
tau_d = zeros(3,l);
for j=t_array
    i = j-t_array(1)+1;
    ts(:,i) = agent_data(j,agent_number).ts;  
    
    ts(3:4,i) = wrap_plus_minus_pi(ts(3:4,i));
    ts_d(:,i) = agent_data(j,agent_number).ts_d;    
    ts_d(3:4,i) = wrap_plus_minus_pi(ts_d(3:4,i));

    poly = parameters.path_following_model_params.thruster_force_polynomial;
    tau(:,i) = [cos(ts(3,i)), cos(ts(4,i));...
              sin(ts(3,i)), sin(ts(4,i));...
              1.2*sin(ts(3,i)), -1.2*sin(ts(4,i))]*[thruster_force(ts(1,i), poly);thruster_force(ts(2,i), poly)];
    tau_d(:,i) = [cos(ts_d(3,i)), cos(ts_d(4,i));...
          sin(ts_d(3,i)), sin(ts_d(4,i));...
          1.2*sin(ts_d(3,i)), -1.2*sin(ts_d(4,i))]*[thruster_force(ts_d(1,i), poly);thruster_force(ts_d(2,i), poly)];
end

subplot(3,1,1), title('Thrust');
hold on, grid on;
plot(time,  ts(1:2,:));
plot(time,  ts_d(1:2,:));
legend('$\omega_1$','$\omega_2$','$\omega_1d$','$\omega_2d$','interpreter','latex');

subplot(3,1,2), title('Azimuth Angle');
hold on, grid on;
plot(time,  (rad2deg((ts(3:4,:)))));
plot(time,  (rad2deg((ts_d(3:4,:)))));
legend('$\alpha_1$','$\alpha_2$','$\alpha_1d$','$\alpha_2d$','interpreter','latex');

% plot(time, exitflag(1:end-1))


% subplot(3,1,3), title('Tau and tau d');
hold on, grid on;
plot(time,  tau);
plot(time,  tau_d);
legend('X','Y','N', 'Xd','Yd', 'Nd','interpreter','latex');







%%
% figure(202),title('Vessel states');
% clf(202);
eta = zeros(3,l);
eta_d = zeros(3,l);
eta_ref = zeros(3,l);
for j=t_array
    i = j-t_array(1)+1;
    eta(:,i) = agent_data(j,agent_number).eta;   
    eta_d(:,i) = agent_data(j,agent_number).eta_d;
    eta_ref(:,i) = agent_data(j,agent_number).eta_ref;
end
% 
% subplot(2,1,1), title('North East');
% hold on, grid on;
% plot(time,  eta(1:2,:));
% plot(time,  eta_d(1:2,:));
% subplot(2,1,2), title('Heading');
% hold on, grid on;
% plot(time,  rad2deg(eta(3,:)),'b');
% plot(time,  rad2deg(eta_d(3,:)),'r');
% plot(time,  rad2deg(eta_ref(3,:)),'g');
% legend('\psi', '\psi_d', '\psi_{ref}');

% %%
% figure(203),title('Vessel states');
% clf(203);
eta_dot = zeros(3,l);
eta_dot_d = zeros(3,l);
eta_dot_ref = zeros(3,l);
for j=t_array
    i = j-t_array(1)+1;
    eta_dot(:,i) = agent_data(j,agent_number).eta_dot;   
    eta_dot_d(:,i) = agent_data(j,agent_number).eta_dot_d;
    eta_dot_ref(:,i) = agent_data(j,agent_number).eta_dot_ref;
end
% 
% subplot(2,1,1), title('eta dot ');
% hold on, grid on;
% plot(time,  eta_dot(1:2,:));
% plot(time,  eta_dot_d(1:2,:));
% subplot(2,1,2), title('Yaw-rate');
% hold on, grid on;
% plot(time,  rad2deg(eta_dot(3,:)),'b');
% plot(time,  rad2deg(eta_dot_d(3,:)),'r');
% plot(time,  rad2deg(eta_dot_ref(3,:)),'g');

% %%
% figure(204),
% clf(204);
speed = zeros(1,l);
speed_d = zeros(1,l);
speed_ref = zeros(1,l);
for j=t_array
    i = j-t_array(1)+1;
    speed(1,i) = norm(agent_data(j,agent_number).eta_dot(1:2),2);   
    speed_d(1,i) = norm(agent_data(j,agent_number).eta_dot_d(1:2,1),2);
    speed_ref(1,i) = norm(agent_data(j,agent_number).eta_dot_ref(1:2,1),2);
end
% 
% % subplot(2,1,1), title('eta dot ');
% hold on, grid on;
% title('Speed and reference');
% plot(time,  speed(1,:),'b');
% plot(time,  speed_d(1,:),'y');
% plot(time,  speed_ref(1,:),'r');
% legend('u', 'u_d', 'u_{ref}')
% subplot(2,1,2), title('Yaw-rate');
% hold on, grid on;
% plot(time,  rad2deg(eta_dot(3,:)),'b');
% plot(time,  rad2deg(eta_dot_d(3,:)),'r');
% plot(time,  rad2deg(eta_dot_ref(3,:)),'g');

% %%
% figure(205),title('Vessel states');
% clf(205);
% 
% subplot(3,1,1), title('North East error');
% hold on, grid on;
% plot(time,  eta(1:2,:)-eta_d(1:2,:));
% % plot(time,  eta_d(1:2,:));
% 
% subplot(3,1,2), title('Position error');
% hold on, grid on;
% eta_err = eta(1:2,:)-eta_d(1:2,:);
% eta_err_ref = eta(1:2,:)-eta_ref(1:2,:);
% 
% pos_err = zeros(1,size(eta_err,2));
% pos_err_ref = zeros(1,size(eta_err,2));
% for i=1:size(eta_err,2)
%     pos_err(i) = norm(eta_err(:,i),2);
%     pos_err_ref(i) = norm(eta_err_ref(:,i),2);
% end
% eta_err = norm(eta_err,2);
% plot(time,  pos_err,'b');
% plot(time, pos_err_ref,'r');
% 
% 
% subplot(3,1,3), title('Velocity error');
% hold on, grid on;
% tilde_eta_dot = eta_dot(1:3,:)-eta_dot_d(1:3,:);
% plot(time, tilde_eta_dot);
% % legend('\tilde{p}');


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
%         tau(:,i) = agent_data(j,agent_number).tau;
%         tau_d(:,i) = agent_data(j,agent_number).tau_d;
%         pid_P(:,i) = R*agent_data(j,agent_number).pid_P;
%         pid_I(:,i) = R*agent_data(j,agent_number).pid_I;
%         pid_D(:,i) = R*agent_data(j,agent_number).pid_D;
%         pid_ff_vel(:,i) = agent_data(j,agent_number).pid_ff_vel;
%         pid_ff_acc(:,i) = agent_data(j,agent_number).pid_ff_acc;
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

    
for j=t_array
    i = j-t_array(1)+1;
    nu(:,i) = agent_data(j,agent_number).nu;
    nud(:,i) = agent_data(j,agent_number).nud;
    d_nud(:,i) = agent_data(j,agent_number).d_nud;
    dd_nud(:,i) = agent_data(j,agent_number).dd_nud;
    speed_ref(:,i) = agent_data(j,agent_number).speed_ref;
    course_ref(:,i) = agent_data(j,agent_number).course_ref;
    tilde_x(:,i) = agent_data(j,agent_number).tilde_x;
    
    d_nu(:,i) = agent_data(j,agent_number).d_nu;
    tilde_d_nu(:,i) = agent_data(j,agent_number).tilde_d_nu;
    z_1(:,i) = agent_data(j,agent_number).z_1;
    d_z_1(:,i) = agent_data(j,agent_number).d_z_1;
    alpha_1(:,i) = agent_data(j,agent_number).alpha_1;
    d_alpha_1(:,i) = agent_data(j,agent_number).d_alpha_1;
    dd_alpha_1(:,i) = agent_data(j,agent_number).dd_alpha_1;
    alpha2(:,i) = agent_data(j,agent_number).alpha2;
    d_alpha2(:,i) = agent_data(j,agent_number).d_alpha2;
    z_2(:,i) = agent_data(j,agent_number).z_2;
    rhod(:,i) = agent_data(j,agent_number).rhod;
    b_rho_ = agent_data(j,agent_number).b_rho;
    b_rho(:,i) = b_rho_(:);
    
    
    
end
l_2dof = size(agent_data_2DOF,1);
eta_2dof = zeros(3,l_2dof);
eta_dot_2dof = zeros(3,l_2dof);
nu_2dof = zeros(3,l_2dof);
d_nu_2dof = zeros(3,l_2dof);
for i=t_array
    j = i-t_array(1)+1;
    eta_2dof(:,j) = agent_data_2DOF(i).eta;
    eta_dot_2dof(:,j) = agent_data_2DOF(i).eta_dot;
    nu_2dof(:,j) = agent_data_2DOF(i).nu;
    d_nu_2dof(:,j) = agent_data_2DOF(i).d_nu;

end


l_pid = size(t_array,2);
eta_pid = zeros(3,l_pid);
eta_dot_pid = zeros(3,l_pid);
nu_pid = zeros(3,l_pid);
d_nu_pid = zeros(3,l_pid);
for i=t_array
    j = i-t_array(1)+1;
    eta_pid(:,j) = sim_output_PID(i).eta;
    eta_dot_pid(:,j) = sim_output_PID(i).eta_dot;
    nu_pid(:,j) = sim_output_PID(i).nu;
    d_nu_pid(:,j) = sim_output_PID(i).d_nu;

end







figure(301)
clf(301)
subplot(3,1,1)
title('$\nu$ and $\nu_d$')
hold on;
plot(time, nud)
plot(time, nu)
legend({'$u_d$','$v_d$','$r_d$','$u$','$v$','$r$'},'interpreter','latex');


subplot(3,1,2)
hold on;
title('$\dot{\nu}_d$')
plot(time, d_nud)
plot(time, d_nu);
legend({'$\dot{u}_d$','$\dot{v}_d$','$\dot{r}_d$','$\dot{u}$','$\dot{v}$','$\dot{r}$'},'interpreter','latex');


subplot(3,1,3)
hold on;
title('$\ddot{\nu}_d$')
plot(time, dd_nud)
legend({'$u$','$v$','$r$'},'interpreter','latex');


%%
figure(302)
clf(302)
hold on;
% set(gca, 'Position', get(gca, 'OuterPosition') - ...
%     get(gca, 'TightInset') * [-1 0 1 0; 0 -1 0 1; 0 0 1 0; 0 0 0 1]);

u_ref = sqrt(nud(1,:).^2 + nud(2,:).^2);
speed_2 = sqrt(nu(1,:).^2 + nu(2,:).^2);

subplot(2,1,1)
hold on;
plot(time, speed_2, 'b')
plot(time, u_ref, 'r')
xlabel('Time [$s$]', 'fontsize',14);
ylabel('Speed [$m/s$]', 'fontsize',14);
axis([0,t_end-t_start, 0, max(speed*1.1)])
legend({'$U$','$U_{ref}$'}, 'fontsize',14,'interpreter','latex', 'location','southeast');

set(gca, 'Position', get(gca, 'OuterPosition') - ...
    get(gca, 'TightInset') * [-1 0 1 0; 0 -1 0 1; 0 0 1 0; 0 0 0 1]);

subplot(2,1,2)
hold on;
eta_wrapped = eta;
for i=1:size(eta_wrapped,2)
   if(eta_wrapped(3,i) > 2)
       eta_wrapped(3,i) = eta_wrapped(3,i)-2*pi;
   end
end
course = course_ref;
for i=1:size(eta_dot,2)
    course(1,i) = atan2(eta_dot(2,i), eta_dot(1,i));
    if(course(1,i) > 2)
        course(1,i) = course(1,i) - 2*pi;
    end
end

eta_ref_filtered = eta_ref(3,:);
eta_ref_filtered(eta_ref_filtered >1.7) = eta_ref_filtered(eta_ref_filtered >1.7)-2*pi;
% 
alpha = 0.95;
for i=1:(size(eta_ref,2)-1)
   eta_ref_filtered(1,i+1) = alpha*eta_ref_filtered(1,i)+(1-alpha)*eta_ref_filtered(1,i+1);
    
end



course_ref(course_ref >1.7) = course_ref(course_ref >1.7)-2*pi;
plot(time, rad2deg(eta_wrapped(3,:)), 'b');
plot(time, rad2deg(eta_ref_filtered(1,:)), 'r')
% plot(time, rad2deg(course))
xlabel('Time [$s$]', 'fontsize',14);
ylabel(' Heading [$deg$]', 'fontsize',14);
axis([0,t_end-t_start, rad2deg(min(course_ref(1,:)))*1.1, rad2deg(max(course_ref(1,:)))*1.1])
legend({'$\psi$','$\psi_{ref}$'}, 'fontsize',14,'interpreter','latex','location','southeast');

set(gca, 'Position', get(gca, 'OuterPosition') - ...
    get(gca, 'TightInset') * [-1 0 1 0; 0 -1 0 1; 0 0 1 0; 0 0 0 1]);




%%


figure(118)
hold on;
clf(10)
set(gca, 'Position', get(gca, 'OuterPosition') - ...
get(gca, 'TightInset') * [-1 0 1 0; 0 -1 0 1; 0 0 1 0; 0 0 0 1]);

subplot(2,1,1);
hold on,% grid on;
plot(time, speed_2, 'b')
plot(time, u_ref, 'r')
xlabel('Time [$s$]', 'fontsize',14);
ylabel('Speed [$m/s$]', 'fontsize',14);
axis([0,t_end-t_start, 0, max(speed*1.1)])
legend({'$U$','$U_{ref}$'}, 'fontsize',14,'interpreter','latex', 'location','southeast');

set(gca, 'Position', get(gca, 'OuterPosition') - ...
    get(gca, 'TightInset') * [-1 0 1 0; 0 -1 0 1; 0 0 1 0; 0 0 0 1]);

subplot(2,1,2);
hold on, %grid on;
plot(time, rad2deg(eta_wrapped(3,:)), 'b');
plot(time, rad2deg(eta_ref_filtered(1,:)), 'r')
% plot(time, rad2deg(course))
xlabel('Time [$s$]', 'fontsize',14);
ylabel(' Heading [$deg$]', 'fontsize',14);
axis([0,t_end-t_start, rad2deg(min(course_ref(1,:)))*1.1, rad2deg(max(course_ref(1,:)))*1.1])
legend({'$\psi$','$\psi_{ref}$'}, 'fontsize',14,'interpreter','latex','location','southeast');

set(gca, 'Position', get(gca, 'OuterPosition') - ...
    get(gca, 'TightInset') * [-1 0 1 0; 0 -1 0 1; 0 0 1 0; 0 0 0 1]);


%%
figure(303)
clf(303)
hold on;
title('$\tilde{x}$');
plot(time, tilde_x);
legend({'$\tilde{u}$','$\tilde{v}$','$\tilde{\psi}$'},'interpreter','latex');

%%

figure(304)
clf(304)
hold on;
axis('equal')
% title('Transit Path')
% plot(eta_ref(2,:), eta_ref(1,:), 'r');
plot(eta(2,:), eta(1,:), 'b', 'linewidth' , 0.5);
wp = agent_data(1,1).wp;
plot(eta_2dof(2,:), eta_2dof(1,:), 'color', get_rgb('orange'), 'linewidth',0.5)
plot(eta_pid(2,:), eta_pid(1,:), 'color', get_rgb('forest_green'), 'linewidth',0.5)

plot(wp(2,:), wp(1,:),'r', 'linewidth' , 0.5);
plot(eta(2,:), eta(1,:), 'b', 'linewidth' , 0.5);

legend({'3DOF','2DOF','PID','reference path'}, 'NumColumns',2,'location', 'northwest', 'fontsize',12,'interpreter','latex');
% legend({'vessel path','reference path','vessel path 2DOF'}, 'location', 'north', 'fontsize',14);

X = [0.2 0.3];
Y = [0.8 0.6];
annotation('arrow',X,Y);
text(-310,-120, 'Direction of external force')
 
X = [0.7 0.6];
Y = [0.285 0.21];
annotation('arrow',X,Y);
text(-230,-220, 'Transit direction')
 
xlabel('East [$m$]', 'fontsize',12);
ylabel('North [$m$]', 'fontsize',12);
axis([-330, -170, -240, -70]);
set(gca, 'Position', get(gca, 'OuterPosition') - ...
    get(gca, 'TightInset') * [-1 0 1 0; 0 -1 0 1; 0 0 1 0; 0 0 0 1]);

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

figure(7)
clf(7)
set(gca, 'Position', get(gca, 'OuterPosition') - ...
    get(gca, 'TightInset') * [-1 0 1 0; 0 -1 0 1; 0 0 1 0; 0 0 0 1]);

subplot(3,1,1)
hold on;
plot(time, rhod(1,:))
ylabel('$F_d$ [N]', 'fontsize',12);
xlabel('Time [$s$]', 'fontsize',12);
% title('Control inputs', 'fontsize',14);
axis([0,t_end-t_start,0, max(rhod(1,:)*1.1)])

set(gca, 'Position', get(gca, 'OuterPosition') - ...
    get(gca, 'TightInset') * [-1 0 1 0; 0 -1 0 1; 0 0 1 0; 0 0 0 1]);


subplot(3,1,2)
hold on;
plot(time, rad2deg(rhod(2,:)))
ylabel('$\alpha_d$ [$deg$]', 'fontsize',12);
xlabel('Time [$s$]', 'fontsize',12);
axis([0,t_end-t_start, min(rad2deg(rhod(2,:)))*1.1, max(rad2deg(rhod(2,:))*1.1)])
set(gca, 'Position', get(gca, 'OuterPosition') - ...
    get(gca, 'TightInset') * [-1 0 1 0; 0 -1 0 1; 0 0 1 0; 0 0 0 1]);

subplot(3,1,3)
hold on;
plot(time, rad2deg(rhod(3,:)))
ylabel('$\gamma_d$ [$deg$]', 'fontsize',12);
xlabel('Time [$s$]', 'fontsize',12);
axis([0,t_end-t_start, min(rad2deg(rhod(3,:)))*1.1, max(rad2deg(rhod(3,:))*1.1)])
set(gca, 'Position', get(gca, 'OuterPosition') - ...
    get(gca, 'TightInset') * [-1 0 1 0; 0 -1 0 1; 0 0 1 0; 0 0 0 1]);
% newfigsize([10, 10])


figure(8)
clf(8)
subplot(3,1,1)
title('$\nu$ and $\nu_d$')
hold on;
plot(time, nu(1:2,:))
plot(time, nud(1:2,:))
ylabel('$[m/s]$')
xlabel('time [$s$]', 'fontsize',14);

legend({'$u$','$v$','$u_d$','$v_d$'});

subplot(3,1,2)
hold on;
title('$\dot{\nu}_d$')
plot(time, d_nu(3,:));
plot(time, d_nud(3,:))
legend({'$r$','$r_d$'});
ylabel('$[m/s^2]$')
xlabel('time [$s$]', 'fontsize',14);


subplot(3,1,3)
hold on;
title('$\ddot{\nu}_d$')
plot(time, dd_nud)
legend({'$u$','$v$','$r$'});
xlabel('time [$s$]', 'fontsize',14);

%%
set(0,'defaultTextInterpreter','latex'); %trying to set the default
figure(9)
clf(9)
hold on;
subplot(3,1,1)
% title('velocity error');
% title('$\nu$ and $\nu_d$')
hold on;
vel_err = nu(1:2,:)-nud(1:2,:);
plot(time, vel_err, 'linewidth',1.0);
legend({'$\dot{\tilde{x}}_u$','$\dot{\tilde{x}}_v$'}, 'fontsize',14, 'interpreter','latex');
ylabel('$[m/s]$', 'fontsize',14);
xlabel('time [$s$]', 'fontsize',14);
axis([0,t_end-t_start, min(vel_err(2,:))*1.1, max(vel_err(2,:)*1.1)])
set(gca, 'Position', get(gca, 'OuterPosition') - ...
get(gca, 'TightInset') * [-1 0 1 0; 0 -1 0 1; 0 0 1 0; 0 0 0 1]);

subplot(3,1,2)
hold on;
% title('yaw rate error')
yaw_rate_error = rad2deg(d_nu(3,:)-d_nud(3,:));
plot(time, yaw_rate_error, 'linewidth',1.0);
% plot(time, rad2deg(nu(3,:)), 'linewidth',1.0);
% plot(time, rad2deg(nu_2dof(3,(1:size(time,2)))),  'linewidth',1.0);
legend({'$\dot{\tilde{x}}_r$'}, 'fontsize',14, 'interpreter','latex','location', 'southeast');
ylabel('$[deg/s]$', 'fontsize',14);
xlabel('time [$s$]', 'fontsize',14);
axis([0,t_end-t_start,min(yaw_rate_error)*1.1, max(yaw_rate_error*1.1)])
set(gca, 'Position', get(gca, 'OuterPosition') - ...
get(gca, 'TightInset') * [-1 0 1 0; 0 -1 0 1; 0 0 1 0; 0 0 0 1]);

subplot(3,1,3)
hold on;
% title('Slip angle')
beta = rad2deg(atan2(nu(2,:), nu(1,:)));
beta_2dof = rad2deg(atan2(nu_2dof(2,:), nu_2dof(1,:)));
beta_pid = rad2deg(atan2(nu_pid(2,:), nu_pid(1,:)));
plot(time, beta, 'linewidth',1.0);
plot(time, beta_2dof(1:size(time,2))); 
plot(time, beta_pid(1:size(time,2))); 
legend({'$\beta_{3{DOF}}$', '$\beta_{2{DOF}}$', '$\beta_{PID}$'}, 'fontsize',11, 'interpreter','latex', 'NumColumns',3, 'location', 'NorthWest');
ylabel('$\beta$[$deg$]', 'fontsize',14);
xlabel('Time [$s$]', 'fontsize',14);
axis([0,t_end-t_start, min(beta_pid)*1.1, max(beta_pid*1.1)])
set(gca, 'Position', get(gca, 'OuterPosition') - ...
get(gca, 'TightInset') * [-1 0 1 0; 0 -1 0 1; 0 0 1 0; 0 0 0 1]);

%%


figure(18)
hold on;
clf(10)
set(gca, 'Position', get(gca, 'OuterPosition') - ...
get(gca, 'TightInset') * [-1 0 1 0; 0 -1 0 1; 0 0 1 0; 0 0 0 1]);

subplot(3,1,1);
hold on,% grid on;
vel_err = nu(1:2,:)-nud(1:2,:);
plot(time, vel_err, 'linewidth',1.0);
legend({'$\dot{\tilde{x}}_u$','$\dot{\tilde{x}}_v$'}, 'fontsize',14, 'interpreter','latex');
ylabel('$[m/s]$', 'fontsize',12);
xlabel('Time [$s$]', 'fontsize',12);
axis([0,t_end-t_start, min(vel_err(2,:))*1.1, max(vel_err(2,:)*1.1)])

set(gca, 'Position', get(gca, 'OuterPosition') - ...
    get(gca, 'TightInset') * [-1 0 1 0; 0 -1 0 1; 0 0 1 0; 0 0 0 1]);

subplot(3,1,2);
hold on, %grid on;
yaw_rate_error = rad2deg(d_nu(3,:)-d_nud(3,:));
plot(time, yaw_rate_error, 'linewidth',1.0);
% plot(time, rad2deg(nu(3,:)), 'linewidth',1.0);
% plot(time, rad2deg(nu_2dof(3,(1:size(time,2)))),  'linewidth',1.0);
legend({'$\dot{\tilde{x}}_r$'}, 'fontsize',14, 'interpreter','latex','location', 'southeast');
ylabel('$[deg/s]$', 'fontsize',12);
xlabel('Time [$s$]', 'fontsize',12);
axis([0,t_end-t_start,min(yaw_rate_error)*1.1, max(yaw_rate_error*1.1)])



set(gca, 'Position', get(gca, 'OuterPosition') - ...
    get(gca, 'TightInset') * [-1 0 1 0; 0 -1 0 1; 0 0 1 0; 0 0 0 1]);

subplot(3,1,3);
hold on,% grid on;

beta = rad2deg(atan2(nu(2,:), nu(1,:)));
beta_2dof = rad2deg(atan2(nu_2dof(2,:), nu_2dof(1,:)));
beta_pid = rad2deg(atan2(nu_pid(2,:), nu_pid(1,:)));
plot(time, beta, 'linewidth',1.0);
plot(time, beta_2dof(1:size(time,2))); 
plot(time, beta_pid(1:size(time,2))); 
legend({'$\beta_{3{DOF}}$', '$\beta_{2{DOF}}$', '$\beta_{PID}$'}, 'fontsize',11, 'interpreter','latex', 'NumColumns',3, 'location', 'NorthWest');
ylabel('$\beta$[$deg$]', 'fontsize',12);
xlabel('Time [$s$]', 'fontsize',12);
axis([0,t_end-t_start, min(beta_pid)*1.1, max(beta_pid*1.1)])

set(gca, 'Position', get(gca, 'OuterPosition') - ...
    get(gca, 'TightInset') * [-1 0 1 0; 0 -1 0 1; 0 0 1 0; 0 0 0 1]);


%%
figure(10)
hold on;
clf(10)
set(gca, 'Position', get(gca, 'OuterPosition') - ...
get(gca, 'TightInset') * [-1 0 1 0; 0 -1 0 1; 0 0 1 0; 0 0 0 1]);

subplot(3,1,1);
hold on, %grid on;
plot(time,  ts(1:2,:), 'linewidth',0.4);
plot(time,  ts_d(1:2,:),'linewidth',0.4);
legend('$\omega_f$','$\omega_a$','$\omega_{fd}$','$\omega_{ad}$', 'fontsize',14, 'interpreter','latex');
ylabel('Propeller speed [RPM]', 'fontsize',12);
xlabel('Time [$s$]', 'fontsize',12);

axis([0,t_end-t_start, 70,690])
set(gca, 'Position', get(gca, 'OuterPosition') - ...
    get(gca, 'TightInset') * [-1 0 1 0; 0 -1 0 1; 0 0 1 0; 0 0 0 1]);

subplot(3,1,2);
hold on, %grid on;
plot(time,  (rad2deg((ts_d(3,:)))), 'linewidth',0.4);
plot(time,  (rad2deg((ts(3,:)))), 'linewidth',0.4);
legend('$\alpha_{fd}$','$\alpha_{f}$', 'fontsize',14, 'interpreter','latex');
ylabel('Azimuth angle [$deg$]', 'fontsize',12);
xlabel('Time [$s$]', 'fontsize',12);
axis([0,t_end-t_start, rad2deg((min(ts_d(4,:))))*1.1, rad2deg(max(ts_d(4,:)))*1.1])


set(gca, 'Position', get(gca, 'OuterPosition') - ...
    get(gca, 'TightInset') * [-1 0 1 0; 0 -1 0 1; 0 0 1 0; 0 0 0 1]);

subplot(3,1,3);
hold on, %grid on;

plot(time,  (rad2deg((ts_d(4,:)))), 'linewidth',0.4);
plot(time,  (rad2deg((ts(4,:)))), 'linewidth',0.4);
legend('$\alpha_{ad}$','$\alpha_{a}$', 'fontsize',12, 'interpreter','latex');
ylabel('Azimuth angle [$deg$]', 'fontsize',14);
xlabel('Time [$s$]', 'fontsize',12);
axis([0,t_end-t_start, rad2deg((min(ts_d(4,:))))*1.1, rad2deg(max(ts_d(4,:)))*1.1])

set(gca, 'Position', get(gca, 'OuterPosition') - ...
    get(gca, 'TightInset') * [-1 0 1 0; 0 -1 0 1; 0 0 1 0; 0 0 0 1]);


%%
figure(11)
hold on;
clf(11)
subplot(2,1,1);
hold on, grid on;
plot(time,  z_1(1:2,:), 'linewidth',1.0);
legend('$\omega_1$','$\omega_2$','$\omega_{1d}$','$\omega_{2d}$', 'fontsize',14);
ylabel('$z_1$', 'fontsize',14);
xlabel('time [$s$]')

% axis([0,t_end-t_start, 70,690])


subplot(2,1,2);
hold on, grid on;
plot(time,  z_2, 'linewidth',1.0);
legend('$\alpha_1$','$\alpha_2$','$\alpha_{1d}$','$\alpha_{2d}$', 'fontsize',14);
ylabel('$z_2$','fontsize',14);
xlabel('time [$s$]')
% axis([0,t_end-t_start, rad2deg((min(ts_d(4,:))))*1.1, rad2deg(max(ts_d(4,:)))*1.1])


    
end