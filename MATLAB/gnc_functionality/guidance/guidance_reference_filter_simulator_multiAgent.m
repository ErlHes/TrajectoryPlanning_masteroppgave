function [agent] = guidance_reference_filter_simulator_multiAgent...
         (agent, dt,reset_filter_states)

% pos_state = agent.eta_d; 
pos_state = agent.eta_ref; 
pos_state(3) = agent.eta_d(3);

vel_state = agent.eta_dot_d;
acc_state = agent.eta_ddot_d;

if(reset_filter_states)
    pos_state = agent.eta;
    vel_state = agent.eta_dot;
    acc_state = [0,0,0]';
end
     

ref_filter_params = no_101_get_params();

% Filter Parameters
delta = eye(3);
% I = eye(3);
omega = 1;

jerk_upper_lim = ref_filter_params.jerk_upper_lim;
jerk_lower_lim = ref_filter_params.jerk_lower_lim;
a_max = 1.1;
a_max_yaw = 0.1;
A_max = [a_max,a_max,a_max_yaw]';
A_min = -A_max;
vel_upper_lim = ref_filter_params.vel_upper_lim;
vel_lower_lim = ref_filter_params.vel_lower_lim;






%% Velocity reference filter
ref_vel = agent.eta_dot_ref;
err_vel = ref_vel - vel_state;

velocity_jerk = -2*delta*omega*acc_state + omega^2 *err_vel;

% velocity_jerk = saturate_val(velocity_jerk, jerk_upper_lim, jerk_lower_lim);
velocity_jerk = min(jerk_upper_lim, max(velocity_jerk, jerk_lower_lim));
% calculate and saturate acceleration reference    
acc_state = acc_state+velocity_jerk*dt;
% acc_state = saturate_val(acc_state, A_max, A_min);
acc_state = min(A_max, max(acc_state, A_min));

% calculate and saturate velocity reference
vel_state = vel_state+acc_state*dt;
vel_state = min(vel_upper_lim, max(vel_state, vel_lower_lim));


% %% Heading reference filter
% pos_state(1) = agent.eta_ref(1);
% pos_state(2) = agent.eta_ref(2);
heading_ref = wrap_plus_minus_pi(agent.eta_ref(3));
% heading_error = heading_ref-pos_state(3,1);
heading_error = get_angle_error(heading_ref,agent.eta_d(3));

heading_jerk = -(2*delta(3,3)+1)*omega*acc_state(3) +(2*delta(3,3)+1)*omega^2*err_vel(3) + omega^3*heading_error;
heading_jerk = min(jerk_upper_lim(3), max(heading_jerk, jerk_lower_lim(3)));

% heading_jerk = saturate_val(heading_jerk, jerk_upper_lim(3), jerk_lower_lim(3));

acc_state(3,1) = acc_state(3,1)+heading_jerk*dt;
% acc_state(3,1) = saturate_val(acc_state(3,1), A_max(3), A_min(3));
acc_state(3,1) = min(A_max(3), max(acc_state(3,1), A_min(3)));


vel_state(3,1) = vel_state(3,1)+acc_state(3,1)*dt;
% vel_state(3,1) = saturate_val(vel_state(3,1), vel_upper_lim(3), vel_lower_lim(3));
vel_state(3,1) = min(vel_upper_lim(3), max(vel_state(3,1), vel_lower_lim(3)));

pos_state = pos_state + vel_state*dt;
pos_state(3,1) = wrap_plus_minus_pi(pos_state(3,1));

% pos_err = agent.eta - pos_state

%% Assign Outputs

agent.eta_d = pos_state; %[eta(1);eta(2);pos_state(3)];
agent.eta_dot_d = vel_state;
agent.eta_ddot_d = acc_state;

end

function err = get_angle_error(desired_angle, current_angle)
% Calculates shortest path angle error between desired angle and current angle
err = desired_angle - current_angle;
if(err <= -pi)
    err = err+2*pi;
elseif err > pi
    err = err-2*pi;
end
end


function ref_filter_params = no_101_get_params()
ref_filter_params = struct;
ref_filter_params.delta = eye(3);
ref_filter_params.I = eye(3);
ref_filter_params.omega= eye(3);
ref_filter_params.dt = 0.1;

ref_filter_params.jerk_upper_lim = [ 3, 3, 3]';
ref_filter_params.jerk_lower_lim = [-3,-3,-3]';

%ref_filter_params.vel_upper_lim = [2.93,2,1]';
%ref_filter_params.vel_lower_lim = [-2.93,-2,-1]';

ref_filter_params.vel_upper_lim = [ 3, 3, 0.4]';
ref_filter_params.vel_lower_lim = [-3,-3,-0.4]';
end
