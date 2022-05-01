function [vessel] = simulate_agent_iteration...
    (vessel,tracks,settings,parameters,iteration)

    if((vessel.model == 1) && (vessel.gnc == 1)) % PID controller and normal TA
        vessel = pid_controller_and_ta(vessel, settings, parameters,iteration);  
    elseif(vessel.model == 2) && (vessel.gnc == 2) % Use reference filter as "vessel model"
        vessel = kinematic_model(vessel,settings,parameters);
    elseif(vessel.model == 2) && (vessel.gnc == 3) % MPC with assist
        vessel = autonomous_agent_trajectory_planner(vessel,tracks,settings,parameters,iteration);
    end

end
%% Supporting functions

function vessel = simulate_vessel_dynamics(vessel,settings)


    % Simulate vessel dynamics based on new actuator setpoints.
      x = [
     vessel.ts;...
     vessel.ts_d;...
     vessel.nu;...
     vessel.eta;...
    ];

    [t, x] = ode45(@milliAmpere_vessel_dynamics_surge_decopuled,[0,settings.dt],x);
    vessel.eta_dot = ((x(end,12:14) - x(end-1,12:14))/(t(end)-t(end-1)))';
    x = x(end,:)';


    % 
    vessel.ts = x(1:4,1);
    vessel.ts(3) = wrap_zero_to_2pi(vessel.ts(3));
    vessel.ts(4) = wrap_zero_to_2pi(vessel.ts(4));
    vessel.nu = x(9:11,1);
    vessel.eta = x(12:14,1);
    vessel.eta(3) = wrap_zero_to_2pi(vessel.eta(3));

end

function vessel = kinematic_model(vessel,settings, parameters)

dt = settings.dt;

vessel = guidance_simulator_multiAgent(vessel,parameters.guidance);
if(vessel.end_of_path)
    vessel.nu = [0,0,0]';
    vessel.eta_dot = [0,0,0]';
    return; 
end

[vessel] = guidance_reference_filter_simulator_multiAgent...
         (vessel,dt,0);

% Set vessel states from guidance model
vessel.eta = vessel.eta + vessel.eta_dot_d*dt;
vessel.eta_dot = vessel.eta_dot_d;
     
end

function vessel = autonomous_agent_trajectory_planner(vessel,tracks,settings,parameters,iteration)

persistent iteration_counter
persistent resulting_trajectory
persistent time_since_start
persistent ocp_eta_ref
persistent ocp_eta_dot_ref
persistent ocp_eta_ddot_ref

if(isempty(time_since_start) || iteration == 1)
    time_since_start = 0;
end

if(isempty(ocp_eta_dot_ref))
    ocp_eta_dot_ref = zeros(3,100);
end

if(isempty(ocp_eta_ref))
    ocp_eta_ref = zeros(3,100);
end
if(isempty(ocp_eta_ddot_ref))
    ocp_eta_ddot_ref = zeros(3,100);
end

dt = settings.dt;

vessel = guidance_simulator_multiAgent(vessel, parameters.guidance);

if(vessel.end_of_path)
    vessel.nu = [0,0,0]';
    vessel.eta_dot = [0,0,0]';
    return; 
end
%[vessel] = guidance_reference_filter_simulator_multiAgent...
%         (vessel,dt,0,no_101_get_params);
[vessel] = guidance_reference_filter_simulator_multiAgent...
         (vessel,dt,0);

if(isempty(iteration_counter) || iteration == 1)
    iteration_counter = inf;
    resulting_trajectory = zeros(6,100);
end

mid_level_period = 1;

if(iteration_counter > mid_level_period/dt) % Time to run mid level colav

    iteration_counter = 1;
    
    [vessel, resulting_trajectory] = MPC_with_Assist(vessel, tracks, parameters, settings); % YEP WORK TO DO
 
    
%   [vessel, resulting_trajectory] = MPC_with_Assist...
%   (vessel, tracks, static_obs, ocp_parameters);

%[ocp_eta_ref, ocp_eta_dot_ref,ocp_eta_ddot_ref] = interpolate_optimal_states_from_ocp(vessel.eta, vessel.eta_dot, w_opt, mid_level_period, opc_period, dt,6);

% else
%     vessel.eta = resulting_trajectory(1:3,iteration_counter);
%     vessel.nu = resulting_trajectory(4:6,iteration_counter);
%     vessel.eta_dot = rotZ(vessel.eta(3))*vessel.nu;
end


%vessel.eta = ocp_eta_ref(:,iteration_counter);
%vessel.eta_dot = ocp_eta_dot_ref(:,iteration_counter);
%vessel.eta_ddot = ocp_eta_ddot_ref(:,iteration_counter); % error :)


% Get vessel states from current plan;
iteration_counter = iteration_counter +1;
end

function vessel = pid_controller_and_ta(vessel, settings, parameters, iteration)

persistent tilde_x;
if(isempty(tilde_x) || iteration <= 1)
    tilde_x = zeros(3,1);
end

dt = settings.dt;
parameters.guidance.delta_lookahead = 6;
vessel = guidance_simulator_multiAgent(vessel,parameters.guidance);

if(vessel.end_of_path)
    vessel.nu = [0,0,0]';
    vessel.eta_dot = [0,0,0]';
    return; 
end
[vessel] = guidance_reference_filter_simulator_multiAgent...
         (vessel,dt,0);

vessel = pidController(vessel, parameters.pid_controller);

last_thrust = vessel.ts(1:2);
last_alpha = vessel.ts(3:4);

thruster_setpoints = thrust_allocation_novel(vessel.tau_d, last_thrust, last_alpha);


% Assign states to vessel
 vessel.ts_d = [thruster_setpoints(:,1);thruster_setpoints(:,2)];
 
 vessel = simulate_vessel_dynamics(vessel,settings);

end
