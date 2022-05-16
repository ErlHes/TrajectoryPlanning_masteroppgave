function [vessel, resulting_trajectory] = MPC_with_Assist(vessel, tracks, parameters, settings)
import casadi.*

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% INITIAL CONDITIONS and persistent variables
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    persistent previous_w_opt
    persistent F
    persistent firsttime
    persistent obstacle_state
    persistent cflags
%     persistent previous_feasibility
       
    % Initialize CasADi
    
    if(isempty(firsttime))
        firsttime = 1;
        obstacle_state = false; % No obstacles on first iteration
        previous_w_opt = [];
        cflags = [];
%         previous_feasibility = 0;
    end
    

    %Initialize COLREGs flag.
    if(isempty(cflags)) % THIS CAN BE USED TO HARDCODE FLAGS IF NEEDED:
         cflags = zeros([1,size(tracks,2)]);
%          cflags = [2, 1];
    end

    %% Settings
    simple = 0; % Enable to discard all traffic pattern assistance.
    chaos = 0; % Do not use
    %%
    
    if ~isempty(tracks)
        dynamic_obs(size(tracks,2)) = struct;
    else
        dynamic_obs = []; % Failsafe in case there are no dynamic obstacles present.
    end
    
    for i = 1:size(tracks,2)
        
        if simple
            tracks(i).wp(1:2) = [tracks(i).eta(1);tracks(i).eta(2)];
            tracks(i).wp(3:4) = [tracks(i).eta(1);tracks(i).eta(2)] +...
                1000 * [cos(tracks(i).eta(3)) , sin(tracks(i).eta(3))]';
            tracks(i).wp = [tracks(i).wp(1:2)' tracks(i).wp(3:4)']; % Truncate excess waypoints.
            tracks(i).current_wp = 1;
        end
    
        [dynamic_obs(i).cflag, dynamic_obs(i).dcpa, dynamic_obs(i).tcpa] = COLREGs_assessment(vessel,tracks(i),cflags(i));
        cflags(i) = dynamic_obs(i).cflag; % Save flag in persistent variable for next iteration.
    end

    [N,h] = DynamicHorizon(vessel, dynamic_obs);
%     T = N * h;

    if(isempty(F))
        F = CasadiSetup(h,N);
    end
    
    
    %% Feasibility check
%     if N < 180
%         fixed_feas = 1;
%     else
%         feasibility = 1;
%     end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % previous feas. | Feasibility | obstacle state %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %       1        |      1      |        1       %
    %       0        |      1      |        0       %
    %       1        |      0      |        0       %
    %       0        |      0      |        0       %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if ~isempty(previous_w_opt)
        feasibility = feasibility_check(previous_w_opt);
    else
        feasibility = 1;
    end
    
%     obstacle_state = false;    
%     if previous_feasibility && feasibility
%         obstacle_state = true;
%     end
%     previous_feasibility = feasibility;

%     feasibility = 1;

    %%
    
    % Initialize position and reference trajectory.
    initial_pos = vessel.eta;
    initial_vel = vessel.nu;

    % reference LOS for OS and TS
    [reference_trajectory_los, ~] = reference_trajectory_from_dynamic_los_guidance(vessel, parameters, h, N, feasibility);
    for i = 1:size(tracks,2)
    dynamic_obs(i).traj = reference_trajectory_from_dynamic_los_guidance(tracks(i),parameters, h, N, feasibility);
    end
    
    %% Obstacles
    enable_Static_obs = obstacle_state;
    enable_dynamic_obs = obstacle_state;
    static_obs = get_global_map_data();
%     interpolated_static_obs = Interpolate_static_obs(static_obs);
%     Static_obs_constraints = Static_obstacles_check(static_obs, reference_trajectory_los); 
%     THIS CHECK IS HANDELED IN THE MAIN LOOP NOW
    
        
    %% NLP initialization.
    % Start with empty NLP.
    w={};
    w0 = []; % Initial guess.
    lbw = [];
    ubw = [];
    J = 0;
    g={};
    lbg = [];
    ubg = [];
    
    % "lift" initial conditions.
    Xk = MX.sym('X0',6);
    w = {w{:}, Xk};
    lbw = [lbw; -inf; -inf; -inf; -2.5; -2.5; -pi/4];
    ubw = [ubw; inf; inf; inf; 2.5; 2.5; pi/4];
    w0 = [w0; initial_pos(1); initial_pos(2); initial_pos(3); initial_vel(1); initial_vel(2); initial_vel(3)];


%     Uk = MX.sym('U0',3);
%     w = {w{:}, Uk};
%     lbw = [lbw; -2.5; -2.5; -pi/4];
%     ubw = [ubw; 2.5; 2.5; pi/4];
%     w0 = [w0; 0; 0; 0];
    
    g = [g, {[initial_pos; initial_vel] - Xk}];
    lbg = [lbg; 0; 0; 0; 0; 0; 0];
    ubg = [ubg; 0; 0; 0; 0; 0; 0];
    
%     g = [g, {initial_vel - Xk}];
%     lbg = [lbg; 0; 0; 0];
%     ubg = [ubg; 0; 0; 0];
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% MAIN LOOP
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
loopdata = zeros(N+1,7);
static_obs_collection = [];
NaNs = [NaN; NaN; NaN];
c_origins = [];
c_radius = [];
%loopdata = [k xref_i uref_i]
    for k = 0:N-1
        % New NLP variable for control.
        
        Tauk = MX.sym(['Tau_' num2str(k)], 3);
        w = {w{:}, Tauk};
        lbw = [lbw; -800;  -800;   -800];
        ubw = [ubw;  800;   800;    800];
        w0 = [w0; 0; 0; 0];
        
        % Integrate until the end of the interval.
        eta_dot_ref = [reference_trajectory_los(3:4,k+1);...
                  (atan2(reference_trajectory_los(4,k+2),reference_trajectory_los(3,k+2)) - ...
                   atan2(reference_trajectory_los(4,k+1),reference_trajectory_los(3,k+1))) / h];
        
        surge_ref = sqrt(eta_dot_ref(1)^2 + eta_dot_ref(2)^2);
        nu_ref = [surge_ref;0;eta_dot_ref(3)]; %Burde være vessel.speed som referanse.
%         nu_ref = [sqrt(eta_dot_ref(1)^2 + eta_dot_ref(2)^2); 0; eta_dot_ref(3)];
%         nu_ref = vessel.eta_dot_ref;
        
        eta_ref = [reference_trajectory_los(1:2,k+1); atan2(eta_dot_ref(2),eta_dot_ref(1))];
        
        xref_i = [eta_ref; nu_ref];
        
        Fk = F('x0', Xk, 'tau', Tauk, 'Xd', xref_i);
        Xk_end = Fk.xf;
        J = J + Fk.qf;
        
        % New NLP variable for state at the end of interval.
        Xk = MX.sym(['X_' num2str(k+1)], 6);
        w = [w, {Xk}];
        lbw = [lbw; -inf; -inf; -inf; -2.3; -2.3; -pi/4];
        ubw = [ubw; inf; inf; inf; 2.3; 2.3; pi/4];
        w0 = [w0; xref_i(1); xref_i(2); xref_i(3); xref_i(4); xref_i(5); xref_i(6)];
        
%         Uk = MX.sym(['U_' num2str(k+1)], 3);
%         w = {w{:}, Uk};
%         lbw = [lbw; -2.5; -2.5; -pi/4];
%         ubw = [ubw; 2.5; 2.5; pi/4];
%         w0 = [w0; 0; 0; 0];
        
        % Add constraints.
        g = [g, {Xk_end - Xk}];
        lbg = [lbg; 0; 0; 0; 0; 0; 0];
        ubg = [ubg; 0; 0; 0; 0; 0; 0];
        
        
        if ~isempty(dynamic_obs) && ~firsttime && enable_dynamic_obs
        
        for i = 1:size(dynamic_obs,2)
           
            if dynamic_obs(i).cflag == 1 % HEAD ON
                if (k > (floor(dynamic_obs(i).tcpa/h) - floor(30/h))) && (k < (floor(dynamic_obs(i).tcpa/h) + floor(30/h)))
                    %% Constraint rundt båten, origo offset til styrbord
                    %Constraint 1:
                    offsetang = atan2(dynamic_obs(i).traj(4,k+1),dynamic_obs(i).traj(3,k+1)) + pi/2;
                    offsetdir = [cos(offsetang);sin(offsetang)];
                    offsetdist = 13;
                    offsetvektor = offsetdist*offsetdir;
                    c_orig = dynamic_obs(i).traj(1:2,k+1) + offsetvektor;
                    c_rad = 20;
                    g = [g, {(Xk(1:2) - c_orig)'*(Xk(1:2) - c_orig)}];
                    lbg = [lbg; c_rad^2];
                    ubg = [ubg; inf];
                    c_origins = [c_origins, c_orig];
                    c_radius = [c_radius, c_rad];
                    
                    %Constraint 2:
                    offsetdist = 38;
                    offsetvektor = offsetdist*offsetdir;
                    c_orig = dynamic_obs(i).traj(1:2,k+1) + offsetvektor;
                    c_rad = 5;
                    g = [g, {(Xk(1:2) - c_orig)'*(Xk(1:2) - c_orig)}];
                    lbg = [lbg; c_rad^2];
                    ubg = [ubg; inf];
                    c_origins = [c_origins, c_orig];
                    c_radius = [c_radius, c_rad];
                end
            elseif dynamic_obs(i).cflag == 2 % GIVE WAY
                if (k > (floor(dynamic_obs(i).tcpa/h) - floor(20/h))) && (k < (floor(dynamic_obs(i).tcpa/h) + floor(20/h)))
                    %% Forbudt å snike seg forbi forran target ship  
                    %c_orig = place_dyn_constraint(dynamic_obs, control
                    %                           interval, TS id, angle
                    %                           offset, distance offset)
                    c_orig = place_dyn_constraint(dynamic_obs, k, i, pi/8, 10);
                    c_rad = 18;
                    g = [g, {(Xk(1:2) - c_orig)'*(Xk(1:2) - c_orig)}];
                    lbg = [lbg; c_rad^2];
                    ubg = [ubg; inf];
                    c_origins = [c_origins, c_orig];
                    c_radius = [c_radius, c_rad];
                    
                    %Constraint 2:
                    c_orig = place_dyn_constraint(dynamic_obs, k, i, pi/12, 33);
                    c_rad = 10;
                    g = [g, {(Xk(1:2) - c_orig)'*(Xk(1:2) - c_orig)}];
                    lbg = [lbg; c_rad^2];
                    ubg = [ubg; inf];
                    c_origins = [c_origins, c_orig];
                    c_radius = [c_radius, c_rad];
                end                
            elseif dynamic_obs(i).cflag == 3 % STAND ON
                if (k > (floor(dynamic_obs(i).tcpa/h) - floor(20/h))) && (k < (floor(dynamic_obs(i).tcpa/h) + floor(20/h)))
                    %% Contraint rundt TS som sikkerhetsmargin
                    offsetang = atan2(dynamic_obs(i).traj(4,k+1),dynamic_obs(i).traj(3,k+1)) + pi/6;
                    offsetdir = [cos(offsetang);sin(offsetang)];
                    offsetdist = 0; % Should ideally be based some function of Involved vessel's speeds
                    offsetvektor = offsetdist*offsetdir;
                    c_orig = dynamic_obs(i).traj(1:2,k+1) + offsetvektor;
%                     c_rad = sqrt(tracks(i).size*tracks(i).size');
                    c_rad = 8;
                    g = [g, {(Xk(1:2) - c_orig)'*(Xk(1:2) - c_orig)}];
                    lbg = [lbg; c_rad^2];
                    ubg = [ubg; inf];
                    c_origins = [c_origins, c_orig];
                    c_radius = [c_radius, c_rad];
                end
            elseif dynamic_obs(i).cflag == 4 % OVERTAKING
                if (k > (floor(dynamic_obs(i).tcpa/h) - floor(20/h))) && (k < (floor(dynamic_obs(i).tcpa/h) + floor(20/h)))
                    %% Constraint rundt TS som sikkerhetsmargin
                    offsetang = atan2(dynamic_obs(i).traj(4,k+1),dynamic_obs(i).traj(3,k+1)) + pi/6;
                    offsetdir = [cos(offsetang);sin(offsetang)];
                    offsetdist = 0; % Should ideally be based some function of Involved vessel's speeds
                    offsetvektor = offsetdist*offsetdir;
                    c_orig = dynamic_obs(i).traj(1:2,k+1) + offsetvektor;
                    c_rad = 8;
                    g = [g, {(Xk(1:2) - c_orig)'*(Xk(1:2) - c_orig)}];
                    lbg = [lbg; c_rad^2];
                    ubg = [ubg; inf];
                    c_origins = [c_origins, c_orig];
                    c_radius = [c_radius, c_rad];
                end
            elseif dynamic_obs(i).cflag == 5 % SAFE
                if dynamic_obs(i).dcpa < 20
                    if (k > (floor(dynamic_obs(i).tcpa/h) - floor(20/h))) && (k < (floor(dynamic_obs(i).tcpa/h) + floor(20/h)))
                        offsetang = atan2(dynamic_obs(i).traj(4,k+1),dynamic_obs(i).traj(3,k+1)) + pi/6;
                        offsetdir = [cos(offsetang);sin(offsetang)];
                        offsetdist = 0; % Should ideally be based some function of Involved vessel's speeds
                        offsetvektor = offsetdist*offsetdir;
                        c_orig = dynamic_obs(i).traj(1:2,k+1) + offsetvektor;
                        c_rad = 8;
                        g = [g, {(Xk(1:2) - c_orig)'*(Xk(1:2) - c_orig)}];
                        lbg = [lbg; c_rad^2];
                        ubg = [ubg; inf];
                        c_origins = [c_origins, c_orig];
                        c_radius = [c_radius, c_rad];
                    end
                end
            end
        end
        end
    
       %static obstacle constraints:
        if(enable_Static_obs) && ~firsttime && (~isempty(static_obs))
            selected_trajectory = reference_trajectory_los;
            if(~isempty(previous_w_opt))
                selected_trajectory = previous_w_opt;
            end
            static_obs_constraints = Static_obstacles_check_Iterative(static_obs, selected_trajectory, k);
            static_obs_collection = [static_obs_collection, static_obs_constraints, NaNs];
            for i = 1:size(static_obs_constraints,2)
                static_obs_y1 = static_obs_constraints(1,i);
                static_obs_x1 = static_obs_constraints(2,i);
                pi_p = static_obs_constraints(3,i);
                
                Static_obs_crosstrack_distance = abs(-(Xk(2)-static_obs_x1) * cos(pi_p) + (Xk(1) - static_obs_y1) * sin(pi_p));
                g = [g, {Static_obs_crosstrack_distance}];
                lbg = [lbg; 5];
                ubg = [ubg; inf];
            end
            
% %             OLD CODE:
%             [~, cols] = size(Static_obs_constraints);
%             for i = 1:cols
%                 g = [g, {(Xk(1:2) - Static_obs_constraints(:,i))'*(Xk(1:2) - Static_obs_constraints(:,i)) - 5^2}]; % Endre constraints
%                 lbg = [lbg; 0];
%                 ubg = [ubg; inf];
%             end
        end
        
        
        loopdata(k+1,:) = [k, xref_i'];
        
    end
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Optimal solution and updating states
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    loopdata(end,:) = [k+1, xref_i'];

    % Create an NLP solver.
    prob = struct('f', J, 'x', vertcat(w{:}), 'g', vertcat(g{:}));
    options = struct;
    options.ipopt.max_iter = 400;
    options.ipopt.print_level = 4;
%     options.ipopt.nlp_scaling_method = 'none';
    options.ipopt.dual_inf_tol = 5;
    options.ipopt.tol = 5e-3;
    options.ipopt.constr_viol_tol = 1e-1;
%     options.ipopt.hessian_approximation = 'limited-memory';

    if(firsttime)
        options.ipopt.max_iter = 200;
        firsttime = 0;
    end
    solver = nlpsol('solver', 'ipopt', prob, options);
    

%     			  "print_level": 0, 
% 				  "tol": 5e-1, 
% 				  "dual_inf_tol": 5.0, 
% 				  "constr_viol_tol": 1e-1,
% 				  "compl_inf_tol": 1e-1, 
% 				  "acceptable_tol": 1e-2, 
% 				  "acceptable_constr_viol_tol": 0.01, 
% 				  "acceptable_dual_inf_tol": 1e10,
% 				  "acceptable_compl_inf_tol": 0.01,
% 				  "acceptable_obj_change_tol": 1e20,
% 				  "diverging_iterates_tol": 1e20}
    if(~isempty(previous_w_opt) && ~chaos && feasibility)
        endindex = min(size(lbw,1),size(previous_w_opt,1));
        temp = w0;
        w0 = previous_w_opt(1:endindex);
        if endindex < size(lbw,1)
            differentialindex = size(lbw,1)-size(previous_w_opt,1);
            w0 = [w0' zeros(1,differentialindex)]';
%             temp = w0(end-differentialindex+1:end,:);
%             w0 = [w0;temp];
        end
    end
    
    % Solve the NLP.
    clock = tic;
    sol = solver('x0', w0, 'lbx', lbw, 'ubx', ubw,...
                'lbg', lbg, 'ubg', ubg);
    Solvertime = toc(clock); %Check here to see how long it took to calculate w_opt. if Solvertime exceeds for example 6 seconds we know something might have went wrong.
    w_opt = full(sol.x);
    
    previous_w_opt = w_opt;
    if Solvertime > 30
%         previous_w_opt = [];
    end
    %% Variables for plotting
    ploteverything(loopdata,w_opt, vessel, tracks, reference_trajectory_los, c_origins, c_radius, settings, static_obs_collection);
    
    obstacle_state = true;
    
    %% Update vessel states  
    vessel.eta = w_opt(10:12);
%     vessel.nu = w_opt(4:6);
    vessel.nu = w_opt(13:15);
    vessel.eta_dot = rotZ(vessel.eta(3))*vessel.nu;
    resulting_trajectory = [w_opt(10:9:end), w_opt(11:9:end), w_opt(12:9:end), w_opt(13:9:end), w_opt(14:9:end), w_opt(15:9:end)]';     %  TODO
