function [vessel, resulting_trajectory] = MPC_with_Assist(vessel, tracks, parameters, settings)
import casadi.*

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% INITIAL CONDITIONS and persistent variables
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    persistent previous_w_opt
    persistent previous_w_opt_F
    persistent F
    persistent firsttime
    persistent obstacle_state
    persistent cflags
    persistent previous_eta_ref
%     persistent pimultiplier
%     persistent previous_feasibility
       
    % Initialize CasADi
    
    if(isempty(firsttime))
        firsttime = 1;
        obstacle_state = false; % No obstacles on first iteration
        previous_w_opt = [];
        cflags = [];
        previous_w_opt_F = [];
        previous_eta_ref = [];
%         pimultiplier = 0;
%         previous_feasibility = 0;
    end
    

    %Initialize COLREGs flag.
    if(isempty(cflags)) % THIS CAN BE USED TO HARDCODE FLAGS IF NEEDED:
         cflags = zeros([1,size(tracks,2)]);
%          cflags = [2, 1];
    end

    %% Settings
    simple = settings.simple; % Enable to discard all traffic pattern assistance.
%     chaos = 0; % Do not use
%     pimultiplier = 0;
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
                1852 * [cos(tracks(i).eta(3)) , sin(tracks(i).eta(3))]';
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
    
%   OLD AND OUTDATED STUFF
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % previous feas. | Feasibility | obstacle state %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %       1        |      1      |        1       %
    %       0        |      1      |        0       %
    %       1        |      0      |        0       %
    %       0        |      0      |        0       %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if ~isempty(previous_w_opt_F)
        feasibility = feasibility_check(previous_w_opt_F);
    else
        feasibility = 1;
    end

%   OLD AND OUTDATED STUFF    
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
    % I have no idea what this is supposed to accomplish, I don't think the
    % second If sentence will ever occur since ssa() in rads can't return an angle
    % greater than pi.

%     if wrapTo2Pi(initial_pos(3)) < pi/6
% %         initial_pos(3) = wrapTo2Pi(initial_pos(3)); % THIS NEEDS MORE WORK
%         if ~isempty(previous_w_opt) && ssa(initial_pos(3)-previous_w_opt(3)) > pi
%             if initial_pos(3) > previous_w_opt(3)
%                 initial_pos(3) = wrapTo2Pi(initial_pos(3));
%             end
%         elseif ~isempty(previous_w_opt)
%             initial_pos(3) = wrapTo2Pi(initial_pos(3));
%         end
%     end


    % reference LOS for OS and TS
    [reference_trajectory_los, ~] = reference_trajectory_from_dynamic_los_guidance(vessel, parameters, h, N, feasibility);
    for i = 1:size(tracks,2)
    dynamic_obs(i).traj = reference_trajectory_from_dynamic_los_guidance(tracks(i),parameters, 0.5, N, feasibility);
    end
    
    %% Obstacles
    enable_Static_obs = obstacle_state; %Obstacle state is purely for debugging.
    enable_dynamic_obs = obstacle_state;
    static_obs = get_global_map_data();
%     interpolated_static_obs = Interpolate_static_obs(static_obs);
%     Static_obs_constraints = Static_obstacles_check(static_obs, reference_trajectory_los); 
%     THIS CHECK IS HANDELED IN THE MAIN LOOP NOW
    
        
    %% NLP initialization.
    % Start with empty NLP.
    w={};
    w0 = zeros(9*N+6,1); % Initial guess.
    lbw = zeros(9*N+6,1);
    ubw = zeros(9*N+6,1);
    J = 0;
    g={};
    lbg = zeros(50*N+6,1);
    ubg = zeros(50*N+6,1);
    
    % "lift" initial conditions.
    Xk = MX.sym('X0',6);
    w = [w {Xk}];
    lbw(1:6) = [-inf; -inf; -inf; -2.5; -2.5; -pi/4];
    ubw(1:6) = [ inf;  inf;  inf;  2.5;  2.5;  pi/4];
    w0(1:6) = [initial_pos(1); initial_pos(2); initial_pos(3); initial_vel(1); initial_vel(2); initial_vel(3)];


%     Uk = MX.sym('U0',3);
%     w = {w{:}, Uk};
%     lbw = [lbw; -2.5; -2.5; -pi/4];
%     ubw = [ubw; 2.5; 2.5; pi/4];
%     w0 = [w0; 0; 0; 0];
    
    g = [g, {[initial_pos; initial_vel] - Xk}];
    lbg(1:6) = [0; 0; 0; 0; 0; 0]';
    ubg(1:6) = [0; 0; 0; 0; 0; 0]';
    
%     g = [g, {initial_vel - Xk}];
%     lbg = [lbg; 0; 0; 0];
%     ubg = [ubg; 0; 0; 0];
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% MAIN LOOP
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
loopdata = zeros(N+1,7);
static_obs_collection = [];
NaNs = [NaN; NaN; NaN];
c_origins = zeros(2,50*N+6);
c_radius = zeros(50*N+6,1);
c_counter = 1;
g_counter = 7;
%loopdata = [k xref_i uref_i]
    for k = 0:N-1
        % New NLP variable for control.
        
        Tauk = MX.sym(['Tau_' num2str(k)], 3);
        w = [w {Tauk}]; %#ok<AGROW> 
        lbw(7+k*9:9+k*9)= [-800;  -800;   -800];
        ubw(7+k*9:9+k*9) = [800;   800;    800];
        w0(7+k*9:9+k*9) = [0; 0; 0];
        
        % Integrate until the end of the interval.
        eta_dot_ref = [reference_trajectory_los(3:4,k+1);...
                  (atan2(reference_trajectory_los(4,k+2),reference_trajectory_los(3,k+2)) - ...
                   atan2(reference_trajectory_los(4,k+1),reference_trajectory_los(3,k+1))) / h];
        
        surge_ref = sqrt(eta_dot_ref(1)^2 + eta_dot_ref(2)^2);
        nu_ref = [surge_ref;0;eta_dot_ref(3)];
%         nu_ref = [sqrt(eta_dot_ref(1)^2 + eta_dot_ref(2)^2); 0; eta_dot_ref(3)];
%         nu_ref = vessel.eta_dot_ref;
        
        eta_ref = [reference_trajectory_los(1:2,k+1); atan2(eta_dot_ref(2),eta_dot_ref(1))]; 
%         eta_ref = [reference_trajectory_los(1:2,k+1); wrapTo2Pi(atan2(eta_dot_ref(2),eta_dot_ref(1)))];

        % We want the reference to start close to initial position.
        if k == 0
            unwrap_diff = abs(eta_ref(3) - initial_pos(3));
            wrap_diff = abs(wrapTo2Pi(eta_ref(3)) - initial_pos(3));

            if unwrap_diff > wrap_diff % check if distance between ref and init_pos is greater when unwrapped
                eta_ref(3) = wrapTo2Pi(eta_ref(3));
            end
            previous_eta_ref = eta_ref;
        end

        
        %% Heading control
        if k > 0
            eta_ref(3) = previous_eta_ref(3) + ssa(eta_ref(3) - previous_eta_ref(3));
            previous_eta_ref = eta_ref; 

%             unwrap_diff = abs(eta_ref(3) - previous_eta_ref(3));
%             wrap_diff = abs(wrapTo2Pi(eta_ref(3)) - previous_eta_ref(3));
% 
%             if unwrap_diff > wrap_diff % check if distance between ref and init_pos is greater when unwrapped
%                 eta_ref(3) = wrapTo2Pi(eta_ref(3));
%             end
%             previous_eta_ref = eta_ref;
        end            
%         if k > 0
%             if wrapTo2Pi(previous_eta_ref(3)) > 21*pi/12 && wrapTo2Pi(eta_ref(3)) < 3*pi/12 % Positive wrap
%                 pimultiplier = pimultiplier + 2*pi;
%             end
%             if wrapTo2Pi(previous_eta_ref(3)) < 3*pi/12 && wrapTo2Pi(eta_ref(3)) > 21*pi/12 % Negative wrap
%                 pimultiplier = pimultiplier - 2*pi;
%             end
%         end
%         eta_ref(3) = eta_ref(3) + pimultiplier;
%         previous_eta_ref = eta_ref;
        %%
%         eta_ref = [reference_trajectory_los(1:2,k+1); 0];
        
        xref_i = [eta_ref; nu_ref];
        
        Fk = F('x0', Xk, 'tau', Tauk, 'Xd', xref_i);
        Xk_end = Fk.xf;
        J = J + Fk.qf;
        
        % New NLP variable for state at the end of interval.
        Xk = MX.sym(['X_' num2str(k+1)], 6);
        w = [w {Xk}]; %#ok<AGROW> 
        lbw(10+k*9:15+k*9) = [-inf; -inf; -inf; -2.3; -2.3; -pi/4];
        ubw(10+k*9:15+k*9) = [inf; inf; inf; 2.3; 2.3; pi/4];
        w0(10+k*9:15+k*9) = [xref_i(1); xref_i(2); xref_i(3); xref_i(4); xref_i(5); xref_i(6)];
        
%         Uk = MX.sym(['U_' num2str(k+1)], 3);
%         w = {w{:}, Uk};
%         lbw = [lbw; -2.5; -2.5; -pi/4];
%         ubw = [ubw; 2.5; 2.5; pi/4];
%         w0 = [w0; 0; 0; 0];
        
        % Add constraints.
        g = [g {Xk_end - Xk}]; %#ok<AGROW> 
        lbg(g_counter:g_counter+5) = [0; 0; 0; 0; 0; 0];
        ubg(g_counter:g_counter+5)= [0; 0; 0; 0; 0; 0];
        g_counter = g_counter + 6;
        
        
        if ~isempty(dynamic_obs) && ~firsttime && enable_dynamic_obs
        
        for i = 1:size(dynamic_obs,2)
           
            if dynamic_obs(i).cflag == 1 % HEAD ON
                if (k > (floor(dynamic_obs(i).tcpa/h) - floor(30/h))) && (k < (floor(dynamic_obs(i).tcpa/h) + floor(30/h)))
                    %% Constraint rundt båten, origo offset til styrbord
                    %Constraint 1:
                    c_orig = place_dyn_constraint(dynamic_obs, k, i, pi/2, 13);
                    c_rad = 22;
                    g = [g {(Xk(1:2) - c_orig)'*(Xk(1:2) - c_orig)}]; %#ok<AGROW> 
                    lbg(g_counter) = c_rad^2;
                    ubg(g_counter) = inf;
                    g_counter = g_counter + 1;
                    c_origins(:,c_counter) = c_orig;
                    c_radius(c_counter) = c_rad;
                    c_counter = c_counter + 1;
                    
                    %Constraint 2:
                    c_orig = place_dyn_constraint(dynamic_obs, k, i, pi/2, 38);
                    c_rad = 5;
                    g = [g {(Xk(1:2) - c_orig)'*(Xk(1:2) - c_orig)}]; %#ok<AGROW> 
                    lbg(g_counter) = c_rad^2;
                    ubg(g_counter) = inf;
                    g_counter = g_counter + 1;
                    c_origins(:,c_counter) = c_orig;
                    c_radius(c_counter) = c_rad;
                    c_counter = c_counter + 1;
                end
            elseif dynamic_obs(i).cflag == 2 % GIVE WAY
                if (k > (floor(dynamic_obs(i).tcpa/h) - floor(20/h))) && (k < (floor(dynamic_obs(i).tcpa/h) + floor(20/h)))
                    %% Forbudt å snike seg forbi forran target ship  
                    %c_orig = place_dyn_constraint(dynamic_obs, control
                    %                           interval, TS id, angle
                    %                           offset, distance offset)
                    c_orig = place_dyn_constraint(dynamic_obs, k, i, pi/8, 10);
                    c_rad = 18;
                    g = [g {(Xk(1:2) - c_orig)'*(Xk(1:2) - c_orig)}]; %#ok<AGROW> 
                    lbg(g_counter) = c_rad^2;
                    ubg(g_counter) = inf;
                    g_counter = g_counter + 1;
                    c_origins(:,c_counter) = c_orig;
                    c_radius(c_counter) = c_rad;
                    c_counter = c_counter + 1;
                    
                    %Constraint 2:
                    c_orig = place_dyn_constraint(dynamic_obs, k, i, pi/12, 33);
                    c_rad = 10;
                    g = [g {(Xk(1:2) - c_orig)'*(Xk(1:2) - c_orig)}]; %#ok<AGROW> 
                    lbg(g_counter) = c_rad^2;
                    ubg(g_counter) = inf;
                    g_counter = g_counter + 1;
                    c_origins(:,c_counter) = c_orig;
                    c_radius(c_counter) = c_rad;
                    c_counter = c_counter + 1;
                end                
            elseif dynamic_obs(i).cflag == 3 % STAND ON
                if (k > (floor(dynamic_obs(i).tcpa/h) - floor(20/h))) && (k < (floor(dynamic_obs(i).tcpa/h) + floor(20/h)))
                    %% Contraint rundt TS som sikkerhetsmargin
                    c_orig = place_dyn_constraint(dynamic_obs, k, i, pi, 0); 
                    c_rad = 7;
                    g = [g {(Xk(1:2) - c_orig)'*(Xk(1:2) - c_orig)}]; %#ok<AGROW> 
                    lbg(g_counter) = c_rad^2;
                    ubg(g_counter) = inf;
                    g_counter = g_counter + 1;
                    c_origins(:,c_counter) = c_orig;
                    c_radius(c_counter) = c_rad;
                    c_counter = c_counter + 1;
                end
            elseif dynamic_obs(i).cflag == 4 % OVERTAKING
                if (k > (floor(dynamic_obs(i).tcpa/h) - floor(20/h))) && (k < (floor(dynamic_obs(i).tcpa/h) + floor(20/h)))
                    %% Constraint rundt TS som sikkerhetsmargin
                    c_orig = place_dyn_constraint(dynamic_obs, k, i, 0, 0);
                    c_rad = 10;
                    g = [g {(Xk(1:2) - c_orig)'*(Xk(1:2) - c_orig)}]; %#ok<AGROW> 
                    lbg(g_counter) = c_rad^2;
                    ubg(g_counter) = inf;
                    g_counter = g_counter + 1;
                    c_origins(:,c_counter) = c_orig;
                    c_radius(c_counter) = c_rad;
                    c_counter = c_counter + 1;
                end
            elseif dynamic_obs(i).cflag == 5 % SAFE
                if dynamic_obs(i).dcpa < 20
                    if (k > (floor(dynamic_obs(i).tcpa/h) - floor(20/h))) && (k < (floor(dynamic_obs(i).tcpa/h) + floor(20/h)))
                        c_orig = place_dyn_constraint(dynamic_obs, k, i, 0, 0);
                        c_rad = 8;
                        g = [g {(Xk(1:2) - c_orig)'*(Xk(1:2) - c_orig)}]; %#ok<AGROW> 
                        lbg(g_counter) = c_rad^2;
                        ubg(g_counter) = inf;
                        g_counter = g_counter + 1;
                        c_origins(:,c_counter) = c_orig;
                        c_radius(c_counter) = c_rad;
                        c_counter = c_counter + 1;
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
            static_obs_collection = [static_obs_collection, static_obs_constraints, NaNs]; %#ok<AGROW> 
            for i = 1:size(static_obs_constraints,2)
                static_obs_y1 = static_obs_constraints(1,i);
                static_obs_x1 = static_obs_constraints(2,i);
                pi_p = static_obs_constraints(3,i);
                
                Static_obs_crosstrack_distance = abs(-(Xk(2)-static_obs_x1) * cos(pi_p) + (Xk(1) - static_obs_y1) * sin(pi_p));
                 g = [g {Static_obs_crosstrack_distance}]; %#ok<AGROW> 
                    lbg(g_counter) = 5;
                    ubg(g_counter) = inf;
                    g_counter = g_counter + 1;
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

    % Truncate lbg, ubg:
    lbg = lbg(1:g_counter-1);
    ubg = ubg(1:g_counter-1);

    % Create an NLP solver.
    prob = struct('f', J, 'x', vertcat(w{:}), 'g', vertcat(g{:}));
%     options = struct;
    options.ipopt.max_iter = 400;
    options.ipopt.print_level = 0;
%     options.ipopt.nlp_scaling_method = 'none';
%     options.ipopt.dual_inf_tol = 5;
%     options.ipopt.tol = 5e-3;
%     options.ipopt.constr_viol_tol = 1e-1;
% %     options.ipopt.hessian_approximation = 'limited-memory';
%     options.ipopt.compl_inf_tol = 1e-1;
%     options.ipopt.acceptable_tol = 1e-2;
%     options.ipopt.constr_viol_tol = 0.01;
%     options.ipopt.acceptable_dual_inf_tol = 1e10;
%     options.ipopt.acceptable_compl_inf_tol = 0.01;
%     options.ipopt.acceptable_obj_change_tol = 1e20;
%     options.ipopt.diverging_iterates_tol = 1e20;

    if(firsttime)
        options.ipopt.max_iter = 200;
        options.ipopt.print_level =  5;
        firsttime = 0;
    end
    solver = nlpsol('solver', 'ipopt', prob, options);

    % Replace w0 with previous_w_opt:
%     if(~isempty(previous_w_opt)) && feasibility == previous_feasibility && feasibility
    if(~isempty(previous_w_opt)) && feasibility
        endindex = min(size(lbw,1),size(previous_w_opt,1));
        if endindex < size(lbw,1)
            % Add back w0 from NLP construction to fill the gap:
            previous_w_opt(end+1:size(lbw,1)) = w0(size(previous_w_opt,1)+1:end);
            endindex = size(lbw,1);
        end
        w0 = previous_w_opt(1:endindex);
    end
    
    % Solve the NLP.
    clock = tic;
    sol = solver('x0', w0, 'lbx', lbw, 'ubx', ubw,...
                'lbg', lbg, 'ubg', ubg);
    Solvertime = toc(clock); %Check here to see how long it took to calculate w_opt. if Solvertime exceeds for example 6 seconds we know something might have went wrong.
    w_opt = full(sol.x);
%     w_opt(3:9:end) = wrapTo2Pi(w_opt(3:9:end));
    
    previous_w_opt = w_opt;
    previous_w_opt_F = w_opt;
%     previous_feasibility = feasibility;
    if Solvertime > 30
        previous_w_opt = [];
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
