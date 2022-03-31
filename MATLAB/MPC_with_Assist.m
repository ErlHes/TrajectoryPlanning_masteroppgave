function [vessel, resulting_trajectory] = MPC_with_Assist(vessel, tracks, parameters, settings)
import casadi.*

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% INITIAL CONDITIONS and persistent variables
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    persistent previous_w_opt
    persistent F
    persistent firsttime
       
    % Initialize CasADi
    
    if(isempty(firsttime))
        firsttime = 1;
    end
    
        persistent cflags
    %Initialize COLREGs flag.
    if(isempty(cflags)) % THIS CAN BE USED TO HARDCODE FLAGS IF NEEDED:
         cflags = zeros([1,size(tracks,2)]);
%          cflags = [2, 1];
    end

    simple = 1; % Enable to discard all traffic pattern assistance.
    
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
    T = N * h;

    if(isempty(F))
        F = CasadiSetup(h,T,N);
    end


    % Initialize position and reference trajectory.
    initial_pos = vessel.eta;
    initial_vel = vessel.nu;

    % reference LOS for OS and TS
    [reference_trajectory_los, ~] = reference_trajectory_from_dynamic_los_guidance(vessel, parameters, h, N);
    for i = 1:size(tracks,2)
    dynamic_obs(i).traj = reference_trajectory_from_dynamic_los_guidance(tracks(i),parameters, h, N);
    end
    
    %% Static obstacles
    static_obs = get_global_map_data();
    interpolated_static_obs = Interpolate_static_obs(static_obs);
    
        
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
    w0 = [w0; initial_pos(1); initial_pos(2); initial_pos(3); 2; 0; 0];


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
        
        nu_ref = [2;0;eta_dot_ref(3)]; %Burde være vessel.speed som referanse.
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
        w0 = [w0; xref_i(1); xref_i(2); xref_i(3); 2; 0; 0];
        
%         Uk = MX.sym(['U_' num2str(k+1)], 3);
%         w = {w{:}, Uk};
%         lbw = [lbw; -2.5; -2.5; -pi/4];
%         ubw = [ubw; 2.5; 2.5; pi/4];
%         w0 = [w0; 0; 0; 0];
        
        % Add constraints.
        g = [g, {Xk_end - Xk}];
        lbg = [lbg; 0; 0; 0; 0; 0; 0];
        ubg = [ubg; 0; 0; 0; 0; 0; 0];
        
        % Her må det komme kode for dynamiske og statiske hindringer, men
        % det blir en jobb for litt senere. Få det grunnleggende til å
        % fungere først!.
        if ~isempty(dynamic_obs) %Fy faen så styggt dette ble as, gjør om til en funksjon please
        
        for i = 1:size(dynamic_obs,2)
           
            if dynamic_obs(i).cflag == 1 % HEAD ON
                %% Constraint rundt båten, origo offset til styrbord
                offsetang = atan2(dynamic_obs(i).traj(4,k+1),dynamic_obs(i).traj(3,k+1)) + pi/2;
                offsetdir = [cos(offsetang);sin(offsetang)];
                offsetdist = 3;
                offsetvektor = offsetdist*offsetdir;
                c_orig = dynamic_obs(i).traj(1:2,k+1) + offsetvektor;
                c_rad = 13;
                g = [g, {(Xk(1:2) - c_orig)'*(Xk(1:2) - c_orig)}];
                lbg = [lbg; c_rad^2];
                ubg = [ubg; inf];
                c_origins = [c_origins, c_orig];
                c_radius = [c_radius, c_rad];
                %% Forbudt å være forran target ship
%                 offsetang = atan2(dynamic_obs(i).traj(4,k+1),dynamic_obs(i).traj(3,k+1));
%                 offsetdir = [cos(offsetang);sin(offsetang)];
%                 offsetdist = 5; % Should ideally be based some function of Involved vessel's speeds
%                 offsetvektor = offsetdist*offsetdir;
%                 c_orig = dynamic_obs(i).traj(1:2,k+1) + offsetvektor;
%                 c_rad = 10;
%                 g = [g, {(Xk(1:2) - c_orig)'*(Xk(1:2) - c_orig)}];
%                 lbg = [lbg; c_rad^2];
%                 ubg = [ubg; inf];
%                 c_origins = [c_origins, c_orig];
%                 c_radius = [c_radius, c_rad];
                %% Vanskeligere å komme seg til target's styrbord
%                 offsetang = atan2(dynamic_obs(i).traj(4,k+1),dynamic_obs(i).traj(3,k+1)) + pi/2;
%                 offsetdir = [cos(offsetang);sin(offsetang)];
%                 offsetdist = 5; % Should ideally be based some function of Involved vessel's speeds
%                 offsetvektor = offsetdist*offsetdir;
%                 c_orig = dynamic_obs(i).traj(1:2,k+1) + offsetvektor;
%                 c_rad = 10;
%                 g = [g, {(Xk(1:2) - c_orig)'*(Xk(1:2) - c_orig)}];
%                 lbg = [lbg; c_rad^2];
%                 ubg = [ubg; inf];
%                 c_origins = [c_origins, c_orig];
%                 c_radius = [c_radius, c_rad];

                %% Forbudt å være rett bak target ship, for komfort's skyld.
%                 offsetang = atan2(dynamic_obs(i).traj(4,k+1),dynamic_obs(i).traj(3,k+1)) + pi;
%                 offsetdir = [cos(offsetang);sin(offsetang)];
%                 offsetdist = 5; % Should ideally be based some function of Involved vessel's speeds
%                 offsetvektor = offsetdist*offsetdir;
%                 c_orig = dynamic_obs(i).traj(1:2,k+1) + offsetvektor;
%                 c_rad = 10;
%                 g = [g, {(Xk(1:2) - c_orig)'*(Xk(1:2) - c_orig)}];
%                 lbg = [lbg; c_rad^2];
%                 ubg = [ubg; inf];
%                 c_origins = [c_origins, c_orig];
%                 c_radius = [c_radius, c_rad];

            elseif dynamic_obs(i).cflag == 2 % GIVE WAY
                %% Forbudt å snike seg forbi forran target ship          
                offsetang = atan2(dynamic_obs(i).traj(4,k+1),dynamic_obs(i).traj(3,k+1)) + pi/6;
                offsetdir = [cos(offsetang);sin(offsetang)];
                offsetdist = 15; % Should ideally be based some function of Involved vessel's speeds
                offsetvektor = offsetdist*offsetdir;
                c_orig = dynamic_obs(i).traj(1:2,k+1) + offsetvektor;
                c_rad = 18;
                g = [g, {(Xk(1:2) - c_orig)'*(Xk(1:2) - c_orig)}];
                lbg = [lbg; c_rad^2];
                ubg = [ubg; inf];
                c_origins = [c_origins, c_orig];
                c_radius = [c_radius, c_rad];
                
                %% Liten constraint bak TS for å hindre å komme for nære når vi svinger tilbake mot vår opprinelige kurs
%                 offsetang = atan2(dynamic_obs(i).traj(4,k+1),dynamic_obs(i).traj(3,k+1)) + pi;
%                 offsetdir = [cos(offsetang);sin(offsetang)];
%                 offsetdist = 3; % Should ideally be based some function of Involved vessel's speeds
%                 offsetvektor = offsetdist*offsetdir;
%                 c_orig = dynamic_obs(i).traj(1:2,k+1) + offsetvektor;
%                 c_rad = 4;
%                 g = [g, {(Xk(1:2) - c_orig)'*(Xk(1:2) - c_orig)}];
%                 lbg = [lbg; c_rad^2];
%                 ubg = [ubg; inf];
%                 c_origins = [c_origins, c_orig];
%                 c_radius = [c_radius, c_rad];
                                
                
            elseif dynamic_obs(i).cflag == 3 % STAND ON
                %% Forbudt å endre heading?
            elseif dynamic_obs(i).cflag == 4 % OVERTAKING
                %% Forbudt å kjøre forbi target's babord
                
                %% Ikke vær for nære bakenden på TS
                
                %% Ikke sving inn forran TS for tidlig
                
            elseif dynamic_obs(i).cflag == 5 % SAFE
                %% No additional constraints
            end
        end
        end
    
    %    static obstacle constraints:
%         if ~isempty(interpolated_static_obs)
%             [~, cols] = size(interpolated_static_obs);
%             for i = 1:cols
%                 g = [g, {(Xk(1:2) - interpolated_static_obs(:,i))'*(Xk(1:2) - interpolated_static_obs(:,i)) - 16^2}];
%                 lbg = [lbg; 0];
%                 ubg = [ubg; inf];
%             end
%         end
        
        
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
    if(~isempty(previous_w_opt))
        w0 = previous_w_opt(1:size(lbw,1)); % somehow: Index exceeds the number of array elements. Index must not exceed 1095. TODO fix this.
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
    ploteverything(loopdata,w_opt, vessel, tracks, reference_trajectory_los, c_origins, c_radius, settings);
    
    %% Update vessel states  
    vessel.eta = w_opt(10:12);
    vessel.nu = w_opt(4:6);
    vessel.eta_dot = rotZ(vessel.eta(3))*vessel.nu;
    resulting_trajectory = zeros(6,100);     %  TODO
