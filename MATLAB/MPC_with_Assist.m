function [vessel, resulting_trajectory] = MPC_with_Assist(vessel, tracks, parameters, settings)
import casadi.*

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% INITIAL CONDITIONS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    persistent previous_w_opt
    persistent cflags
    persistent firsttime
    
    N = 80;
    h = 0.5;
    T = N * h;
    
    %Initialize COLREGs flag.
    if(isempty(cflags)) % THIS CAN BE USED TO HARDCODE FLAGS IF NEEDED:
         cflags = zeros([1,size(tracks,2)]);
    end
    
    %Initialize firsttime variable.
    if(isempty(firsttime))
        firsttime = 1;
    end
    
    %Initialize position and reference trajectory.
    initial_pos = vessel.eta(1:2);
    [reference_trajectory_los, ~] = reference_trajectory_from_dynamic_los_guidance(vessel, parameters);
    
    %% Static obstacles
    static_obs = get_global_map_data();
    interpolated_static_obs = Interpolate_static_obs(static_obs);
    
    %% Dynamic obstacles
    simple = 1; % Enable to discard all traffic pattern assistance.
    
    if ~isempty(tracks)
        dynamic_obs(size(tracks,2)) = struct;
        tracks2 = tracks;
    else
        dynamic_obs = []; % Failsafe in case there are no dynamic obstacles present.
    end
    
    for i = 1:size(tracks,2)
        
        if simple
            tracks(i).wp(1:2) = [tracks(i).eta(1);tracks(i).eta(2)];
            tracks(i).wp(3:4) = [tracks2(i).eta(1);tracks2(i).eta(2)] +...
                1000 * [cos(tracks2(i).eta(3)) , sin(tracks2(i).eta(3))]';
            tracks(i).wp = [tracks(i).wp(1:2)' tracks(i).wp(3:4)']; % Truncate excess waypoints.
        end
        dynamic_obs(i).traj = reference_trajectory_from_dynamic_los_guidance(tracks(i),parameters);
        % Her må vi legge til litt mer logikk for å skjekke at det faktisk
        % er en situasjon å klassifisere. deretter trengs det logikk for å
        % skjekke når situasjonen er klarert slik at den kan bli safe.
        dynamic_obs(i).cflag = COLREGs_assessment(vessel,tracks(i),clfags(i));
        cflags(i) = dynamic_obs(i).cflag; % Save flag in persistent variable for next iteration.
    end
    
    %% CasADi setup
    
    % System matrices.
    x = SX.sym('x',3); % x = [N, E, psi]'
    u = SX.sym('u',2); % u = [Surge, psi_dot]'
    xref = SX.sym('xref',2); % xref = [Nref, Eref]'
    uref = SX.sym('uref',2); % uref = [Surge_ref, Psi_dot_ref]'
    % There is no need for a heading reference.
    
    % System dynamics.    
    R = [cos(x(3))   0;...
         sin(x(3))   0;...
            0       1];
    xdot = R*u;
    
    % Objective function.
    P = [x(1), x(2)]'; % Position in NED.
    Kp = diag([10, 10]); % Tuning parameter for positional reference deviation.
    Ku = 1; % Tuning parameter for surge reference deviation.
    Kr = 0.1; % Tuning parameter for yaw rate reference deviation.
    L = Kp * norm(P - xref)^2 + Ku  * (u(1) - uref(1))^2 + Kr * (u(2) - uref(2))^2;
    % Continous time dynamics.
    f = Function('f', {x, u, xref, uref}, {xdot, L});
    
    % Discrete time dynamics.
    M = 4; %RK4 steps per interval
    DT = T/N/M;
    X0 = MX.sym('X0',3);
    U = MX.sym('U',2);
    Xd = MX.sym('Xd',2);
    Ud = MX.sym('Ud',2);
    X = X0;
    Q = 0;
    for j=1:M
        [k1, k1_q] = f(X, U, Xd, Ud);
        [k2, k2_q] = f(X + DT/2 * k1, U, Xd, Ud);
        [k3, k3_q] = f(X + DT/2 * k2, U, Xd, Ud);
        [k4, k4_q] = f(X + DT * k3, U, Xd, Ud);
        X=X+DT/6*(k1 +2*k2 +2*k3 +k4);
        Q = Q + DT/6*(k1_q + 2*k2_q + 2*k3_q + k4_q);
    end
    F = Function('F', {X0, U, Xd, Ud}, {X, Q}, {'x0','u', 'Xd', 'Ud'}, {'xf', 'qf'});
    
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
    Xk = MX.sym('X0',3);
    w = {w{:}, Xk};
    lbw = [lbw; -inf; -inf; -inf];
    ubw = [ubw; inf; inf; inf];
    w0 = [w0; 0; 0; 0];

    %g = [g, {initial_pos - Xk(1:2)}];
    %lbg = [lbg; 0; 0];
    %ubg = [ubg; 0; 0];
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% MAIN LOOP
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    for k = 0:N-1
        % New NLP variable for control.
        Uk = MX.sym(['U_' num2str(k)], 2);
        w = {w{:}, Uk};
        lbw = [lbw; -10; -deg2rad(30)];
        ubw = [ubw; 10; deg2rad(30)];
        w0 = [w0; 0; 0];
        
        % Integrate until the end of the interval.
        xref_i = reference_trajectory_los(1:2,k+2); % Positional reference.
        uref_i = [norm(reference_trajectory_los(3:4,k+2));...
                  (atan2(reference_trajectory_los(4,k+3),reference_trajectory_los(3,k+3)) - ...
                   atan2(reference_trajectory_los(4,k+2),reference_trajectory_los(3,k+2))) / h]; % Surge and yaw rate reference.
        Fk = F('x0', Xk, 'u', Uk, 'Xd', xref_i, 'Ud', uref_i);
        Xk_end = Fk.xf;
        J = J + Fk.qf;
        
        % New NLP variable for state at the end of interval.
        Xk = MX.sym(['X_' num2str(k+1)], 3);
        w = [w, {Xk}];
        lbw = [lbw; -inf; -inf; -inf];
        ubw = [ubw; inf; inf; inf];
        w0 = [w0; 0; 0; 0];
        
        % Add constraints.
        g = [g, {Xk_end - Xk}];
        lbg = [lbg; 0; 0; 0];
        ubg = [ubg; 0; 0; 0];
        
        % Her må det komme kode for dynamiske og statiske hindringer, men
        % det blir en jobb for litt senere. Få det grunnleggende til å
        % fungere først!.
        
        
        
        if ~isempty(interpolated_static_obs)
            [~, cols] = size(interpolated_static_obs);
            for i=1:cols
                g = [g, {(Xk(1:2) - interpolated_static_obs(:,i))'*(Xk(1:2) - interpolated_static_obs(:,i)) - 16^2}];
                lbg = [lbg; 0];
                ubg = [ubg; inf];
            end
        end
        
    end
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Optimal solution and updating states
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Create an NLP solver.
    prob = struct('f', J, 'x', vertcat(w{:}), 'g', vertcat(g{:}));
    solver = nlpsol('solver', 'ipopt', prob);
    
    if(~isempty(previous_w_opt))
        w0 = previous_w_opt;
    end
    
    % Solve the NLP.
    sol = solver('x0', w0, 'lbx', lbw, 'ubx', ubw,...
                'lbg', lbg, 'ubg', ubg);
    w_opt = full(sol.x);
    
    previous_w_opt = w_opt;
    firsttime = 0;

    % Plot the solution
    figure(999);
    clf;
    north_opt = w_opt(1:5:end);
    east_opt = w_opt(2:5:end);
    hold on
    plot(east_opt, north_opt, '*');
    plot(vessel.wp(2,:),vessel.wp(1,:),'g');
    plot(reference_trajectory_los(2,:),reference_trajectory_los(1,:) , 'r-.');
    axis(settings.axis);
    title('Projected future trajectory');
    xlabel('East [m]');
    ylabel('North [m]');
    legend('W_{opt}', 'Transit path', 'reference trajectory');
    grid;
    %plot(tgrid, east_opt, '-')
    %stairs(tgrid, [u_opt; nan], '-.')
    %xlabel('t')
    %legend('x1','x2','u')

    vessel.eta(1:2) = w_opt(5:6);
    vessel.eta_dot(1:2) = w_opt(3:4);
    vessel.eta_dot(3) = 0;
    vessel.eta(3) = atan2(vessel.eta_dot(2),vessel.eta_dot(1));
    vessel.nu = rotZ(vessel.eta(3))*vessel.eta_dot;
    resulting_trajectory = zeros(6,100);    

        
        
        
        
        
        
    
    
    
    
    