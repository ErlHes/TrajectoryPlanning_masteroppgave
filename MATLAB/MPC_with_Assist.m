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
    u = SX.sym('u',3); % u = [u, v, r]'
    tau = SX.sym('tau',3); % tau = [Fx, Fy, Fn];
    xref = SX.sym('xref',3); % xref = [Nref, Eref, Psi_ref]'
    uref = SX.sym('uref',3); % uref = [Surge_ref, sway_ref, r_ref]'
    
    % Model Parameters.
    Xu = -68.676;   % Kg/s
    Xuu = -50.08;   % Kg/m
    Xuuu = -14.93;  % Kgs/(m^2)
    Xv = -25.20;    % Kg/s
    Xr = -145.3;    % Kgm/s
    Yu = 90.15;     % Kg/s
    Yv = -8.69;     % Kg/s
    Yvv = -189.08;  % Kg/m
    Yvvv = -0.00613;% Kgs/(s^2) ? Kgs/(m^2)?
    Yrv = -3086.95; % Kg
    Yr = -24.09;    % Kgm/s
    Yvr = -338.32;  % Kg
    Yrr = 1372.06;  % Kg(m^2)
    Nu = -38.00;    % Kgm/s
    Nv = -97.26;    % Kgm/s
    Nvv = -18.85;   % Kg
    Nrv = 5552.23;  % Kgm
    Nr = -230.19;   % Kg(m^2)/s
    Nrr = -0.0063;  % Kg(m^2)
    Nrrr = -0.00067;% Kgms
    Nvr = -5888.89; % Kgm
    
    m11 = 2131.80;  % Kg
    m12 = 1.00;     % Kg
    m13 = 141.02;   % Kgm
    m21 = -15.87;   % Kg
    m22 = 2231.89;  % Kg
    m23 = -1244.35; % Kgm
    m31 = -423.76;  % Kgm
    m32 = -397.64;  % Kgm
    m33 = 4351.56;  % Kg(m^2)
    
    c13 = -m22*u(2); % should be -m22 * sway.
    c23 = m11*u(1);
    c31 = -c13;
    c32 = -c23*u(2); % should be -c23 * sway.
    
    d11 = -Xu - Xuu * abs(u(1)) - Xuuu*u(1)^2;
    d22 = -Yv - Yvv*abs(u(2)) - Yvvv*u(2)^2;
    d23 = d22;
    d32 = -Nvv*abs(u(2)) - Nrv *abs(u(3));
    d33 = -Nr - Nrr*abs(u(3)) - Nrrr*u(3)^2;
    
    
    % System dynamics.
    R = [cos(x(3))    -sin(x(3))    0;...
         sin(x(3))    cos(x(3))     0;...
            0           0       1];    
    M = [m11   m12     m13;...
         m21   m22     m23;...
         m31   m32     m33];
    C = [0    0   c13;...
         0    0   c23;...
         c31 c32    0];
    D = [d11     0        0;...
         0      d22     d23;...
         0      d32     d33];
    
%     tau = [cos(theta_front)         cos(theta_rear);...
%            sin(theta_front)         sin(theta_rear);...
%            -l_TA*sin(theta_front)   -l_TA*sin(theta_rear)] * [F_front;F_rear];
    
    nudot = M\(tau -(C+D)*u); 
    %Mangler å oppdatere Nu, altså u, for å fullføre overgangen: 
    %nudot (udot) -> Nu -> etadot(xdot)
    nu = u + h*nudot;
    xdot = R*nu;  % eta_dot
    % Objective function.
    %P = [x(1), x(2)]'; % Position in NED.
    Kp = diag([10, 10, 1]); % Tuning parameter for positional reference deviation.
    Ku = 1; % Tuning parameter for surge reference deviation.
    Kr = 0.1; % Tuning parameter for yaw rate reference deviation.
    Kt = 2;
    %L = Kp * norm(P - xref)^2 + Ku  * (u(1) - uref(1))^2 + Kr * (u(2) - uref(2))^2;
    %L = (P - xref)'* Kp * (P - xref) + Ku * (u_0'*u_0 - uref(1)'*uref(1))^2;
    %L = (P - xref)'* Kp * (P - xref) + Ku * (u(1) - uref(1))^2 + Kr * (u(2) - uref(2))^2;
    L = (x - xref)'* Kp * (x - xref) + Kt * abs(tau'*tau) + Ku * (u'*u - uref'*uref)^2;
    
    % Continous time dynamics.
    f = Function('f', {x, u, tau, xref, uref}, {xdot, L});
    
    % Discrete time dynamics.
    M = 4; %RK4 steps per interval
    DT = T/N/M;
    f = Function('f', {x, u, tau, xref, uref}, {xdot, L});
    X0 = MX.sym('X0',3);
    U = MX.sym('U',3);
    Tau = MX.sym('Tau',3);
    Xd = MX.sym('Xd',3);
    Ud = MX.sym('Ud',3);
    X = X0;
    Q = 0;
    for j=1:M
        [k1, k1_q] = f(X, U, Tau, Xd, Ud);
        [k2, k2_q] = f(X + DT/2 * k1, U, Tau, Xd, Ud);
        [k3, k3_q] = f(X + DT/2 * k2, U, Tau, Xd, Ud);
        [k4, k4_q] = f(X + DT * k3, U, Tau, Xd, Ud);
        X=X+DT/6*(k1 +2*k2 +2*k3 +k4);
        Q = Q + DT/6*(k1_q + 2*k2_q + 2*k3_q + k4_q);
    end
    F = Function('F', {X0, U, Tau, Xd, Ud}, {X, Q}, {'x0','u', 'tau' 'Xd', 'Ud'}, {'xf', 'qf'});
    
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
    lbw = [lbw; -inf; -inf; 0];
    ubw = [ubw; inf; inf; 2*pi];
    w0 = [w0; 0; 0; 0];

%     g = [g, {initial_pos - Xk(1:2)}];
%     lbg = [lbg; 0; 0];
%     ubg = [ubg; 0; 0];
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% MAIN LOOP
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
loopdata = zeros(N+1,7);
%loopdata = [k xref_N xref_E uref_i]
    for k = 0:N-1
        % New NLP variable for control.
        Uk = MX.sym(['U_' num2str(k)], 3);
        w = {w{:}, Uk};
        lbw = [lbw; -10; -10; -pi];
        ubw = [ubw; 10; 10; pi];
        w0 = [w0; 0; 0; 0];
        
        Tauk = MX.sym(['Tau_' num2str(k)], 3);
        w = {w{:}, Tauk};
        lbw = [lbw; -100; -60; -40];
        ubw = [ubw; 100; 60; 40];
        w0 = [w0; 0; 0; 0];
        
        % Integrate until the end of the interval.
%         xref_i(1:2) = reference_trajectory_los(1:2,k+2); % Positional reference.
        eta_dot_ref = [reference_trajectory_los(3:4,k+2);...
                  (atan2(reference_trajectory_los(4,k+3),reference_trajectory_los(3,k+3)) - ...
                   atan2(reference_trajectory_los(4,k+2),reference_trajectory_los(3,k+2))) / h];
        
        uref_i = [sqrt(eta_dot_ref(1)^2 + eta_dot_ref(2)^2); 0; eta_dot_ref(3)];
%         xref_i(3) = atan2(uref_i(2),uref_i(1));
        xref_i = [reference_trajectory_los(1:2,k+2); atan2(eta_dot_ref(2),eta_dot_ref(1))];
        %uref_i = R\uref_i;
        Fk = F('x0', Xk, 'u', Uk, 'tau', Tauk, 'Xd', xref_i, 'Ud', uref_i);
        Xk_end = Fk.xf;
        J = J + Fk.qf;
        
        % New NLP variable for state at the end of interval.
        Xk = MX.sym(['X_' num2str(k+1)], 3);
        w = [w, {Xk}];
        lbw = [lbw; -inf; -inf; 0];
        ubw = [ubw; inf; inf; 2*pi];
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
%                 g = [g, {(Xk(1:2) - interpolated_static_obs(:,i))'*(Xk(1:2) - interpolated_static_obs(:,i)) - 16^2}];
%                 lbg = [lbg; 0];
%                 ubg = [ubg; inf];
            end
        end
        
        loopdata(k+1,:) = [k, xref_i' uref_i'];
        
    end
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Optimal solution and updating states
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    loopdata(end,:) = [k+1, xref_i' uref_i']; %off by one error hack.
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
    
    t = loopdata(:,1);
    xref_N = loopdata(:,2);
    xref_E = loopdata(:,3);
    psi_ref = loopdata(:,4);
    surge_ref = loopdata(:,5);
    sway_ref = loopdata(:,6);
    r_ref = loopdata(:,7);
    
    
    figure(999);
    clf;
    north_opt = w_opt(1:9:end);
    east_opt = w_opt(2:9:end);
    psi_opt = w_opt(3:9:end);
    surge_opt = w_opt(4:9:end);
    sway_opt = w_opt(5:9:end);
    r_opt = w_opt(6:9:end);
    hold on;
    plot(east_opt, north_opt, '*');
    plot(vessel.wp(2,:),vessel.wp(1,:),'g');
    plot(reference_trajectory_los(2,:),reference_trajectory_los(1,:) , 'r-.');
    axis(settings.axis);
    title('Projected future trajectory');
    xlabel('East [m]');
    ylabel('North [m]');
    legend('W_{opt}', 'Transit path', 'reference trajectory');
    grid;
    
    figure(10);
    clf;
    subplot(3,1,1);
    plot(t,xref_N);
    hold on;
    plot(t,north_opt,'*');
    hold off;
    grid;
    title('North ref and North Opt');
    xlabel('Discretized time [k]');
    ylabel('North [m]');
    legend('North ref','North Opt');
    
    subplot(3,1,2);
    plot(t,xref_E);
    hold on;
    plot(t,east_opt,'*');
    hold off;
    grid;
    title('East ref and East opt');
    xlabel('Discretized Time [k]');
    ylabel('East [m]');
    legend('East ref','East opt');
    
    subplot(3,1,3);
    plot(t,psi_ref);
    hold on;
    plot(t,psi_opt,'*');
    hold off;
    grid;
    title('psi ref and psi opt');
    xlabel('Discretized time [k]');
    ylabel('Psi (rad)');
    legend('Psi ref','Psi opt');
    
    figure(11);
    clf;
    subplot(3,1,1);
    plot(t,surge_ref);
    hold on;
    plot(t(2:end),surge_opt,'*');
    hold off;
    grid;
    title('surge ref and surge Opt');
    xlabel('Discretized time [k]');
    ylabel('Surge [m/s]');
    legend('Surge ref','Surge Opt');

    subplot(3,1,2);
    plot(t,sway_ref);
    hold on;
    plot(t(2:end),sway_opt,'*');
    hold off;
    grid;
    title('sway ref and sway Opt');
    xlabel('Discretized time [k]');
    ylabel('sway [m/s]');
    legend('sway ref','sway Opt');

    subplot(3,1,3);
    plot(t,r_ref);
    hold on;
    plot(t(2:end),r_opt,'*');
    hold off;
    grid;
    title('yaw rate ref and yaw rate Opt');
    xlabel('Discretized time [k]');
    ylabel('yaw rate [rad/s]');
    legend('Yaw rate ref','Yaw Rate Opt');
    
%     plot(tgrid, east_opt, '-') ?????
%     stairs(tgrid, [u_opt; nan], '-.') ????
%     xlabel('t') ?????
%     %legend('x1','x2','u') ?????? 
    
    vessel.eta = w_opt(10:12);
    vessel.nu = w_opt(4:6);
    vessel.eta_dot = rotZ(vessel.eta(3))*vessel.nu;
    resulting_trajectory = zeros(6,100);     %  TODO

        
        
        
        
        
        
    
    
    
    
    