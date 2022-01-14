function [vessel, resulting_trajectory] = MPC_with_Assist(vessel, tracks, parameters, settings)
import casadi.*
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% INITIAL CONDITIONS - COVERED
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
persistent previous_w_opt
persistent cflags
persistent dontrunfirstime
    Ns = 80;   %number of samples (-) %c
    h = 0.5;    %sampling time (s) %c
    T = Ns * h; %c
    % 1 = HO
    % 2 = GW
    % 3 = SO
    % 4 = OT
    % 5 = SF 
    if(isempty(cflags)) % THIS CAN BE USED TO HARDCODE FLAGS IF NEEDED:
%         cflags = zeros([1,size(tracks,2)]);
        cflags = [2 3 1]; % FOR StraitCross_HO

    end
    
    if(isempty(dontrunfirstime))
        dontrunfirstime = 1;
    end
    
    initial_pos = vessel.eta(1:2);
    [reference_trajectory_los, ~] = reference_trajectory_from_dynamic_los_guidance(vessel, parameters);
    
    %% Static obstacles - COVERED
    static_obs = get_global_map_data();
    interpolated_static_obs = Interpolate_static_obs(static_obs);
    
    %% Dynamic obstacles - COVERED
    simpletraj = 1; %Used to see behaviour if no traffic ASSIST.
    if ~isempty(tracks)
        dynamic_obs(size(tracks,2)) = struct;
        tracks2 = tracks;
    else
        dynamic_obs = [];
    end
    if ~dontrunfirstime
        for i = 1:size(tracks,2)
            tracks2(i).wp(3:4) = [tracks2(i).eta(1);tracks2(i).eta(2)] + 1000* [cos(tracks2(i).eta(3)) , sin(tracks2(i).eta(3))]'; % UFERDIG
            dynamic_obs(i).id = tracks(i).id;
            if simpletraj == 1
                dynamic_obs(i).Traj = reference_trajectory_from_dynamic_los_guidance(tracks2(i),parameters);
            else
                dynamic_obs(i).Traj = reference_trajectory_from_dynamic_los_guidance(tracks(i), parameters);
                dynamic_obs(i).STraj = reference_trajectory_from_dynamic_los_guidance(tracks2(i),parameters);
            end
            dynamic_obs(i).cflag = COLREGs_assessment(vessel, tracks(i),cflags(i));
            cflags(i) = dynamic_obs(i).cflag;
        end 
    end
    %% Casadi setup - COVERED
    
    % x = [eta'] = [x, y, psi] in NED
    % u = [eta_dot'] [u, v, r] in NED
    x1 = SX.sym('x1');      % two states
    x2 = SX.sym('x2');
    u1 = SX.sym('u1');
    u2 = SX.sym('u2');
    x_ref1 = SX.sym('x_ref1');
    x_ref2 = SX.sym('x_ref2');
    u_ref1 = SX.sym('u_ref1');
    u_ref2 = SX.sym('u_ref2');  
    x_ref = [x_ref1;x_ref2];
    u_ref = [u_ref1;u_ref2];
    x = [x1; x2];
    u = [u1; u2];
%     x_ref = SX.sym('x_ref', 2);
%     u_ref = SX.sym('u_ref', 2);
%    x = [vessel.eta(1), vessel.eta(2)]; % Current position
%    u = [vessel.eta_dot(1), vessel.eta_dot(2)];
    xdot = u;     
    %Objective function 
    a =  diag([100,100]); %gain for distance to reference trajectory.    
    b = 0.5;    
    %F = (x - x_ref)'* a *(x - x_ref) + (u-u_ref)'*b*(u-u_ref);   
    L = (x - x_ref)'* a * (x - x_ref) + b * (u'*u - u_ref'*u_ref)^2;   
    % continous time dynamics
    f = Function('f', {x, u, x_ref, u_ref}, {xdot, L});
    
    % Formulate discrete time dynamics
if false
   % CVODES from the SUNDIALS suite
   dae = struct('x',x,'p',u,'ode',xdot,'quad',L);
   opts = struct('tf',T/Ns);
   F = integrator('F', 'cvodes', dae, opts);
else
   % Fixed step Runge-Kutta 4 integrator
   M = 4; % RK4 steps per interval
   DT = T/Ns/M;
   f = Function('f', {x, u, x_ref, u_ref}, {xdot, L});
   X0 = MX.sym('X0', 2);
   U = MX.sym('U', 2);
   Xd = MX.sym('Xd', 2);
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
    F = Function('F', {X0, U, Xd, Ud}, {X, Q}, {'x0','p', 'Xd', 'Ud'}, {'xf', 'qf'});
end

% Start with an empty NLP
w={};
w0 = []; % Initial guess
lbw = [];
ubw = [];
J = 0;
g={};
lbg = [];
ubg = [];

% "Lift" initial conditions
Xk = MX.sym('X0', 2);
w = {w{:}, Xk};
lbw = [lbw; -inf; -inf];
ubw = [ubw; inf; inf];
w0 = [w0; 0; 0];
%X_goal = vessel.wp(:,end);

    g = [g, {initial_pos-Xk}];
    lbg = [lbg; 0; 0];
    ubg = [ubg; 0; 0];
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% MAIN LOOP - COVERED
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for k = 0:Ns-1
    % New NLP variable for the control
    Uk = MX.sym(['U_' num2str(k)], 2);
    w = {w{:}, Uk};
    lbw = [lbw; -10; -10];
    ubw = [ubw;  10; 10];
    w0 = [w0;  0; 0];

    % Integrate till the end of the interval
    x_ref_i = reference_trajectory_los(1:2,k+2);
    u_ref_i = reference_trajectory_los(3:4,k+2);
    Fk = F('x0', Xk, 'p', Uk, 'Xd', x_ref_i, 'Ud', u_ref_i);
    Xk_end = Fk.xf;
    J=J+Fk.qf;

    % New NLP variable for state at end of interval
    Xk = MX.sym(['X_' num2str(k+1)], 2);
    w = [w, {Xk}];
    lbw = [lbw; -inf; -inf];
    ubw = [ubw;  inf;  inf];
    w0 = [w0; 0; 0];
    
    % Add constraints  
    % 1 = HO
    % 2 = GW
    % 3 = SO
    % 4 = OT
    % 5 = SF   
    g = [g, {Xk_end-Xk}];
    lbg = [lbg; 0; 0];
    ubg = [ubg; 0; 0];
    
    if ~isempty(dynamic_obs) && ~dontrunfirstime %Fy faen så styggt dette ble as, gjør om til en funksjon please
        
        for i = 1:size(dynamic_obs,2)
            g = [g, {(Xk - dynamic_obs(i).Traj(1:2,k+1))'*(Xk - dynamic_obs(i).Traj(1:2,k+1)) - 10^2}];
            lbg = [lbg; 0];
            ubg = [ubg; inf];
            
            if dynamic_obs(i).cflag == 1 % HEAD ON
                %% Forbudt å være forran target ship
                offsetang = atan2(dynamic_obs(i).Traj(4,k+1),dynamic_obs(i).Traj(3,k+1));
                offsetdir = [cos(offsetang);sin(offsetang)];
                offsetdist = 5;
                offsetvektor = offsetdist*offsetdir;
                g = [g, {(Xk - (dynamic_obs(i).Traj(1:2,k+1)+ offsetvektor))'*(Xk - (dynamic_obs(i).Traj(1:2,k+1) + offsetvektor)) - 8^2}];
                lbg = [lbg; 0];
                ubg = [ubg; inf];
                offsetdist = 10;
                offsetvektor = offsetdist*offsetdir;
                g = [g, {(Xk - (dynamic_obs(i).Traj(1:2,k+1)+ offsetvektor))'*(Xk - (dynamic_obs(i).Traj(1:2,k+1) + offsetvektor)) - 8^2}];
                lbg = [lbg; 0];
                ubg = [ubg; inf];
                offsetdist = 15;
                offsetvektor = offsetdist*offsetdir;
                g = [g, {(Xk - (dynamic_obs(i).Traj(1:2,k+1)+ offsetvektor))'*(Xk - (dynamic_obs(i).Traj(1:2,k+1) + offsetvektor)) - 8^2}];
                lbg = [lbg; 0];
                ubg = [ubg; inf];
                offsetdist = 25;
                offsetvektor = offsetdist*offsetdir;
                g = [g, {(Xk - (dynamic_obs(i).Traj(1:2,k+1)+ offsetvektor))'*(Xk - (dynamic_obs(i).Traj(1:2,k+1) + offsetvektor)) - 12^2}];
                lbg = [lbg; 0];
                ubg = [ubg; inf];
                offsetdist = 35;
                offsetvektor = offsetdist*offsetdir;
                g = [g, {(Xk - (dynamic_obs(i).Traj(1:2,k+1)+ offsetvektor))'*(Xk - (dynamic_obs(i).Traj(1:2,k+1) + offsetvektor)) - 12^2}];
                lbg = [lbg; 0];
                ubg = [ubg; inf];
                offsetdist = 45;
                offsetvektor = offsetdist*offsetdir;
                g = [g, {(Xk - (dynamic_obs(i).Traj(1:2,k+1)+ offsetvektor))'*(Xk - (dynamic_obs(i).Traj(1:2,k+1) + offsetvektor)) - 12^2}];
                lbg = [lbg; 0];
                ubg = [ubg; inf];
                %% Vanskeligere å komme seg til target's styrbord
                offsetang = atan2(dynamic_obs(i).Traj(4,k+1),dynamic_obs(i).Traj(3,k+1))+ pi/20;
                offsetdir = [cos(offsetang);sin(offsetang)];
                offsetdist = 45;
                offsetvektor = offsetdist*offsetdir;
                g = [g, {(Xk - (dynamic_obs(i).Traj(1:2,k+1)+ offsetvektor))'*(Xk - (dynamic_obs(i).Traj(1:2,k+1) + offsetvektor)) - 15^2}];
                lbg = [lbg; 0];
                ubg = [ubg; inf];
                %% Forbudt å være rett bak target ship, for komfort's skyld.
                offsetang = atan2(dynamic_obs(i).Traj(4,k+1),dynamic_obs(i).Traj(3,k+1)) + pi;
                offsetdir = [cos(offsetang);sin(offsetang)];
                offsetdist = 10;
                offsetvektor = offsetdist*offsetdir;
                g = [g, {(Xk - (dynamic_obs(i).Traj(1:2,k+1)+ offsetvektor))'*(Xk - (dynamic_obs(i).Traj(1:2,k+1) + offsetvektor)) - 12^2}];
                lbg = [lbg; 0];
                ubg = [ubg; inf];
                offsetdist = 20;
                offsetvektor = offsetdist*offsetdir;
                g = [g, {(Xk - (dynamic_obs(i).Traj(1:2,k+1)+ offsetvektor))'*(Xk - (dynamic_obs(i).Traj(1:2,k+1) + offsetvektor)) - 12^2}];
                lbg = [lbg; 0];
                ubg = [ubg; inf];
                offsetdist = 30;
                offsetvektor = offsetdist*offsetdir;
                g = [g, {(Xk - (dynamic_obs(i).Traj(1:2,k+1)+ offsetvektor))'*(Xk - (dynamic_obs(i).Traj(1:2,k+1) + offsetvektor)) - 12^2}];
                lbg = [lbg; 0];
                ubg = [ubg; inf];

                %%
            elseif dynamic_obs(i).cflag == 2 % GIVE WAY
                %%Forbudt å snike seg forbi forran target ship
                offsetang = atan2(dynamic_obs(i).Traj(4,k+1),dynamic_obs(i).Traj(3,k+1));
                offsetdir = [cos(offsetang);sin(offsetang)];
                offsetdist = 10;
                offsetdist2 = 20;
                offsetdist3 = 30;
                offsetvektor = offsetdist*offsetdir;
                offsetvektor2 = offsetdist2*offsetdir;
                offsetvektor3 = offsetdist3*offsetdir;
                g = [g, {(Xk - (dynamic_obs(i).Traj(1:2,k+1)+ offsetvektor))'*(Xk - (dynamic_obs(i).Traj(1:2,k+1) + offsetvektor)) - 20^2}];
                lbg = [lbg; 0];
                ubg = [ubg; inf];
                g = [g, {(Xk - (dynamic_obs(i).Traj(1:2,k+1)+ offsetvektor2))'*(Xk - (dynamic_obs(i).Traj(1:2,k+1) + offsetvektor2)) - 20^2}];
                lbg = [lbg; 0];
                ubg = [ubg; inf];
                g = [g, {(Xk - (dynamic_obs(i).Traj(1:2,k+1)+ offsetvektor3))'*(Xk - (dynamic_obs(i).Traj(1:2,k+1) + offsetvektor3)) - 20^2}];
                lbg = [lbg; 0];
                ubg = [ubg; inf];
                
                
            elseif dynamic_obs(i).cflag == 3 % STAND ON
%                 offsetang = atan2(dynamic_obs(i).Traj(4,k+1),dynamic_obs(i).Traj(3,k+1));
%                 offsetdir = [cos(offsetang);sin(offsetang)];
%                 offsetdist = -10;
%                 offsetvektor = offsetdist*offsetdir;
%                 g = [g, {(Xk - (dynamic_obs(i).Traj(1:2,k+1)+ offsetvektor))'*(Xk - (dynamic_obs(i).Traj(1:2,k+1) + offsetvektor)) - 8^2}];
%                 lbg = [lbg; 0];
%                 ubg = [ubg; inf];
            elseif dynamic_obs(i).cflag == 4 % OVERTAKING
                offsetang = atan2(dynamic_obs(i).Traj(4,k+1),dynamic_obs(i).Traj(3,k+1)) + 0;
                offsetdir = [cos(offsetang);sin(offsetang)];
                offsetdist = 15;
                offsetvektor = offsetdist*offsetdir;
                g = [g, {(Xk - (dynamic_obs(i).Traj(1:2,k+1)+ offsetvektor))'*(Xk - (dynamic_obs(i).Traj(1:2,k+1) + offsetvektor)) - 12^2}];
                lbg = [lbg; 0];
                ubg = [ubg; inf];

                offsetang = atan2(dynamic_obs(i).Traj(4,k+1),dynamic_obs(i).Traj(3,k+1)) + pi;
                offsetdir = [cos(offsetang);sin(offsetang)];
                offsetdist = 15;
                offsetvektor = offsetdist*offsetdir;
                g = [g, {(Xk - (dynamic_obs(i).Traj(1:2,k+1)+ offsetvektor))'*(Xk - (dynamic_obs(i).Traj(1:2,k+1) + offsetvektor)) - 12^2}];
                lbg = [lbg; 0];
                ubg = [ubg; inf];
                
                offsetang = atan2(dynamic_obs(i).Traj(4,k+1),dynamic_obs(i).Traj(3,k+1)) - pi/2;
                offsetdir = [cos(offsetang);sin(offsetang)];
                offsetdist = 5;
                offsetvektor = offsetdist*offsetdir;
                g = [g, {(Xk - (dynamic_obs(i).Traj(1:2,k+1)+ offsetvektor))'*(Xk - (dynamic_obs(i).Traj(1:2,k+1) + offsetvektor)) - 5^2}];
                lbg = [lbg; 0];
                ubg = [ubg; inf];
                
            elseif dynamic_obs(i).cflag == 5 % SAFE
                % No additional constraints
            end
        end
    end
    
%    static obstacle constraints:
    if ~isempty(interpolated_static_obs)
        [~, cols] = size(interpolated_static_obs);
        for i = 1:cols
            g = [g, {(Xk - interpolated_static_obs(:,i))'*(Xk - interpolated_static_obs(:,i)) - 16^2}];
            lbg = [lbg; 0];
            ubg = [ubg; inf];
        end
    end


   
end

%% Optimal solution and updating states
% Create an NLP solver
prob = struct('f', J, 'x', vertcat(w{:}), 'g', vertcat(g{:}));
solver = nlpsol('solver', 'ipopt', prob);

if(~isempty(previous_w_opt))
    w0 = previous_w_opt;
end

% Solve the NLP
sol = solver('x0', w0, 'lbx', lbw, 'ubx', ubw,...
            'lbg', lbg, 'ubg', ubg);
w_opt = full(sol.x);

previous_w_opt = w_opt;
dontrunfirstime = 0;

%previous_w_opt = reshape(reference_trajectory_los,[],1);
%previous_w_opt = previous_w_opt(1:end-2,:);


% Plot the solution
figure(999);
clf;
north_opt = w_opt(1:4:end);
east_opt = w_opt(2:4:end);
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