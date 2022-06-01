function F = CasadiSetup(h, N)
import casadi.*

T = h * N;

    %% CasADi setup
    
    % System matrices.
    x = SX.sym('x',6); % x = [N, E, psi, u, v, r]'
    tau = SX.sym('tau',3); % tau = [Fx, Fy, Fn]';
    xref = SX.sym('xref',6); % xref = [Nref, Eref, Psi_ref, Surge_ref, sway_ref, r_ref]'
    
    
%     [R, M, C, D] = SystemDynamics(x, u); % Usikker på hvorvidt det funker
%     å sende CasADi systemer inn i en subfunksjon. Burde jo gå, men lar
%     være for nå.
    % Model Parameters.
    Xu = -68.676;   % Kg/s
    Xuu = -50.08;   % Kg/m
    Xuuu = -14.93;  % Kgs/(m^2)
%     Xv = -25.20;    % Kg/s
%     Xr = -145.3;    % Kgm/s
%     Yu = 90.15;     % Kg/s
    Yv = -8.69;     % Kg/s
    Yvv = -189.08;  % Kg/m
    Yvvv = -0.00613;% Kgs/(s^2) ? Kgs/(m^2)?
%     Yrv = -3086.95; % Kg
%     Yr = -24.09;    % Kgm/s
%     Yvr = -338.32;  % Kg
%     Yrr = 1372.06;  % Kg(m^2)
%     Nu = -38.00;    % Kgm/s
%     Nv = -97.26;    % Kgm/s
    Nvv = -18.85;   % Kg
    Nrv = 5552.23;  % Kgm
    Nr = -230.19;   % Kg(m^2)/s
    Nrr = -0.0063;  % Kg(m^2)
    Nrrr = -0.00067;% Kgms
%     Nvr = -5888.89; % Kgm
    
    m11 = 2131.80;  % Kg
    m12 = 1.00;     % Kg
    m13 = 141.02;   % Kgm
    m21 = -15.87;   % Kg
    m22 = 2231.89;  % Kg
    m23 = -1244.35; % Kgm
    m31 = -423.76;  % Kgm
    m32 = -397.64;  % Kgm
    m33 = 4351.56;  % Kg(m^2)
    
    c13 = -m22*x(5);
    c23 = m11*x(4);
    c31 = -c13;
    c32 = -c23*x(5);
    
    d11 = -Xu - Xuu * abs(x(4)) - Xuuu*(x(4)^2);
    d22 = -Yv - Yvv*abs(x(5)) - Yvvv*(x(5)^2);
    d23 = d22;
    d32 = -Nvv*abs(x(5)) - Nrv *abs(x(6));
    d33 = -Nr - Nrr*abs(x(6)) - Nrrr*(x(6)^2);
    
    
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
%     D = [d11     0        0;...
%          0      d22     d23;...
%          0      d32     50*d33];
% 
%      M = eye(3)*1000;
     D = diag([200, 200, 1000]);
%      C = zeros(3);
    
%     Tau = pickthree(tau); %failed experiement.
    nu_dot = M\(tau -(C+D)*x(4:6)); 
    nu = x(4:6) + h*nu_dot;
    eta_dot = R*nu;
    
    xdot = [eta_dot; nu_dot];
    
%     Funker bra:
%     Kp = diag([8*10^-1, 8*10^-1]);
%     Ku = 6*10^2;
%     Kv = 8*10^2;
    
    % Objective function.
    Kp = diag([8*10^-1, 8*10^-1]); % Tuning parameter for positional reference deviation.
    Ku = 6.7*10^2; % Tuning parameter for surge reference deviation.
    Kv = 7.2*10^2;
%     Kv = 0;
%     Kr = 3*10^2; % Tuning parameter for yaw rate reference deviation.
%     Kt = 10^2;
    R2 = [cos(x(3))    -sin(x(3));...
          sin(x(3))    cos(x(3))];
    Error = R2'*(x(1:2) - xref(1:2));
    Kfy = 1 * 10^-5;

    %Test for heading
    K_phi = 1*10^-5;

    %L = Kp * norm(P - xref)^2 + Ku  * (u(1) - uref(1))^2 + Kr * (u(2) - uref(2))^2;
    %L = (P - xref)'* Kp * (P - xref) + Ku * (u_0'*u_0 - uref(1)'*uref(1))^2;
    %L = (P - xref)'* Kp * (P - xref) + Ku * (u(1) - uref(1))^2 + Kr * (u(2) - uref(2))^2;
    L = Error'* Kp * Error + Ku * (x(4)-xref(4))^2 + Kv * (x(5)-xref(5))^2 + Kfy * tau(2)^2 + K_phi * (x(3)-xref(3))^2;% + Kr * (x(6) - xref(6))^2 + Kt * (tau'*tau) + Ku * (x(4) - xref(4))^2;
    
    % Continous time dynamics.
    f = Function('f', {x, tau, xref}, {xdot, L});
    
    % Discrete time dynamics.
    M = 4; %RK4 steps per interval
    DT = T/N/M;
    f = Function('f', {x, tau, xref}, {xdot, L});
    X0 = MX.sym('X0',6);
    Tau = MX.sym('Tau',3);
    Xd = MX.sym('Xd',6);
    X = X0;
    Q = 0;
    for j=1:M
        [k1, k1_q] = f(X, Tau, Xd);
        [k2, k2_q] = f(X + DT/2 * k1, Tau, Xd);
        [k3, k3_q] = f(X + DT/2 * k2, Tau, Xd);
        [k4, k4_q] = f(X + DT * k3, Tau, Xd);
        X=X+DT/6*(k1 +2*k2 +2*k3 +k4);
        Q = Q + DT/6*(k1_q + 2*k2_q + 2*k3_q + k4_q);
    end
    
    F = Function('F', {X0, Tau, Xd}, {X, Q}, {'x0', 'tau', 'Xd'}, {'xf', 'qf'});
end

