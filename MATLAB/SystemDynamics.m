function [R, M, C, D] = SystemDynamics(x, u)
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
end