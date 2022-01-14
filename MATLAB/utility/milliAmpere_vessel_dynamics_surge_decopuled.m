function dxdt = milliAmpere_vessel_dynamics_surge_decopuled(t,x)
% x is a vector of vessel states;
% x(1:4,1) are the thruster states: [w1, w2, alpha1, alpha2]
% x(5:8,1) are the thruster setpoints/desired thruster states/control inputs: [w1d, w2d, alpha1_d, alpha2_d]
% x(9:11,1) are the body velocities (nu) : [surge, sway, yaw_rate]
% x(12:14,1) are the position/pose (eta) : [north, east, heading]


xd = x(5:8,1); % Desired thruster states;
dxdt = zeros(14,1); % Allocate zero array.

% Multiply K_omega by 5, since K_omega is calculated for the RPM after the
% gear with a gear ration of 1:5;
K_omega = 4*[0.5630;0.5910]; %params.actuator_1.propeller.K;


% Propeller dynamics
dxdt(1,1) = K_omega(1)*(xd(1,1)-x(1,1));
dxdt(2,1) = K_omega(1)*(xd(2,1)-x(2,1));


%Azimuth dynamics
K_azi = 4*[34.46;37.53];%params.actuator_1.azimuth.K;
eps = [6.277;7.271];%params.actuator_1.azimuth.eps;
% xd(3:4,1) = rad2deg(xd(3:4,1)); % Transform to degrees, since parameters are found for degrees
% x(3:4,1) = rad2deg(x(3:4,1)); 
delta_1 = rad2deg(shortest_angle_path(x(3,1), xd(3,1)));
delta_2 = rad2deg(shortest_angle_path(x(4,1), xd(4,1)));


dxdt(3,1) = K_azi(1)*(delta_1)/sqrt((delta_1)^2 + eps(1)^2);
dxdt(4,1) = K_azi(2)*(delta_2)/sqrt((delta_2)^2 + eps(2)^2);
dxdt(3:4,1) = deg2rad(dxdt(3:4,1)); % Transform derivative back to radians. 


%nu_dynamics, dxdt(9:11,1)
L = 1.2;%thruster_param.arm_to_cg; % Physical dimension on milliAmpere
[M,C,D] = comp_matrices_surge_decoupled(x(9:11,1));
D(3,3) = D(3,3)*30;
thruster_tau = [cos(x(3)) cos(x(4));
                sin(x(3)) sin(x(4));
                L*sin(x(3)) -L*sin(x(4));...
]*x(1:2,1);

% Calculate the nu dynamics
dxdt(9:11,1) = M\(thruster_tau - (C+D)*x(9:11,1));


% eta_dynamics dxdt(12:14,1);
dxdt(12:14,1) = [cos(x(14,1)),-sin(x(14,1)), 0;...
                 sin(x(14,1)), cos(x(14,1)), 0;...
                 0           , 0           , 1]*x(9:11,1);

             

end