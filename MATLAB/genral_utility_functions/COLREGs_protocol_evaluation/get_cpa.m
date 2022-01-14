function [cpa, tcpa,p,pc, beta, alpha] = get_cpa(eta,u, eta_ts, u_ts, params)
%GET_CPA Summary of this function goes here
%   Detailed explanation goes here
theta = eta(3);
theta_c = eta_ts(3);

k2 = u^2+ u_ts^2 - 2*sin(eta(3))*sin(eta_ts(3))*u*u_ts-2*cos(eta(3))*cos(eta_ts(3))*u*u_ts;
k1 = 2*(sin(theta)*u*(eta(2)-eta_ts(2))...
       -sin(theta_c)*u_ts*(eta(2)-eta_ts(2))...
       +cos(theta)*u*(eta(1)-eta_ts(1))...
       -cos(theta_c)*u_ts*(eta(1)-eta_ts(1)));
k0 = (eta(1)-eta_ts(1))^2 + (eta(2)-eta_ts(2))^2;


if(k2 == 0) % The vessels are moving at paralell paths with same speed, giving a constant range.
    tcpa = 0;
else
    tcpa = -k1/(2*k2);
end

if tcpa <0 % If the tcpa has allready happened, used current time.
    tcpa = 0;
elseif tcpa > params.tcpa_horizon
    tcpa = params.tcpa_horizon;
end

cpa = sqrt(k2*tcpa^2 + k1*tcpa + k0);

% Calculate relative bearing of TS from OS
p = eta(1:2)+u*[cos(eta(3));sin(eta(3))]*tcpa;
pc = eta_ts(1:2)+u_ts*[cos(eta_ts(3));sin(eta_ts(3))]*tcpa;
dp = pc-p;
if(dp(1) == 0)
    beta = wrap_plus_minus_pi(eta_ts(3)-pi/2-eta(3));
    alpha = wrap_plus_minus_pi(-beta);
else
    beta = wrap_plus_minus_pi(atan2(dp(2),dp(1)) - eta(3));
    % Calculate contanct angle at CPA (the bearing of OS from the TS)
    alpha = wrap_plus_minus_pi(atan2(-dp(2),-dp(1)) - eta_ts(3)); 
end
    



end

