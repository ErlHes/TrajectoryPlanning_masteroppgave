function setpoints = thrust_allocation_novel(tau, last_thrust, last_alpha)
    % predefine output
    alpha1 = 0;
    alpha2 = 0;
%     thrust1 = 0;
%     thrust2 = 0;
    Lx = 1.80;
    max_thrust = 500.3;
    
    Fy1 = saturate((tau(3) + Lx*tau(2))/(2*Lx),-max_thrust,max_thrust);
    Fy2 = saturate((Lx*tau(2) - tau(3))/(2*Lx),-max_thrust,max_thrust);

    Fx1_min = -max_thrust;
    Fx1_max = max_thrust;
    
    %% Forward thrust on both
    reverse1 = false; reverse2 = false;
    f_wrapper = @(x) f(x, Fy1, Fy2, tau(1), last_thrust(1), last_thrust(2), ...
                     last_alpha(1), last_alpha(2), reverse1, reverse2);    
    [Fx1_1,cost1] = fminbnd(f_wrapper, Fx1_min, Fx1_max);
    
    %% Reversed thrust on Thruster 1
    reverse1 = true; reverse2 = false;
    f_wrapper = @(x) f(x, Fy1, Fy2, tau(1), last_thrust(1), last_thrust(2), ...
                     last_alpha(1), last_alpha(2), reverse1, reverse2);
    [Fx1_2,cost2] = fminbnd(f_wrapper, Fx1_min, Fx1_max);
    
    %% Reversed thrust on Thruster 2
    reverse1 = false; reverse2 = true;
    f_wrapper = @(x) f(x, Fy1, Fy2, tau(1), last_thrust(1), last_thrust(2), ...
                     last_alpha(1), last_alpha(2), reverse1, reverse2);
    [Fx1_3,cost3] = fminbnd(f_wrapper, Fx1_min, Fx1_max);
    
    %% Reversed thrust on both
    reverse1 = true; reverse2 = true;
    f_wrapper = @(x) f(x, Fy1, Fy2, tau(1), last_thrust(1), last_thrust(2), ...
                     last_alpha(1), last_alpha(2), reverse1, reverse2);
    [Fx1_4,cost4] = fminbnd(f_wrapper, Fx1_min, Fx1_max);
    
    %% Choose best solution   
    costs = [cost1 cost2 cost3 cost4];
    Fx1s = [Fx1_1 Fx1_2 Fx1_3 Fx1_4];   
    [~, best_idx] = min(costs);
    Fx1 = Fx1s(best_idx);
    Fx2 = tau(1) - Fx1;
    thrust1 = sqrt(Fx1^2 + Fy1^2);
    thrust2 = sqrt(Fx2^2 + Fy2^2);
    a1 = atan2(Fy1,Fx1);
    a2 = atan2(Fy2,Fx2);
    switch best_idx
        case 1%Forward thrust on both
            alpha1 = last_alpha(1) + shortest_angle_path(last_alpha(1),a1);
            alpha2 = last_alpha(2) + shortest_angle_path(last_alpha(2),a2);
        case 2%Reversed thrust on Thruster 1
            thrust1 = -thrust1;
            alpha1 = last_alpha(1) + shortest_angle_path(last_alpha(1),a1+pi);
            alpha2 = last_alpha(2) + shortest_angle_path(last_alpha(2),a2);
        case 3%Reversed thrust on Thruster 2
            thrust2 = -thrust2;
            alpha1 = last_alpha(1) + shortest_angle_path(last_alpha(1),a1);
            alpha2 = last_alpha(2) + shortest_angle_path(last_alpha(2),a2+pi);
        case 4%Reversed thrust on both
            thrust1 = -thrust1;
            thrust2 = -thrust2;
            alpha1 = last_alpha(1) + shortest_angle_path(last_alpha(1),a1+pi);
            alpha2 = last_alpha(2) + shortest_angle_path(last_alpha(2),a2+pi);
        otherwise
    end
    
    setpoints = [thrust1 alpha1; thrust2 alpha2];    
end

%% Cost function
function cost = f(Fx1, Fy1, Fy2, tau_x, F1_last, F2_last, a1_last, a2_last, reverse1, reverse2)
    a = [1000000 1000000];%Cost of change of angle
    b = [10 10];%Cost of thrust usage
    c = [10 10];%Cost of thrust change
    d = [1000000 1000000];%Cost of deviation from home angle
    e = [1 1];%Cost of running in reverse
    f = [0 0];%Cost of deviation from mean thrust
    
    home_angle = [0 0];
    mean_thrust = [0 0];
    
    Fx2 = tau_x - Fx1;
    a1 = atan2(Fy1,Fx1);
    a2 = atan2(Fy2,Fx2);
    F1 = sqrt(Fx1^2 + Fy1^2);
    F2 = sqrt(Fx2^2 + Fy2^2);
    
    if reverse1
       a1 = a1 + pi;
       F1 = -F1;
    end
    
    if reverse2
        a2 = a2 + pi;
        F2 = -F2;
    end
    
    move1 = shortest_angle_path(a1_last,a1);
    move2 = shortest_angle_path(a2_last,a2);
    
    cost_angle_change = a(1)*move1^2 + a(2)*move2^2;
    cost_thrust = b(1)*F1^2 + b(2)*F2^2;
    cost_thrust_change = c(1)*(F1-F1_last)^2 + c(2)*(F2-F2_last)^2;
    cost_angle_dev = d(1)*(home_angle(1) - (a1_last + move1))^2 + d(2)*(home_angle(2) - (a2_last + move2))^2;
    cost_reverse = e(1)*double(reverse1)*F1^2 + e(2)*double(reverse2)*F2^2;
    cost_thrust_dev = f(1)*(mean_thrust(1)-F1)^2 + f(2)*(mean_thrust(2)-F2)^2;
    
    cost = cost_angle_change + cost_thrust + cost_thrust_change + cost_angle_dev + cost_reverse + cost_thrust_dev;
end