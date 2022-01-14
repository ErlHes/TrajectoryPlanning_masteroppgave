function [vessel,return_without_further_guidance] = guidance_simulator_multiAgent(vessel,parameters)

[vessel, return_without_further_guidance] = waypoint_handling_and_edge_case_guidance(vessel,parameters);

if(return_without_further_guidance)
    return;
end

vessel = guidance_reference(vessel,parameters);

end

%% Supporting function

function vessel = guidance_reference(vessel,parameters)
persistent plot_handle
persistent integrated_heading_error
persistent plot_counter
if(isempty(plot_handle))
    plot_handle = [];
    integrated_heading_error = 0;
    plot_counter = 1000;
end

   
dt = parameters.dt;


path = vessel.wp(:,1:(size(vessel.wp,2)-sum(isnan(vessel.wp(1,:)))));
speeds = vessel.speed(:,1:(size(vessel.speed,2)-sum(isnan(vessel.speed(1,:)))));
current_waypoint = vessel.current_wp;
current_leg_course =atan2(path(2,current_waypoint+1)-path(2,current_waypoint), path(1,current_waypoint+1)-path(1,current_waypoint));

p = vessel.eta(1:2,1);
eta =vessel.eta;
along_and_cross_track_error = rot2(current_leg_course)'*(p-path(:,current_waypoint));
integrated_heading_error = integrated_heading_error + along_and_cross_track_error(2)*dt;
% Determine course reference

delta_lookahead = parameters.delta_lookahead;
yaw_rate_lim = parameters.yaw_rate_lim;
Kp_course = parameters.Kp_course;
if(vessel.gnc == 1) || (vessel.gnc == 5) 
    theta_d = current_leg_course - atan(along_and_cross_track_error(2)/delta_lookahead);
    vessel_course = atan2(vessel.eta_dot(2,1), vessel.eta_dot(1,1));
    course_error = shortest_angle_path(vessel_course,theta_d);
    guidance_yaw_rate = Kp_course*(course_error);
    guidance_yaw_rate = max(-yaw_rate_lim, min(yaw_rate_lim,guidance_yaw_rate));
    theta_i = vessel_course + dt*guidance_yaw_rate;
    
elseif(vessel.gnc == 3) || (vessel.gnc == 4) %|| (vessel.gnc == 10)
    if(abs(along_and_cross_track_error(2)) > 100)
        lookahead_distance = delta_lookahead;
    elseif(abs(along_and_cross_track_error(2)) < 5)
        lookahead_distance = delta_lookahead*0.2;
    else
        lookahead_distance = delta_lookahead*(0.2 + 0.8*(abs(along_and_cross_track_error(2))-5)/95);
    end
    theta_d = current_leg_course - atan(along_and_cross_track_error(2)/lookahead_distance);
    course_error = shortest_angle_path(vessel.eta_ref(3),theta_d);
    guidance_yaw_rate = Kp_course*(course_error);
    guidance_course_rate = max(-yaw_rate_lim, min(yaw_rate_lim,guidance_yaw_rate));
    theta_i = vessel.eta_ref(3) + dt*guidance_course_rate;
    
elseif(0)
    ki = 1.5;
    integrated_heading_error = integrated_heading_error + along_and_cross_track_error(2)*dt;
    theta_i = current_leg_course - atan(along_and_cross_track_error(2)/delta_lookahead)- ki*integrated_heading_error;
    disp(theta_i);
    disp(integrated_heading_error);
    
elseif(vessel.gnc == 2)    
    lookahead_distance = delta_lookahead;
    theta_d = current_leg_course - atan(along_and_cross_track_error(2)/lookahead_distance);
    course_error = shortest_angle_path(vessel.eta(3),theta_d);
    guidance_yaw_rate = Kp_course*(course_error);
    guidance_course_rate = max(-yaw_rate_lim, min(yaw_rate_lim,guidance_yaw_rate));
    theta_i = vessel.eta(3) + dt*guidance_course_rate;
    
else
    theta_i = current_leg_course - atan(along_and_cross_track_error(2)/delta_lookahead);
end
% 
% plot_counter = plot_counter +1;
% if((vessel.gnc == 4) && plot_counter > 30)
%     plot_counter = 0;
%     figure(600)
%     delete(plot_handle)
%     vessel_course = atan2(vessel.eta_dot(2,1), vessel.eta_dot(1,1));
% 
%     handle_1 = plot_line(vessel.eta(1:2,1), current_leg_course, 50 , 600, get_rgb('blue'));
%     handle_2 = plot_line(vessel.eta(1:2,1), vessel_course, 50 , 600, get_rgb('red'));
%     handle_3 = plot_line(vessel.eta(1:2,1), vessel.eta(3,1), 50 , 600, get_rgb('green'));
%     handle_4 = plot_line(vessel.eta(1:2,1), theta_i, 50 , 600, get_rgb('violet'));
% 
% 
%      plot_handle = [plot_handle, handle_1,handle_2,handle_3,handle_4];
% end

if(size(speeds,2) >= current_waypoint)
    U_d = speeds(1,current_waypoint);
else
    U_d = speeds(1,end);
end


eta_dot_ref = [U_d*cos(theta_i);U_d*sin(theta_i);0];

eta_ref = zeros(3,1);
eta_ref(1) = eta(1)+eta_dot_ref(1)*dt;
eta_ref(2) = eta(2)+eta_dot_ref(2)*dt;
eta_ref(3) = theta_i;

vessel.eta_ref = eta_ref;
vessel.eta_dot_ref = eta_dot_ref;

end

function [vessel, return_without_further_guidance] = waypoint_handling_and_edge_case_guidance(vessel,parameters)

dt = parameters.dt;

return_without_further_guidance = false;
eta = vessel.eta;

path = vessel.wp(:,1:(size(vessel.wp,2)-sum(isnan(vessel.wp(1,:)))));
speeds = vessel.speed(:,1:(size(vessel.speed,2)-sum(isnan(vessel.speed(1,:)))));

current_waypoint = vessel.current_wp;


% Special case if there is only one or zero waypoints, since this will not work with the guidance law.
% The vessel wil then move towards the point and slow down when it is
% whithin 5 meters of the waypoint. The heading will "live free". 
if(size(path,2) == 1) 
    if(vessel.end_of_path == 1)
        dp = path(:,1)-vessel.eta(1:2,1);
        theta_i = atan2(dp(2), dp(1));
        U_d =min(0.5, 0.2*norm(dp,2));
        vessel.eta_dot_ref = [U_d*cos(theta_i);U_d*sin(theta_i);0];
        eta_ref = zeros(3,1); % Do not care about position. Use path-following for now.
        eta_ref(1) = eta(1)+vessel.eta_dot_ref(1)*dt;
        eta_ref(2) = eta(2)+vessel.eta_dot_ref(2)*dt;
        eta_ref(3) = vessel.eta(3);
        vessel.eta_ref = eta_ref;
        
        return_without_further_guidance = true;
        return;
        
    end
    
    dp = path(:,1)-vessel.eta(1:2,1);
    
    theta_i = atan2(dp(2), dp(1));
    U_d = speeds(1,1);
    
    eta_dot_ref = [U_d*cos(theta_i);U_d*sin(theta_i);0];

    eta_ref = zeros(3,1); % Do not care about position. Use path-following for now.
    eta_ref(1) = eta(1)+eta_dot_ref(1)*dt;
    eta_ref(2) = eta(2)+eta_dot_ref(2)*dt;
    eta_ref(3) = theta_i;

    vessel.eta_ref = eta_ref;
    vessel.eta_dot_ref = eta_dot_ref;
    
    if(norm(dp,2) < 5)
        vessel.end_of_path = 1;
    end

    return_without_further_guidance = true;

    return;
    
elseif (size(path,2) == 0) % Path is empty
    vessel.eta_ref = vessel.eta;
    vessel.eta_dot_ref = [0,0,0]';
    return_without_further_guidance = true;

    return;
end

if(current_waypoint >= size(path,2) || vessel.end_of_path == 1) % Stop at end of path. 
    vessel.end_of_path = 1;
    psi_i = atan2(path(2,end)-path(2,end-1), path(1,end)-path(1,end-1));
    vessel.eta_ref = [path(1,end),path(2,end),psi_i]';
    vessel.eta_dot_ref = [0,0,0]';
    return_without_further_guidance = true;

    return;
end


%%% Iterate Waypoint
% Course of current leg
psi_i =atan2(path(2,current_waypoint+1)-path(2,current_waypoint), path(1,current_waypoint+1)-path(1,current_waypoint));
p = vessel.eta(1:2,1);

% Epsilon holds the along track distance and cross track error.
epsilon = rot2(psi_i)'*(p-path(:,current_waypoint));
current_path_leg_length = norm(path(:,current_waypoint) - path(:,current_waypoint+1));
switch_waypoint_tolerance = 3;

if(vessel.gnc == 2)
    switch_waypoint_tolerance = 8;
end

% Calculate along-path distance for next leg of the path
if(current_waypoint < size(path,2)-2)
    psi_i_2 =atan2(path(2,current_waypoint+2)-path(2,current_waypoint+1), path(1,current_waypoint+2)-path(1,current_waypoint+1));
    epsilon_2 = rot2(psi_i_2)'*(p-path(:,current_waypoint+1));
    along_path_distance_from_next_waypoint = epsilon_2(1);

   
else
    along_path_distance_from_next_waypoint = 0;
end

along_path_distance_to_next_waypoint = (current_path_leg_length-epsilon(1));

% Switch waypoint if within acceptance tolerance, or the along-path
% distance along the next leg is further than the remaining distance on the
% current leg. 
if((along_path_distance_to_next_waypoint < switch_waypoint_tolerance)||(along_path_distance_from_next_waypoint >along_path_distance_to_next_waypoint )) && (current_waypoint < size(path,2))% Passed the "perpendicular line" of the next waypoint.
    vessel.current_wp = vessel.current_wp +1;
%      disp('Switching to waypoint:'), disp(agent.current_wp); 
    if(vessel.current_wp >= size(path,2)) % Vessel it at the end of the path
        vessel.end_of_path = 1;
        psi_i = atan2(path(2,end)-path(2,end-1), path(1,end)-path(1,end-1));
        vessel.eta_ref = [path(1,end),path(2,end),psi_i]';
        vessel.eta_dot_ref = [0,0,0]';
        return_without_further_guidance = true;
        return;
    end
end


end