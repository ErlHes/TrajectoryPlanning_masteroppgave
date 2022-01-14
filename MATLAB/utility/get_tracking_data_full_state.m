function tracking_data = get_tracking_data_full_state(OS,agents, parameters)
tracking_data = [];
static_obs = get_global_map_data();
for k=1:size(agents,2)
    if(agents(k).id ~=OS.id)
        if(parameters.line_of_sight_visibility && size(static_obs,1)>0)
            visible = check_line_of_sight_visibility(OS.eta(1:2,1),agents(k).eta(1:2,1),static_obs);
        else
            visible = true;
        end
        
        if(visible)   
            agents(k).eta(3,1) = atan2(agents(k).eta_dot(2,1), agents(k).eta_dot(1,1));
            agents(k).vel = norm(agents(k).eta_dot(1:2,1),2);
            tracking_data = [tracking_data, agents(k)];
        end
    end
end
end


%% Supporting functions

function visible = check_line_of_sight_visibility(p1,p2,static_obs)
visible = false;

line_between_vessels = [p1,p2];

[xi,~] = polyxpoly(line_between_vessels(1,:),line_between_vessels(2,:),static_obs(1,:),static_obs(2,:));
if(isempty(xi))
    visible = true;
end
end