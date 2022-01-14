function set_global_tracking_data(OS,agents)
global tracking_data
tracking_data = [];
for k=1:size(agents,2)
    if(agents(k).id ~=OS.id) && (agents(k).visible == 1)
        tracking_data = [tracking_data, agents(k)];
    end
end
end