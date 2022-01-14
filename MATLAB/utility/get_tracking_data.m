function tracking_data = get_tracking_data(OS,agents)
tracking_data = [];

for k=1:size(agents,2)
    if(agents(k).id ~=OS.id) && (agents(k).visible == 1)
        track_course = atan2(agents(k).eta_dot(2,1), agents(k).eta_dot(1,1));
        track_speed = norm(agents(k).eta_dot(1:2,1),2);
        track = get_tracking_data_struct(agents(k).id,[agents(k).eta(1:2,1);track_course] , track_speed, agents(k).size);
        tracking_data = [tracking_data, track];
    end
end
end