function feasibility = feasibility_check(previous_w_opt)
    north_opt = previous_w_opt(1:9:end);
    east_opt = previous_w_opt(2:9:end);

    feasibility = 1;

    for i = 1:length(north_opt)-1
        x1 = east_opt(i);
        x2 = east_opt(i+1);
        y1 = north_opt(i);
        y2 = north_opt(i+1);
        dist = sqrt((x2-x1)^2 + (y2-y1)^2);
        if dist > 5
            feasibility = 0;
        end
        %if distance to next point > 5
            % BIG ERROR, NOT FEASIBLE
        %else
            %feasible.

    end
end 