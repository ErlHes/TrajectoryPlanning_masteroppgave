function [pos_TS, vel_TS] = whereisTS(tracks,wptstimer)
    pos_TS = tracks.eta(1:2);
    vel_TS = tracks.eta_dot(1:2);
    WPlim = size(tracks.wp,2);

    pos = tracks.eta(1:2);
    distance = wptstimer * norm(tracks.nu(1:2),2);
    WPindex = tracks.current_wp;
    if WPindex < WPlim
        distancetonextWP = sqrt((tracks.wp(1,WPindex+1) - pos(1))^2 + ((tracks.wp(2,WPindex+1) - pos(2))^2));
    else
        distancetonextWP = 0;
    end
    while distance > 0
        if distance > distancetonextWP
            pos = tracks.wp(1:2,WPindex+1);
            distance = distance - distancetonextWP;
            WPindex = WPindex + 1;
            if WPindex < WPlim
                distancetonextWP = sqrt((tracks.wp(1,WPindex+1) - pos(1))^2 + ((tracks.wp(2,WPindex+1) - pos(2))^2));
            else
                distancetonextWP = 0;
                pos_TS = pos;
                vel_TS = [0,0]';
                distance = 0;
            end
        else
            %Beveg oss (distane) langs banen til neste WP
            %sett distance til null.
            direction = tracks.wp(:,WPindex+1) - pos;
            travel = distance / distancetonextWP;
            pos_TS = pos + travel * direction;
            vel_TS = rotZ(atan2(direction(2),direction(1))) * tracks.nu;
            vel_TS = vel_TS(1:2);
            distance = 0;
        end
    end
end



