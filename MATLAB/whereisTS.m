function [pos_TS, vel_TS] = whereisTS(tracks,wptstimer)
    startpos = tracks.eta(1:2);
    totaldistance = wptstimer * norm(tracks.nu(1:2),2);
    WPindex = tracks.current_wp;
    distancetonextWP = sqrt((tracks.wp(1,WPindex+1) - startpos(1))^2 + (vessel.wp(2,WPindex+1) - startpos(2))^2);
    leftoverdistance = totaldistance - distancetonextWP;
    while leftoverdistance > 0
        WPindex = WPindex + 1;
        distancetonextWP = sqrt((tracks.wp(1,WPindex+1) - tracks.wp(1,WPindex))^2 + (vessel.wp(2,WPindex+1) - tracks.wp(2,WPindex))^2);
        leftoverdistance = leftoverdistance - distancetonextWP;
    end
end