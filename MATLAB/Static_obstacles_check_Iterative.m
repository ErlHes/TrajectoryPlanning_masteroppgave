function static_obs_constraints = Static_obstacles_check_Iterative(obsmatrix, trajectory, k)
%First check if we're using reference LOS trajectory or previous w_opt
    w_opt = 0;
    if size(trajectory,1) > 4
        w_opt = 1;
    end

    %% Initialize
%     startpos = trajectory(1:2,2);
%     heading = vessel.eta(3);
%     heading = atan2(trajectory(2,2)-vessel.eta(2),trajectory(1,2)-vessel.eta(1));
    x = [];
    y = [];
    %% Polygons
    xbox = obsmatrix(2,:);
    ybox = obsmatrix(1,:);

    %Find position
    if(w_opt)
        ypos = trajectory(1:9:end);
        xpos = trajectory(2:9:end);
        if k < length(xpos)
            pos = [ypos(k+1);xpos(k+1)];
        else
            pos = [ypos(length(ypos));xpos(length(xpos))]; % <- Contingency that should never occur.
        end
    else
        pos = trajectory(1:2,k+1);        
    end
    
    %Generate intersection scan lines
    for j = -pi:pi/6:pi
            ang = j;
            dir = [cos(ang);sin(ang)];
            %% RADIUS OF SCAN HERE
            dist = 50;
            %%
            vektor = dist*dir;
            checkpos = pos + vektor;
            x = [x, pos(2), checkpos(2), NaN];
            y = [y, pos(1), checkpos(1), NaN];
    end

    [xi, yi, ii] = polyxpoly(x,y,xbox,ybox);
    % Keep first hit:
    A = [xi, yi, ii];
    [~,uidx] = unique(A(:,3),'stable');
    A_without_dup = A(uidx,:);
    xi = A_without_dup(:,1);
    yi = A_without_dup(:,2);
    ii = A_without_dup(:,3:4);
    
    %% TEST CODE
%     testx = [];
%     testy = [];
%     mapshow(xbox,ybox,'DisplayType','polygon','LineStyle','none')
%     mapshow(x,y,'Marker','+')
%     mapshow(xi,yi,'DisplayType','point','Marker','o')
%     for i = 1:length(xi)
%         testpoint = [yi(i);xi(i)];
%         line = testpoint - pos;
%         line = [-line(2); line(1)];
%         point1 = testpoint + line;
%         point2 = testpoint - line;
%         testx = [testx, point1(2), point2(2), NaN];
%         testy = [testy, point1(1), point2(1), NaN];
%         mapshow(testx,testy,'Marker','x')
%     end
%%
    %% Generate lines:
    static_obs_constraints = zeros(3,length(xi));
    for i = 1:length(xi)
        intersectionpoint = [yi(i); xi(i)];
        %horrible 2am spaghetti:
        line = pos - intersectionpoint; % The vector that takes us from intersection point current position
        transposedline = [-line(2);line(1)]; % Get Orthogonal of said vector.
        tangent = intersectionpoint + transposedline; % create point along orthogonal vector
        
%         pi_p = atan2(tangent(1) - intersectionpoint(1), tangent(2) - intersectionpoint(2)); % THIS COULD BE OPTIMIZED WITH A TABLE, 
        pi_p = atan2(tangent(2) - intersectionpoint(2), tangent(1) - intersectionpoint(1));
        % check line ID -> lookup corresponding angle :)
        static_obs_constraints(:,i) = [intersectionpoint(1); intersectionpoint(2); pi_p];
    end
end