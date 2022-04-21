function Static_obs_constraints = Static_obstacles_check(obsmatrix, trajectory, vessel)
% Check for static obstacles within a circular area around each waypoint in
% the reference trajectory, We only care about obstacles that occur "in
% front of" the vessel, and thus in turn in front of the waypoint.
    %% Initialize
%     startpos = trajectory(1:2,2);
%     heading = vessel.eta(3);
%     heading = atan2(trajectory(2,2)-vessel.eta(2),trajectory(1,2)-vessel.eta(1));
    x = [];
    y = [];
    %% Polygons
    xbox = obsmatrix(2,:);
    ybox = obsmatrix(1,:);

    %% LOOP
    for i = 2:25:length(trajectory)
        pos = trajectory(1:2,i);
        heading = atan2(trajectory(2,i)-trajectory(2,i-1),trajectory(1,i)-trajectory(1,i-1));

        for j = -pi/3:0.2:pi/3
            ang = heading + j;
            dir = [cos(ang);sin(ang)];
            %% RADIUS OF SCAN HERE
            dist = 50;
            %%
            vektor = dist*dir;
            checkpos = pos + vektor;
            x = [x, pos(2), checkpos(2), NaN];
            y = [y, pos(1), checkpos(1), NaN];
        end

    end
    [xi, yi, ii] = polyxpoly(x,y,xbox,ybox);
    %liste med punkter, + hvilken polygon de tilhører i første kolonne i
    %ii.
    iii = ii(:,2); %We only care about the obstacle index
    iii = [iii; NaN];
    constraintindex = 1;

    %% Generate list if points that are all on the same polygon edge
    intersect =  [];
    for k = 1:length(iii)-1
        if ~isnan(iii(k+1))
            if iii(k) == iii(k+1)
                intersect = [intersect; yi(k), xi(k)]; % GOT A BUG HERE:
                %Error using vertcat
                %Dimensions of arrays being concatenated are not consistent.
            elseif k == 1
                intersect = [yi(k),xi(k)]';
            else
                intersect = [intersect; yi(k), xi(k)];
                intersect = intersect';
                
                % DO SOMETHING
                %something something
                %Static_obs_constraints = something;
                if size(intersect,2) > 1
                count = 1;
                for kk = 1:length(intersect)
                    for jj = 1 + 1:length(intersect)
                        distance(count) = sqrt((intersect(2,kk) - intersect(2,jj))^2 + (intersect(1,kk) - intersect(1,jj))^2);
                        Matrix(count, :) = [intersect(2,kk) intersect(1,kk) intersect(2,jj) intersect(1,jj) distance(count)];
                        count = count + 1;
                    end
                end
                SortedMatrix = sortrows(Matrix,5);
                MaxDist = SortedMatrix(size(Matrix,1), :);
                else
                   distance(count) = sqrt((intersect(2) - intersect(2))^2 + (intersect(1) - intersect(1))^2);
                   MaxDist = [intersect(2) intersect(1) intersect(2) intersect(1) distance(count)];
                end

                PolyAng = atan2(MaxDist(1)-MaxDist(3),MaxDist(2)-MaxDist(4));
                PolyDir = [cos(PolyAng);sin(PolyAng)];

                %% place distance here
                PlaceDistance = 10;
                %%
                for n = 0:floor(MaxDist(5)/PlaceDistance)
                    Static_obs_constraints(:,constraintindex) = [MaxDist(4);MaxDist(3)] + n * PlaceDistance*PolyDir;
                    constraintindex = constraintindex + 1;
                end
                Matrix = [];
                distance = [];
                intersect = []; % Clear for next polygon edge.
            end
        else
            %Last point in list, doesn't matter if it's part of the same edge
            %as previous, the operation will be the same.
            intersect = [intersect; yi(k), xi(k)];
            intersect = intersect';
            % DO SOMETHING
            if size(intersect,2) > 1
            count = 1;
            for kk = 1:length(intersect)
                for jj = 1 + 1:length(intersect)
                    distance(count) = sqrt((intersect(2,kk) - intersect(2,jj))^2 + (intersect(1,kk) - intersect(1,jj))^2);
                    Matrix(count, :) = [intersect(2,kk) intersect(1,kk) intersect(2,jj) intersect(1,jj) distance(count)];
                    count = count + 1;
                end
            end
            SortedMatrix = sortrows(Matrix,5);
            MaxDist = SortedMatrix(size(Matrix,1), :);
            else
               distance(count) = sqrt((intersect(2) - intersect(2))^2 + (intersect(1) - intersect(1))^2);
               MaxDist = [intersect(2) intersect(1) intersect(2) intersect(1) distance(count)];
            end

            PolyAng = atan2(MaxDist(1)-MaxDist(3),MaxDist(2)-MaxDist(4));
            PolyDir = [cos(PolyAng);sin(PolyAng)];
            PlaceDistance = 20;

            for n = 0:floor(MaxDist(5)/PlaceDistance)
                Static_obs_constraints(:,constraintindex) = [MaxDist(4);MaxDist(3)] + n * PlaceDistance*PolyDir;
                constraintindex = constraintindex + 1;
            end
            Matrix = [];
            distance = [];
            intersect = []; % Clear for next polygon edge.
            %END
        end
    end

    
    %% Display for testing purposes:
    mapshow(xbox,ybox,'DisplayType','polygon','LineStyle','none')
    mapshow(x,y,'Marker','+')
    mapshow(xi,yi,'DisplayType','point','Marker','o')
    mapshow(Static_obs_constraints(2,:),Static_obs_constraints(1,:),'DisplayType','point','Marker','x')
end