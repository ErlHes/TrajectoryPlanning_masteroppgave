function interpolated = Interpolate_static_obs(matrix)

if ~isempty(matrix)

    [rows, cols] = size(matrix);
    interpolated = zeros(rows, cols*100);
    k = 1;

    for i = 1:cols
        if ~isnan(matrix(1,i))
            if ~isnan(matrix(1,i+1))
                dist = matrix(:,i+1) - matrix(:,i);
                if norm(dist) < 5
                    temp = sign(dist)*1;
                elseif norm(dist) < 10
                    temp = sign(dist)*2;
                elseif norm(dist) < 50
                    temp = sign(dist)*10;
                elseif norm(dist) < 100
                    temp = sign(dist)*20;
                else
                    temp = sign(dist)*50;
                end
                interpolated(:,k) = matrix(:,i);
                count = 1;
                k = k + 1;
                while dist(1) > -1 && dist(2) > -1
                    interpolated(:,k) = matrix(:,i)+ count * temp;
                    k = k + 1;
                    dist = dist - temp;
                    count = count + 1;
                end
            end
        end
    end

    interpolated = interpolated(:,1:find(interpolated(1,:)~=0,1,'last'));
else
    interpolated = [];
end