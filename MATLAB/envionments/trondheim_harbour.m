
data = loadjson('trondheim_harbour_map.json');


figure(680)
clf(680)
hold on;
grid on;
axis('equal')
axis([-500,200,-500,0])

origin = [-550,-300]';
map_list = [1 2 66 80 81 107 108 109 110 111]; % The parts of the map that are relevant

% static_env_trondheim_harbour = cell(size(map_list,2),1);

separator = [nan;nan];
static_obs = separator;

for j=1:size(map_list,2)
    i = map_list(j);
    coord = flip(round( data.features{1,i}.geometry.coordinates),1)'; % Flip to get coordinates in clockwise direciton. 
    for k=1:size(coord,2)
        coord(:,k) = coord(:,k)-origin;
    end
    static_obs = [static_obs,coord, separator];
    
    
%       pgon = polyshape(coord(2,:), coord(1,:));
%       plot(pgon);
%     static_env_trondheim_harbour{j} = coord;
end
load('static_env_trondheim_harbour.mat');


plot_static_obs(static_obs, 680);



