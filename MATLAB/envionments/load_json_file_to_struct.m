
data = loadjson('trondheim_harbour_map.json');



piren_frame = true;
if(piren_frame)
    origin = [0,0]'; % Piren frame
else
    origin = [-550,-300]';
end
figure(680)
clf(680)
hold on;
grid on;


% Select relevant map polygons
harbour_indices = [1 2 4 5 6 7 8 9 10 11 12 13];
land_index_offset = 10;
land_indices = [66,66 80 81 107 108 109 110 111] + land_index_offset;

map_list = [harbour_indices land_indices];







separator = [nan;nan];
static_obs = separator;

for j=1:size(map_list,2)
    i = map_list(j);
    coord = data.features{1,i}.geometry.coordinates';
    if~(ispolycw(coord(2,:), coord(1,:)))
        coord = flip(coord,2);
    end
    for k=1:size(coord,2)
        coord(:,k) = coord(:,k)-origin;
    end
    static_obs = [static_obs,coord, separator];
    
    
      pgon = polyshape(coord(2,:), coord(1,:));
      plot(pgon);
end


if(piren_frame)
    save('/home/emilht/MATLAB/simulator_multiAgent/envionments/piren_frame_static_env_trondheim_harbour.mat','static_obs');
else
    save('/home/emilht/MATLAB/simulator_multiAgent/envionments/static_env_trondheim_harbour.mat','static_obs');
end
