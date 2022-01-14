
% Get the Graph structure:
% filename = '/home/emilht/MATLAB/mapdata_trondheim/mapdata/redusert_basisdata_50_trondelag_bolgebryter.gml';
% filename = '/home/emilht/MATLAB/mapdata_trondheim/mapdata/redusert_basisdata_50_trondelag.gml';
% filename = '/home/emilht/MATLAB/mapdata_trondheim/mapdata/Basisdata_50_Trondelag_25833_Dybdedata_GML.gml';
filename = '/home/emilht/MATLAB/mapdata_trondheim/mapdata/Basisdata_50_Trondelag_25832_Dybdedata_GML.gml';

% graph = read_gml_map_data(filename);


range = 2500;
% graph = read_gml_map_data_feature(filename, 'Kystkontur', [-15000,15000,-15000,15000]);
landareal = read_gml_map_data_feature(filename, 'Landareal', [-range,range,-range,range]);
kystkontur = read_gml_map_data_feature(filename, 'Kystkontur', [-range,range,-range,range]);
bolgebryter = read_gml_map_data_feature(filename, 'Bølgebryter', [-range,range,-range,range]);
bygningsmessigAnleggVann = read_gml_map_data_feature(filename, 'BygningsmessigAnleggVann', [-range,range,-range,range]);

% B = read_gml_map_data_feature(filename, 'Kystkontur', [-15000,15000,-15000,15000]);
% graph = [landareal;kystkontur];

%%
% graph = [bolgebryter];
% graph = [kystkontur];
% graph = [bygningsmessigAnleggVann];
graph = [landareal;kystkontur;bygningsmessigAnleggVann];

figure(760)
clf(760)

hold on;
grid on;
c = [0.7,0.7,0.7];
axis('equal')
num_features = size(graph,1);
progress = 0;

for i=1:num_features
    
    coord = graph{i};
    if(size(coord,2) >=3)
%         fill = polyshape([coord(2,:),coord(2,1)], [coord(1,:), coord(1,1)]);
        plot([coord(2,:)], [coord(1,:)],'color', c)    
    
    elseif(size(coord,2) == 2)
        
    elseif(size(coord,2) == 1)
    
    else
        
    end
    if(mod(i,round(num_features/100))==0)
        progress = progress +1;
        disp(strcat("Plotting is at ", num2str(progress), "%"));
    end
    
    
end

