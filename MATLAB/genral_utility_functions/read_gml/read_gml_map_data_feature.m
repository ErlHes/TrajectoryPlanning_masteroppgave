function map_data = read_gml_map_data_feature(filename, feature, axis_limits)
% READ_GML returns hierarchical datastructure of information in % .gml file.

    content = fileread(filename);
    
    %% Remove features that are not relevant
    content = eraseBetween(content,'<?','?>'); % Remove line from start
    content = eraseBetween(content,'<gml:FeatureCollection','>'); % Remove additional info in tag
    content = eraseBetween(content,strcat('<app:',feature),'>'); % Remove additional info in feature tag.
    content = eraseBetween(content,'<gml:FeatureCollection>',strcat('<app:',feature,'>'));    
    
    content = eraseBetween(content,strcat('</app:',feature,'>'),strcat('<app:',feature,'>'));    
    content = eraseBetween(content,strcat('</app:',feature,'>'),strcat('<app:',feature,'>'));
    
    switch feature
        case ( 'Bølgebryter')
            content = eraseBetween(content,'<app:BygningsmessigAnleggVann','</gml:FeatureCollection>');
        case ( 'BygningsmessigAnleggVann' )
            content = eraseBetween(content,'app:Dataavgrensning','</gml:FeatureCollection>');
    end
        
    
    content = replaceBetween(content,'<gml:FeatureCollection>','<gml:posList>' , "    ");
    content = strrep(content,'<gml:FeatureCollection>    <gml:posList>' , '</gml:posList>  <gml:posList>');
    content = replaceBetween(content,'</gml:posList>','<gml:posList>', "  ");
    content = replaceBetween(content,'</gml:posList>','<gml:posList>', "  ");
    content = replaceBetween(content,'</gml:posList>','<gml:posList>', "  ");

%     content = replaceBetween(content,'7034458.98432121</gml:posList>','</gml:FeatureCollection>',  "    ");
    
    content = strrep(content,'</gml:posList>    </gml:FeatureCollection>' , '</gml:posList>  <gml:posList>');

    

    
    
    content = split(content, '</gml:posList>  <gml:posList>');
    
%     origin = [6509744.65,1714990.87] ;
%     origin = [7042294,270275] ;
%     origin = [(97250 + 474940.976664649), (7060873.16133415-25000)]; % Trondheim ish
    
%     origin = [187297.341808508 7089325.11401945];
    origin = [270313,7042302]; % Brattora-ravnkloa from map 25833
    
%     brattora_ravnkloa = [63.434741, 10.393375];
    map_data_ = cell(size(content,1),1);
    j = 0;
    for i=1:size(content,1)
        if(size(content{i},2) > 0)
            coord_list = str2num(content{i});
            coord = [coord_list(2:2:end)-origin(2); coord_list(1:2:end)-origin(1)];
            in_range = (any(coord(1,:)>axis_limits(1))&& any((coord(1,:)<axis_limits(2)))&& any(coord(2,:)>axis_limits(3))&& any(coord(2,:)<axis_limits(4)));
            if(in_range)
                j = j+1;
                map_data_{j} = coord;
            end
        end
    end

    map_data = cell(j,1);

    for i=1:j
        map_data{i} = map_data_{i};
    end
       
   
end