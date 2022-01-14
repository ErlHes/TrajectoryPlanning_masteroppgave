function map_data_ = read_gml_map_data(filename)
% READ_GML returns hierarchical datastructure of information in % .gml file.

    content = fileread(filename);
    
    %% Remove features that are not relevant
    content = deleteBetween(content,'<app:Dybdepunkt','</app:Dybdepunkt>');
    content = deleteBetween(content,'<app:Dybdepunkt','</app:Dybdepunkt>');

    
    
    
    content = replaceBetween(content,'<gml:FeatureCollection>','<gml:posList>' , "    ");
    content = strrep(content,'<gml:FeatureCollection>    <gml:posList>' , '</gml:posList>  <gml:posList>');
    content = replaceBetween(content,'</gml:posList>','<gml:posList>', "  ");
    content = replaceBetween(content,'7034458.98432121</gml:posList>','</gml:FeatureCollection>',  "    ");
    content = strrep(content,'</gml:posList>    </gml:FeatureCollection>' , '</gml:posList>  <gml:posList>');

    
%     content = eraseBetween(content, '<gml:LineString','>')
%     content = replaceBetween(content, '<app:B', '>', 'olgebryter');
%     content = eraseBetween(content, '<app:Bolgebryter', '</app:datauttaksdato>');
%     content = replace(content, '<app:Bolgebryter</app:datauttaksdato>', '<app:Bolgebryter>');
% 
% %     content = eraseBetween(content, '<app:identifikasjon>', '</app:identifikasjon>');
%     content = strrep(content, '<app:identifikasjon></app:identifikasjon>' , '');
    
    
    content = split(content, '</gml:posList>  <gml:posList>');
    
%     origin = [6509744.65,1714990.87] ;
    origin = [7042294,270275] ;
    
%     brattora_ravnkloa = [63.434741, 10.393375];
    map_data_ = cell(size(content,1),1);
    j = 0;
    for i=1:size(content,1)
        if(size(content{i},2) > 0)
            j = j+1;
            coord_list = str2num(content{i});
            coord = [coord_list(2:2:end)-origin(1); coord_list(1:2:end)-origin(2)];
            map_data_{j} = coord;
        end
    end

    map_data = cell(j,1);

    for i=1:j
        map_data{i} = map_data_{i};
    end
       
   
end