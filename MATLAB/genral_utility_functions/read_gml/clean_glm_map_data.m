function graph = clean_glm_map_data(filename)
% READ_GML returns hierarchical datastructure of information in % .gml file.
graph = 10;
    content = fileread(filename);
    content = strrep(content, sprintf('\n'), ''); 

   
end