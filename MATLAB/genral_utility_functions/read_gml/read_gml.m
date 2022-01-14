function graph = read_gml(filename)
% READ_GML returns hierarchical datastructure of information in % .gml file.

    content = fileread(filename);
    content = strrep(content, sprintf('\n'), ''); 
    
    %for consistent input
    content = strrep(content, '[', ' [ ');
    content = strrep(content, ']', ' ] ');
    
    %remove multiple whitespaces in the content:
    loop = 1; 
    i = 1;
    
    while loop
        
        i = i + 1;
        
        if size(strfind(content, repmat(' ', 1, i)), 2) == 0
            
            loop = 0;
            
        end
        
    end
    
    for ii = 1:(i-2)
        
        content = strrep(content, '  ', ' ');
        
    end
    
    content = regexp(content, ' ', 'split');
    
    [i, graph] = get_next_struct(content, 1);
    
    % make next line a comment if information doesnt contain an actual
    % graph.
    graph = graph.graph;
    
end