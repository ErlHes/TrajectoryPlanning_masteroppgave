function [A, dict] = get_adjacency_matrix(graph)
% Returns the Adjacency Matrix of a given Graph.
% The Adjacency Matrix is a NxN-Matrix, where row represents the
% Source, while column represents the Target.

    N = numel(graph.node);
    A = zeros(N);
    dict = [];
    

    for i = 1:N
        
        dict = [dict, graph.node(i).id];
    
    end
    
    E = numel(graph.edge);
    
    for i = 1:E
        
        row = find(graph.edge(i).source == dict, 1);
        column = find(graph.edge(i).target == dict, 1);
        
        if isfield(graph.edge(i), 'value')
            A(row, column) = graph.edge(i).value;
        else
            A(row, column) = 1;
        end
        
	end
    
end