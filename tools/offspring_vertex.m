%% Offspring vertex
% This function takes two groups of vertex and returns another, containing
% features from the given groups. 

function new_vertex = offspring_vertex(vertex1, vertex2)
    
    % Initialize offspring
    new_vertex = choose(vertex1, vertex2);
    
    % Substitute some points
    for c = 1:1:20
        idlow = randi(size(vertex1,1));
        idhigh = randi(size(vertex1,1),idlow);
        subs = choose(vertex1, vertex2);
        new_vertex(idlow:idhigh,:) = subs(idlow:idhigh,:);
    end
    
end