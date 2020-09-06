%% Mutate vertex
% This function takes the vertex corresponding a model and applies them
% some random changes. 

function new_vertex = mutate_vertex(vertex,elements)
    
    mag = 0.002;
    new_vertex = vertex;
    s = size(vertex,1);
    n = COMPUTE_mesh_normals(struct('faces',elements,'vertices',vertex));
    for k = 1:1:floor(s/5)
        if randi(100) <= 1
            % Pick a vertex from the list
            disp('Mutated'); pause(0.0001);
            rp = randi(s);
            curv = vertex(rp,:);
            
            % Find the elements 
            id = find(elements(:,1) == rp | elements(:,2) == rp | elements(:,3) == rp);
            
            % Compute the mean normal
            curn = mean(n(id,:),1);
            
            % var = randn*mag*curn;
            
            % Check whether the variation is valid
            valid = false;
            while(~valid)
            
                % Compute the variation
                var = randn*mag;

                % Mutate the vertex
                if (sign(var) == 1)
                    if (sum(inpolyhedron(elements,vertex, new_vertex(rp,:) + var*curn)) == 0)
                        valid = true;
                    end
                else
                    if (sum(inpolyhedron(elements,vertex, new_vertex(rp,:) + var*curn)) ~= 0)
                        valid = true;
                    end
                end
            
            end
            
            new_vertex(rp,:) = new_vertex(rp,:) + var*curn;
            
        end
    end
end