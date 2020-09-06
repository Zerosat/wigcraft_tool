%% Evolution Algorithm
% This algorithm optimizes the solid (STL) to improve performance (lift)

population = 5;
pitch = 15;

% Loading the model
clear;

% Include
addpath('include\');
include();

% Load the required parameters
partable = readtable('config.dat','HeaderLines',6,'Delimiter','semi','ReadRowNames',true);
parameters = join(erase(string(partable{:, :}), "'"), '', 2);

% File loading
vehicle = stlread(parameters(1));
unicube = stlread(parameters(3));

% Vehicle model creation
[~, ~, centroid, ~] = inertia_tensor(vehicle.Points,vehicle.ConnectivityList);
VFo = vehicle.Points - centroid;
FFo = unifyMeshNormals(vehicle.ConnectivityList,VFo,'alignTo','out');
[VFi, FFi] = deal(VFo*rotx(deg2rad(pitch))*roty(deg2rad(0))*rotz(deg2rad(0)),FFo);

% The algorithm starts
% PatchProps.FaceColor = 'b';PatchProps.EdgeColor = 'none';
% PatchProps.FaceAlpha = 1; cont = 1; figure; hold on;

% Array allocating
VS = cell(1,population);
lifts = zeros(1,population);
FS = FFo;
cont = 1;

% VS Initialization: Mutation sequence
for r = 1:1:population
    for k = 1:1:20
        VS{r} = mutate_vertex(VS{r},FS);
    end
end

% Genetic algorithm
while (1)
    
    % Simulation
    for r = 1:1:population
        VFp = VS{r};
        % fuerzas momentos etc etc
        lifts(r) = curforce(:,2);
    end
    
    % Reproduction & Mutation
    for r = 1:1:population
        P = lifts/sum(lifts);
        parent_1 = VS{find(rand<cumsum(P),1,'first')};
        parent_2 = VS{find(rand<cumsum(P),1,'first')};
        VS{r} = offspring_vertex(parent_1, parent_2);
        VS{r} = mutate_vertex(VS{r},FS);
    end
   
    disp(strcat('Generation completed: ',num2str(cont)));
    pause(0.0001); cont = cont + 1;
end
