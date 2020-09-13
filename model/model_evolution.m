%% Evolution Algorithm
% This algorithm optimizes the solid (STL) to improve performance (lift)

% Loading the model
clear;

population = 5;
pitch = 15;
speed = 5;

% Include
addpath('include\');
include();

% Load the required parameters
partable = readtable('config.dat','HeaderLines',6,'Delimiter','semi','ReadRowNames',true);
parameters = join(erase(string(partable{:, :}), "'"), '', 2);

% File loading
vehicle = stlread(parameters(1));
unicube = stlread(parameters(3));
cube_dim = double(strsplit(parameters(4),','));
cube_zwidth = sum(double(strsplit(parameters(5),',')));

% Vehicle model creation
[~, ~, centroid, ~] = inertia_tensor(vehicle.Points,vehicle.ConnectivityList);
VFo = vehicle.Points - centroid;
FFo = unifyMeshNormals(vehicle.ConnectivityList,VFo,'alignTo','out');

% Air model creation (CUBE)
height_ini = 1;
limits = [cube_dim(1) (cube_zwidth-height_ini) cube_dim(2) cube_dim(3) -height_ini cube_dim(4)];
VCo = aircube_sizing(unicube.Points + [-10 10 -10],[0 0 0],limits);
FCo = unifyMeshNormals(unicube.ConnectivityList,VCo,'alignTo','out');

[VFi, FFi, VCi, FCi] = deal(VFo*rotx(deg2rad(pitch))*roty(deg2rad(0))*rotz(deg2rad(0)),FFo,VCo,FCo);
% The algorithm starts
% PatchProps.FaceColor = 'b';PatchProps.EdgeColor = 'none';
% PatchProps.FaceAlpha = 1; cont = 1; figure; hold on;

% Keycube creation (MINICUBE)
mini_dim = 0.005;
mini_limits = [mini_dim mini_dim mini_dim -mini_dim -mini_dim -mini_dim];
p = {[0 0 0],[1.26 -0.09 0.5],[-1.26 -0.09 0.5],[0 0 1.3],[0 0.2 0.6],...
    [0 -0.2 0.6],[0 0.01 -0.6],[0 -0.19 -0.6],[0 -0.19 -1.3],[0 0.51 -2.1]};
Okeycubes = cell(size(p,2),1);
for k = 1:1:size(p,2)
    Okeycubes{k} = aircube_sizing(unicube.Points + [-10 10 -10],p{k},mini_limits);
end
keycubes = Okeycubes;
for f = 1:1:size(Okeycubes,1)
    keycubes{f} = Okeycubes{f}*rotx(deg2rad(pitch))*roty(deg2rad(0))*rotz(deg2rad(0));
end

% Array allocating
VS = cell(1,5);
for o = 1:1:size(VS,2)
    VS{o} = VFi;
end
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
        [curforce,~,~] = model_flysim(VCi,FCi,keycubes,VFp,FFi,{},speed,0.3,false);
        lifts(r) = curforce(:,2);
    end
    
    % Reproduction & Mutation
    liftnorm = (lifts - mean(lifts))/std(lifts);
    % liftnorm = lifts;
    for r = 1:1:population
        P = liftnorm/sum(liftnorm);
        parent_1 = VS{find(rand<cumsum(P),1,'first')};
        parent_2 = VS{find(rand<cumsum(P),1,'first')};
        VS{r} = offspring_vertex(parent_1, parent_2);
        VS{r} = mutate_vertex(VS{r},FS);
    end
    
    writematrix(lifts,'genetic_algorithm_NUEVO.txt','WriteMode','append');
    disp(strcat('Generation completed: ',num2str(cont)));
    pause(0.0001); cont = cont + 1;
end
