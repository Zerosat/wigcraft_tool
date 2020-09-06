clear; clc;

% Include
addpath('include\');
include();

% CRITERION: [X Z Y]. Vertexes are already in this form. 

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
height_ini = 2;
mini_dim = 0.005;
limits = [cube_dim(1) (cube_zwidth-height_ini) cube_dim(2) cube_dim(3) -height_ini cube_dim(4)];
mini_limits = [mini_dim mini_dim mini_dim -mini_dim -mini_dim -mini_dim];
VCo = aircube_sizing(unicube.Points + [-10 10 -10],[0 0 0],limits);


p = {[0 0 0],[1.26 -0.09 0.5],[-1.26 -0.09 0.5],[0 0 1.3],[0 0.2 0.6],...
    [0 -0.2 0.6],[0 0.01 -0.6],[0 -0.19 -0.6],[0 -0.19 -1.3],[0 0.51 -2.1]};
minicubes = cell(size(p,2),1);

for k = 1:1:size(p,2)
    minicubes{k} = aircube_sizing(unicube.Points + [-10 10 -10],p{k},mini_limits);
end

FCo = unifyMeshNormals(unicube.ConnectivityList,VCo,'alignTo','out');
[VFi, FFi, VCi, FCi] = deal(VFo*rotx(deg2rad(0))*roty(deg2rad(0))*rotz(deg2rad(0)),FFo,VCo,FCo);

names = ['A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P'];

hold on;axis equal;
PatchProps.EdgeColor = 'none';
PatchProps.FaceAlpha = 0.1;
PatchProps.FaceColor = 'g';
patch('Faces',FFi, 'Vertices', VFi, PatchProps);
PatchProps.FaceColor = 'r';

for k = 1:1:size(minicubes,1)
    fprintf("Testing %s minicube... ",num2str(names(k)));
    if all(inpolyhedron(FFi,VFi,minicubes{k}))
        
        cprintf('green',"PASSED\n");
    else
        cprintf('red',"FAILED\n");
    end
    patch('Faces',FCi, 'Vertices', minicubes{k}, PatchProps);
end
view(3);

% figure;view(3);hold on;axis equal;

for k = 1:1:size(p,2)
    curp = p{k};
    for r = 1:1:size(p,2)
        cur2p = p{r};
        if ~isequal(curp,cur2p)
            plot3([curp(:,1);cur2p(:,1)],[curp(:,2);cur2p(:,2)],[curp(:,3);cur2p(:,3)],'k-s','LineWidth',1,'MarkerFaceColor','r');
        end
    end
end
