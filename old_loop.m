%% LOOP()
%-------------------------------------------------------------------------------
%  loop.m -  
% 
%  Copyright (c) 2020, Pablo J. Rosado Junquera (pabloimie@gmail.com)
% 
%  Version: 1.0
%
%-------------------------------------------------------------------------------

% Include
addpath('include\');
include();

% File loading
vehicle = stlread('aircraft_base.stl');
unicube = stlread('cube.stl');

% Center of mass
mass_center = [1.6377 0.46685 2.6168];

% Vehicle model creation
VFi = (vehicle.Points - mass_center);
FFi = unifyMeshNormals(vehicle.ConnectivityList,VFi,'alignTo','out');
VFOri = VFi; VFi = VFi*rotx(deg2rad(15)); 

% Air model creation (CUBE)
limits = [7 7 7 -7 -3 -7];
VCi = aircube_sizing(unicube.Points + [-10 10 -10],[0 0 0],[2 2 7 -2 -2 -5]);
VCi = aircube_sizing(unicube.Points + [-10 10 -10],[0 0 0],limits);
FCi = unifyMeshNormals(unicube.ConnectivityList,VCi,'alignTo','out');

%{
% Air model creation (CYLINDER)
VCi = (unicyl.Points + [0 0 5]).*[0.3 0.3 1];
VCi = VCi*rotx(deg2rad(180)) + [0 0 9];
FCi = unifyMeshNormals(unicyl.ConnectivityList,VCi,'alignTo','out');
%}

%{
vehicle = stlread('sphere.stl');
VFi = vehicle.Points*0.02 + [0.45 0.25 -15];
FFi = vehicle.ConnectivityList;
FFi = unifyMeshNormals(FFi,VFi,'alignTo','out');
%}
%%

%{
view(3);axis equal;light;hold on;
PatchProps.FaceColor = 'g';PatchProps.EdgeColor = 'k'; 
PatchProps.FaceAlpha = 0.3; PatchProps.EdgeAlpha = 1; 
patch('Faces', FFi, 'Vertices',VFi, PatchProps);
PatchProps.FaceColor = 'r';PatchProps.EdgeColor = 'none'; 
PatchProps.FaceAlpha = 0.8; PatchProps.EdgeAlpha = 1; 
patch('Faces', FCi, 'Vertices',VCi, PatchProps);
zdfgdfg
%}

%% PRUEBA PROBLEMA EJEMPLO
%{
addpath('test NS\');
meshbo = load('sphere_mesh2rcm.boundary');
meshele = load('sphere_mesh2rcm.elem');
meshnod = load('sphere_mesh2rcm.node');
inflow_counter  = 0;
outflow_counter = 0;
wall_counter    = 0;

for n = meshbo'
  r2 = meshnod(n,1)^2 + meshnod(n,2)^2;
  % Inflow Boundary Conditions
  if ( abs( meshnod(n,3)+2 ) < 1e-3 )
    inflow_counter = inflow_counter + 1;
    inflow_nodes(inflow_counter) = n;
  elseif ( abs( meshnod(n,3)-7 ) < 1e-3  &&  r2 < 4 -1e-3 )
    outflow_counter = outflow_counter + 1;
    outflow_nodes(outflow_counter) = n;

  else
    wall_counter = wall_counter + 1;
    wall_nodes(wall_counter) = n;
  end
end

steady_navierstokes_3d(2,meshnod,meshele,meshbo,inflow_nodes,outflow_nodes,wall_nodes);
%}
%% PRUEBA PROBLEMA PROPIO

% Perform constructive solid operations


[VAir,FAir] = mesh_boolean(VCi,FCi,VFi,FFi,'minus');

% Repair the mesh
[VAir,FAir,~] = clean_mesh(VAir,FAir);

%{
PatchProps.FaceColor = 'r';PatchProps.EdgeColor = 'none';
PatchProps.FaceAlpha = 0.5;
patch('Faces',FAir, 'Vertices', VAir, PatchProps);
view(3);axis equal;light;
hold on;
quiver3D(VAir(:,1),VAir(:,2),VAir(:,3),normals(:,1),normals(:,2),normals(:,3),3,'LineWidth',1);
%}
% Create the mesh
% [Vm Tm Fm] = tetgen(VAir,FAir,'Flags','-T1e-10-a1-q100');

[Vextra, Textra, Fextra] = model_tetgen(VCi,FCi,[],'Flags','-T1e-8-a0.1-pq');
% [Vextra, ~, ~] = clean_mesh(Vextra,Fextra);
[Valone, ~, Falone] = model_tetgen(VAir,FAir,[],'Flags','-T1e-8-a0.1-S0');
Vextra = setdiff(Vextra,VAir,'rows');
insidx = inpolyhedron(FFi,VFi, Vextra);
Vextra(insidx == 1,:) = [];
[Vm, Tm, Fm] = model_tetgen(VAir,FAir,Vextra,'Flags','-T1e-8-a0.1-S0-i');
%}

% Compute the 10-element mesh
%{
[Vextra, Textra, Fextra] = PR_tetgen(VCi,FCi,[],'Flags','-T1e-8-a0.3-pq-o2');
% [Vextra, ~, ~] = clean_mesh(Vextra,Fextra);
[Valone, ~, ~] = PR_tetgen(VAir,FAir,[],'Flags','-T1e-8-a0.3-S0-o2');
Vextra = setdiff(Vextra,VAir,'rows');
insidx = inpolyhedron(FFi,VFi, Vextra);
Vextra(insidx == 1,:) = [];
[Vm, Tm, Fm] = PR_tetgen(VAir,FAir,Vextra,'Flags','-T1e-8-a0.3-S0-i-o2');

%}

% Compute the boundary conditions

% Walls (CUBE)
[~, fish_coll, ~] = intersect(Vm,Valone,'rows');
casing = [find((Vm(:,1)) == min((Vm(:,1))));find((Vm(:,1)) == max((Vm(:,1))));
          find((Vm(:,2)) == min((Vm(:,2))));find((Vm(:,2)) == max((Vm(:,2))))];
walls = [fish_coll;casing];
% walls = [fish_coll];

%{
% Walls (CYLINDER)
[~, fish_coll, ~] = intersect(Vm,Valone,'rows');
radius = (abs(max(Vm(:,2))) + abs(min(Vm(:,1))))/2;
borders = sqrt((Vm(:,1)).^2+(Vm(:,2)).^2);
casing = find(abs(borders-radius) < 0.1);
walls = [fish_coll;casing];
%}

% Inlet and outlet
inflow = setdiff(find((Vm(:,3) == max((Vm(:,3))))),casing);
outflow = setdiff(find((Vm(:,3) == min((Vm(:,3))))),casing);

hold on;
% scatter3(Vextra(:,1),Vextra(:,2),Vextra(:,3),10,[0.55 0.54 0.54],'.');
% scatter3(Valone(:,1),Valone(:,2),Valone(:,3),10,[0.55 0.54 0.54],'.');
% scatter3(VAir(:,1),VAir(:,2),VAir(:,3),10,[0.55 0.54 0.54],'.');
scatter3(Vm(:,1),Vm(:,2),Vm(:,3),10,[0.55 0.54 0.54],'.');
scatter3(Vm(walls,1),Vm(walls,2),Vm(walls,3),2,'r.');
scatter3(Vm(inflow,1),Vm(inflow,2),Vm(inflow,3),30,[0.9290, 0.6940, 0.1250],'^','filled');
scatter3(Vm(outflow,1),Vm(outflow,2),Vm(outflow,3),30,[0, 0.4470, 0.7410],'^','filled');
axis equal;view(3);

fghfgh

model_solve_navierstokes(Vm,Tm,inflow',outflow',walls');

% steady_navierstokes_3d_10ele(2,Vm,Tm,[inflow;outflow]',inflow',outflow',walls');