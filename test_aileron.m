clear; clc;

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

% Aileron loading
ail_backR = stlread('aileron_back_right.STL');
ail_backL = stlread('aileron_back_left.STL');
ail_backU = stlread('aileron_back_up.STL');
ail_wingR = stlread('aileron_wing_right.STL');
ail_wingL = stlread('aileron_wing_left.STL');


% Vehicle model creation
[~, ~, centroid, ~] = inertia_tensor(vehicle.Points,vehicle.ConnectivityList);
VFo = vehicle.Points - centroid;
FFo = unifyMeshNormals(vehicle.ConnectivityList,VFo,'alignTo','out');

[ABRVo, ABRFo] = deal(ail_backR.Points + VFo(100,:) - ail_backR.Points(24,:), ail_backR.ConnectivityList);
[ABLVo, ABLFo] = deal(ail_backL.Points + VFo(467,:) - ail_backL.Points(26,:), ail_backL.ConnectivityList);
[ABUVo, ABUFo] = deal(ail_backU.Points + VFo(26,:) - ail_backU.Points(9,:) + [-0.01 0 0], ail_backU.ConnectivityList);
[AWRVo, AWRFo] = deal(ail_wingR.Points + VFo(838,:) - ail_wingR.Points(44,:), ail_wingR.ConnectivityList);
[AWLVo, AWLFo] = deal(ail_wingL.Points + VFo(630,:) - ail_wingL.Points(41,:), ail_wingL.ConnectivityList);

% BACKU Aileron
ABUval = mean(ABUVo,1);
ABUval = [ABUval(1) ABUval(2) max(ABUVo(:,3))];
ABUvct = [1 0 0];
ABUVo = rodrigues_rot(ABUVo - ABUval,ABUvct,deg2rad(0)) + ABUval;
% BACKR Aileron
ABRval = mean([-0.1883 -0.10615 -1.421;-0.34995 0.22635 -1.8125]);
ABRvct = [-0.1883 -0.10615 -1.421] - [-0.34995 0.22635 -1.8125];
ABRVo = rodrigues_rot(ABRVo - ABRval,ABRvct,deg2rad(0)) + ABRval;
% BACKL Aileron
ABLval = mean([0.1883 -0.10615 -1.421;0.34995 0.22635 -1.8125]);
ABLvct = [0.1883 -0.10615 -1.421] - [0.34995 0.22635 -1.8125];
ABLVo = rodrigues_rot(ABLVo - ABLval,ABLvct,deg2rad(0)) + ABLval;
% WINGR Aileron
AWRval = mean([-1.314 -0.1152 0.3634;-1.625 0.2472 0.22]);
AWRvct = [-1.314 -0.1152 0.3634] - [-1.625 0.2472 0.22];
AWRVo = rodrigues_rot(AWRVo - AWRval,AWRvct,deg2rad(0)) + AWRval;
% WINGL Aileron
AWLval = mean([1.314 -0.1152 0.3634;1.625 0.2472 0.22]);
AWLvct = [1.314 -0.1152 0.3634] - [1.625 0.2472 0.22];
AWLVo = rodrigues_rot(AWLVo - AWLval,AWLvct,deg2rad(0)) + AWLval;


hold on; axis equal;
PatchProps.FaceColor = 'r';PatchProps.EdgeColor = 'k'; 
PatchProps.FaceAlpha = 0.9; PatchProps.EdgeAlpha = 1; 
latb = patch('Faces', FFo, 'Vertices',VFo, PatchProps);

% ROLL CONTROL
PatchProps.FaceColor = 'b';PatchProps.EdgeColor = 'k';
lat4 = patch('Faces', AWRFo, 'Vertices',AWRVo, PatchProps);
lat5 = patch('Faces', AWLFo, 'Vertices',AWLVo, PatchProps);

% YAW CONTROL
PatchProps.FaceColor = 'g';PatchProps.EdgeColor = 'k';
lat1 = patch('Faces', ABRFo, 'Vertices',ABRVo, PatchProps);
lat2 = patch('Faces', ABLFo, 'Vertices',ABLVo, PatchProps);

% PITCH CONTROL
PatchProps.FaceColor = 'y';PatchProps.EdgeColor = 'k'; 
lat3 = patch('Faces', ABUFo, 'Vertices',ABUVo, PatchProps);


view(3);
