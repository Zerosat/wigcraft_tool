clear;

% Include
addpath('include\');
include();

results = readmatrix("results_old2.txt");

% Load the required parameters
partable = readtable('config.dat','HeaderLines',6,'Delimiter','semi','ReadRowNames',true);
parameters = join(erase(string(partable{:, :}), "'"), '', 2);

% File loading
vehicle = stlread(parameters(1));

[~, ~, centroid, ~] = inertia_tensor(vehicle.Points,vehicle.ConnectivityList);
VFo = (vehicle.Points - centroid);
FFo = unifyMeshNormals(vehicle.ConnectivityList,VFo,'alignTo','out');
figure('Position', [300 300 900 600]);
%{
%% TERRAIN GENERATION
[xm, ym] = meshgrid(linspace(-5, 5, 513));
im = zeros(64, 64);
im = perlin_noise(im);
J = imresize(im,[513 513]);
hm = 0.003*xm.^2.*J - 0.05;
cm = generate_terrain_colors(hm);
hmp = max(hm, 0);
h_land = patch(surf2patch(xm*15, ym*15, hmp*15, cm));
[nx, ny, nz] = surfnorm(hmp);
land_normals = [nx(:), ny(:), nz(:)];
set(h_land, 'VertexNormals',    land_normals, ...
            'DiffuseStrength',  0.8, ...      % Reacts to light direction
            'SpecularStrength', 0, ...        % Not shiny
            'AmbientStrength',  0.3, ...      % Reacts to ambient light
            'BackFaceLighting', 'unlit');     % Don't illuminate reverse
patch('Faces',           [1 2 3 4], ...
      'Vertices',        [-1 -1 -0.01; ...
                           1 -1 -0.01; ...
                           1  1 -0.01; ...
                          -1  1 -0.01], ...
      'FaceVertexCData', 0.25*ones(4, 3));
current_time = 0.5;
sun_color    = sun_tones(current_time); sky_color    = sun_tones(0.5);
[xs, ys, zs] = sphere(16); sky_scale = 99;
sky_patch = surf2patch(sky_scale*xs, sky_scale*ys, sky_scale*zs, ...
                       repmat(reshape(sky_color, [1 1 3]), [17 17 1]));
% h_sky = patch(sky_patch);
% set(h_sky, 'DiffuseStrength',  0.3, ...
%           'SpecularStrength', 0, ...
%           'AmbientStrength',  1, ...
%           'BackFaceLighting', 'unlit');
h_light = lightangle(90, 360*(current_time - 0.25));
set(h_light, 'Color', sun_color);
set(gca(), 'AmbientLightColor', sun_color); lighting gouraud;
camera_target   = [0 0 mean(hmp(:)) + 0.5*std(hmp(:))];
camorbit(360*rand(), 0); shading interp; drawnow();
if exist('FigureRotator', 'class');
    f = FigureRotator(gca());
end
%}
hold on; light;

%% MAIN LOOP

for k = 1:1:size(results,1)
    
    % Update the model
    position = results(k,1:3);
    force = results(k,10:12)/norm(results(k,10:12));
    pitch = results(k,7);
    yaw = results(k,8);
    roll = results(k,9);
    VFi = VFo*rotx(deg2rad(pitch))*roty(deg2rad(yaw))*rotz(deg2rad(roll)) + [position(1),position(3),position(2)]; 
    FFi = FFo;
    
    % Plot the data
    if k > 1
        delete(lat); delete(qui);
        delete(ann1); delete(ann2);
        delete(ann3);
    end
    % view([145,30]);
    view([145,30]);
    view([90,0]);
    axis equal;
    PatchProps.FaceColor = 'r';PatchProps.EdgeColor = 'none'; 
    PatchProps.FaceAlpha = 1; PatchProps.EdgeAlpha = 1; 
    scatter3(position(1),position(2),position(3),'ro','filled');
    lat = patch('Faces', FFi, 'Vertices',[VFi(:,1) VFi(:,3) VFi(:,2)], PatchProps);hold on;
    qui = quiver3(position(1),position(2),position(3),force(1),force(3),force(2),'Color','r');
    ann1 = annotation('textbox',[0.05 0.03 0.2 0.05],'String',strcat("Time: ",num2str(results(k,19))," s"));
    ann2 = annotation('textbox',[0.05 0.085 0.2 0.05],'String',strcat("Vspeed: ",num2str(results(k,5))," m/s"));
    ann3 = annotation('textbox',[0.05 0.145 0.2 0.05],'String',strcat("Lift: ",num2str(results(k,11))," N"));
    xlabel('X axis');ylabel('Y axis');zlabel('Z axis');axis equal;
    xlim([-5 5]);ylim([-2 20]);zlim([0 10]);
    % xlim([-50 50]);ylim([-40 40]);zlim([0 10]);
    pause(0.05);
    
end