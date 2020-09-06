%% TERRAIN GENERATION
[xm, ym] = meshgrid(linspace(-5, 5, 513));
im = zeros(128, 128);
im = perlin_noise(im);
J = imresize(im,[513 513]);
hm = 0.003*xm.^2.*J - 0.05;
% hm = J;
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
hold on; axis equal; view(3);