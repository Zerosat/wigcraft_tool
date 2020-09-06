%% SELECT VALID FACETS

% Compute area of triangle in 3D coordinates

% The 1st point is given by the 1st index in the facet list, and so on

tol = 0.005; % is the minimum valid area

F = FFi; % is the facet list
V = VFi; % is the vertex list

[st, nd, rd, Fcont] = deal(V(F(:,1),:),V(F(:,2),:),V(F(:,3),:),1);

x = [st(:,1),nd(:,1),rd(:,1)];
y = [st(:,2),nd(:,2),rd(:,2)];
z = [st(:,3),nd(:,3),rd(:,3)];

hold on;
for plo = 1:1:size(F,1)
    [xi,yi,zi] = deal([st(plo,1),nd(plo,1),rd(plo,1)],...
    [st(plo,2),nd(plo,2),rd(plo,2)],[st(plo,3),nd(plo,3),rd(plo,3)]);
    A(plo) = 0.5*sqrt(det([xi;yi;ons])^2 + det([yi;zi;ons])^2 + det([zi;xi;ons])^2);
    
    if A(plo) >= 0.005
        col = 'w';
        Vcent(Fcont,:) = [xi,yi,zi];
        Fsel(Fcont,:) = F(plo,:);
        Asel(Fcont) = A(plo);
        Fcont = Fcont + 1;
    else
        col = 'r';
    end
    fill3(xi,yi,zi,col);
end
axis equal;

%% QUERY AND PRESSURE COMPUTATION

shift = 0.1; % is the gap vehicle - calculation point


dir = COMPUTE_mesh_normals(struct('vertices',V,'faces',Fsel))*shift;
% Computation points
Vcomp = [mean(Vcent(:,1:3),2),mean(Vcent(:,4:6),2),mean(Vcent(:,7:9),2)] + dir*shift;


% scatter3(Vcomp(:,1),Vcomp(:,2),Vcomp(:,3),'b');
% quiver3(mean(Vcent(:,1:3),2),mean(Vcent(:,4:6),2),mean(Vcent(:,7:9),2),dir(:,1),dir(:,2),dir(:,3),1,'LineWidth',1,'Color','b');

% Check if there are points inside the model
insidx = inpolyhedron(FFi,VFi, Vcomp);
Vdefc = Vcomp; Vdefc(insidx == 1,:) = [];
dirdefc = -dir; dirdefc(insidx == 1,:) = [];
Aseldefc = Asel; Aseldefc(insidx == 1) = [];

%% COMPUTE FORCES AND MOMENTS
load('ns_3d.mat');
data_nofilt = [x u v w p];
% Noise filtering
[~,pos] = rmoutliers(data_nofilt(:,7),'quartiles');
dat = data_nofilt(~pos,:);
izero = find(dat(:,7) < 0);
dat(izero,:) = [];

Pdefc = griddatan([dat(:,1) dat(:,2) dat(:,3)],dat(:,7),Vdefc);
Fdefc = Pdefc.*dirdefc.*Aseldefc';
Fsum = sum(Fdefc);

quiver3D(Vdefc(:,1),Vdefc(:,2),Vdefc(:,3),Fdefc(:,1),Fdefc(:,2),Fdefc(:,3),'LineWidth',3);
quiver3(0,0,0,Fsum(:,1),Fsum(:,2),Fsum(:,3),'Color','r');
