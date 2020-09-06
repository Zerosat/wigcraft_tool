dat = [x u v w p];
data_nofilt = [x u v w p];
% Noise filtering
% histogram(mydat(:,7));
[~,pos] = rmoutliers(data_nofilt(:,7),'quartiles');
dat = data_nofilt(~pos,:);
% histogram(filtdata);


idx = find(dat(:,1) >= 0);
mydat = dat(idx,:);
quiver3D(mydat(:,1),mydat(:,3),mydat(:,2),mydat(:,4),mydat(:,6),mydat(:,5),3,'LineWidth',2);
axis equal; view([-1 0 0]); zlim([-5 5]); ylim([-4 20]);
PatchProps.FaceColor = 'k';PatchProps.EdgeColor = 'none';
PatchProps.FaceAlpha = 1;
patch('Faces',FFi, 'Vertices', [VFi(:,1) VFi(:,3) VFi(:,2)], PatchProps);


scatter3(mydat(:,1),mydat(:,3),mydat(:,2),100,mydat(:,7),'filled');axis equal;colormap;

[~,idx,~] = intersect(mydat(:,1:3),Valone,'rows');
scatter3(mydat(idx,1),mydat(idx,3),mydat(idx,2),10,mydat(idx,7),'filled');axis equal;colormap;
PatchProps.FaceColor = 'k';PatchProps.EdgeColor = 'none';hold on;
PatchProps.FaceAlpha = 0.1;
patch('Faces',FFi, 'Vertices', [VFi(:,1) VFi(:,3) VFi(:,2)], PatchProps);









% X
c = find(dat(:,1) > -10000);

gridx = (min(dat(c,1))):0.2:(max(dat(c,1)));
gridy = (min(dat(c,2))):0.2:(max(dat(c,2)));
gridz = (min(dat(c,3))):0.2:(max(dat(c,3)));
spdmod = sqrt(sum((dat(:,4:6)).^2,2));

[actX, actY, actZ] = ndgrid(gridx(:),gridy(:),gridz(:));
mygrid = [actX(:), actY(:), actZ(:)];
gridp = reshape(griddatan([dat(:,1) dat(:,2) dat(:,3)],dat(:,7),mygrid),size(actX));
grids = reshape(griddatan([dat(:,1) dat(:,2) dat(:,3)],spdmod,mygrid),size(actX));
gridp(isnan(gridp)) = 0;


[xlist,ylist,zlist] = deal(actX(:),actY(:),actZ(:));
idpos = find(xlist >= 0);

% PRESIÓN
scatter3(xlist(idpos),zlist(idpos),ylist(idpos),30,gridp(idpos),'filled'); hold on;
scatter3(xlist(idpos),zlist(idpos),ylist(idpos),30,gridp(idpos),'filled');
axis equal; view([-1 0 0]); zlim([-5 5]); ylim([-4 20]);
PatchProps.FaceColor = 'k';PatchProps.EdgeColor = 'none';
PatchProps.FaceAlpha = 1;
patch('Faces',FFi, 'Vertices', [VFi(:,1) VFi(:,3) VFi(:,2)], PatchProps);

% VELOCIDAD
scatter3(xlist(idpos),zlist(idpos),ylist(idpos),30,grids(idpos),'filled'); axis equal;
axis equal; view([-1 0 0]); zlim([-5 5]); ylim([-4 20]);
PatchProps.FaceColor = 'k';PatchProps.EdgeColor = 'none';
PatchProps.FaceAlpha = 1;
patch('Faces',FFi, 'Vertices', [VFi(:,1) VFi(:,3) VFi(:,2)], PatchProps);

% SLICE BASE
figure;
contourslice(actX,actY,actZ,gridp,[],[],-1:0.1:1);hold on;
axis equal; view(3); zlim([-5 5]); ylim([-4 20]);
PatchProps.FaceColor = 'k';PatchProps.EdgeColor = 'none';
PatchProps.FaceAlpha = 1;
patch('Faces',FFi, 'Vertices', [VFi(:,1) VFi(:,2) VFi(:,3)], PatchProps);




% PUNTOS SLICE AVION
posmod = find(VFi(:,2) >= 0);
zetas = griddata(VFOri(posmod,1),VFOri(posmod,3),VFOri(posmod,2),xmod,ymod);
scatter3(xmod(:),ymod(:),zetas(:),'r.');hold on;axis equal;view(3);
zetas = griddata(VFOri(:,1),VFOri(:,3),VFOri(:,2),xmod,ymod);
scatter3(xmod(:),ymod(:),zetas(:),'g.');hold on;axis equal;view(3);



ed = edges(Fm);
plot_edges(Vm,Fm);

tet_mesh_display(-1, size(Vm,1), Vm', 4, size(Tm,1), Tm');




for l = 1:1:size(gridx,2)
    [xlist,ylist,zlist,gridlist] = deal(actX(:),actY(:),actZ(:),grids(:));
    contourf(squeeze(grids(:,l,:)),20);colorbar;axis equal;pause(0.1);
    hold off;
end


for l = 1:1:size(gridx,2)
    [xlist,ylist,zlist,gridlist] = deal(actX(:),actY(:),actZ(:),gridp(:));
    contourf(squeeze(gridp(:,l,:)),20);axis equal;colorbar;pause(0.1);
    hold off;
end


for l = 1:1:size(gridz,2)
    [xlist,ylist,zlist,gridlist] = deal(actX(:),actY(:),actZ(:),gridp(:));
    contourf(squeeze(gridp(:,:,l)));axis equal;colorbar;pause(0.1);
    hold off;
end
