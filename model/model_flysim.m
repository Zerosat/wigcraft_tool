%% model_flysim

function [Fsum,Msum,field] = model_flysim(VCi,FCi,keycubes,VFi,FFi,ails,airspd,precision,toggleplot)

    %% PROBLEM DEFINITION AND SOLVING
    
    % Perform solid operations
    [VAir,FAir] = mesh_boolean(VCi,FCi,VFi,FFi,'minus');
    [mVAir,mFAir] = deal(VCi,FCi);
    for k = 1:1:size(keycubes,1)
        [mVAir,mFAir] = mesh_boolean(mVAir,mFAir,keycubes{k},FCi,'minus');
    end
    
    % Subtract the ailerons
    for r = 1:1:(size(ails,2)/2)
        [VAir,FAir] = mesh_boolean(VAir,FAir,ails{r},ails{r + size(ails,2)/2},'minus');
    end
    
    %{
    PatchProps.FaceColor = 'r';PatchProps.EdgeColor = 'none';light;
    patch('Faces', FFi, 'Vertices',[VFi(:,1) VFi(:,3) VFi(:,2)], PatchProps); hold on; axis equal;
    set(gca,'CameraPosition',[-21.168 25.68 17.235]);set(gca,'XColor', 'none','YColor','none','ZColor', 'none');
    fhgj
    %}
    
    %{
    PatchProps.FaceColor = 'b';PatchProps.EdgeColor = 'none';light;
    patch('Faces', FAir, 'Vertices',[VAir(:,1) VAir(:,3) VAir(:,2)], PatchProps); hold on; axis equal;
    set(gca,'CameraPosition',[-21.168 25.68 17.235]);set(gca,'XColor', 'none','YColor','none','ZColor', 'none');
    jkghjk
    %}
    
    
    % Repair the mesh (Removed: it causes Tetgen to fail in certain situations)
    % [VAir,FAir,~] = clean_mesh(VAir,FAir);
    
    % Generate the mesh
    [Vextra, Textra, ~] = model_tetgen(mVAir,mFAir,[],[],[],'Flags','-pq1.1a0.5');
    %{
    pdemesh([Vextra(:,1) Vextra(:,3) Vextra(:,2)]',Textra');
    title(strcat("Problem with ",num2str(size(Textra,1))," elements"));
    set(gca,'CameraPosition',[-21.168 25.68 17.235]);set(gca,'XColor', 'none','YColor','none','ZColor', 'none');
    sdgdfg
    %}
    
    [Valone, ~, ~] = model_tetgen(VAir,FAir,[],[],[],'Flags',['-T1e-9-a' num2str(precision) '-S0']);
    Vextra = setdiff(Vextra,VAir,'rows');
    insidx = inpolyhedron(FFi,VFi,Vextra);
    Vextra(insidx == 1,:) = [];
    [Vm, Tm, ~] = model_tetgen(VAir,FAir,Vextra,[],[],'Flags',['-T1e-9-a' num2str(precision) '-S0-i']);
    
    % Compute boundary conditions
        % Casing
    [~, fish_coll, ~] = intersect(Vm,Valone,'rows');
    casing = [find((Vm(:,1)) == min((Vm(:,1))));find((Vm(:,1)) == max((Vm(:,1))));
              find((Vm(:,2)) == min((Vm(:,2))));find((Vm(:,2)) == max((Vm(:,2))))];
    walls = [fish_coll;casing];
    
        % Inlet and outlet
    inflow = setdiff(find((Vm(:,3) == max((Vm(:,3))))),casing);
    outflow = setdiff(find((Vm(:,3) == min((Vm(:,3))))),casing);
    
    %{
    pdemesh([Vm(:,1) Vm(:,3) Vm(:,2)]',Tm');
    title(strcat("Problem with ",num2str(size(Tm,1))," elements"));
    set(gca,'CameraPosition',[-21.168 25.68 17.235]);set(gca,'XColor', 'none','YColor','none','ZColor', 'none');
    sdgdfg
    %}
   
    %{
    hold on;
    scatter3(Vm(:,1),Vm(:,3),Vm(:,2),10,[0.55 0.54 0.54],'.');
    scatter3(Vm(walls,1),Vm(walls,3),Vm(walls,2),2,'r.');
    scatter3(Vm(inflow,1),Vm(inflow,3),Vm(inflow,2),30,[0.9290, 0.6940, 0.1250],'^','filled');
    scatter3(Vm(outflow,1),Vm(outflow,3),Vm(outflow,2),30,[0, 0.4470, 0.7410],'^','filled');
    axis equal;view(3);xlabel('X axis');ylabel('Y axis');zlabel('Z axis');
    set(gca,'CameraPosition',[-21.168 25.68 17.235]);set(gca,'XColor', 'none','YColor','none','ZColor', 'none');
    sdsdf
    %}
    disp(strcat("[model_flysim]: solving problem with ",num2str(size(Tm,1))," elements...")); pause(0.001);
    field = model_solve_navierstokes(Vm,Tm,inflow',outflow',walls',airspd);
    disp("[model_flysim]: problem solved. Computing pressure..."); pause(0.001);
    
    %% PRESSURE COMPUTATION FACETS

    % Compute area of triangle in 3D coordinates
    % The 1ast point is given by the 1ast index in the facet list, and so on

    % tol = 0.005; % is the minimum valid area
    tol = 0;
    [st, nd, rd, Fcont] = deal(VFi(FFi(:,1),:),VFi(FFi(:,2),:),VFi(FFi(:,3),:),1);

    for plo = 1:1:size(FFi,1)
        [xi,yi,zi] = deal([st(plo,1),nd(plo,1),rd(plo,1)],...
        [st(plo,2),nd(plo,2),rd(plo,2)],[st(plo,3),nd(plo,3),rd(plo,3)]);
        ons = ones(1,size(xi,2));
        A(plo) = 0.5*sqrt(det([xi;yi;ons])^2 + det([yi;zi;ons])^2 + det([zi;xi;ons])^2);

        if A(plo) >= tol
            col = 'w';
            Vcent(Fcont,:) = [xi,yi,zi];
            Fsel(Fcont,:) = FFi(plo,:);
            Asel(Fcont) = A(plo);
            Fcont = Fcont + 1;
        else
            col = 'r';
        end
        if (toggleplot)
            hold on;
            fill3(xi,yi,zi,col);
            axis equal;
        end
    end

    %% PRESSURE COMPUTATION POINTS

    shift = 0.3; % is the gap vehicle - calculation point
    dir = COMPUTE_mesh_normals(struct('vertices',VFi,'faces',Fsel));
    
    % Computation points
    Vcomp = [mean(Vcent(:,1:3),2),mean(Vcent(:,4:6),2),mean(Vcent(:,7:9),2)] + dir*shift;

    % scatter3(Vcomp(:,1),Vcomp(:,2),Vcomp(:,3),'b');
    % quiver3(mean(Vcent(:,1:3),2),mean(Vcent(:,4:6),2),mean(Vcent(:,7:9),2),dir(:,1),dir(:,2),dir(:,3),1,'LineWidth',1,'Color','b');

    % Check if there are points inside the model. Remove them from the calculation array. 
    insidx = inpolyhedron(FFi,VFi, Vcomp);
    Vdefc = Vcomp; Vdefc(insidx == 1,:) = [];
    dirdefc = -dir; dirdefc(insidx == 1,:) = [];
    Aseldefc = Asel; Aseldefc(insidx == 1) = [];

    %% COMPUTE FORCES AND MOMENTS
    
    load('ns_3d.mat');
    dat = [x u v w p];

    % Noise filtering (Old, it uses the default sub-solved pressure)
    [~,pos] = rmoutliers(dat(:,7),'percentiles',[30 95]);
    dat = dat(~pos,:);
    izero = find(dat(:,7) < 0);
    dat(izero,:) = [];
    
    % Remove the NaN values
    idNaN = find(isnan(dat(:,7)));
    dat(idNaN,:) = [];
    
    % METHOD 1 (It is more accurate theoretically, but not in practice)
    %{
    barpoints = 3;
    id = setdiff(1:1:size(Vcomp,1),find(insidx == 1));
    dims = [7 25 121 673];
    N = [0 0 0];
    for g = 1:1:size(Vdefc,1)
        cutr = [Vcent(id(g),[1 4 7]);Vcent(id(g),[2 5 8]);Vcent(id(g),[3 6 9])];
        N = [N;get_barycenter_inners(cutr,barpoints)];
    end
    N(1,:) = [];
    Pmed = mean(griddatan([dat(:,1) dat(:,2) dat(:,3)],dat(:,7),N),2);
    for pm = 1:1:(length(Pmed))/dims(barpoints)
        Pdefc(pm) = mean(Pmed(pm*dims(barpoints)-(dims(barpoints)-1):pm*dims(barpoints)));
    end
    %}
    
    % METHOD 2 (Faster, while less accurate)
    Pdefc = griddatan(dat(:,1:3),dat(:,7),Vdefc);
    
    % FORCE AND MOMENT COMPUTATION
    Fdefc = dirdefc.*(Aseldefc(:)).*(Pdefc(:));
    Fsum = sum(Fdefc);
    rad = Vdefc-dirdefc*shift;
    Mdefc = cross(rad,Fdefc,2);
    Msum = sum(Mdefc);
    
    %% PLOT THE RESULTS (if asked to do so)
    
    if (toggleplot)
        quiver3(Vdefc(:,1),Vdefc(:,2),Vdefc(:,3),rad(:,1),rad(:,2),rad(:,3),'Color',[0.9290 0.6940 0.1250]);
        quiver3(Vdefc(:,1),Vdefc(:,2),Vdefc(:,3),-Fdefc(:,1),-Fdefc(:,2),-Fdefc(:,3),'Color','m');
        quiver3(0,0,0,Fsum(:,1),Fsum(:,2),Fsum(:,3),'Color','r');
        quiver3(0,0,0,Msum(:,1),Msum(:,2),Msum(:,3),'Color','b');
        zlim([-5 5]);xlim([-5 5]);ylim([-5 5]);
        fgfgh;
    end 

end