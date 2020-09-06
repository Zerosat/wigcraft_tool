function area = get_projected_area(doiplot)

    if (nargin<1)
        doiplot = false;
    end
    
    % Include
    addpath('include\');
    include();

    % Load the required parameters
    partable = readtable('config.dat','HeaderLines',6,'Delimiter','semi','ReadRowNames',true);
    parameters = join(erase(string(partable{:, :}), "'"), '', 2);

    % File loading
    vehicle = stlread(parameters(1));

    % Vehicle model creation
    [~, ~, centroid, ~] = inertia_tensor(vehicle.Points,vehicle.ConnectivityList);
    VFo = (vehicle.Points - centroid);

    k = boundary(VFo(:,1),VFo(:,3));
    area = polyarea(VFo(k,1),VFo(k,3));
    
    if (doiplot)
        hold on;
        % plot(VFo(:,1),VFo(:,3),'k.'); axis equal;
        plot(VFo(k,1),VFo(k,3),'k-','Linewidth',1); axis equal;
    end
    
end