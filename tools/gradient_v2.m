function val = gradient_v2(field,ix,iy,iz,step)
    
    % X AXIS
    [in1,~,inn1] = get_gradient_points(size(field,1),ix);
    dx = (field(in1,iy,iz)-field(inn1,iy,iz))/2*step;
    % Y AXIS
    [in1,~,inn1] = get_gradient_points(size(field,2),iy);
    dy = (field(ix,in1,iz)-field(ix,inn1,iz))/2*step;
    % Z AXIS
    [in1,~,inn1] = get_gradient_points(size(field,3),iz);
    dz = (field(ix,iy,in1)-field(ix,iy,inn1))/2*step;
    
    val = [dx dy dz];
    
end