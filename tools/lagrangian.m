function val = lagrangian(field,ix,iy,iz,step)

    % X AXIS
    [in2,in1,in0,inn1,inn2] = get_lagrangian_points(size(field,1),ix);
    d2x = (-field(in2,iy,iz)+16*field(in1,iy,iz)-30*field(in0,iy,iz)+16*field(inn1,iy,iz)-field(inn2,iy,iz))/(12*step^2);
    
    % Y AXIS
    [in2,in1,in0,inn1,inn2] = get_lagrangian_points(size(field,2),iy);
    d2y = (-field(ix,in2,iz)+16*field(ix,in1,iz)-30*field(ix,in0,iz)+16*field(ix,inn1,iz)-field(ix,inn2,iz))/(12*step^2);
    
    % Z AXIS
    [in2,in1,in0,inn1,inn2] = get_lagrangian_points(size(field,3),iz);
    d2z = (-field(ix,iy,in2)+16*field(ix,iy,in1)-30*field(ix,iy,in0)+16*field(ix,iy,inn1)-field(ix,iy,inn2))/(12*step^2);
    
    val = [d2x d2y d2z];
end