function val = get_median_value(field,ix,iy,iz)
    % X dir
    xp = clamp(ix+1,1,size(field,1));
    xn = clamp(ix-1,1,size(field,1));
    % Y dir
    yp = clamp(iy+1,1,size(field,2));
    yn = clamp(iy-1,1,size(field,2));
    % Zdir
    zp = clamp(iz+1,1,size(field,3));
    zn = clamp(iz-1,1,size(field,3));
    
    val = mean([field(xp,iy,iz)
                field(xn,iy,iz)
                field(ix,yp,iz)
                field(ix,yn,iz)
                field(ix,iy,zp)
                field(ix,iy,zn)]);
end