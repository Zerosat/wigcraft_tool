%% AIRCUBE_SIZING()
%-------------------------------------------------------------------------------
%  aircube_sizing.m - Edits a given cube mesh to fit specified dimensions.
%  The algorithm presumes the cube is unitary and centered in (0,0,0). 
% 
%  Copyright (c) 2020, Pablo J. Rosado Junquera (pabloimie@gmail.com)
% 
%  Version: 1.0
%
%  USAGE:   
% 
%  new_vertex = aircube_sizing(vertex,pos,limits);
%
%  INPUTS:  
%  
%  vertex: The initial points, in the form of a Nx3 matrix, being N the
%  total number of points. Each row is the(x,y,z) coordinate of the point.
% 
%  pos: the desired cube geometric center, expressed in [x y z]. 
% 
%  limits: A 1x6 matrix containing the desired dimensions. Contents: 
%  [xmax, ymax, zmax, xmin, ymin, zmin]
%  xmax, xmin: The cube bounds in the x axis. Width is xmax + xmin (abs)
%  ymax, ymin: The cube bounds in the y axis. Depth is ymax + ymin (abs)
%  zmax, zmin: The cube bounds in the z axis. Height is zmax + zmin (abs)
% 
%  OUTPUTS:
%
%  new_vertex: The processed new points. 
%-------------------------------------------------------------------------------

function new_vertex = aircube_sizing(vertex,pos,lim)
    
    % Substitute our points
    [xx,yy,zz] = deal(vertex(:,1),vertex(:,2),vertex(:,3));
    new_vertex(:,1) = -clamp(sign(xx),-1,0)*lim(4) + clamp(sign(xx),0,1)*lim(1) + pos(1);
    new_vertex(:,2) = -clamp(sign(yy),-1,0)*lim(5) + clamp(sign(yy),0,1)*lim(2) + pos(2);
    new_vertex(:,3) = -clamp(sign(zz),-1,0)*lim(6) + clamp(sign(zz),0,1)*lim(3) + pos(3);
    
end
