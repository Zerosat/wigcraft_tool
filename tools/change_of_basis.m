%% CHANGE_OF_BASIS()
%-------------------------------------------------------------------------------
%  change_of_basis.m - Changes the basis of a given vector xi to B1. The
%  algorithm presumes the given data is represented in the cartesian basis. 
% 
%  Copyright (c) 2020, Pablo J. Rosado Junquera (pabloimie@gmail.com)
% 
%  Version: 1.0
%
%  USAGE:   
% 
%  xo = change_of_basis(xi,B0,B1);
%
%  INPUTS:  
%  
%  xi: The vector to be projected.  
% 
% 
%  max: The final basis. 
% 
%  OUTPUTS:
%
%  xo: The projected vector
%  
%-------------------------------------------------------------------------------

function xo = change_of_basis(xi,B1)
    
    xo = inv(B1)*xi;
    
end