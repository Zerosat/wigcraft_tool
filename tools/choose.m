%% CHOOSE()
%-------------------------------------------------------------------------------
%  choose.m - Returns one on the provided value. The chance of chosing a value
%  is the same for every one.
% 
%  Copyright (c) 2020, Pablo J. Rosado Junquera (pabloimie@gmail.com)
% 
%  Version: 1.0
%
%  USAGE:   
% 
%  val = choose(a,b,c,...);
%
%  INPUTS:  
%  
%  a,b,c,...: The value pool to choose from.  
%  
% 
%  OUTPUTS:
%
%  val: The chosen value
%  
%-------------------------------------------------------------------------------
function val = choose(varargin)

    % Pick randomnly an argument
    idx = randi(nargin, 1);
    val = varargin{idx};
    
end