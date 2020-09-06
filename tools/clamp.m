%% CLAMP()
%-------------------------------------------------------------------------------
%  clamp.m - Ensures a specified value is within the interval (min,max). If
%  that condition is not met, the algorithm coerces "val" to max or min. 
% 
%  Copyright (c) 2020, Pablo J. Rosado Junquera (pabloimie@gmail.com)
% 
%  Version: 1.0
%
%  USAGE:   
% 
%  new_val = clamp(val,min,max);
%
%  INPUTS:  
%  
%  val: A matrix with the queried values. 
% 
%  min: The lower limit of the interval (scalar).
% 
%  max: The upper limit of the interval (scalar). 
% 
%  OUTPUTS:
%
%  new_val: The coarced values. Cases: 
%  
%  - If min < val < max, new_val = val
%  - If val < min, new_val = min
%  - If val > max, new_val = max
%-------------------------------------------------------------------------------

function new_val = clamp(val,min,max)
    
    new_val = zeros(size(val));
    for k = 1:1:length(val)
        curval = val(k);
        if (curval >= min) && (curval <= max)
            new_val(k) = curval;
        elseif (curval < min)
            new_val(k) = min;
        else
            new_val(k) = max;
        end
    end
    
end