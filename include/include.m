%% include
%-------------------------------------------------------------------------------
%  include.m - Includes to the path all needed folders to use the model. 
% 
%  Copyright (c) 2020, Pablo J. Rosado Junquera (pabloimie@gmail.com)
% 
%  Version: 1.0
%
%  USAGE:   
% 
%  include();
%-------------------------------------------------------------------------------

function [] = include()
    % Include
    addpath('geometry\surround');
	addpath('geometry\vehicle');
    addpath('geometry\others');
    addpath('model\');
	addpath('parties\matlab');
	addpath('parties\c++');
    addpath('tools\');
    addpath('results\');
end

