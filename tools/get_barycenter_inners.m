%% GET_BARYCENTER_INNERS()
%-------------------------------------------------------------------------------
%  get_barycenter_inners.m - Extract some keypoints from a given triangle. 
% 
%  Copyright (c) 2020, Pablo J. Rosado Junquera (pabloimie@gmail.com)
% 
%  Version: 1.0
%
%  USAGE:   
% 
%  P = get_barycenter_inners(vertex,N);
%
%  INPUTS:  
%  
%  vertex: A 3x3 array containing the three vertexes of the triangle in XYZ coordinates in rows. 
% 
%  N: the number of times the algorithm will be executed. 
%  WARNING! A number beyond 9 will generate a huge amount of points (>10M). 
% 
%  OUTPUTS:
%
%  P: A Nx3 array containing the resulting points. 

function [innerpoints] = get_barycenter_inners(V,num)
    if (num == 0)
        innerpoints = [];
    else
        B = barycenter(V,[1 2 3]);
        M1 = (V(1,:) + V(2,:))/2;
        M2 = (V(2,:) + V(3,:))/2;
        M3 = (V(1,:) + V(3,:))/2;
		[k,vs,ms] = deal(cell(6,1),[1 3 3 2 2 1],{M3,M2,M1});
		for j = 1:1:6
			k{j} = get_barycenter_inners([V(vs(j),:);B;ms{ceil(j/2)}],num-1);
		end
        innerpoints = unique([V;B;M1;M2;M3;k{1};k{2};k{3};k{4};k{5};k{6}],'rows');
    end
end