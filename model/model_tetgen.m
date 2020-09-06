%% model_tetgen
%-------------------------------------------------------------------------------
%  model_tetgen.m - Configure and calls TETGEN to generate a tetrahedra mesh 
%  for the given model. Script highly based on the one given by jburkardt, 
%  "tetgen.m".  
% 
%  Copyright (c) 2020, Pablo J. Rosado Junquera (pabloimie@gmail.com)
% 
%  Version: 1.0
%
%  USAGE:   
% 
%  [V,T,F] = model_tetgen(SV,SF,userdefp,sizing_nodes,sizing_elements,'ParameterName',ParameterValue, ...)
%
%  INPUTS:  
%  
%  SV: List of surface vertex positions of exterior mesh, # vertices by 3
% 
%  SF: List of surface face indices of exterior triangle mesh, # faces by 3
% 
%  userdefp: List of internal contrained vertices
% 
%  sizing_nodes: List of nodes used to apply a custom sizing function
%
%  sizing_elements: List of elements used to apply a custom sizing function
% 
%  OUTPUTS:
% 
%  V:  List of tetrahedra vertices
% 
%  T:  List of tetrahedra indices
% 
%  F:  List of faces of 3D volume mesh
%-------------------------------------------------------------------------------

function [V,T,F] = model_tetgen(SV,SF,userdefp,sizing_nodes,sizing_elements,varargin)

  if ~isempty(varargin)
    assert(ischar(varargin{1}), ...
      'First optional arg not char.');
  end
  
  SH = [];
  % default values
  flags = '-q2';
  verbose = false;
  % Map of parameter names to variable names
  params_to_variables = containers.Map( ...
    {'Flags','Holes','Verbose'}, {'flags','SH','verbose'});
  v = 1;
  while v <= numel(varargin)
    param_name = varargin{v};
    if isKey(params_to_variables,param_name)
      assert(v+1<=numel(varargin));
      v = v+1;
      % Trick: use feval on anonymous function to use assignin to this workspace 
      feval(@()assignin('caller',params_to_variables(param_name),varargin{v}));
    else
      error('Unsupported parameter: %s',varargin{v});
    end
    v=v+1;
  end

  % get a temporary file name prefix
  %off_filename = [prefix '.off'];
  %writeOFF(off_filename,SV,SF);
  
  % Try to mesh with all faces included directly

  prefix = tempname;
  
 
  % Write the userdef points file
  writeNODE([prefix '.a.node'],userdefp);
  
  poly_filename = [prefix '.poly'];
  writePOLY_tetgen(poly_filename,SV,SF,SH,'BoundaryMarkers',ones(size(SF,1),1));
  mesh_flags = '-Cpg ';
  mesh_flags = '';
  
  
  % Write the sizing function files
  if (~isempty(sizing_nodes) && ~isempty(sizing_elements))
	
	% Compute the function
	sizes = 0.1*clamp(sqrt(sizing_nodes(:,1).^2 + sizing_nodes(:,2).^2 + sizing_nodes(:,3).^2),0.0001,10);
	
	% Write the files
	mtr_node_filename = [prefix '.b.node'];
	mtr_ele_filename = [prefix '.b.ele'];
	mtr_siz_filename = [prefix '.b.mtr'];
	
	writeNODE(mtr_node_filename,sizing_nodes);
	writeELE(mtr_ele_filename,sizing_elements);
	write_mtr(mtr_siz_filename,sizes);
	
  end
  
  % call tetgen
  command = ['C:\dev\tetgen\build\Debug\tetgen.exe' ' ' mesh_flags ' ' flags ' ' poly_filename];
  if verbose
    fprintf('%s\n',command);
  end
  [status, result] = system(command);
  if status~=0
    % warning(command)
    error(result)
  elseif verbose
    fprintf('%s\n',result);
  end
  % tetgen always writes output to file:
  %   xxxx.1.ele  tetrahedra
  %   xxxx.1.node tetrahedra vertices
  %   xxxx.1.face  surface faces
  ele_filename = [prefix '.1.ele'];
  face_filename = [prefix '.1.face'];
  node_filename = [prefix '.1.node'];

  if exist(face_filename,'file')
    F = readFACE(face_filename);
    % reverse faces because tetgen uses backwards order
    F = fliplr(F);
  end
  % I guess this is 1-indexed because we're using a .off file rather than a
  % .poly file
  T = readELE(ele_filename);
  V = readNODE(node_filename);
  if min(T(:)) == 0 && max(T(:))<size(V,1)
    error
    % make 1-indexed
    T = T + 1;
  else if min(T(:)) >= 1
    %warning('min(T) >= 1, leaving indices as is');
  end

  delete(poly_filename);
  %if(internal_constraints)
  %  delete(inode_filename);
  %end
  delete(ele_filename);
  if exist(face_filename,'file')
    delete(face_filename);
  end
  delete(node_filename);
  delete([prefix '.a.node']);
  delete([prefix '.1.edge']);
  if isfile([prefix '.1.mesh'])
    delete([prefix '.1.mesh']);
  end
  if (~isempty(sizing_nodes) && ~isempty(sizing_elements))
	delete([prefix '.b.node']);
	delete([prefix '.b.ele']);
	delete([prefix '.b.mtr']);
  end
end