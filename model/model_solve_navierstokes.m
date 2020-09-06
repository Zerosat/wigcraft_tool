%% model_solve_navierstokes
%-------------------------------------------------------------------------------
%  model_solve_navierstokes.m - Solves the 3D Navier-Stokes equations for the
%  given mesh and boundary conditions. This script is highly based on the one 
%  given by Jeff Borggaard, Virginia Tech (2008) "steady_navierstokes_3d.m"
% 
%  Copyright (c) 2020, Pablo J. Rosado Junquera (pabloimie@gmail.com)
% 
%  Version: 1.0
%
%  USAGE:   
% 
%  steady_navierstokes_3d(vertex,pos,limits);
%
%  INPUTS:  
%  
%  x: The mesh vertex list. Typically named "V" or "vertices". 
% 
%  e_conn: The mesh tetrahedron list. Each element contains 4 vertices. 
% 
%  inflow_nodes: A list of indexes corresponding to the inflow elements. 
%
%  outflow_nodes: A list of indexes corresponding to the outflow elements. 
% 
%  wall_nodes: A list of indexes corresponding to the wall (solid) elements. 
% 
%  OUTPUTS:
%
%  The script has no outputs, but instead it creates a file with the solution.  
%-------------------------------------------------------------------------------

function [uvw] = model_solve_navierstokes(x,e_conn,inflow_nodes,outflow_nodes,wall_nodes,sp)
%-------------------------------------------------------------------------------
%  Define "Input" Parameters
%-------------------------------------------------------------------------------
  
	n_gauss = 4;        % number of quadrature points
	max_iterations = 5;  
	max_elem_per_partition = 10000;  % max num elements per partition 10000
	epsilon = 0.0005;   % penalty parameter
	% Re = sp*4.0145/14.88e-6;
    Re = 1;
	mu = 1/Re;
  
%-------------------------------------------------------------------------------
%  Geometry Module
%-------------------------------------------------------------------------------

	nd_val = length([inflow_nodes outflow_nodes wall_nodes]);
  
%-------------------------------------------------------------------------------
%  Solver Module
%-------------------------------------------------------------------------------

  [n_nodes , n_dimensions] = size(x);
  [n_elements, nel_dof] = size(e_conn);
  
  %-----------------------------------------------------------------------------
  %  Determine equation numbers, set up boundary condition information
  %-----------------------------------------------------------------------------
  
	n_diru = 0;
	n_equations = 0;
	ide_u = zeros(n_nodes,3);
	ide_p = zeros(n_nodes,1);
	dir_u = zeros(nd_val,1);

	for n_nd=inflow_nodes
	  r2 = x(n_nd,1)^2 + x(n_nd,2)^2;
	  
	  n_diru = n_diru + 1;
	  ide_u(n_nd,1) = -n_diru;
	  dir_u(n_diru) = 0;
	  
	  n_diru = n_diru + 1;
	  ide_u(n_nd,2) = -n_diru;
	  dir_u(n_diru) = 0;
	  
	  n_diru = n_diru + 1;
	  ide_u(n_nd,3) = -n_diru;
	  dir_u(n_diru) = -sp;
	end

	for n_nd=wall_nodes
	  for j=1:3
		n_diru = n_diru + 1;
		ide_u(n_nd,j) = -n_diru;
		dir_u(n_diru) = 0;
	  end
	end  

	for n_nd=1:n_nodes
	  if (ide_u(n_nd,1)==0)
		for j=1:3
		  n_equations   = n_equations + 1;
		  ide_u(n_nd,j) = n_equations;
		end
	  end
	end

	n_equations_u = n_equations;

	for n_el=1:n_elements
	vertex = e_conn(n_el,1:4);
	for n_c=1:4
	  if (ide_p(vertex(n_c))==0)
		n_equations = n_equations + 1;
		ide_p(vertex(n_c)) = n_equations;
	  end
	end
	end
	n_equations_p = n_equations - n_equations_u;
  
%-------------------------------------------------------------------------------
%  Begin Newton Iterations
%-------------------------------------------------------------------------------

	%  Initial Guess (must satisfy boundary conditions)
	u = zeros(n_nodes,1); 
	v = zeros(n_nodes,1);
	w = zeros(n_nodes,1);

	for n=1:n_nodes
		index = ide_u(n,1);
		if (index<0)
		  u(n) = dir_u(-index);
		end

		index = ide_u(n,2);
		if (index<0)
		  v(n) = dir_u(-index);
		end

		index = ide_u(n,3);
		if (index<0)
		  w(n) = dir_u(-index);
		end
	end
  
	% initialize the pressure
	p = zeros(n_nodes,1);

	converged = 0;
	diverged  = 0;
	iteration = 0;
  
	while (~converged && ~diverged)
  
		iteration = iteration + 1;

		%  Integrate finite element matrices
		
		%  Define mesh partitions for integration
		n_part = floor(n_elements/(max_elem_per_partition+1)) + 1;
		elem_segment = floor(linspace(0,n_elements,n_part+1));
		max_part_size = max(diff(elem_segment));
			
		res  = zeros(n_equations,1); % weak residual vector

		[r,s,t,wt] = threed_gauss(n_gauss);
		one        = ones(length(r),1);

		for n_pt=1:n_part
		  II = zeros(34*34*max_part_size,1);
		  JJ = zeros(34*34*max_part_size,1);
		  XX = zeros(34*34*max_part_size,1);
		  ntriplets = 0;
		

		for n_el=elem_segment(n_pt)+1:elem_segment(n_pt+1)
		  % compute value of each test function and their spatial derivaties
		  % at the integration points
		  nodes_local = e_conn(n_el,:);
		  x_local = x(nodes_local,:);
		  [x_g,wt_g,phi,phi_x,phi_y,phi_z] = threed_shape(x_local,r,s,t,wt);
		  [x_g,wt_g,psi,psi_x,psi_y,psi_z] = threed_shape(x_local(1:4,:),r,s,t,wt);
		  
		  % compute the value of each variable at the Gauss points
		  mu_g = mu*one;
		  epsilon_g = epsilon*one;
		  fx_g = fx_function(x_g,mu);
		  fy_g = fy_function(x_g);
		  fz_g = fz_function(x_g);
		  u_g  = phi*u(nodes_local);
		  u_xg = phi_x*u(nodes_local);
		  u_yg = phi_y*u(nodes_local);
		  u_zg = phi_z*u(nodes_local);   
		  v_g  = phi*v(nodes_local);
		  v_xg = phi_x*v(nodes_local);
		  v_yg = phi_y*v(nodes_local);
		  v_zg = phi_z*v(nodes_local); 
		  w_g  = phi*w(nodes_local);
		  w_xg = phi_x*w(nodes_local);
		  w_yg = phi_y*w(nodes_local);
		  w_zg = phi_z*w(nodes_local);
		  p_g = psi*p(nodes_local(1:4));
		  
		  %-------------------------------------------------------------------------
		  %  Assemble the weak form of the equations (element contributions)
		  %-------------------------------------------------------------------------
		  a11_loc =-threed_bilinear(    2*mu_g, phi_x, phi_x, wt_g) ...
				  - threed_bilinear(      mu_g, phi_y, phi_y, wt_g) ...
				  - threed_bilinear(      mu_g, phi_z, phi_z, wt_g) ...
				  - threed_bilinear(       u_g, phi_x, phi  , wt_g) ...
				  - threed_bilinear(       v_g, phi_y, phi  , wt_g) ...
				  - threed_bilinear(       w_g, phi_z, phi  , wt_g) ...
				  - threed_bilinear(      u_xg, phi  , phi  , wt_g) ;
					
		  a12_loc =-threed_bilinear(      mu_g, phi_x, phi_y, wt_g) ...
				  - threed_bilinear(      u_yg, phi  , phi  , wt_g) ;
		  
		  a13_loc =-threed_bilinear(      mu_g, phi_x, phi_z, wt_g) ...
				  - threed_bilinear(      u_zg, phi  , phi  , wt_g) ;
		  
		  a21_loc =-threed_bilinear(      mu_g, phi_y, phi_x, wt_g) ...
				  - threed_bilinear(      v_xg, phi  , phi  , wt_g) ;
		
		  a22_loc =-threed_bilinear(      mu_g, phi_x, phi_x, wt_g) ...
				  - threed_bilinear(    2*mu_g, phi_y, phi_y, wt_g) ...
				  - threed_bilinear(      mu_g, phi_z, phi_z, wt_g) ...
				  - threed_bilinear(       u_g, phi_x, phi  , wt_g) ...
				  - threed_bilinear(       v_g, phi_y, phi  , wt_g) ...
				  - threed_bilinear(       w_g, phi_z, phi  , wt_g) ...
				  - threed_bilinear(      v_yg, phi  , phi  , wt_g) ;
				  
		  a23_loc =-threed_bilinear(      mu_g, phi_y, phi_z, wt_g) ...
				  - threed_bilinear(      v_zg, phi  , phi  , wt_g) ;
		  
		  a31_loc =-threed_bilinear(      mu_g, phi_z, phi_x, wt_g) ...
				  - threed_bilinear(      w_xg, phi  , phi  , wt_g) ;
				  
		  a32_loc =-threed_bilinear(      mu_g, phi_z, phi_y, wt_g) ...
				  - threed_bilinear(      w_yg, phi  , phi  , wt_g) ;
				  
		  a33_loc =-threed_bilinear(      mu_g, phi_x, phi_x, wt_g) ...
				  - threed_bilinear(      mu_g, phi_y, phi_y, wt_g) ...
				  - threed_bilinear(    2*mu_g, phi_z, phi_z, wt_g) ...
				  - threed_bilinear(       u_g, phi_x, phi  , wt_g) ...
				  - threed_bilinear(       v_g, phi_y, phi  , wt_g) ...
				  - threed_bilinear(       w_g, phi_z, phi  , wt_g) ...
				  - threed_bilinear(      w_zg, phi  , phi  , wt_g) ;
					
		  b1_loc  = threed_bilinear(       one, phi_x, psi  , wt_g) ;
		  b2_loc  = threed_bilinear(       one, phi_y, psi  , wt_g) ;
		  b3_loc  = threed_bilinear(       one, phi_z, psi  , wt_g) ;
		
		  m_loc   = threed_bilinear( epsilon_g./mu_g, psi  , psi  , wt_g) ;
		
		  f1_loc  = threed_f_int(                              fx_g, phi  , wt_g) ...
				  - threed_f_int( u_g.*u_xg + v_g.*u_yg + w_g.*u_zg, phi  , wt_g) ...
				  + threed_f_int(                               p_g, phi_x, wt_g) ...
				  - threed_f_int(                      2*mu_g.*u_xg, phi_x, wt_g) ...
				  - threed_f_int(                 mu_g.*(u_yg+v_xg), phi_y, wt_g) ...
				  - threed_f_int(                 mu_g.*(u_zg+w_xg), phi_z, wt_g) ;
		  
		  f2_loc  = threed_f_int(                              fy_g, phi  , wt_g) ...
				  - threed_f_int( u_g.*v_xg + v_g.*v_yg + w_g.*v_zg, phi  , wt_g) ...
				  + threed_f_int(                               p_g, phi_y, wt_g) ...
				  - threed_f_int(                 mu_g.*(u_yg+v_xg), phi_x, wt_g) ...
				  - threed_f_int(                      2*mu_g.*v_yg, phi_y, wt_g) ...
				  - threed_f_int(                 mu_g.*(v_zg+w_yg), phi_z, wt_g) ;
		  
		  f3_loc  = threed_f_int(                              fz_g, phi  , wt_g) ...
				  - threed_f_int( u_g.*w_xg + v_g.*w_yg + w_g.*w_zg, phi  , wt_g) ...
				  + threed_f_int(                               p_g, phi_z, wt_g) ...
				  - threed_f_int(                 mu_g.*(u_zg+w_xg), phi_x, wt_g) ...
				  - threed_f_int(                 mu_g.*(v_zg+w_yg), phi_y, wt_g) ...
				  - threed_f_int(                      2*mu_g.*w_zg, phi_z, wt_g) ;

		  f4_loc  = threed_f_int(                u_xg + v_yg + w_zg, psi  , wt_g) ...
				  + epsilon/mu*...
					threed_f_int(                               p_g, psi  , wt_g) ;
		  
		  %-------------------------------------------------------------------------
		  %  Assemble contributions into the global system matrix
		  %-------------------------------------------------------------------------
		  
		  % u-momentum equations
		  for n_t=1:4
			n_test = ide_u(nodes_local(n_t),1);
			if (n_test > 0)  % this is an unknown, fill the row
			  for n_u=1:4
				n_unku = ide_u(nodes_local(n_u),1);
				n_unkv = ide_u(nodes_local(n_u),2);
				n_unkw = ide_u(nodes_local(n_u),3);
				if (n_unku > 0)
				  ntriplets = ntriplets + 1;
				  II( ntriplets ) = n_test;
				  JJ( ntriplets ) = n_unku;
				  XX( ntriplets ) = a11_loc(n_t,n_u);
				end
				%
				if (n_unkv > 0)
				  ntriplets = ntriplets + 1;
				  II( ntriplets ) = n_test;
				  JJ( ntriplets ) = n_unkv;
				  XX( ntriplets ) = a12_loc(n_t,n_u);
				end
				%
				if (n_unkw > 0)
				  ntriplets = ntriplets + 1;
				  II( ntriplets ) = n_test;
				  JJ( ntriplets ) = n_unkw;
				  XX( ntriplets ) = a13_loc(n_t,n_u);
				end
			  end
			  for n_p=1:4
				n_unkp = ide_p(nodes_local(n_p));

				ntriplets = ntriplets + 1;
				II( ntriplets ) = n_test;
				JJ( ntriplets ) = n_unkp;
				XX( ntriplets ) = b1_loc(n_p,n_t);
			  end
			  res(n_test) = res(n_test) + f1_loc(n_t);
			end
		  end

		  
		  % v-momentum equations
		  for n_t=1:4
			n_test = ide_u(nodes_local(n_t),2);
			if (n_test > 0)  % this is an unknown, fill the row
			  for n_u=1:4
				n_unku = ide_u(nodes_local(n_u),1);
				n_unkv = ide_u(nodes_local(n_u),2);
				n_unkw = ide_u(nodes_local(n_u),3);
				if (n_unku > 0)
				  ntriplets = ntriplets + 1;
				  II( ntriplets ) = n_test;
				  JJ( ntriplets ) = n_unku;
				  XX( ntriplets ) = a21_loc(n_t,n_u);
				end
				%
				if (n_unkv > 0)
				  ntriplets = ntriplets + 1;
				  II( ntriplets ) = n_test;
				  JJ( ntriplets ) = n_unkv;
				  XX( ntriplets ) = a22_loc(n_t,n_u);
				end
				%
				if (n_unkw > 0)
				  ntriplets = ntriplets + 1;
				  II( ntriplets ) = n_test;
				  JJ( ntriplets ) = n_unkw;
				  XX( ntriplets ) = a23_loc(n_t,n_u);
				end
			  end
			  for n_p=1:4
				n_unkp = ide_p(nodes_local(n_p));

				ntriplets = ntriplets + 1;
				II( ntriplets ) = n_test;
				JJ( ntriplets ) = n_unkp;
				XX( ntriplets ) = b2_loc(n_p,n_t);
			  end
			  res(n_test) = res(n_test) + f2_loc(n_t);
			end
		  end

		  % w-momentum equations
		  for n_t=1:4
			n_test = ide_u(nodes_local(n_t),3);
			if (n_test > 0)  % this is an unknown, fill the row
			  for n_u=1:4
				n_unku = ide_u(nodes_local(n_u),1);
				n_unkv = ide_u(nodes_local(n_u),2);
				n_unkw = ide_u(nodes_local(n_u),3);
				if (n_unku > 0)
				  ntriplets = ntriplets + 1;
				  II( ntriplets ) = n_test;
				  JJ( ntriplets ) = n_unku;
				  XX( ntriplets ) = a31_loc(n_t,n_u);
				end
				%
				if (n_unkv > 0)
				  ntriplets = ntriplets + 1;
				  II( ntriplets ) = n_test;
				  JJ( ntriplets ) = n_unkv;
				  XX( ntriplets ) = a32_loc(n_t,n_u);
				end
				%
				if (n_unkw > 0)
				  ntriplets = ntriplets + 1;
				  II( ntriplets ) = n_test;
				  JJ( ntriplets ) = n_unkw;
				  XX( ntriplets ) = a33_loc(n_t,n_u);
				end
			  end
			  for n_p=1:4
				n_unkp = ide_p(nodes_local(n_p));

				ntriplets = ntriplets + 1;
				II( ntriplets ) = n_test;
				JJ( ntriplets ) = n_unkp;
				XX( ntriplets ) = b3_loc(n_p,n_t);
			  end
			  res(n_test) = res(n_test) + f3_loc(n_t);
			end
		  end

		  % continuity equations
		  for n_t=1:4
			n_test = ide_p(nodes_local(n_t));
			for n_u=1:4
			  n_unku = ide_u(nodes_local(n_u),1);
			  n_unkv = ide_u(nodes_local(n_u),2);
			  n_unkw = ide_u(nodes_local(n_u),3);
			  if (n_unku > 0)
				ntriplets = ntriplets + 1;
				II( ntriplets ) = n_test;
				JJ( ntriplets ) = n_unku;
				XX( ntriplets ) = b1_loc(n_t,n_u);
			  end
			  %
			  if (n_unkv > 0)
				ntriplets = ntriplets + 1;
				II( ntriplets ) = n_test;
				JJ( ntriplets ) = n_unkv;
				XX( ntriplets ) = b2_loc(n_t,n_u);
			  end
			  %
			  if (n_unkw > 0)
				ntriplets = ntriplets + 1;
				II( ntriplets ) = n_test;
				JJ( ntriplets ) = n_unkw;
				XX( ntriplets ) = b3_loc(n_t,n_u);
			  end
			end
			for n_p=1:4
			  n_unkp = ide_p(nodes_local(n_p));

			  ntriplets = ntriplets + 1;
			  II( ntriplets ) = n_test;
			  JJ( ntriplets ) = n_unkp;
			  XX( ntriplets ) = m_loc(n_t,n_p);
			end
			res(n_test) = res(n_test) + f4_loc(n_t);
		  end

		end % element loop
		
		if ( n_pt==1 )
		  J = sparse( II(1:ntriplets), JJ(1:ntriplets), XX(1:ntriplets), ...
					 n_equations, n_equations);
		else
		  J = J + sparse( II(1:ntriplets), JJ(1:ntriplets), XX(1:ntriplets), ...
					 n_equations, n_equations);
		end
		
		end % part
		
		clear II JJ XX

		%  Direct solve (for small system sizes)
		
		uvw = -J\res;

		% setup.type='nofill';   % to save storage
		% [JL,JU] = ilu(J,setup);
		% uvw = gmres(J,-res,50,1e-6,500,JL,JU);
		
		for n=1:n_nodes
		  index = ide_u(n,1);
		  if (index>0)
			u(n) = u(n) + uvw(index);
		  end

		  index = ide_u(n,2);
		  if (index>0)
			v(n) = v(n) + uvw(index);
		  end

		  index = ide_u(n,3);
		  if (index>0)
			w(n) = w(n) + uvw(index);
		  end
		end

		for n=1:n_nodes
		  index = ide_p(n,1);
		  if (index>0)
			p(n) = p(n) + uvw(index);
		  end
		end

		resid_test = norm(res);
		step_test  = norm(uvw);
		% converged = ( resid_test < 1e-5 & step_test < 1e-5 );
		
		converged = ( resid_test < 1e-9 & step_test < 1e-9 );
		
		% converged = ( resid_test < 1e-15 & step_test < 1e-15 );
		
		diverged  = iteration>max_iterations;
    
	
  end % Newton iterations
  
  save ns_3d  x e_conn u v w p

end % function
  
%-------------------------------------------------------------------------------
%  Supporting Functions
%-------------------------------------------------------------------------------

function f = fx_function(x,mu)
  f = zeros(length(x),1);
end

function f = fy_function(x)
  f = zeros(length(x),1);
end

function f = fz_function(x)
  f = zeros(length(x),1);
end

function u = d_function(x)
  u = 16*x(:,2).*(1-x(:,2))*x(:,3)*(1-x(:,3));
end
