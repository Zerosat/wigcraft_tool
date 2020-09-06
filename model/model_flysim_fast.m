 function [curforce curmoment] = model_flysim_fast(curspd,curpitch,curyaw,curroll,curbackangle,RPY,m_40,m_m40)
	% We do know the current speed and pitch

	%% LIFT COMPUTATION
	% Compute the lift for the current pitch
	lift_star = liftstar(curpitch,curspd);
	% Compute the amplification factor D
	D = clamp(abs(liftstar(curpitch,curspd))/abs(liftstar(curpitch,5)),1,100);
	% Compute the aileron correction
	
	val40 = unique(sortrows([m_40(:,7) m_40(:,2)],1,'ascend'),'rows');
	valm40 = unique(sortrows([m_m40(:,7) m_m40(:,2)],1,'ascend'),'rows');
	val40(:,2) = movmean(val40(:,2),5);
	valm40(:,2) = movmean(valm40(:,2),5);
	ms = (val40(:,2) - valm40(:,2))/80;
	ns = 0.5*(val40(:,2) + valm40(:,2));
	curms = interp1(m_40(:,7),ms,clamp(curpitch,-20,20));
	curns = interp1(m_40(:,7),ns,clamp(curpitch,-20,20));
	delta_L = curms*curbackangle + curns;
	% Compute the correction factor
	
	RPY_val = [RPY(:,[2,7,8,9])];
	num = griddatan(RPY_val(:,2:4),movmean(RPY_val(:,1),3),[clamp(curpitch,-20,20) clamp(curyaw,-20,20) clamp(curroll,-20,20)]);
	den = griddatan(RPY_val(:,2:4),movmean(RPY_val(:,1),3),[clamp(curpitch,-20,20) 0 0]);
	F = num/den;
	% Compute the lift
	curlift = (lift_star + delta_L*D)*F;
    
    
	%% LAT COMPUTATION
	% Compute the lat for the current pitch
	lat_star = latstar(curpitch,curspd);
	% Compute the aileron correction
	delta_La = 0;
	% Compute the correction factor
	RPY_vala = [RPY(:,[1,7,8,9])];
	numa = griddatan(RPY_vala(:,2:4),movmean(RPY_vala(:,1),3),[clamp(curpitch,-20,20) clamp(curyaw,-20,20) clamp(curroll,-20,20)]);
	dena = griddatan(RPY_vala(:,2:4),movmean(RPY_vala(:,1),3),[clamp(curpitch,-20,20) 0 0]);
	Fa = numa/dena;
	curlat = (lat_star + delta_La*D)*Fa;

	%% DRAG COMPUTATION
	% Compute the drag for the current pitch
	curdrag = dragstar(curpitch,curspd);
    
    
	%% PITCHING MOMENT COMPUTATION
	% Compute the aileron correction (a line deducted after some simulations)
	mback = 0.01*curbackangle - 0.1;
	delta_R = mback*clamp(curpitch,-20,20);
	% Compute the correction factor (base pitching moment)
	RPY_valr = [RPY(:,[4,7,8,9])];
	Fr = griddatan(RPY_valr(:,2:4),movmean(RPY_valr(:,1),3),[clamp(curpitch,-20,20) clamp(curyaw,-20,20) clamp(curroll,-20,20)]);
	curpitch_moment = Fr + delta_R*D;
	% curpitch_moment = 0;

	%% ROLLING MOMENT COMPUTATION
	curroll_moment = 0;

	%% YAWING MOMENT COMPUTATION
	curyaw_moment = 0;

    
	curforce = [curlat curlift curdrag];
	curmoment = [curpitch_moment curyaw_moment curroll_moment];



	% plot results (delta_L)
	%{
	kkk = -100:1:100;
	A = ms*kkk + ns;
	hold on;
	for l = 1:1:size(A,1)
	plot(kkk,A(l,:));
	end
	plot(curpitch,delta_L,'ro');
    dghfjf
	%}
	% plot results (LIFT* interp)
	%{
	hold on;
	c = readmatrix('lift_vs_pitch_5ms');
	plot(c(:,1),c(:,2));
	c = readmatrix('lift_vs_pitch_10ms');
	plot(c(:,1),c(:,2));
	c = readmatrix('lift_vs_pitch_15ms');
	plot(c(:,1),c(:,2));
	c = readmatrix('lift_vs_pitch_20ms');
	plot(c(:,1),c(:,2));
	c = readmatrix('lift_vs_pitch_30ms');
	plot(c(:,1),c(:,2));
	plot(lift_vs_pitch_cursp(:,1),lift_vs_pitch_cursp(:,2),'g--');
	plot(curpitch,lift_star,'ro');
	%}
 end