function liftval = liftstar(pitch,speed)
	 
	 pitch = clamp(pitch,-20,20);
	 speed = clamp(speed,5,30);
	 
	 %% Compute our lift with speed & pitch
	 % Load the simulated data
	 lift_vs_pitch = ...
	 [readmatrix('lift_vs_pitch_5ms.txt');
	 readmatrix('lift_vs_pitch_10ms.txt');
	 readmatrix('lift_vs_pitch_15ms.txt');
	 readmatrix('lift_vs_pitch_20ms.txt');
	 readmatrix('lift_vs_pitch_30ms.txt')];
	 simspeeds = [5 10 15 20 30];
	 simpitchs = [-20 -15 -13 -10 -8 -7 -3 0 3 8 10 13 15 20];
	 
	 % Compute a new lift-pitch curve for the current speed
	 lift_vs_pitch_cursp = zeros(size(simpitchs,2),2);
	 
	 for k = 1:1:size(simpitchs,2)
		lift_vs_speed = [simspeeds' lift_vs_pitch(lift_vs_pitch(:,1) == simpitchs(k),2)];
		lift_vs_pitch_cursp(k,:) = [simpitchs(k) interp1(lift_vs_speed(:,1),lift_vs_speed(:,2),speed)];
	 end
	 
	 % Compute the lift for the current pitch
	 liftval = interp1(lift_vs_pitch_cursp(:,1),lift_vs_pitch_cursp(:,2),pitch);
end