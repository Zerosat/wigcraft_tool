function latval = latstar(pitch,speed)
	 
	 pitch = clamp(pitch,-20,20);
	 speed = clamp(speed,5,30);
	 
	 %% Compute our lat with speed & pitch
	 % Load the simulated data
	 lat_vs_pitch = ...
	 [readmatrix('lat_vs_pitch_5ms.txt');
	 readmatrix('lat_vs_pitch_10ms.txt');
	 readmatrix('lat_vs_pitch_15ms.txt');
	 readmatrix('lat_vs_pitch_20ms.txt');
	 readmatrix('lat_vs_pitch_30ms.txt')];
	 simspeeds = [5 10 15 20 30];
	 simpitchs = [-20 20];
	 
	 % Compute a new lat-pitch curve for the current speed
	 lat_vs_pitch_cursp = zeros(size(simpitchs,2),2);
	 
	 for k = 1:1:size(simpitchs,2)
		lat_vs_speed = [simspeeds' lat_vs_pitch(lat_vs_pitch(:,1) == simpitchs(k),2)];
		lat_vs_pitch_cursp(k,:) = [simpitchs(k) interp1(lat_vs_speed(:,1),lat_vs_speed(:,2),speed)];
	 end
	 
	 % Compute the lat for the current pitch
	 latval = interp1(lat_vs_pitch_cursp(:,1),lat_vs_pitch_cursp(:,2),pitch);
end