clear; clc;

% Include
addpath('include\');
include();

% CRITERION: [X Z Y]. Vertexes are already in this form. 

% Load the required parameters
partable = readtable('config.dat','HeaderLines',6,'Delimiter','semi','ReadRowNames',true);
parameters = join(erase(string(partable{:, :}), "'"), '', 2);

% File loading
vehicle = stlread(parameters(1));
unicube = stlread(parameters(3));
cube_dim = double(strsplit(parameters(4),','));
cube_zwidth = sum(double(strsplit(parameters(5),',')));

% Aileron loading
load_ailerons();

% Vehicle model creation
[~, ~, centroid, ~] = inertia_tensor(vehicle.Points,vehicle.ConnectivityList);
VFo = vehicle.Points - centroid;
FFo = unifyMeshNormals(vehicle.ConnectivityList,VFo,'alignTo','out');
initialize_ailerons();

% Air model creation (CUBE)
height_ini = 1;
limits = [cube_dim(1) (cube_zwidth-height_ini) cube_dim(2) cube_dim(3) -height_ini cube_dim(4)];
VCo = aircube_sizing(unicube.Points + [-10 10 -10],[0 0 0],limits);
FCo = unifyMeshNormals(unicube.ConnectivityList,VCo,'alignTo','out');


% Keycube creation (MINICUBE)
mini_dim = 0.005;
mini_limits = [mini_dim mini_dim mini_dim -mini_dim -mini_dim -mini_dim];
p = {[0 0 0],[1.26 -0.09 0.5],[-1.26 -0.09 0.5],[0 0 1.3],[0 0.2 0.6],...
    [0 -0.2 0.6],[0 0.01 -0.6],[0 -0.19 -0.6],[0 -0.19 -1.3],[0 0.51 -2.1]};
Okeycubes = cell(size(p,2),1);
for k = 1:1:size(p,2)
    Okeycubes{k} = aircube_sizing(unicube.Points + [-10 10 -10],p{k},mini_limits);
end

% Initial values
mass = 30;                      % Mass (kg)
inipos = [0 0 height_ini];      % Position
inispd = [0 10 0];              % Speed
iniang = [10 0 0];              % Angle (deg)
iniwsp = [0 0 0];               % Angular speed (rad/s)
delta_time = 0.05;              % Time step (s)
ini_wang = [0 0 0 75];          % Control angles (deg)

% Simulation initialization
[curpitch, curyaw, curroll] = deal(iniang(1), iniang(2), iniang(3));
[angle_back,angle_wing_R,angle_wing_L,angle_up] = deal(ini_wang(1), ini_wang(2), ini_wang(3), ini_wang(4));
[VFi, FFi, VCi, FCi] = deal(VFo*rotx(deg2rad(curpitch))*roty(deg2rad(curyaw))*rotz(deg2rad(curroll)),FFo,VCo,FCo);

% Ailerons
[ABRFi, ABLFi, ABUFi, AWRFi, AWLFi] = deal(ABRFo, ABLFo, ABUFo, AWRFo, AWLFo);
update_ailerons;
% ails = {ABRVi, ABLVi, ABUVi, AWRVi, AWLVi, ABRFi, ABLFi, ABUFi, AWRFi, AWLFi};
ails = {ABUVi,ABUFi};

keycubes = Okeycubes;
for f = 1:1:size(Okeycubes,1)
    keycubes{f} = Okeycubes{f}*rotx(deg2rad(curpitch))*roty(deg2rad(curyaw))*rotz(deg2rad(curroll));
end
curpos = inipos(:);
curspd = inispd(:);
curwsp = iniwsp(:);
m = 1;

while (m < 2) 

    % Simulate the problem
    [curforce,curmoment,field] = model_flysim(VCi,FCi,keycubes,VFi,FFi,ails,curspd(2),0.3,false);

    % Compute dynamics
    % Forces
    curacc = ([curforce(1) curforce(3) curforce(2)] + [0 0 -9.80*mass])/mass;
    % Moments
    [I, ax, ~, ~] = inertia_tensor(VFi,FFi);
    [pax_mom,pax_I,pax_ws] = deal(change_of_basis(curmoment(:),ax'),eig(I),change_of_basis(curwsp(:),ax'));
    pax_ac(1) = (pax_mom(1) - (pax_I(3)-pax_I(2))*mass*pax_ws(2)*pax_ws(3))/(pax_I(1)*mass);
    pax_ac(2) = (pax_mom(2) - (pax_I(1)-pax_I(3))*mass*pax_ws(3)*pax_ws(1))/(pax_I(2)*mass);
    pax_ac(3) = (pax_mom(3) - (pax_I(2)-pax_I(1))*mass*pax_ws(1)*pax_ws(2))/(pax_I(3)*mass);
    curwac = change_of_basis(pax_ac(:),inv(ax'));
    curwac = [curwac(1) curwac(3) curwac(2)]';
    
   % Compute cinematics
   % Translate
   curspd = curspd(:) + curacc(:)*delta_time;
   curspd(2) = inispd(2);
   curpos = curpos(:) + curspd(:)*delta_time;
   % Rotate
   curwsp = curwsp(:) + curwac(:)*delta_time;
   curpitch = curpitch + rad2deg(curwsp(1))*delta_time; 
   curyaw = curyaw + rad2deg(curwsp(3))*delta_time;
   curroll = curroll + rad2deg(curwsp(2))*delta_time;
   
   % Update the vehicle angle 
   VFi = VFo*rotx(deg2rad(curpitch))*roty(deg2rad(0))*rotz(deg2rad(curroll)); 
   for f = 1:1:size(keycubes,1)
       keycubes{f} = Okeycubes{f}*rotx(deg2rad(curpitch))*roty(deg2rad(0))*rotz(deg2rad(curroll));
   end
   
   % Control module
   update_ailerons;
    
   % Update the aircube
   limits = [cube_dim(1) (cube_zwidth-curpos(3)) cube_dim(2) cube_dim(3) -curpos(3) cube_dim(4)];
   VCi = aircube_sizing(unicube.Points + [-10 10 -10],[0 0 0],limits);
   
   disp("=====================STEP SOLVED===================");
   disp(strcat("Current position is:            ",num2str(curpos(:)')));
   disp(strcat("Current lift is:                ",num2str(curforce(2))));
   disp(strcat("Current angles are (p y r):     ",num2str([curpitch curyaw curroll])));
   disp(strcat("Current acceleration is:        ",num2str(curacc(:)')));
   disp(strcat("Current time is:                ",num2str(m*delta_time)));
   disp("===================================================");
   pause(0.00001);
   writematrix([curpos(:)' curspd(:)' curpitch curyaw curroll curforce(:)' curmoment(:)' curwsp(:)' m*delta_time],'results','WriteMode','append');
   
   m = m + 1;
end

