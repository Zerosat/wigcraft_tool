clear; clc;

% Include
addpath('include\');
include();

% Load the required parameters
partable = readtable('config.dat','HeaderLines',6,'Delimiter','semi','ReadRowNames',true);
parameters = join(erase(string(partable{:, :}), "'"), '', 2);
m_40 = readmatrix("ails_back_40_s5_90K.txt");
m_m40 = readmatrix("ails_back_m40_s5_90K.txt");
RPY = readmatrix("sim_RPY_5ms_90K.txt");

% File loading
vehicle = stlread(parameters(1));
unicube = stlread(parameters(3));

% Aileron loading
load_ailerons();

% Vehicle model creation
[~, ~, centroid, ~] = inertia_tensor(vehicle.Points,vehicle.ConnectivityList);
VFo = vehicle.Points - centroid;
FFo = unifyMeshNormals(vehicle.ConnectivityList,VFo,'alignTo','out');
initialize_ailerons();

% Initial values
mass = 25;                       % Mass (kg)
inipos = [0 0 2];             % Position
inispd = [0 28 0];               % Speed
iniang = [11 0 0];               % Angle (deg)
iniwsp = [0 0 0];                % Angular speed (rad/s)
delta_time = 0.01;               % Time step (s)
ini_wang = [0 0 0 0];            % Control angles (deg)

% Simulation initialization
[curpitch, curyaw, curroll] = deal(iniang(1), iniang(2), iniang(3));
[angle_back,angle_wing_R,angle_wing_L,angle_up] = deal(ini_wang(1), ini_wang(2), ini_wang(3), ini_wang(4));
[VFi, FFi] = deal(VFo*rotx(deg2rad(curpitch))*roty(deg2rad(curyaw))*rotz(deg2rad(curroll)),FFo);

% Ailerons
[ABRFi, ABLFi, ABUFi, AWRFi, AWLFi] = deal(ABRFo, ABLFo, ABUFo, AWRFo, AWLFo);
update_ailerons;
% ails = {ABRVi, ABLVi, ABUVi, AWRVi, AWLVi, ABRFi, ABLFi, ABUFi, AWRFi, AWLFi};
ails = {ABUVi,ABUFi};

curpos = inipos(:);
curspd = inispd(:);
curwsp = iniwsp(:);
m = 1;
[cumm,cumm2,cumm3,cumm4,cumm5,cumm6] = deal(0);
thrust = 0;

% Initialize terrain generation
% figure('units','normalized','outerposition',[0 0 1 1]);
% set(gca,'color',[0.3010 0.7450 0.9330]);
% model_terrain();

% Control module
pitch_setpoint = 10;
wspeed_setpoint = 0;
height_setpoint = 2;


while (curpos(3) > -1 && m < 1e9) 

    % Simulate the problem
    [curforce,curmoment] = model_flysim_fast(curspd(2),curpitch,curyaw,curroll,angle_up,RPY,m_40,m_m40);
    
    % [curforce,curmoment] = deal([0 0 100],[0 0 0]);
    
    % Compute dynamics
    % Forces
    curacc = ([curforce(1) curforce(3) curforce(2)] + [0 thrust -9.80*mass])/mass;
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
   curpos = curpos(:) + curspd(:)*delta_time;
    % Rotate
   curwsp = curwsp(:) + curwac(:)*delta_time;
   curpitch = curpitch + rad2deg(curwsp(1))*delta_time; 
   curyaw = curyaw + rad2deg(curwsp(3))*delta_time;
   curroll = curroll + rad2deg(curwsp(2))*delta_time;

   % curpos(3) = 2;
   
   % Update the vehicle angle 
   VFi = VFo*rotx(deg2rad(curpitch))*roty(deg2rad(curyaw))*rotz(deg2rad(curroll)); 
   
   
   % Height control
   [pitch_med,cumm4] = pid_control(curspd(3),curacc(3),0,0.8,0,0,delta_time,cumm4,m);
   pitch_setpoint = pitch_setpoint + pitch_med*delta_time;
   
   % Pitch control
   [angle_up_med2,cumm] = pid_control(curpitch,curwsp(1),pitch_setpoint,25,25,0,delta_time,cumm,m);
   [angle_up_med,cumm2] = pid_control(curwsp(1),curwac(1),-wspeed_setpoint,-15,0.4,-3,delta_time,cumm2,m);
   angle_up = angle_up_med + angle_up_med2;
   
   % Speed control
   [thrust_med,cumm3] = pid_control(curspd(2),curacc(2),28,2900,5400,0,delta_time,cumm3,m);
   thrust = 100 + thrust_med;
   
   angle_up = clamp(angle_up,-80,80);
   disp(curpos(3));
   update_ailerons;
   
   figure(1);
   hold on;
   plot(m,curpos(3),'m.');
   
   figure(2);
   hold on;
   plot(m,curpitch,'m.');
   plot(m,pitch_setpoint,'r.');
   
    % plot(m,curspd(3),'m.');
    % plot(m,0,'r.');
    
   %{
   % Plot the data
    if m > 1
        delete(lat);
        delete(aiback);
    end
    sc = 0.5;
    PatchProps.FaceColor = 'r';PatchProps.EdgeColor = 'k'; 
    PatchProps.FaceAlpha = 1; PatchProps.EdgeAlpha = 1; 
    lat = patch('Faces', FFi, 'Vertices',sc*([VFi(:,1) VFi(:,3) VFi(:,2)] + [curpos(1),curpos(2),curpos(3)]), PatchProps);
    aiback = patch('Faces', ABUFi, 'Vertices',sc*([ABUVi(:,1) ABUVi(:,3) ABUVi(:,2)] + [curpos(1),curpos(2),curpos(3)]), PatchProps);
    
    axis equal;
    if (curpos(3) + 1 < 0)
        set(gca,'color','b');
    end
    % set(gca, 'CameraPosition', sc*[curpos(1),curpos(2) - 5,curpos(3) + 1.5]);
    set(gca, 'CameraPosition', sc*[curpos(1) - 5,curpos(2),curpos(3) + 1.5]);
    set(gca, 'CameraTarget', sc*[curpos(1),curpos(2),curpos(3)]);
    set(gca, 'CameraViewAngle', 50);
    set(gca, 'Projection', 'perspective');
    scatter3(sc*curpos(1),sc*curpos(2),sc*curpos(3),'ro','filled');
    %}
  
    pause(0.001);
    m = m + 1;
end

