clear; clc; 

% Configuration
precision = 0.3;
height = 10;
filename = 'height_10m.txt';

pitches = [-20 -10 -5 0 5 10 20];
rolls = [0];
yaws = [0];

speeds = 5;
ini_wang = [0 0 0 0];

% Include
addpath('include\');
include();

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
VFo = (vehicle.Points - centroid);
FFo = unifyMeshNormals(vehicle.ConnectivityList,VFo,'alignTo','out');
initialize_ailerons();

% Air model creation (CUBE)
height_ini = height;
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
[VFi, FFi, VCi, FCi] = deal(VFo,FFo,VCo,FCo);
keycubes = Okeycubes;

% Ailerons
[angle_back,angle_wing_R,angle_wing_L,angle_up] = deal(ini_wang(1), ini_wang(2), ini_wang(3), ini_wang(4));
[ABRFi, ABLFi, ABUFi, AWRFi, AWLFi] = deal(ABRFo, ABLFo, ABUFo, AWRFo, AWLFo);
[curpitch, curyaw, curroll] = deal(pitches(1), yaws(1), rolls(1));
update_ailerons;
% ails = {ABRVi, ABLVi, ABUVi, AWRVi, AWLVi, ABRFi, ABLFi, ABUFi, AWRFi, AWLFi};
ails = {AWRVi, AWLVi, AWRFi, AWLFi};
ails = {};

numcount = 1;
problemsize = size(speeds,2)*size(pitches,2)*size(rolls,2)*size(yaws,2);

for s = 1:size(speeds,2)
    
    curspd = speeds(s);
    
    for h = 1:size(yaws,2)
        
        curyaw = yaws(h);

        for t = 1:size(rolls,2)

            curroll = rolls(t);

            for m = 1:size(pitches,2)

               curpitch = pitches(m);

               % Modify the desired variable
               % Attack angle (pitch)
               VFi = VFo*rotx(deg2rad(curpitch))*roty(deg2rad(curyaw))*rotz(deg2rad(curroll));
               for f = 1:1:size(keycubes,1)
                   keycubes{f} = Okeycubes{f}*rotx(deg2rad(curpitch))*roty(deg2rad(curyaw))*rotz(deg2rad(curroll));
               end

               % Flying height
               %{
               limits = [cube_dim(1) (cube_zwidth-curpos(3)) cube_dim(2) cube_dim(3) -curpos(3) cube_dim(4)];
               VCi = aircube_sizing(unicube.Points + [-10 10 -10],[0 0 0],limits);
               %}

               % Simulate the problem
               [curforce,curmoment] = model_flysim(VCi,FCi,keycubes,VFi,FFi,ails,curspd,precision,false);
               
               if (~all(~isnan(curforce)))
                    error('NaN force');
               end

               % Save our data
               writematrix([curforce(:)' curmoment(:)' curpitch curyaw curroll curspd angle_back angle_wing_R angle_wing_L angle_up],filename,'WriteMode','append');
               
               disp(strcat("Iteration completed: ",num2str(numcount)," out of ", num2str(problemsize),". Expected time: ",num2str(3*(problemsize - numcount))," min."));
               numcount = numcount + 1;
               pause(0.00001);
               
            end
        end
    end
end