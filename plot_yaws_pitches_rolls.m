
%% YAW CONSTANT
% Load matrix
K = readmatrix("angles_RPY_vs_lift_s20_90K.txt");

% Find our indexes
yawval = 0;
a = find(K(:,8) == yawval & K(:,9) == -15);
b = find(K(:,8) == yawval & K(:,9) == 0);
c = find(K(:,8) == yawval & K(:,9) == 15);

A = unique(sortrows([K(a,7) K(a,2)],1,'ascend'),'rows');
B = unique(sortrows([K(b,7) K(b,2)],1,'ascend'),'rows');
C = unique(sortrows([K(c,7) K(c,2)],1,'ascend'),'rows');

hold on;
plot(A(:,1),movmean(A(:,2),15),'r-');
plot(B(:,1),movmean(B(:,2),15),'b-');
plot(C(:,1),movmean(C(:,2),15),'m-');

hold on;
plot(A(:,1),A(:,2),'r-');
plot(B(:,1),B(:,2),'b-');
plot(C(:,1),C(:,2),'m-');



%% ROLL CONSTANT
% Load matrix
K = readmatrix("angles_RPY_vs_lift_s20_90K.txt");

% Find our indexes
rollval = 15;
a = find(K(:,9) == rollval & K(:,8) == -15);
b = find(K(:,9) == rollval & K(:,8) == 0);
c = find(K(:,9) == rollval & K(:,8) == 15);

A = unique(sortrows([K(a,7) K(a,2)],1,'ascend'),'rows');
B = unique(sortrows([K(b,7) K(b,2)],1,'ascend'),'rows');
C = unique(sortrows([K(c,7) K(c,2)],1,'ascend'),'rows');

hold on;
plot(A(:,1),movmean(A(:,2),15),'r-');
plot(B(:,1),movmean(B(:,2),15),'b-');
plot(C(:,1),movmean(C(:,2),15),'m-');

%% NO YAW NO ROLL: BASE SIMULATION

K = readmatrix("lift_angle_s10_90K.txt");
A = unique(sortrows([K(:,4) K(:,2)],2,'ascend'),'rows');
plot(A(:,1),movmean(A(:,2),15),'k-');

%% SPEEDS VS PITCH

addpath('results\');

K = readmatrix("pitches_vs_speeds_90K.txt");
area = get_projected_area();

a = find(K(:,10) == 5);
b = find(K(:,10) == 10);
c = find(K(:,10) == 15);
d = find(K(:,10) == 20);
e = find(K(:,10) == 30);

% The array contents are: [PITCH LIFT DRAG]

A = unique(sortrows([K(a,7) 2*K(a,2)/(1.22*(5^2)*area) 2*K(a,3)/(1.22*(5^2)*area) K(a,4) K(a,2) K(a,3) K(a,1)],2,'ascend'),'rows');
B = unique(sortrows([K(b,7) 2*K(b,2)/(1.22*(10^2)*area) 2*K(b,3)/(1.22*(10^2)*area) K(b,4) K(b,2) K(b,3) K(b,1)],2,'ascend'),'rows');
C = unique(sortrows([K(c,7) 2*K(c,2)/(1.22*(15^2)*area) 2*K(c,3)/(1.22*(15^2)*area) K(c,4) K(c,2) K(c,3) K(c,1)],2,'ascend'),'rows');
D = unique(sortrows([K(d,7) 2*K(d,2)/(1.22*(20^2)*area) 2*K(d,3)/(1.22*(20^2)*area) K(d,4) K(d,2) K(d,3) K(d,1)],2,'ascend'),'rows');
E = unique(sortrows([K(e,7) 2*K(e,2)/(1.22*(30^2)*area) 2*K(e,3)/(1.22*(30^2)*area) K(e,4) K(e,2) K(e,3) K(e,1)],2,'ascend'),'rows');

% LIFT COEFFICIENT
hold on;
plot(A(:,1),movmean(A(:,2),3),'m.--');
plot(B(:,1),movmean(B(:,2),3),'b.--');
plot(C(:,1),movmean(C(:,2),3),'r.--');
plot(D(:,1),movmean(D(:,2),3),'k.--');
plot(E(:,1),movmean(E(:,2),3),'g.--');

% DRAG COEFFICIENT
hold on;
plot(A(:,1),movmean(-A(:,3),3),'m.--');
plot(B(:,1),movmean(-B(:,3),3),'b.--');
plot(C(:,1),movmean(-C(:,3),3),'r.--');
plot(D(:,1),movmean(-D(:,3),3),'k.--');
plot(E(:,1),movmean(-E(:,3),3),'g.--');

% CD vs CL
hold on;
plot(movmean(-A(:,3),3),movmean(A(:,2),3),'m.--');
plot(movmean(-B(:,3),3),movmean(B(:,2),3),'b.--');
plot(movmean(-C(:,3),3),movmean(C(:,2),3),'r.--');
plot(movmean(-D(:,3),3),movmean(D(:,2),3),'k.--');
plot(movmean(-E(:,3),3),movmean(E(:,2),3),'g.--');

% Lift to drag ratio
hold on;
plot(A(:,1),movmean(-A(:,2)./A(:,3),3),'m.--');
plot(B(:,1),movmean(-B(:,2)./B(:,3),3),'b.--');
plot(C(:,1),movmean(-C(:,2)./C(:,3),3),'r.--');
plot(D(:,1),movmean(-D(:,2)./D(:,3),3),'k.--');
plot(E(:,1),movmean(-E(:,2)./E(:,3),3),'g.--');

% Pitching moment vs pitch
hold on;
plot(A(:,1),movmean(A(:,4),6),'m.--');
plot(B(:,1),movmean(B(:,4),6),'b.--');
plot(C(:,1),movmean(C(:,4),6),'r.--');
plot(D(:,1),movmean(D(:,4),6),'k.--');
plot(E(:,1),movmean(E(:,4),6),'g.--');

% Lift vs pitch
hold on;
plot(A(:,1),movmean(A(:,5),6),'m.--');
plot(B(:,1),movmean(B(:,5),6),'b.--');
plot(C(:,1),movmean(C(:,5),6),'r.--');
plot(D(:,1),movmean(D(:,5),6),'k.--');
plot(E(:,1),movmean(E(:,5),6),'g.--');

% Drag vs pitch
hold on;
plot(A(:,1),-movmean(A(:,6),6),'m.--');
plot(B(:,1),-movmean(B(:,6),6),'b.--');
plot(C(:,1),-movmean(C(:,6),6),'r.--');
plot(D(:,1),-movmean(D(:,6),6),'k.--');
plot(E(:,1),-movmean(E(:,6),6),'g.--');

% Lat vs pitch
hold on;
ka = movmean(A(:,7),10);
kb = movmean(B(:,7),10);
kc = movmean(C(:,7),10);
kd = movmean(D(:,7),10);
ke = movmean(E(:,7),10);

plot(A([1 end],1),ka([1 end],1),'m.--');
plot(B([1 end],1),kb([1 end],1),'b.--');
plot(C([1 end],1),kc([1 end],1),'r.--');
plot(D([1 end],1),kd([1 end],1),'k.--');
plot(E([1 end],1),ke([1 end],1),'g.--');







% Pitching moment (Y) vs lift (X)
hold on;
plot(movmean(A(:,5),3),movmean(A(:,4),3),'m.--');
plot(movmean(B(:,5),3),movmean(B(:,4),3),'b.--');
plot(movmean(C(:,5),3),movmean(C(:,4),3),'r.--');
plot(movmean(D(:,5),3),movmean(D(:,4),3),'k.--');
plot(movmean(E(:,5),3),movmean(E(:,4),3),'g.--');



%% COMPARISON AILERON BACK AND NOTHING

% Lifts (is reasonable not to take it into account)
addpath('results\');
hold on;
mvm = 5;
% K = readmatrix("ails_back_0_s20_90K.txt"); 
% A = unique(sortrows([K(:,7) K(:,2)],1,'ascend'),'rows');
plot(A(8:14,1),movmean(A(8:14,2),mvm),'r.-');
K = readmatrix("ails_back_15_s20_90K.txt"); 
A = unique(sortrows([K(:,7) K(:,2)],1,'ascend'),'rows');
plot(A(:,1),movmean(A(:,2),mvm),'b.-');
K = readmatrix("ails_back_35_s20_90K.txt"); 
A = unique(sortrows([K(:,7) K(:,2)],1,'ascend'),'rows');
plot(A(:,1),movmean(A(:,2),mvm),'m.-');
K = readmatrix("ails_back_55_s20_90K.txt"); 
A = unique(sortrows([K(:,7) K(:,2)],1,'ascend'),'rows');
plot(A(:,1),movmean(A(:,2),mvm),'k.-');
K = readmatrix("ails_back_75_s20_90K.txt"); 
A = unique(sortrows([K(:,7) K(:,2)],1,'ascend'),'rows');
plot(A(:,1),movmean(A(:,2),mvm),'c.-');
K = readmatrix("ails_back_90_s20_90K.txt"); 
A = unique(sortrows([K(:,7) K(:,2)],1,'ascend'),'rows');
plot(A(:,1),movmean(A(:,2),mvm),'g.-');


addpath('results\');
hold on;
mvm = 6;

K = readmatrix("ails_back_15_s20_90K.txt"); 
A = unique(sortrows([K(:,7) K(:,4)],1,'ascend'),'rows');
plot(A(:,1),movmean(A(:,2),mvm),'b.-');
K = readmatrix("ails_back_35_s20_90K.txt"); 
A = unique(sortrows([K(:,7) K(:,4)],1,'ascend'),'rows');
plot(A(:,1),movmean(A(:,2),mvm),'m.-');
K = readmatrix("ails_back_55_s20_90K.txt"); 
A = unique(sortrows([K(:,7) K(:,4)],1,'ascend'),'rows');
plot(A(:,1),movmean(A(:,2),mvm),'k.-');
K = readmatrix("ails_back_75_s20_90K.txt"); 
A = unique(sortrows([K(:,7) K(:,4)],1,'ascend'),'rows');
plot(A(:,1),movmean(A(:,2),mvm),'c.-');
K = readmatrix("ails_back_90_s20_90K.txt"); 
A = unique(sortrows([K(:,7) K(:,4)],1,'ascend'),'rows');
plot(A(:,1),movmean(A(:,2),mvm),'g.-');
K = readmatrix("ails_back_m40_s20_90K.txt"); 
A = unique(sortrows([K(:,7) K(:,4)],1,'ascend'),'rows');
plot(A(:,1),movmean(A(:,2),mvm),'y.-');