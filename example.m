
% Demonstration of use of both functions

% computeComplexityTraj
fprintf("---------- Using computeComplexityTraj func ----------\n")
d = [0.360;0.0;0.420;0.0;0.4;0.0;0.126+0.16]*1000;
a = [0;0;0;0;0;0;0];
alpha = [0;deg2rad(-90);deg2rad(90);deg2rad(90);deg2rad(-90);deg2rad(-90);deg2rad(90)];

DH = struct();
DH.d = d;
DH.a = a;
DH.alpha = alpha;

% Right Arm
rpy=[0.5236 0 1.5708];
%Left Arm
%rpyLeft=[-0.5236 0 -1.5708];

computeComplexityTraj(DH,"Milho/P1_0.task",rpy);

% computeComplexity
fprintf("---------- Using computeComplexity func ----------\n")
computeComplexity(500,200,300,1000,0.2,0.5,0.7,1106);


