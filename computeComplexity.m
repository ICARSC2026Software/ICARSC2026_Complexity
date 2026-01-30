function [complexOutput] = computeComplexity(distance,xf,yf,zf,droll,dpitch,dyaw,lengthArm,filename)
% Compute Complexity score given the 7 variables needed
% -> distance - distance between the final and initial point
% -> xf - final X position of the target position in relation
% to the robot reference frame
% -> yf - final Y position of the target position in relation
% to the robot reference frame
% -> zf - final Z position of the target position in relation
% to the robot reference frame
% -> droll - reorientation in roll(Z) between the final and initial pose of the end-effector;
% -> dpitch - reorientation in pitch(Y) between the final and initial pose of the end-effector;
% -> dyaw - reorientation in yaw(X) between the final and initial pose of the end-effector
% -> lengthArm - length between the shoulder and the palm of the hand
% -> filename (optional) - give a filename if you want to save the data to a file

% Give error if the point is outside of the robot's workspace, as it does
% not make sense to compute complexity for a point where the robot can't
% reach

if(distance > 2*lengthArm)
    fprintf("Error: The distance indicates that the point is outside of the robot's workspace!\nInsert another distance.\n");
    return;
end

if(distance==0)
    fprintf("Error: Distance equal to zero, there is no movement! Insert a positive value between 0 and 2x the arm length\n");
    return;
end

if(xf>lengthArm || yf>lengthArm || zf>lengthArm || xf<0 || yf<-lengthArm || zf<-lengthArm)
    fprintf("Error: The given coordinates are outside of the robot's workspace!\nInsert another target position.\n");
    return;
end

minValues = [0,0,-lengthArm,-lengthArm,deg2rad(-180),deg2rad(-90),deg2rad(-180)];
maxValues = [2*lengthArm,lengthArm,lengthArm,lengthArm,deg2rad(180),deg2rad(90),deg2rad(180)];

complexityMov = [];

vars = [distance,xf,yf,zf,droll,dpitch,dyaw];

% Weights
w_1 = 0.119;
w_2 = 0.122;
w_3 = 0.172;
w_4 = 0.174;

% Normalization of the data
varNorm = (vars - minValues) ./ (maxValues-minValues);
% Complexity Formula
for i=1:size(vars,1)
    complexVal = varNorm(i,5)*w_1 + varNorm(i,6)*w_3 + varNorm(i,7)*w_1 + ...
    varNorm(i,2)*w_2 + varNorm(i,3)* w_2 + varNorm(i,4)*w_3 + varNorm(i,1)*w_4;

    complexityMov = [complexityMov; complexVal];
end

% Return Values

complexOutput = table(distance,xf,yf,zf,droll,dpitch,dyaw,complexityMov, 'VariableNames', {'D','xf','yf','zf','droll','dpitch','dyaw','Complexity'});

complexOutput.Complexity = round(complexOutput.Complexity,4);

for i=1:size(complexOutput,1)
    fprintf("Complexity Score = %.4f\n",complexOutput.Complexity(i));
end

if(nargin==9)
    writetable(complexOutput,filename);
end

end


