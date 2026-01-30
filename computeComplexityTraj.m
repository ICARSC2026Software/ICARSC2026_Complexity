function [complexOutput] = computeComplexityTraj(DH,trajFilename,rpyArm,rpyArmLeft)
% computeComplexity - Get the complexity score of a given trajectory
% -> Denavit-Hartenberg Parameters (Modified Convention) in form of struct
% DH.d = 1xnJoints
% DH.a = 1xnJoints
% DH.alpha = alpha;
% -> Trajectory Filename
% -> rpyArm (optional) - RPY (ZYX) of the arm  - if none given it will be considered
% [0 0 0]
% -> rpyArmLeft (optional) - In case of bimanual movements, is the RPY (ZPY) of
% the left arm - if none given it will be considered [0 0 0]

arguments
    DH
    trajFilename
    rpyArm = [0 0 0]
    rpyArmLeft = [0 0 0]
end

complexityMov = [];
robotFrame = eye(4,4); robotFrameLeft = eye(4,4);
robotFrame(1:3,1:3) = computeRotMatrix(rpyArm);
robotFrameLeft(1:3,1:3) = computeRotMatrix(rpyArmLeft);

lArm = sum(DH.d(2:7));
% D,x,y,z,droll,dpitch,dyaw
minValues = [0,0,-lArm,-lArm,deg2rad(-180),deg2rad(-90),deg2rad(-180)];
maxValues = [2*lArm,lArm,lArm,lArm,deg2rad(180),deg2rad(90),deg2rad(180)];


joints_task = {}; armType_task = []; handPos_task = {};

vars = [];

[joints_traj, arm_type] = extractDataTraj(trajFilename);
joints_task = [joints_task;joints_traj'];
armType_task = [armType_task; arm_type];

% Compute variables for single arm
handPos_task = [handPos_task;getHandPos(joints_task{1}, DH,robotFrame)];
[d_direct] = computeDist(handPos_task{1}(:,1),handPos_task{1}(:,2),handPos_task{1}(:,3));
[delta_rpy] = computeReorientation(joints_task{1},DH,robotFrame);
vars = [vars; armType_task(1),d_direct, handPos_task{1}(end,1), handPos_task{1}(end,2), handPos_task{1}(end,3), delta_rpy];

% It is a bimanual movement - Compute also for the left arm
if(length(handPos_task)==2)
    handPos_task = [handPos_task;getHandPos(joints_task{1}, DH,robotFrameLeft)];
    [d_direct] = computeDist(handPos_task{2}(:,1),handPos_task{2}(:,2),handPos_task{2}(:,3));
    [delta_rpy] = computeReorientation(joints_task{2}, DH, robotFrameLeft);
    vars = [vars; armType_task(2),d_direct, handPos_task{2}(end,1), handPos_task{2}(end,2), handPos_task{2}(end,3), delta_rpy];
end

% Weights
w_1 = 0.119;
w_2 = 0.122;
w_3 = 0.172;
w_4 = 0.174;

% Normalization of the data
varNorm = (vars(2:8) - minValues) ./ (maxValues-minValues);
% Complexity Formula
for i=1:size(vars,1)
    complexVal = varNorm(i,5)*w_1 + varNorm(i,6)*w_3 + varNorm(i,7)*w_1 + ...
    varNorm(i,2)*w_2 + varNorm(i,3)* w_2 + varNorm(i,4)*w_3 + varNorm(i,1)*w_4;

    complexityMov = [complexityMov; complexVal];
end

% Return Values
%complexOutput = [vars(1), complexityMov];
complexOutput = complexityMov;

fprintf("Complexity Score = %.4f\n",complexOutput);
end


% Get the distance between the final and intial points
function [D1] = computeDist(x,y,z)
size_pos = length(x);
dist_direct = sqrt((x(size_pos)-x(1))^2+(y(size_pos)-y(1))^2+(z(size_pos)-z(1))^2);
D1 = dist_direct;
end

% Reorientation of the end-effector between the final and initial point
function [delta_rpy] = computeReorientation(traj,DH,T_init)

% Init Variables
roll = []; pitch = []; yaw = [];
rpy=[];
Rot = [];
ori = [];
%traj(:,8) = [];
traj = deg2rad(traj);

m_size = length(traj);
Rot = zeros(3,3,m_size);

%% Compute Orientation for each step
for i=1:m_size
    DH.theta = traj(i,:)';
    [~,~,~,~,T] = DirKin_robot(DH,T_init);
    Rot(:,:,i) = T(1:3,1:3);
end

for j=1:(m_size-1)

    Rot_points = Rot(:,:,j)'*Rot(:,:,j+1);
    m_rpy = Rot_points;
    
    rpy = computeRPY(m_rpy);

    roll = [roll; rpy(1)];
    pitch = [pitch; rpy(2)];
    yaw = [yaw; rpy(3)];
end

%% Return Variables
    Rot_init = Rot(:,:,1);
    Rot_final = Rot(:,:,m_size);
    Rot_i_f = inv(Rot_init)*Rot_final;
    delta_rpy = computeRPY(Rot_i_f);
    rpy_inicial = computeRPY(Rot(:,:,1));
    rpy_final = computeRPY(Rot(:,:,m_size));
    min_rot = [min(roll) min(pitch) min(yaw)];
    max_rot = [max(roll) max(pitch) max(yaw)]; 
end

% Get the RPY(ZYX) in rad of a Rotation Matrix
function [rpy] = computeRPY(Rot)
% Get rpy from the rotation matrix
% In: Rot
% Out: roll, pitch, yaw

%Init variables
roll = 0;
pitch = 0;
yaw = 0;

if(abs(Rot(1,1))<0.001 && abs(Rot(2,1))<0.001)
    roll = 0;
    pitch = atan2(-Rot(3,1), Rot(1,1));
    yaw = atan2(-Rot(2,3),Rot(2,2));
else
    roll = atan2(Rot(2,1), Rot(1,1));
    sp = sin(roll);
    cp = cos(roll);
    pitch = atan2(-Rot(3,1), cp*Rot(1,1) + sp*Rot(2,1));
    yaw = atan2(sp*Rot(1,3) - cp*Rot(2,3), cp*Rot(2,2) - sp*Rot(1,2));
end

rpy = [roll, pitch, yaw];
end

% Get the rotation matrix of RPY(ZYX) angles in rad
function [Rot] = computeRotMatrix(rpy)
    
    Rot=eye(3,3);
    roll = rpy(1);
    pitch = rpy(2);
    yaw = rpy(3);
    
    % First Row
    Rot(1,1) = cos(roll)*cos(pitch);
    Rot(1,2) = cos(roll)*sin(pitch)*sin(yaw)-sin(roll)*cos(yaw);
    Rot(1,3) = sin(roll)*sin(yaw)+cos(roll)*sin(pitch)*cos(yaw);
    % Second Row
    Rot(2,1) = sin(roll)*cos(pitch); 
    Rot(2,2) = cos(roll)*cos(yaw)+sin(roll)*sin(pitch)*sin(yaw); 
    Rot(2,3) = sin(roll)*sin(pitch)*cos(yaw)-cos(roll)*sin(yaw);
    % Third Row
    Rot(3,1) = -sin(pitch);             
    Rot(3,2) = cos(pitch)*sin(yaw);                                    
    Rot(3,3) = cos(pitch)*cos(yaw);
end

% Direct Kinematics of the robotic arm
function [Hand,Shoulder,Elbow,Wrist,T_h] = DirKin_robot(DH_parameters,T_init)
    % function [T] = DirKin_robot(DH_parameters)
    % Computes the direct kinematics of a planar robot with 7 DOFs
    %
    % Input:
    %   DH_parameters -> 7x4D matrix with de DH parameters 
    %       [θi     di      ai      αi], with i ∈ {1, 2, 3, 4, 5, 6, 7}
    %
    % Output:
    %   T -> 4D matrix with the cartesian coordinates of the end-point and
    %       the rotation ==
    %       [R11    R12    R13    Px]
    %       [R21    R22    R23    Py]
    %       [R31    R32    R33    Pz]
    %       [0      0      0      1 ]

if nargin<2
    T_init = eye(4,4);
end
    %% Variáveis

    theta   = DH_parameters.theta;
    d       = DH_parameters.d;
    alpha   = DH_parameters.alpha;
    a       = DH_parameters.a;
    
    %% Cinemática Direta Joint i
    joint = 7;
    T0_E    = eye(4,4);
    T       = zeros(4,4,joint);

    % i-1Ti = RotX(αi-1).TransX(ai-1).RotZ(θi).TransZ(di)
    for i = 1:joint
        T(:,:,i) = ...
            [                  cos(theta(i))                   (-sin(theta(i)))                    0                       a(i) ;
             (cos(alpha(i)) * sin(theta(i)))    (cos(alpha(i)) * cos(theta(i)))     (-sin(alpha(i)))    (-sin(alpha(i)) * d(i)) ;
             (sin(alpha(i)) * sin(theta(i)))    (sin(alpha(i)) * cos(theta(i)))      (cos(alpha(i)))     (cos(alpha(i)) * d(i)) ;
                                           0                                  0                    0                          1 ];

        T0_E = T0_E * T(:,:,i);
    end


    %% Saída

    T_h = T_init * T(:,:,1)*T(:,:,2)*T(:,:,3)*T(:,:,4)*T(:,:,5)*T(:,:,6)*T(:,:,7); 
    Hand = T_h(1:3,4);
    T_s = T_init*T(:,:,1)*T(:,:,2);
    Shoulder = T_s(1:3,4);
    T_e = T_init*T(:,:,1)*T(:,:,2)*T(:,:,3)*T(:,:,4);
    Elbow = T_e(1:3,4);
    T_w = T_init*T(:,:,1)*T(:,:,2)*T(:,:,3)*T(:,:,4)*T(:,:,5)*T(:,:,6);
    Wrist = T_w(1:3,4);
    
    
end

% Get the Hand Position along the trajectory
function [hand_pos] = getHandPos(joint_traj, DH, T_init)

% Init Variables
hand_pos = [];

joint_traj = deg2rad(joint_traj);
m_size = length(joint_traj);

for i=1:m_size
    
    DH.theta = joint_traj(i,:)';

    [h,~,~,~,~] = DirKin_robot(DH,T_init);
    hand_pos = [hand_pos;h'];   
end

end

% Extract the trajectory steps from a file
function [joints_traj, arm_type] = extractDataTraj(filename)

% Open the file
fid = fopen(filename, 'r');

% Initialize variables
pos_mov = []; % cell value for the joints value for the entire task
vel_mov = [];
pos_matrix = []; % joints values for primitive movement
joints_traj = {}; time_traj = {};
arm_type = [];

time = []; time = [time 0];
step_index = 0;
mov_id = [];
arm_id = [];
stage_change = 0;

position = [];
velocity = [];

% Read the file line by line
while ~feof(fid)
    line = fgetl(fid); % Read a line

    if startsWith(line,'# Planner')
        mov_info = regexp(line, 'Movement:\s*([-\w\s]+).*?Arm:\s*([-\w\s]+)', 'tokens');
        mov_value = mov_info{1}{1};
        arm_value = mov_info{1}{2};

        % Assign arm_id
        if strcmpi(arm_value, 'both')
            arm_id = [arm_id;2];
        elseif strcmpi(arm_value, 'right')
            arm_id = [arm_id;0];
        elseif strcmpi(arm_value, 'left')
            arm_id = [arm_id;1];
        end

        % Assign mov_id
        if strcmpi(mov_value, 'reach-to-grasp')
            mov_id = [mov_id; 1];
            if arm_id(end)==2
                mov_id = [mov_id;1];
            end
        elseif strcmpi(mov_value, 'transport')
            mov_id = [mov_id; 2];
            if arm_id(end)==2
                mov_id = [mov_id;2];
            end
        elseif strcmpi(mov_value, 'Go park') || strcmpi(mov_value, 'Move')
            mov_id = [mov_id; 3];
            if arm_id(end)==2
                mov_id = [mov_id;3];
            end
        elseif strcmpi(mov_value, 'Dual3') % place pick
            mov_id = [mov_id; 2];
            mov_id = [mov_id; 1];
        elseif strcmpi(mov_value, 'Dual4') % pick place
            mov_id = [mov_id; 1];
            mov_id = [mov_id; 2];
        elseif strcmpi(mov_value, 'Dual5') % go-park and place
            mov_id = [mov_id; 3];
            mov_id = [mov_id; 1];
        elseif strcmpi(mov_value, 'Dual7') % go-park and pick
            mov_id = [mov_id; 3];
            mov_id = [mov_id; 1];
        elseif strcmpi(mov_value, 'Dual8') % place and go-park
            mov_id = [mov_id; 2];
            mov_id = [mov_id; 3];
        end

        if ~isempty(pos_matrix)
            % Delete repeated times (because of time_step==0)
            time = unique(time,'stable');
            % Save the movement
            pos_mov{end+1} = pos_matrix;
            vel_mov{end+1} = vel_matrix;
            time_traj{end+1} = time;
            %arm_type = [arm_type;arm_id];
            %mov_type = [mov_type;mov_id];
        end

        pos_matrix = []; position = [];
        vel_matrix = []; velocity = [];
        time = []; time = [time 0];
    end

    if startsWith(line, 'prob_time')

    end
    
    
    if startsWith(line,'Movement')
        stage_info = regexp(line, 'Movement stage:\s*(\w+)', 'tokens');
        stage_value = stage_info{1}{1};

        if strcmpi(stage_value,'approach') || strcmpi(stage_value,'retreat') || strcmpi(stage_value,'approach2')
            stage_change = 1;
        end
    end

    step_match = regexp(line, 'step=\d+, time step=([^,]+), (.+)', 'tokens');
    if ~isempty(step_match)
        step_index = step_index + 1;
        time_value = str2double(step_match{1}{1});
        joint_values = regexp(step_match{1}{2}, 'Joint \d+=([-.\d|]+)', 'tokens');

        time = [time;time(end)+time_value];

        % Extract numerical values for each joint
        joint_matrix = [];
        for j = 1:length(joint_values)
            values = str2double(strsplit(joint_values{j}{1}, '|'));
            %joint_matrix = [joint_matrix; values]; %#ok<AGROW>
            position = [position values(1)];
            velocity = [velocity values(2)];
        end
        if(stage_change~=1)
            pos_matrix = [pos_matrix;position];
            vel_matrix = [vel_matrix;velocity];
        else
            stage_change = 0;
        end
        position = []; velocity = [];
    end

    if startsWith(line,'#END') % To save the final mov or the only movement in the file
        % Delete repeated times (because of time_step==0)
        time = unique(time,'stable');
        % Save the movement
        pos_mov{end+1} = pos_matrix;
        vel_mov{end+1} = vel_matrix;
        time_traj{end+1} = time;
        %arm_type = [arm_type;arm_id];
        %mov_type = [mov_type;mov_id];
    end

    % Close the file
end
fclose(fid);

if arm_id == 2
    pos_mov_right={}; pos_mov_left={};
    vel_mov_right={}; vel_mov_left={};
    mov_type = mov_id;
    for i=1:length(pos_mov)
        pos_mov_right{i} = pos_mov{i}(:,1:7);
        vel_mov_right{i} = vel_mov{i}(:,1:7);
        n_joints = size(pos_mov{i},2);
        if(n_joints==17 || n_joints==18) % SoftHand
            pos_mov_left{i} = pos_mov{i}(:,10:16);
            vel_mov_left{i} = vel_mov{i}(:,10:16);
        else
            pos_mov_left{i} = pos_mov{i}(:,9:15);
            vel_mov_left{i} = vel_mov{i}(:,9:15);
        end
    end
    
    % To join all movements of the same arm
    joints_traj{end+1} = vertcat(pos_mov_right{:});
    vel_traj{end+1} = vertcat(vel_mov_right{:});
    arm_type = [arm_type;0];
    joints_traj{end+1}=vertcat(pos_mov_left{:});
    vel_traj{end+1} = vertcat(vel_mov_left{:});
    arm_type = [arm_type;1];

    % for i=1:length(pos_mov_right)
    %     joints_traj{i}=pos_mov_right{i};
    %     vel_traj{i} = vel_mov_right{i};
    %     arm_type = [arm_type;0];
    % end

    % for i=1:length(pos_mov_left)
    %     joints_traj{end+1}=pos_mov_left{i};
    %     vel_traj{end+1} = vel_mov_left{i};
    %     arm_type = [arm_type;1];
    % end
    time_traj = [time_traj time_traj];
else
    for i=1:length(pos_mov)
        joints_traj{i} = pos_mov{i}(:,1:7);
        vel_traj{i} = vel_mov{i}(:,1:7);
    end
    arm_type = arm_id;
    mov_type = mov_id;
end




end
