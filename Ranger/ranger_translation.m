%% constants
eps = 0.05;
obstacles = [];
obstacles(1).radius = 0.1;
obstacles(1).position_x = 5;
obstacles(1).position_y = 3;
obstacles(1).orientation = 0;
%obstacles(1).locations
obstacles(2).radius = 0.1;
obstacles(2).position_x = 3;
obstacles(2).position_y = 5;
obstacles(2).orientation = 0;
%obstacles(2).locations
numObstacles = length(obstacles);

arena.xwidth = 10;
arena.ywidth = 10;
arena.gradient = 0;
arena.windSpeed = 0;
% arena.locations

tockLength = 1;
timeStep = 0.2;
collisionEps = 0.05;

clear comp;

%% EnvironmentLoop

% state:
% 1: robot_position_x
% 2: robot_position_y
% 3: robot_velocity_x
% 4: robot_velocity_y
% 5: robot_acceleration_x
% 6: robot_acceleration_y
% 7: robot_orientation
% 8: robot_angularVelocity
% 9: robot_angularAcceleration
% 10: time
% 11: stepTimer
% 12: tockTimer
% 13: obstacleTime
% 14: obstacleOccurred
% 15: inarg1_x
% 16: inarg1_y
% 17: inarg2_x
% 18: inarg2_y
% 19: inarg3_x
% 20: inarg3_y
% 21: inarg4
% 22: inarg5
% 23: inarg6
% 24: inarg7
% 25: inarg8
% 26: _timer
% 27: obstacleTriggered_EL_oB_hidden_arg
% 28: getRobotOrientation_EL_mS_hidden_arg
state_size = 28;

% inputs:
% 1: moveTime
% 2: moveOccurred
% 3: obstacleTrig
% 4: inarg9
% 5: setRobotVelocity_EL_mS_hidden_x
% 6: setRobotVelocity_EL_mS_hidden_y
% 7: setRobotAngularVelocity_EL_mS_hidden
input_size = 28; % extend to maximum input size

% default dynamics
A = zeros(state_size,state_size);
B = zeros(state_size,input_size);
c = zeros(state_size,1);
c(26) = 1; % _timer' (26) == 1
C = eye(state_size);
D = zeros(state_size,input_size);
k = zeros(state_size,1);
default_dyn = linearSys(A, B, c, C, D, k);

% identity reset
identity_reset.A = eye(state_size);
identity_reset.c = zeros(state_size,1);

vars = sym('x',[state_size,1]);
eq = [vars(1) - arena.xwidth; vars(2) - arena.ywidth];
compOp = {'<=', '<='};
state_invariant = levelSet(eq,vars,compOp);
for i = 1:length(obstacles)
    eq = - (vars(1) - obstacles(i).position_x)^2 - (vars(2) - obstacles(i).position_y)^2 + obstacles(i).radius^2;
    compOp = {'<='};
    state_invariant = state_invariant & levelSet(eq,vars,compOp);
end

clear locs;

%% EnvironmentLoop, Loc 1: EnvironmentLoop_2 assignment start (-> 3)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
reset.A(10,10) = 0; % time (10) = 0
reset.A(11,11) = 0; % stepTimer (11) = 0
reset.A(12,12) = 0; % tockTimer (12) = 0
reset.A(13,13) = 0; % obstacleTime (13) = 0
reset.A(14,14) = 0; % obstacleOccurred (14) = false
trans(1) = transition(guard, reset, 3, 'sync2');

locs(1) = location(inv, trans, dyn);

%% EnvironmentLoop, Loc 2: unused end location

inv = fullspace(state_size);
dyn = default_dyn;
trans = transition();

locs(2) = location(inv, trans, dyn);

%% EnvironmentLoop, Loc 3: RobotMovement continuous schema (-> 4)

inv = state_invariant;
for i = 1:length(obstacles)
    % obs_radius^2 < (pos_x (1) - obs_pos_x)^2 + (pos_y (2) - obs_pos_y)^2
    eq = - (vars(1) - obstacles(i).position_x)^2 - (vars(2) - obstacles(i).position_y)^2 + obstacles(i).radius^2;
    compOp = {'<'};
    inv = inv & levelSet(eq,vars,compOp);
end
% stepTimer (11) < timeStep
eq = vars(11) - timeStep;
compOp = {'<'};
inv = inv & levelSet(eq,vars,compOp);

A = zeros(state_size,state_size);
B = zeros(state_size,input_size);
c = zeros(state_size,1);
A(1,1) = 0; A(1,3) = 1; % pos_x' (1) == vel_x (3)
A(2,2) = 0; A(2,4) = 1; % pos_y' (2) == vel_y (4)
A(3,3) = 0; A(3,5) = 1;% vel_x' (3) == acc_x (5)
A(4,4) = 0; A(4,6) = 1; % vel_y' (4) == acc_y (6)
A(5,5) = 0; % acc_x' (5) == 0
A(6,6) = 0; % acc_y' (6) == 0
A(7,7) = 0; A(7,8) = 1; % ori' (7) == angVel (8)
A(8,8) = 0; A(8,9) = 1; % angVel' (8) == angAcc (9)
A(9,9) = 0; % angAcc' (9) == 0
c(10) = 1; % time' (10) = 1
c(11) = 1; % stepTimer' (11) == 1
c(12) = 1; % tockTimer' (12) == 1
c(26) = 1; % _timer' (26) = 1
C = eye(state_size);
D = zeros(state_size,input_size);
k = zeros(state_size,1);
dyn = linearSys(A, B, c, C, D, k);

trans = transition();% exists obs : obstacles @ 
%     (obs.pos.x - robot.pos.x)^2 + (obs.pos.x - robot.pos.x)^2 <= obs.radius^2
% 1:  robot_position_x
% 2:  robot_position_y
for i = 1:length(obstacles)
    eq = (vars(1) - obstacles(i).position_x)^2 + (vars(2) - obstacles(i).position_y)^2 - obstacles(i).radius^2;
    compOp = {'<='};
    guard = levelSet(eq,vars,compOp);
    reset = identity_reset;
    trans(i) = transition(guard, reset, 4);
end
% stepTimer >= timeStep
% 11: stepTimer
eq = timeStep - vars(11);
compOp = {'<='};
guard = levelSet(eq,vars,compOp);
reset = identity_reset;
trans(length(obstacles)+1) = transition(guard, reset, 4);

locs(3) = location(inv, trans, dyn);

%% EnvironmentLoop, Loc 4: EnvironmentLoop_5 conditional (-> 6, 7)

inv = emptySet(state_size);
dyn = default_dyn;

trans = transition();
% stepTimer < timeStep - eps
% 11: stepTimer
eq = - (timeStep - eps) + vars(11);
compOp = {'<'};
guard = levelSet(eq,vars,compOp);
reset = identity_reset;
trans(1) = transition(guard, reset, 6);

% stepTimer >= timeStep - eps
% 11: stepTimer
eq = (timeStep - eps) - vars(11);
compOp = {'<='};
guard = levelSet(eq,vars,compOp);
reset = identity_reset;
trans(2) = transition(guard, reset, 7);

locs(4) = location(inv, trans, dyn);

%% EnvironmentLoop, Loc 5: unused RobotMovement end location (-> 4)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
trans(1) = transition(guard, reset, 4);

locs(5) = location(inv, trans, dyn);

%% EnvironmentLoop, Loc 6: HandleCollision_1 assignment (-> 3)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = struct('f', @(x, u) HandleCollision_1_start_trans1_reset(x, u, collisionEps));
trans(1) = transition(guard, reset, 3);

locs(6) = location(inv, trans, dyn);

%% EnvironmentLoop, Loc 7: obstacle_InputEventMapping_1 conditional (-> 9, 10, 11)

inv = emptySet(state_size);
dyn = default_dyn;

trans = transition();
% (exists obstacle1 : ran obstacles @ exists loc : obstacle1.locations @
%     (norm(loc - robot.position) < 0.5 - eps))
% 1:  robot_position_x
% 2:  robot_position_y
for i = 1:length(obstacles)
    eq = (vars(1) - obstacles(i).position_x)^2 + (vars(2) - obstacles(i).position_y)^2 - 0.5^2 + eps;
    compOp = {'<'};
    guard = levelSet(eq,vars,compOp);
    reset = identity_reset;
    trans(i) = transition(guard, reset, 9);
end

% not (exists obstacle1 : ran obstacles @ exists loc : obstacle1.locations @
%     (norm(loc - robot.position) < 0.5))
% 1:  robot_position_x
% 2:  robot_position_y
guard = fullspace(state_size);
for i = 1:length(obstacles)
    eq = - (vars(1) - obstacles(i).position_x)^2 - (vars(2) - obstacles(i).position_y)^2 + 0.5^2;
    compOp = {'<='};
    guard = guard & levelSet(eq,vars,compOp);
end
reset = identity_reset;
trans(length(obstacles)+1) = transition(guard, reset, 10);

% not (exists obstacle1 : ran obstacles @ exists loc : obstacle1.locations @
%     (norm(loc - robot.position) < 0.5 - eps))
%  /\ (exists obstacle1 : ran obstacles @ exists loc : obstacle1.locations @
%     (norm(loc - robot.position) < 0.5))
guard1 = fullspace(state_size);
for i = 1:length(obstacles)
    % iterate over universal quantifier to make combined guard
    eq = - (vars(1) - obstacles(i).position_x)^2 - (vars(2) - obstacles(i).position_y)^2 + 0.5^2 - eps;
    compOp = {'<='};
    guard1 = guard1 & levelSet(eq,vars,compOp);
end
for i = 1:length(obstacles)
    % iterate over existential quantifier to make separate transitions
    eq = (vars(1) - obstacles(i).position_x)^2 + (vars(2) - obstacles(i).position_y)^2 - 0.5^2;
    compOp = {'<'};
    guard2 = levelSet(eq,vars,compOp);
    guard = guard1 & guard2;
    reset = identity_reset;
    trans(length(obstacles)+1+i) = transition(guard, reset, 11);
end

locs(7) = location(inv, trans, dyn);

%% EnvironmentLoop, Loc 8: Communication_1 external choice start (-> 20, 25, 29, 35)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
reset.A(28,28) = 0; reset.A(28,7) = 1; % getRobotOrientation_EL_mS_hidden_arg (28) := robot_orientation (7)
trans(1) = transition(guard, reset, 17, 'getRobotOrientation_EL_mS_hidden_start');

guard = fullspace(state_size);
reset = identity_reset;
trans(2) = transition(guard, reset, 19, 'setRobotVelocity_EL_mS_hidden_start');

guard = fullspace(state_size);
reset = identity_reset;
trans(3) = transition(guard, reset, 21, 'setRobotAngularVelocity_EL_mS_hidden_start');

guard = fullspace(state_size);
reset = identity_reset;
trans(4) = transition(guard, reset, 23, 'proceed_EL_mS_hidden');

locs(8) = location(inv, trans, dyn);

%% EnvironmentLoop, Loc 9: obstacle_InputEventMapping_2 obstacleTriggered comm start (-> 12)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
reset.A(27,27) = 0; reset.c(27) = 1;  % obstacleTriggered_EL_oB_hidden_arg (27) := true
trans(1) = transition(guard, reset, 14, 'obstacleTriggered_EL_oB_hidden_start');

locs(9) = location(inv, trans, dyn);

%% EnvironmentLoop, Loc 10: obstacle_InputEventMapping_3 obstacleTriggered comm start (-> 14)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
reset.A(27,27) = 0; % obstacleTriggered_EL_oB_hidden_arg (27) := false
trans(1) = transition(guard, reset, 14, 'obstacleTriggered_EL_oB_hidden_start');

locs(10) = location(inv, trans, dyn);

%% EnvironmentLoop, Loc 11: obstacle_InputEventMapping_4 Skip (-> 8)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
trans(1) = transition(guard, reset, 8);

locs(11) = location(inv, trans, dyn);

%% EnvironmentLoop, Loc 12: obstacle_InputEventMapping_2 obstacleTriggered comm intermediate (-> 13)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
trans(1) = transition(guard, reset, 13, 'obstacleTriggered_EL_oB_hidden_end');

locs(12) = location(inv, trans, dyn);

%% EnvironmentLoop, Loc 13: obstacle_InputEventMapping_5 assignment (-> 8)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
reset.A(14,14) = 0; reset.c(14) = 1; % obstacleOccurred (14) := true
reset.A(13,13) = 0; reset.A(13,10) = 1; % obstacleTime (13) := time (10)
trans(1) = transition(guard, reset, 8, 'sync1');

locs(13) = location(inv, trans, dyn);

%% EnvironmentLoop, Loc 14: obstacle_InputEventMapping_3 targetTriggered comm intermediate (-> 15)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
trans(1) = transition(guard, reset, 15, 'obstacleTriggered_EL_oB_hidden_end');

locs(14) = location(inv, trans, dyn);

%% EnvironmentLoop, Loc 15: obstacle_InputEventMapping_3 Skip (-> 8)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
trans(1) = transition(guard, reset, 8, 'sync2');

locs(15) = location(inv, trans, dyn);

%% EnvironmentLoop, Loc 16: CheckTock_1 conditional (-> 25, 26)

inv = emptySet(state_size);
dyn = default_dyn;

trans = transition();
eq = (tockLength - eps) - vars(12); % tockTimer >= tockLength - eps
compOp = {'<='};
guard = levelSet(eq,vars,compOp);
reset = identity_reset;
trans(1) = transition(guard, reset, 25);

eq = - (tockLength - eps) + vars(12); % tockTimer < tockLength - eps
compOp = {'<'};
guard = levelSet(eq,vars,compOp);
reset = identity_reset;
trans(2) = transition(guard, reset, 26);

locs(16) = location(inv, trans, dyn);

%% EnvironmentLoop, Loc 17: Communication_1 getRobotOrientation comm intermediate (-> 18)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
trans(1) = transition(guard, reset, 18, 'getRobotOrientation_EL_mS_hidden_end');

locs(17) = location(inv, trans, dyn);

%% EnvironmentLoop, Loc 18: Communication_21 Skip start (-> 8)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
trans(1) = transition(guard, reset, 8');

locs(18) = location(inv, trans, dyn);

%% EnvironmentLoop, Loc 19: Communication_1 setRobotVelocity comm intermediate (-> 20)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
reset.B = zeros(state_size, input_size);
reset.A(17,17) = 0; reset.B(17,5) = 1; % inarg_2_x (17) := setRobotVelocity_EL_mS_hidden_x (in5)
reset.A(18,18) = 0; reset.B(18,6) = 1; % inarg_2_y (18) := setRobotVelocity_EL_mS_hidden_y (in6)
trans(1) = transition(guard, reset, 20, 'setRobotVelocity_EL_mS_hidden_end');

locs(19) = location(inv, trans, dyn);

%% EnvironmentLoop, Loc 20: Communication_25 assignment start (-> 8)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
% robot_velocity := inarg2
reset = identity_reset;
reset.A(3,3) = 0; reset.A(3,17) = 1; % robot_velocity_x (3) := inarg2_x (17)
reset.A(4,4) = 0; reset.A(4,18) = 1; % robot_velocity_y (4) := inarg2_y (18)
trans(1) = transition(guard, reset, 8);

locs(20) = location(inv, trans, dyn);

%% EnvironmentLoop, Loc 21: Communication_1 setRobotAngularVelocity comm intermediate (-> 22)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
reset.B = zeros(state_size, input_size);
reset.A(22,22) = 0; reset.B(22,7) = 1; % inarg5 (22) = setRobotAngularVelocity_EL_mS_hidden (in7)
trans(1) = transition(guard, reset, 22, 'setRobotAngularVelocity_EL_mS_hidden_end');

locs(21) = location(inv, trans, dyn);

%% EnvironmentLoop, Loc 22: Communication_28 assignment start (-> 8)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
% robot_angularVelocity := inarg5
reset = identity_reset;
reset.A(8,8) = 0; reset.A(8,22) = 1; % robot_angularVelocity (8) := inarg5 (22)
trans(1) = transition(guard, reset, 8);

locs(22) = location(inv, trans, dyn);

%% EnvironmentLoop, Loc 23: Communication_17 Skip start (-> 16)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
trans(1) = transition(guard, reset, 16);

locs(23) = location(inv, trans, dyn);

%% EnvironmentLoop, Loc 24: EnvironmentLoop_9 assignment (-> 3)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
reset.A(11,11) = 0; % stepTimer (11) := 0
trans(1) = transition(guard, reset, 3);

locs(24) = location(inv, trans, dyn);

%% EnvironmentLoop, Loc 25: CheckTock_2 tock comm (-> 27)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
trans(1) = transition(guard, reset, 27, 'tock_EL');

locs(25) = location(inv, trans, dyn);

%% EnvironmentLoop, Loc 26: CheckTock_3 Skip (-> 24)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
trans(1) = transition(guard, reset, 24);

locs(26) = location(inv, trans, dyn);

%% EnvironmentLoop, Loc 27: CheckTock_4 assignment (-> 24)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
reset.A(12,12) = 0; % tockTimer (12) := 0
trans(1) = transition(guard, reset, 24);

locs(27) = location(inv, trans, dyn);

comp(1) = hybridAutomaton(locs);

%% obstacle_Buffer

% state:
% 1: obstacleTrig
% 2: inarg9
% 3: _timer
% 4: obstacle_arg
state_size = 4;

% input:
% 1: robot_position_x
% 2: robot_position_y
% 3: robot_velocity_x
% 4: robot_velocity_y
% 5: robot_acceleration_x
% 6: robot_acceleration_y
% 7: robot_orientation
% 8: robot_angularVelocity
% 9: robot_angularAcceleration
% 10: time
% 11: stepTimer
% 12: tockTimer
% 13: obstacleTime
% 14: obstacleOccurred
% 15: moveTime
% 16: moveOccurred
% 17: inarg1_x
% 18: inarg1_y
% 19: inarg2_x
% 20: inarg2_y
% 21: inarg3_x
% 22: inarg3_y
% 23: inarg4
% 24: inarg5
% 25: inarg6
% 26: inarg7
% 27: inarg8
% 28: obstacleTriggered_inarg
input_size = 28;

% default dynamics
A = zeros(state_size,state_size);
B = zeros(state_size,input_size);
c = zeros(state_size,1);
c(3) = 1; % _timer' (3) == 1
C = eye(state_size);
D = zeros(state_size,input_size);
k = zeros(state_size,1);
default_dyn = linearSys(A, B, c, C, D, k);

% identity reset
identity_reset.A = eye(state_size);
identity_reset.c = zeros(state_size,1);

vars = sym('x',[state_size,1]);

clear locs;

%% obstacle_Buffer, Loc 1: obstacle_Buffer_2 assignment (-> 3)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
reset.A(1,1) = 0; % obstacleTrig (1) := false
trans(1) = transition(guard, reset, 3);

locs(1) = location(inv, trans, dyn);

%% obstacle_Buffer, Loc 2: unused end location

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();

locs(2) = location(inv, trans, dyn);

%% obstacle_Buffer, Loc 3: obstacle_Buffer_3 conditional (-> 4, 5)

inv = emptySet(state_size);
dyn = default_dyn;

trans = transition();
eq = 0.5 - vars(1);
compOp = {'<'};
guard = levelSet(eq,vars,compOp);
reset = identity_reset;
trans(1) = transition(guard, reset, 4, 'sync1');

eq = -0.5 + vars(1);
compOp = {'<'};
guard = levelSet(eq,vars,compOp);
reset = identity_reset;
trans(2) = transition(guard, reset, 5, 'sync2');

locs(3) = location(inv, trans, dyn);

%% obstacle_Buffer, Loc 4: obstacle_Buffer_4 external choice (-> 6, 8)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
trans(1) = transition(guard, reset, 6, 'obstacleTriggered_EL_oB_hidden_start');

guard = fullspace(state_size);
reset = identity_reset;
reset.A(4,4) = 0; % obstacle_arg (4) := in (const 0)
trans(2) = transition(guard, reset, 8, 'obstacle_oB_start');

locs(4) = location(inv, trans, dyn);

%% obstacle_Buffer, Loc 5: obstacle_Buffer_5 obstacleTriggered comm start (-> 10)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
trans(1) = transition(guard, reset, 10, 'obstacleTriggered_EL_oB_hidden_start');

locs(5) = location(inv, trans, dyn);

%% obstacle_Buffer, Loc 6: obstacle_Buffer_4 obstacleTriggered comm intermediate (-> 7)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
reset.B = zeros(state_size,input_size);
reset.A(2,2) = 0; reset.B(2,28) = 1; % inarg9 (2) := obstacleTriggered_inarg (in 28)
trans(1) = transition(guard, reset, 7, 'obstacleTriggered_EL_oB_hidden_end');

locs(6) = location(inv, trans, dyn);

%% obstacle_Buffer, Loc 7: obstacle_Buffer_9 assignment (-> 3)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
reset.A(1,1) = 0; reset.A(1,2) = 1; % obstacleTrig (1) := inarg9 (2)
trans(1) = transition(guard, reset, 3);

locs(7) = location(inv, trans, dyn);

%% obstacle_Buffer, Loc 8: obstacle_Buffer_4 obstacle comm intermediate (-> 9)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
trans(1) = transition(guard, reset, 9, 'obstacle_oB_end');

locs(8) = location(inv, trans, dyn);

%% obstacle_Buffer, Loc 9: obstacle_Buffer_10 Skip (-> 3)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
trans(1) = transition(guard, reset, 3);

locs(9) = location(inv, trans, dyn);

%% obstacle_Buffer, Loc 10: obstacle_Buffer_5 obstacleTriggered comm intermediate (-> 11)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
reset.B = zeros(state_size,input_size);
reset.A(2,2) = 0; reset.B(2,28) = 1; % inarg9 (2) := obstacleTriggered_inarg (in28)
trans(1) = transition(guard, reset, 11, 'obstacleTriggered_EL_oB_hidden_end');

locs(10) = location(inv, trans, dyn);

%% obstacle_Buffer, Loc 12: obstacle_Buffer_11 assignment (-> 3)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
reset.A(1,1) = 0; reset.A(1,2) = 1; % obstacleTrig (1) := inarg9 (2)
trans(1) = transition(guard, reset, 3);

locs(11) = location(inv, trans, dyn);

comp(2) = hybridAutomaton(locs);

%% move_Buffer

% state:
% 1: moveTime
% 2: moveOccurred
% 3: _timer
state_size = 3;

% input:
% 1: robot_position_x
% 2: robot_position_y
% 3: robot_velocity_x
% 4: robot_velocity_y
% 5: robot_acceleration_x
% 6: robot_acceleration_y
% 7: robot_orientation
% 8: robot_angularVelocity
% 9: robot_angularAcceleration
% 10: time
% 11: stepTimer
% 12: tockTimer
% 13: obstacleTime
% 14: obstacleOccurred
% 15: inarg1_x
% 16: inarg1_y
% 17: inarg2_x
% 18: inarg2_y
% 19: inarg3_x
% 20: inarg3_y
% 21: inarg4
% 22: inarg5
% 23: inarg6
% 24: inarg7
% 25: inarg8
% 26: obstacleTrig
% 27: inarg9
input_size = 28; % expand to maximum input size

% default dynamics
A = zeros(state_size,state_size);
B = zeros(state_size,input_size);
c = zeros(state_size,1);
c(3) = 1; % _timer' (3) == 1
C = eye(state_size);
D = zeros(state_size,input_size);
k = zeros(state_size,1);
default_dyn = linearSys(A, B, c, C, D, k);

% identity reset
identity_reset.A = eye(state_size);
identity_reset.c = zeros(state_size,1);

vars = sym('x',[state_size,1]);

clear locs;

%% move_Buffer, Loc 1: move_Buffer_2 assignment (-> 3)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
reset.A(1,1) = 0; % moveTime (1) := 0
reset.A(2,2) = 0; % moveOccurred (2) := false (const 0)
trans(1) = transition(guard, reset, 3);

locs(1) = location(inv, trans, dyn);

%% move_Buffer, Loc 2: unused end location

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();

locs(2) = location(inv, trans, dyn);

%% move_Buffer, Loc 3: move_Buffer_3 moveHappened comm (-> 4)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
trans(1) = transition(guard, reset, 4, 'moveHappened_mB_mM_hidden');

locs(3) = location(inv, trans, dyn);

%% move_Buffer, Loc 4: move_Buffer_5 assignment (-> 3)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
reset.B = zeros(state_size,input_size);
reset.A(2,2) = 0; % moveOccurred (2) := true (const 0)
reset.A(1,1) = 0; reset.B(1,10) = 1; % moveTime (1) := time (in10)
trans(1) = transition(guard, reset, 3);

locs(4) = location(inv, trans, dyn);

comp(3) = hybridAutomaton(locs);

%% move_Semantics

% state:
% 1: inarg10
% 2: inarg11
% 3: inarg12
% 4: _timer
% 5: setRobotVelocity_arg_x
% 6: setRobotVelocity_arg_y
% 7: setRobotAngularVelocity_arg
state_size = 7;

% input:
% 1: inarg13
% 2: inarg14
% 3: moveCall_inarg1
% 4: moveCall_inarg2
% 5: getRobotOrientation_inarg
input_size = 28; % expand to maximum input size

% default dynamics
A = zeros(state_size,state_size);
B = zeros(state_size,input_size);
c = zeros(state_size,1);
c(4) = 1; % _timer' (4) == 1
C = eye(state_size);
D = zeros(state_size,input_size);
k = zeros(state_size,1);
default_dyn = linearSys(A, B, c, C, D, k);

% identity reset
identity_reset.A = eye(state_size);
identity_reset.c = zeros(state_size,1);

vars = sym('x',[state_size,1]);

clear locs;

%% move_Semantics, Loc 1: move_Semantics_1 external choice (-> 3, 1)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
trans(1) = transition(guard, reset, 3, 'moveCall_mS_mM_start');

guard = fullspace(state_size);
reset = identity_reset;
trans(2) = transition(guard, reset, 1, 'proceed_EL_mS_hidden');

locs(1) = location(inv, trans, dyn);

%% move_Semantics, Loc 2: unused end location

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();

locs(2) = location(inv, trans, dyn);

%% move_Semantics, Loc 3: move_Semantics_1 moveCall comm intermediate (-> 4)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
reset.B = zeros(state_size,input_size);
reset.A(1,1) = 0; reset.B(1,3) = 1; % inarg10 (1) := moveCall_inarg1 (in3)
reset.A(2,2) = 0; reset.B(2,4) = 1; % inarg11 (2) := moveCall_inarg2 (in4)
trans(1) = transition(guard, reset, 4, 'moveCall_mS_mM_end');

locs(3) = location(inv, trans, dyn);

%% move_Semantics, Loc 4: move_Semantics_2 getOri comm start (-> 5)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
trans(1) = transition(guard, reset, 5, 'getRobotOrientation_EL_mS_hidden_start');

locs(4) = location(inv, trans, dyn);

%% move_Semantics, Loc 5: move_Semantics_2 getOri comm intermediate (-> 6)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
reset.B = zeros(state_size,input_size);
reset.A(3,3) = 0; reset.B(3,5) = 1; % inarg12 (3) := getRobotOrientation_inarg (in5)
trans(1) = transition(guard, reset, 6, 'getRobotOrientation_EL_mS_hidden_end');

locs(5) = location(inv, trans, dyn);

%% move_Semantics, Loc 6: move_Semantics_4 setVel comm start (-> 8)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = struct('f', @move_Semantics_4_setRobotVelocity_start_reset);
trans(1) = transition(guard, reset, 8, 'setRobotVelocity_EL_mS_hidden_start');

locs(6) = location(inv, trans, dyn);

%% move_Semantics, Loc 7: move_Semantics_5 setAngVel comm start (-> 10)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
reset.A(7,7) = 0; reset.A(7,2) = 1; % setRobotAngularVelocity_arg (7) := inarg11 (2)
trans(1) = transition(guard, reset, 10, 'setRobotAngularVelocity_EL_mS_hidden_start');

locs(7) = location(inv, trans, dyn);

%% move_Semantics, Loc 8: move_Semantics_4 setVel comm intermediate (-> 9)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
trans(1) = transition(guard, reset, 9, 'setRobotVelocity_EL_mS_hidden_end');

locs(8) = location(inv, trans, dyn);

%% move_Semantics, Loc 9: move_Semantics_6 Skip (-> 7)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
trans(1) = transition(guard, reset, 7);

locs(9) = location(inv, trans, dyn);

%% move_Semantics, Loc 10: move_Semantics_5 setAngVel comm intermediate (-> 11)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
trans(1) = transition(guard, reset, 11, 'setRobotAngularVelocity_EL_mS_hidden_end');

locs(10) = location(inv, trans, dyn);

%% move_Semantics, Loc 11: move_Semantics_8 Skip (-> 1)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
trans(1) = transition(guard, reset, 1);

locs(11) = location(inv, trans, dyn);

comp(4) = hybridAutomaton(locs);

%% move_Monitor

% state:
% 1: inarg13
% 2: inarg14
% 3: _timer
state_size = 3;

% input:
% 1: inarg10
% 2: inarg11
% 3: inarg12
% 4: moveCall_inarg1
% 5: moveCall_inarg2
input_size = 28; % expand to maximum input size

% default dynamics
A = zeros(state_size,state_size);
B = zeros(state_size,input_size);
c = zeros(state_size,1);
c(3) = 1; % _timer' (3) == 1
C = eye(state_size);
D = zeros(state_size,input_size);
k = zeros(state_size,1);
default_dyn = linearSys(A, B, c, C, D, k);

% identity reset
identity_reset.A = eye(state_size);
identity_reset.c = zeros(state_size,1);

vars = sym('x',[state_size,1]);

clear locs;

%% move_Monitor, Loc 1: move_Monitor_1 moveCall comm start (-> 3)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
trans(1) = transition(guard, reset, 3, 'moveCall_mS_mM_start');

locs(1) = location(inv, trans, dyn);

%% move_Monitor, Loc 2: unused end location

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();

locs(2) = location(inv, trans, dyn);

%% move_Monitor, Loc 3: move_Monitor_1 moveCall comm end (-> 4)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
reset.B = zeros(state_size,input_size);
reset.A(1,1) = 0; reset.B(1,3) = 1; % inarg13 (1) := moveCall_inarg1 (in3)
reset.A(2,2) = 0; reset.B(2,4) = 1; % inarg14 (2) := moveCall_inarg2 (in4)
trans(1) = transition(guard, reset, 4, 'moveCall_mS_mM_end');

locs(3) = location(inv, trans, dyn);

%% move_Monitor, Loc 4: move_Monitor_1 moveHappened comm  (-> 1)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
trans(1) = transition(guard, reset, 1, 'moveHappened_mB_mM_hidden');

locs(4) = location(inv, trans, dyn);

comp(5) = hybridAutomaton(locs);

%% composition

iBinds = {};

% EnvironmentLoop inputs:
iBinds{1} = [
    [3,1]; % 1: moveTime - link to moveTime (1) in move_Buffer (3)
    [3,2]; % 2: moveOccurred - link to moveOccurred (2) in move_Buffer (3)
    [2,1]; % 3: obstacleTrig - link to obstacleTrig (1) in obstacle_Buffer (2)
    [2,2]; % 4: inarg9 - link to inarg9 (2) in obstacle_Buffer (2)
    [4,5]; % 5: setRobotVelocity_EL_mS_hidden_x
    %       - link to setRobotVelocity_EL_mS_hidden_arg_x (5) in move_Semantics (4)
    [4,6]; % 6: setRobotVelocity_EL_mS_hidden_y
    %       - link to setRobotVelocity_EL_mS_hidden_arg_x (6) in move_Semantics (4)
    [4,7] % 7: setRobotAngularVelocity_EL_mS_hidden
    %       - link to setRobotAngularVelocity_EL_mS_hidden_arg (7) in move_Semantics (4)
    [0,1]; [0,2]; [0,3]; [0,4]; [0,5]; [0,6]; [0,7]; % dummy inputs 8 to 14
    [0,8]; [0,9]; [0,10]; [0,11]; [0,12]; [0,13]; [0,14]; % dummy inputs 15 to 21
    [0,15]; [0,16]; [0,17]; [0,18]; [0,19]; [0,20]; [0,21];  % dummy inputs 22 to 28
];

% obstacle_Buffer input:
iBinds{2} = [
    [1,1]; % 1: robot_position_x - link to robot_position_x (1) in EnvironmentLoop (1)
    [1,2]; % 2: robot_position_y - link to robot_position_y (2) in EnvironmentLoop (1)
    [1,3]; % 3: robot_velocity_x - link to robot_velocity_x (3) in EnvironmentLoop (1)
    [1,4]; % 4: robot_velocity_y - link to robot_velocity_y (4) in EnvironmentLoop (1)
    [1,5]; % 5: robot_acceleration_x - link to robot_acceleration_x (5) in EnvironmentLoop (1)
    [1,6]; % 6: robot_acceleration_y - link to robot_acceleration_y (6) in EnvironmentLoop (1)
    [1,7]; % 7: robot_orientation - link to robot_orientation (7) in EnvironmentLoop (1)
    [1,8]; % 8: robot_angularVelocity - link to robot_angularVelocity (8) in EnvironmentLoop (1)
    [1,9]; % 9: robot_angularAcceleration - link to robot_angularAcceleration (9) in EnvironmentLoop (1)
    [1,10]; % 10: time - link to time (10) in EnvironmentLoop (1)
    [1,11]; % 11: stepTimer - link to stepTimer (11) in EnvironmentLoop (1)
    [1,12]; % 12: tockTimer - link to tockTimer (12) in EnvironmentLoop (1)
    [1,13]; % 13: obstacleTime - link to obstacleTime (13) in EnvironmentLoop (1)
    [1,14]; % 14: obstacleOccurred - link to obstacleOccurred (14) in EnvironmentLoop (1)
    [3,1]; % 15: moveTime - link to moveTime (1) in move_Buffer (3)
    [3,2]; % 16: moveOccurred - link to moveOccurred (2) in move_Buffer (3)
    [1,15]; % 17: inarg1_x - link to inarg1_x (15) in EnviromentLoop (1)
    [1,16]; % 18: inarg1_y - link to inarg1_y (16) in EnviromentLoop (1)
    [1,17]; % 19: inarg2_x - link to inarg2_x (17) in EnviromentLoop (1)
    [1,18]; % 20: inarg2_y - link to inarg2_y (18) in EnviromentLoop (1)
    [1,19]; % 21: inarg3_x - link to inarg3_x (19) in EnviromentLoop (1)
    [1,20]; % 22: inarg3_y - link to inarg3_y (20) in EnviromentLoop (1)
    [1,21]; % 23: inarg4 - link to inarg4 (21) in EnviromentLoop (1)
    [1,22]; % 24: inarg5 - link to inarg5 (22) in EnviromentLoop (1)
    [1,23]; % 25: inarg6 - link to inarg6 (23) in EnviromentLoop (1)
    [1,24]; % 26: inarg7 - link to inarg7 (24) in EnviromentLoop (1)
    [1,25]; % 27: inarg8 - link to inarg8 (25) in EnviromentLoop (1)
    [1,27]; % 28: obstacleTriggered_EL_oB_hidden_inarg
    %       - link to obstacleTriggered_EL_oB_hidden_arg (27) in EnvironmentLoop (1)
];

% move_Buffer input:
iBinds{3} = [
    [1,1]; % 1: robot_position_x - link to robot_position_x (1) in EnvironmentLoop (1)
    [1,2]; % 2: robot_position_y - link to robot_position_y (2) in EnvironmentLoop (1)
    [1,3]; % 3: robot_velocity_x - link to robot_velocity_x (3) in EnvironmentLoop (1)
    [1,4]; % 4: robot_velocity_y - link to robot_velocity_y (4) in EnvironmentLoop (1)
    [1,5]; % 5: robot_acceleration_x - link to robot_acceleration_x (5) in EnvironmentLoop (1)
    [1,6]; % 6: robot_acceleration_y - link to robot_acceleration_y (6) in EnvironmentLoop (1)
    [1,7]; % 7: robot_orientation - link to robot_orientation (7) in EnvironmentLoop (1)
    [1,8]; % 8: robot_angularVelocity - link to robot_angularVelocity (8) in EnvironmentLoop (1)
    [1,9]; % 9: robot_angularAcceleration - link to robot_angularAcceleration (9) in EnvironmentLoop (1)
    [1,10]; % 10: time - link to time (10) in EnvironmentLoop (1)
    [1,11]; % 11: stepTimer - link to stepTimer (11) in EnvironmentLoop (1)
    [1,12]; % 12: tockTimer - link to tockTimer (12) in EnvironmentLoop (1)
    [1,13]; % 13: obstacleTime - link to obstacleTime (13) in EnvironmentLoop (1)
    [1,14]; % 14: obstacleOccurred - link to obstacleOccurred (14) in EnvironmentLoop (1)
    [1,15]; % 15: inarg1_x - link to inarg1_x (15) in EnvironmentLoop (1)
    [1,16]; % 16: inarg1_y - link to inarg1_y (16) in EnvironmentLoop (1)
    [1,17]; % 17: inarg2_x - link to inarg2_x (17) in EnvironmentLoop (1)
    [1,18]; % 18: inarg2_y - link to inarg2_y (18) in EnvironmentLoop (1)
    [1,19]; % 19: inarg3_x - link to inarg3_x (19) in EnvironmentLoop (1)
    [1,20]; % 20: inarg3_y - link to inarg3_y (20) in EnvironmentLoop (1)
    [1,21]; % 21: inarg4 - link to inarg4 (21) in EnvironmentLoop (1)
    [1,22]; % 22: inarg5 - link to inarg5 (22) in EnvironmentLoop (1)
    [1,23]; % 23: inarg6 - link to inarg6 (23) in EnvironmentLoop (1)
    [1,24]; % 24: inarg7 - link to inarg7 (24) in EnvironmentLoop (1)
    [1,25]; % 25: inarg8 - link to inarg8 (25) in EnvironmentLoop (1)
    [2,1]; % 26: obstacleTrig - link to obstacleTrig (1) in obstacle_Buffer (2)
    [2,2]; % 27: inarg9 - link to inarg9 (2) in obstacle_Buffer (2)
    [0,22]; % dummy input 28
];

% move_Semantics input:
iBinds{4} = [
    [5,1]; % 1: inarg13 - link to inarg13 (1) in move_Monitor (5)
    [5,2]; % 2: inarg14 - link to inarg14 (2) in move_Monitor (5)
    [0,1]; % 3: moveCall_mS_mM_inarg1 - link to external input 1
    [0,2]; % 4: moveCall_mS_mM_inarg2 - link to external input 2
    [1,28]; % 5: getRobotOrientation_EL_mS_hidden_inarg
    %       - link to getRobotOrientation_EL_mS_hidden_arg (28) in EnviromentLoop (1)
    [0,23]; [0,24]; [0,25]; [0,26]; [0,27]; [0,28]; [0,1]; [0,2]; % dummy inputs 6 to 13
    [0,3]; [0,4]; [0,5]; [0,6]; [0,7]; [0,8]; [0,9]; [0,10]; % dummy inputs 14 to 21
    [0,11]; [0,12]; [0,13]; [0,14]; [0,15]; [0,16]; [0,17];  % dummy inputs 22 to 28
];

% move_Monitor input:
iBinds{5} = [
    [4,1]; % 1: inarg10 - link to inarg10 (1) in move_Semantics (4)
    [4,2]; % 2: inarg11 - link to inarg11 (2) in move_Semantics (4)
    [4,3]; % 3: inarg12 - link to inarg12 (3) in move_Semantics (4)
    [0,1]; % 4: moveCall_mS_mM_inarg1 - link to external input 1
    [0,2]; % 5: moveCall_mS_mM_inarg2 - link to external input 2
    [0,18]; [0,19]; [0,20]; [0,21]; [0,22]; [0,23]; [0,24]; [0,25]; % dummy inputs 6 to 13
    [0,26]; [0,27]; [0,28]; [0,1]; [0,2]; [0,3]; [0,4]; [0,5]; % dummy inputs 14 to 21
    [0,6]; [0,7]; [0,8]; [0,9]; [0,10]; [0,11]; [0,12];  % dummy inputs 22 to 28
];

pHA = parallelHybridAutomaton(comp, iBinds);

%% reachability checking


c = [...
    % STATE (EnvironmentLoop)
    0; % 1: robot_position_x
    3; % 2: robot_position_y
    0; % 3: robot_velocity_x
    0; % 4: robot_velocity_y
    0; % 5: robot_acceleration_x
    0; % 6: robot_acceleration_y
    0; % 7: robot_orientation
    0; % 8: robot_angularVelocity
    0; % 9: robot_angularAcceleration
    0; % 10: time
    0; % 11: stepTimer
    0; % 12: tockTimer
    0; % 13: obstacleTime
    0; % 14: obstacleOccurred
    0; % 15: inarg1_x
    0; % 16: inarg1_y
    0; % 17: inarg2_x
    0; % 18: inarg2_y
    0; % 19: inarg3_x
    0; % 20: inarg3_y
    0; % 21: inarg4
    0; % 22: inarg5
    0; % 23: inarg6
    0; % 24: inarg7
    0; % 25: inarg8
    0; % 26: _timer
    0; % 27: obstacleTriggered_EL_oB_hidden_arg
    0; % 28: getRobotOrientation_EL_mS_hidden_arg
    % STATE (obstacle_Buffer)
    0; % 29: obstacleTrig
    0; % 30: inarg9
    0; % 31: _timer
    0; % 32: obstacle_arg
    % STATE (move_Buffer)
    0; % 33: moveTime
    0; % 34: moveOccurred
    0; % 35: _timer
    % STATE (move_Semantics)
    0; % 36: inarg10
    0; % 37: inarg11
    0; % 38: inarg12
    0; % 39: _timer
    0; % 40: setRobotVelocity_arg_x
    0; % 41: setRobotVelocity_arg_y
    0; % 42: setRobotAngularVelocity_arg
    % STATE (move_Monitor)
    0; % 43: inarg13
    0; % 44: inarg14
    0; % 45: _timer
];

G = [...
    % STATE (EnvironmentLoop)
    0; % 1: robot_position_x
    0; % 2: robot_position_y
    0; % 3: robot_velocity_x
    0; % 4: robot_velocity_y
    0; % 5: robot_acceleration_x
    0; % 6: robot_acceleration_y
    0; % 7: robot_orientation
    0; % 8: robot_angularVelocity
    0; % 9: robot_angularAcceleration
    0; % 10: time
    0; % 11: stepTimer
    0; % 12: tockTimer
    0; % 13: obstacleTime
    0; % 14: obstacleOccurred
    0; % 15: inarg1_x
    0; % 16: inarg1_y
    0; % 17: inarg2_x
    0; % 18: inarg2_y
    0; % 19: inarg3_x
    0; % 20: inarg3_y
    0; % 21: inarg4
    0; % 22: inarg5
    0; % 23: inarg6
    0; % 24: inarg7
    0; % 25: inarg8
    0; % 26: _timer
    0; % 27: obstacleTriggered_EL_oB_hidden_arg
    0; % 28: getRobotOrientation_EL_mS_hidden_arg
    % STATE (obstacle_Buffer)
    0; % 29: obstacleTrig
    0; % 30: inarg9
    0; % 31: _timer
    0; % 32: obstacle_arg
    % STATE (move_Buffer)
    0; % 33: moveTime
    0; % 34: moveOccurred
    0; % 35: _timer
    % STATE (move_Semantics)
    0; % 36: inarg10
    0; % 37: inarg11
    0; % 38: inarg12
    0; % 39: _timer
    0; % 40: setRobotVelocity_arg_x
    0; % 41: setRobotVelocity_arg_y
    0; % 42: setRobotAngularVelocity_arg
    % STATE (move_Monitor)
    0; % 43: inarg13
    0; % 44: inarg14
    0; % 45: _timer
];

% start time
params.tStart = 0;
% final time
params.tFinal = 10;
% start locations:
% EnvironmentLoop_1 start: 27
% obstacle_Buffer_1 start: 1
% move_Buffer_1 start: 1
% move_Semantics_1 start: 1
% move_Monitor_1 start: 1
params.startLoc = [1; 1; 1; 1; 1];
params.R0 = zonotope(c,G);
params.U = zonotope(zeros(28,1), zeros(28,1));
%params.tu = [0];
% ...

options.verbose = false;
options.timeStep = 0.0001;
options.taylorTerms = 1;
options.zonotopeOrder = 1;
options.linAlg = 'standard';
options.guardOrder = 1;
options.guardIntersect = 'levelSet';
%options.enclose = {'box'};
%options.error = 0.1;

tic
R = reach(pHA, params, options);
tComp = toc;
disp("Analysis time: " + tComp);

% filter out reach sets that don't have time interval sets
Rfiltered = R(arrayfun(@(x) not(isempty(x.timeInterval)), R));

%% checks of properties
disp('');

% avoid {x : R^n | (x1 - 3)^2 + (x2 - 5)^2} <= 0.1^2
vars = sym('x', [pHA.nrOfOutputs,1]);
eq = (vars(1) - 3)^2 + (vars(2) - 3)^2 - 0.1^2;
compOp = {'<='};
S = levelSet(eq, vars, compOp);
spec = specification(S, 'unsafeSet');
check1 = spec.check(Rfiltered);

if check1
    disp('Avoid {x : R^n | (x1 - 3)^2 + (x2 - 5)^2 <= 0.1^2}: true');
else
    disp('Avoid {x : R^n | (x1 - 3)^2 + (x2 - 5)^2 <= 0.1^2}: false');
end

% avoid {x : R^n | (x1 - 5)^2 + (x2 - 3)^2} <= 0.1^2
vars = sym('x', [pHA.nrOfOutputs,1]);
eq = (vars(1) - 3)^2 + (vars(2) - 5)^2 - 0.1^2;
compOp = {'<='};
S = levelSet(eq, vars, compOp);
spec = specification(S, 'unsafeSet');
check2 = spec.check(Rfiltered);

if check2
    disp('Avoid {x : R^n | (x1 - 5)^2 + (x2 - 3)^2 <= 0.1^2}: true');
else
    disp('Avoid {x : R^n | (x1 - 5)^2 + (x2 - 3)^2 <= 0.1^2}: false');
end

check3 = 1;
for i = 1:length(R)
    if R(i).loc(1) == 6
        check3 = 0;
    end
end

if check3
    disp('Avoid location 6: true');
else
    disp('Avoid location 6: false');
end

%% functions

function [x2] = HandleCollision_1_start_trans1_reset(x, u, collisionEps)
    % Assign type of input to output variable, type safety feature
    if isa(x,'double')
	    x_R(1,1) = 0;
    else
	    x_R(1,1) = sym(0);
    end

    inp = u(1);
    
    % robot.position := robot.position-(collisionEps/2)*(2 * robot.velocity - collisionEps * robot.acceleration)
    % robot.velocity := robot.velocity - collisionEps * robot.acceleration
    % STATE
    % 1: robot_position_x
    % 2: robot_position_y
    % 3: robot_velocity_x
    % 4: robot_velocity_y
    % 5: robot_acceleration_x
    % 6: robot_acceleration_y
    % 7: robot_orientation
    % 8: robot_angularVelocity
    % 9: robot_angularAcceleration
    % 10: time
    % 11: stepTimer
    % 12: tockTimer
    % 13: obstacleTime
    % 14: obstacleOccurred
    % 15: inarg1_x
    % 16: inarg1_y
    % 17: inarg2_x
    % 18: inarg2_y
    % 19: inarg3_x
    % 20: inarg3_y
    % 21: inarg4
    % 22: inarg5
    % 23: inarg6
    % 24: inarg7
    % 25: inarg8
    % 26: _timer
    % 27: obstacleTriggered_EL_oB_hidden_arg
    % 28: getRobotOrientation_EL_mS_hidden_arg
    x2(1,1) = x(1) - (collisionEps/2) * (2 * x(3) - collisionEps * x(5));
    x2(2,1) = x(2) - (collisionEps/2) * (2 * x(4) - collisionEps * x(6));
    x2(3,1) = x(3) - collisionEps * x(5);
    x2(4,1) = x(4) - collisionEps * x(6);
    x2(5,1) = x(5);
    x2(6,1) = x(6);
    x2(7,1) = x(7);
    x2(8,1) = x(8);
    x2(9,1) = x(9);
    x2(10,1) = x(10);
    x2(11,1) = x(11);
    x2(12,1) = x(12);
    x2(13,1) = x(13);
    x2(14,1) = x(14);
    x2(15,1) = x(15);
    x2(16,1) = x(16);
    x2(17,1) = x(17);
    x2(18,1) = x(18);
    x2(19,1) = x(19);
    x2(20,1) = x(20);
    x2(21,1) = x(21);
    x2(22,1) = x(22);
    x2(23,1) = x(23);
    x2(24,1) = x(24);
    x2(25,1) = x(25);
    x2(26,1) = x(26);
    x2(27,1) = x(27);
    x2(28,1) = x(28);
end

function [x2] = move_Semantics_4_setRobotVelocity_start_reset(x, u)
    % Assign type of input to output variable, type safety feature
    if isa(x,'double')
	    x_R(1,1) = 0;
    else
	    x_R(1,1) = sym(0);
    end

    inp = u(1:5);

    % setRobotVelocity!(lv * orientationToVector(robotOri))
    % state:
    % 1: inarg10
    % 2: inarg11
    % 3: inarg12
    % 4: _timer
    % 5: setRobotVelocity_arg_x
    % 6: setRobotVelocity_arg_y
    % 7: setRobotAngularVelocity_arg
    x2(1,1) = x(1);
    x2(2,1) = x(2);
    % setRobotVelocity_arg_x := inarg10 (1) * cos(inarg12 (3))
    x2(3,1) = x(1) * cos(x(3));
    % setRobotVelocity_arg_y := inarg10 (1) * sin(inarg12 (3))
    x2(4,1) = x(1) * sin(x(3));
    x2(5,1) = x(5);
    x2(6,1) = x(6);
    x2(7,1) = x(7);
end
