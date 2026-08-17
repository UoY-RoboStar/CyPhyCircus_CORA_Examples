function pHA = navigation_translation(eps, obstacles, numObstacles, arena, tockLength, timeStep, collisionEps)

%% EnvironmentLoop

% EnvironmentLoop state:
% 1: robot_pos_x
% 2: robot_pos_y
% 3: robot_vel_x
% 4: robot_vel_y
% 5: robot_acc_x
% 6: robot_acc_y
% 7: robot_ori
% 8: robot_angVel
% 9: robot_angAcc
% 10: time
% 11: stepTimer
% 12: tockTimer
% 13: obstacleTime
% 14: obstacleOccurred
% 15: value
% 16: inarg1
% 17: inarg2_x
% 18: inarg2_y
% 19: inarg3_x
% 20: inarg3_y
% 21: inarg4_x
% 22: inarg4_y
% 23: inarg5
% 24: inarg6
% 25: inarg7
% 26: inarg8
% 27: inarg9
% 28: _timer
% 29: obstacleTriggered_EL_oB_hidden_arg1
% 30: obstacleTriggered_EL_oB_hidden_arg2
% 31: getRobotPosition_EL_mTLS_hidden_arg_x
% 32: getRobotPosition_EL_mTLS_hidden_arg_y
state_size = 32;

% EnvironmentLoop inputs:
% 1: moveToLocationTime
% 2: moveToLocationOccurred
% 3: obstacleTrig
% 4: params
% 5: inarg10
% 6: inarg11
% 7: obstacleTriggered_EL_oB_hidden_inarg2
% 8: setRobotVelocity_EL_mTLS_hidden_inarg_x
% 9: setRobotVelocity_EL_mTLS_hidden_inarg_y
% 10: setRobotOrientation_EL_mTLS_hidden_inarg
input_size = 31;  % expand to maximum input size


% default dynamics
A = zeros(state_size,state_size);
B = zeros(state_size,input_size);
c = zeros(state_size,1);
c(28) = 1; % _timer' (28) == 1
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

%% EnvironmentLoop, Loc 1: EnvironmentLoopStateInit schema start (-> 3)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
reset.A(10,10) = 0; % time (10) := 0
reset.A(11,11) = 0; % stepTimer (11) := 0
reset.A(12,12) = 0; % tockTImer (12) := 0
reset.A(13,13) = 0; % obstacleTime (13) := 0
reset.A(14,14) = 0; % obstacleOccurred (14) := false (const 0)
trans(1) = transition(guard, reset, 3, 'sync2');

locs(1) = location(inv, trans, dyn);

%% EnvironmentLoop, Loc 2: unused end location

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();

locs(2) = location(inv, trans, dyn);

%% EnvironmentLoop, Loc 3: RobotMovement schema start (-> 4)

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
c(28) = 1; % _timer' (26) = 1
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

locs(5) = location(inv, trans, dyn);

%% EnvironmentLoop, Loc 6: HandleCollision_1 schema start (-> 3)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
% nonlinear reset
reset = struct('f', @(x,u) HandleCollision_start_trans1_reset(x, u, collisionEps));
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

%% EnvironmentLoop, Loc 8: Communication_1 external choice (-> 18, 20, 22, 24)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();

guard = fullspace(state_size);
reset = identity_reset;
reset.A(31,31) = 0; reset.A(31,1) = 1; % getRobotPosition_EL_mTLS_hidden_arg_x (31) := robot_pos_x (1)
reset.A(32,32) = 0; reset.A(32,2) = 1; % getRobotPosition_EL_mTLS_hidden_arg_y (32) := robot_pos_y (2)
trans(1) = transition(guard, reset, 18, 'getRobotPosition_EL_mTLS_hidden_start');

guard = fullspace(state_size);
reset = identity_reset;
trans(2) = transition(guard, reset, 20, 'setRobotVelocity_EL_mTLS_hidden_start');

guard = fullspace(state_size);
reset = identity_reset;
trans(3) = transition(guard, reset, 22, 'setRobotOrientation_EL_mTLS_hidden_start');

guard = fullspace(state_size);
reset = identity_reset;
trans(4) = transition(guard, reset, 24, 'proceed_EL_mTLS_hidden');

locs(8) = location(inv, trans, dyn);

%% EnvironmentLoop, Loc 9: obstacle_InputEventMapping_5_DetermineValue schema start (-> 12)

inv = emptySet(state_size);
dyn = default_dyn;

trans = transition();
for obs1  = 1:length(obstacles)
    guard = fullspace(state_size);
    for obs2 = 1:length(obstacles)
        if obs1 == obs2
            continue
        end
        lhs_exp1_x = obstacles(obs1).position_x; % obs1.position (x component)
        lhs_exp1_y = obstacles(obs1).position_y; % obs1.position (y component)
        lhs_exp2_x = vars(1); % robot.position (x component)
        lhs_exp2_y = vars(2); % robot.position (y component)
        lhs_exp3_x = lhs_exp1_x - lhs_exp2_x; % obs1.position - robot.position (x component)
        lhs_exp3_y = lhs_exp1_y - lhs_exp2_y; % obs1.position - robot.position (y component)
        lhs = lhs_exp3_x^2 + lhs_exp3_y^2; % norm(obs1.position - robot.position)
        rhs_exp1_x = obstacles(obs2).position_x; % obs2.position (x component)
        rhs_exp1_y = obstacles(obs2).position_y; % obs2.position (y component)
        rhs_exp2_x = vars(1); % robot.position (x component)
        rhs_exp2_y = vars(2); % robot.position (y component)
        rhs_exp3_x = rhs_exp1_x - rhs_exp2_x; % obs2.position - robot.position (x component)
        rhs_exp3_y = rhs_exp1_y - rhs_exp2_y; % obs2.position - robot.position (y component)
        rhs = rhs_exp3_x^2 + rhs_exp3_y^2; % norm(obs2.position - robot.position)
        eq = lhs - rhs;
        compOp = '<=';
        guard = guard & levelSet(eq, vars, compOp);
    end
    reset = struct('f', @(x,u) obstacle_InputEventMapping_5_start_trans1_reset(x, u, obstacles(obs1)));
    trans(obs1) = transition(guard, reset, 12);
end

locs(9) = location(inv, trans, dyn);

%% EnvironmentLoop, Loc 10: obstacle_InputEventMapping_3 obstacleTriggered comm start (-> 15)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
reset.A(29, 29) = 0; % obstacleTriggered_EL_oB_hidden_arg1 (29) := false (const 0)
trans(1) = transition(guard, reset, 15, 'obstacleTriggered_EL_oB_hidden_start');

locs(10) = location(inv, trans, dyn);

%% EnvironmentLoop, Loc 11: obstacle_InputEventMapping_4 Skip (-> 8)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
trans(1) = transition(guard, reset, 8);

locs(11) = location(inv, trans, dyn);

%% EnvironmentLoop, Loc 12: obstacle_InputEventMapping_6 obstacleTriggered comm start (-> 13)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
reset.A(29,29) = 0; reset.c(29) = 1; % obstacleTriggered_EL_oB_hidden_arg1 (29) := true (const 1)
reset.A(30,30) = 0; reset.A(30,15) = 1; % obstacleTriggered_EL_oB_hidden_arg2 (30) := value (15)
trans(1) = transition(guard, reset, 13, 'obstacleTriggered_EL_oB_hidden_start');

locs(12) = location(inv, trans, dyn);

%% EnvironmentLoop, Loc 13: obstacle_InputEventMapping_3 obstacleTriggered comm intermediate (-> 14)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
reset.B = zeros(state_size,input_size);
reset.A(16,16) = 0; reset.B(16,7) = 1; % inarg1 (16) := obstacleTriggered_EL_oB_hidden_inarg2 (in7)
trans(1) = transition(guard, reset, 14, 'obstacleTriggered_EL_oB_hidden_end');

locs(13) = location(inv, trans, dyn);

%% EnvironmentLoop, Loc 14: obstacle_InputEventMapping_8 assignment (-> 8)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
reset.A(14,14) = 0; reset.c(14) = 1; % obstacleOccurred (14) := true (const 1)
reset.A(13,13) = 0; reset.A(13,10) = 1; % obstacleTime (13) := time (10)
trans(1) = transition(guard, reset, 8, 'sync1');

locs(14) = location(inv, trans, dyn);

%% EnvironmentLoop, Loc 15: obstacle_InputEventMapping_6 obstacleTriggered comm intermediate (-> 16)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
trans(1) = transition(guard, reset, 16, 'obstacleTriggered_EL_oB_hidden_end');

locs(15) = location(inv, trans, dyn);

%% EnvironmentLoop, Loc 16: obstacle_InputEventMapping_7 Skip (-> 8)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
trans(1) = transition(guard, reset, 8, 'sync2');

locs(16) = location(inv, trans, dyn);

%% EnvironmentLoop, Loc 17: CheckTock_1 conditional (-> 26, 27)

inv = emptySet(state_size);
dyn = default_dyn;

trans = transition();
eq = (tockLength - eps) - vars(12); % tockTimer >= tockLength - eps
compOp = {'<='};
guard = levelSet(eq,vars,compOp);
reset = identity_reset;
trans(1) = transition(guard, reset, 26);

eq = - (tockLength - eps) + vars(12); % tockTimer < tockLength - eps
compOp = {'<'};
guard = levelSet(eq,vars,compOp);
reset = identity_reset;
trans(2) = transition(guard, reset, 27);

locs(17) = location(inv, trans, dyn);

%% EnvironmentLoop, Loc 18: Communication_1 getRobotPosition comm intermediate (-> 19)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
trans(1) = transition(guard, reset, 19, 'getRobotPosition_EL_mTLS_hidden_end');

locs(18) = location(inv, trans, dyn);

%% EnvironmentLoop, Loc 19: Communication_18 Skip (-> 8)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
trans(1) = transition(guard, reset, 8);

locs(19) = location(inv, trans, dyn);

%% EnvironmentLoop, Loc 20: Communication_1 setRobotVelocity comm end (-> 21)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
reset.B = zeros(state_size,input_size);
reset.A(19,19) = 0; reset.B(19,8) = 1; % inarg3_x (19) := setRobotVelocity_EL_mTLS_hidden_inarg_x (in8)
reset.A(20,20) = 0; reset.B(20,9) = 1; % inarg3_y (20) := setRobotVelocity_EL_mTLS_hidden_inarg_y (in9)
trans(1) = transition(guard, reset, 21, 'setRobotVelocity_EL_mTLS_hidden_end');

locs(20) = location(inv, trans, dyn);

%% EnvironmentLoop, Loc 21: Communication_25 assignment (-> 8)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
reset.A(3,3) = 0; reset.A(3,19) = 1; % robot_vel_x (3) := inarg3_x (19)
reset.A(4,4) = 0; reset.A(4,20) = 1; % robot_vel_y (4) := inarg3_y (20)
trans(1) = transition(guard, reset, 8);

locs(21) = location(inv, trans, dyn);

%% EnvironmentLoop, Loc 22: Communication_1 setRobotOrientation comm end (-> 23)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
reset.B = zeros(state_size,input_size);
reset.A(23,23) = 0; reset.B(23,10) = 1; % inarg5 (23) = setRobotOrientation_EL_mTLS_hidden_inarg (in10)
trans(1) = transition(guard, reset, 23, 'setRobotOrientation_EL_mTLS_hidden');

locs(22) = location(inv, trans, dyn);

%% EnvironmentLoop, Loc 23: Communication_27 assignment (-> 8)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
reset.A(7,7) = 0; reset.A(7,23) = 1; % robot_ori (7) := inarg5 (23)
trans(1) = transition(guard, reset, 8);

locs(23) = location(inv, trans, dyn);

%% EnvironmentLoop, Loc 24: Communication_17 Skip (-> 17)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
trans(1) = transition(guard, reset, 17);

locs(24) = location(inv, trans, dyn);

%% EnvironmentLoop, Loc 25: EnvironmentLoop_9 assignment (-> 3)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
reset.A(11,11) = 0; % stepTimer (11) := 0
trans(1) = transition(guard, reset, 3);

locs(25) = location(inv, trans, dyn);

%% EnvironmentLoop, Loc 26: CheckTock_2 tock comm (-> 28)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
trans(1) = transition(guard, reset, 28, 'tock_EL');

locs(26) = location(inv, trans, dyn);

%% EnvironmentLoop, Loc 27: CheckTock_3 Skip (-> 25)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
trans(1) = transition(guard, reset, 25);

locs(27) = location(inv, trans, dyn);

%% EnvironmentLoop, Loc 28: CheckTock_4 assignment (-> 25)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
reset.A(12,12) = 0; % tockTimer (12) := 0
trans(1) = transition(guard, reset, 25);

locs(28) = location(inv, trans, dyn);


comp(1) = hybridAutomaton(locs);


%% obstacle_Buffer

% obstacle_Buffer state: obstacleTrig, params, inarg10, inarg11
% 1: obstacleTrig
% 2: params
% 3: inarg10
% 4: inarg11
% 5: _timer
% 6: obstacle_oB_arg
state_size = 6;

% obstacle_Buffer inputs:
% 1: robot_pos_x
% 2: robot_pos_y
% 3: robot_vel_x
% 4: robot_vel_y
% 5: robot_acc_x
% 6: robot_acc_y
% 7: robot_ori
% 8: robot_angVel
% 9: robot_angAcc
% 10: time
% 11: stepTimer
% 12: tockTimer
% 13: obstacleTime
% 14: obstacleOccurred
% 15: moveToLocationTime
% 16: moveToLocationOccurred
% 17: value
% 18: inarg1
% 19: inarg2_x
% 20: inarg2_y
% 21: inarg3_x
% 22: inarg3_y
% 23: inarg4_x
% 24: inarg4_y
% 25: inarg5
% 26: inarg6
% 27: inarg7
% 28: inarg8
% 29: inarg9
% 30: obstacleTriggered_EL_oB_hidden_inarg1
% 31: obstacleTriggered_EL_oB_hidden_inarg2
input_size = 31;

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
reset.A(1,1) = 0; % obstacleTrig (1) := false (const 0)
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
reset.A(6,6) = 0; % obstacle_oB_arg (6) = in (const 0)
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
reset.B = zeros(state_size, input_size);
reset.A(3,3) = 0; reset.B(3,30) = 1; % inarg10 (3) := obstacleTriggered_EL_oB_hidden_inarg1 (in30)
reset.A(4,4) = 0; reset.B(4,31) = 1; % inarg11 (4) := obstacleTriggered_EL_oB_hidden_inarg2 (in31)
trans(1) = transition(guard, reset, 7, 'obstacleTriggered_EL_oB_hidden_end');

locs(6) = location(inv, trans, dyn);

%% obstacle_Buffer, Loc 7: obstacle_Buffer_9 assignment (-> 3)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
reset.A(1,1) = 0; reset.A(1,3) = 1; % obstacleTrig (1) := inarg10 (3)
reset.A(2,2) = 0; reset.A(2,4) = 1; % params (2) := inarg11 (4)
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
reset.B = zeros(state_size, input_size);
reset.A(3,3) = 0; reset.B(3,30) = 1; % inarg10 (3) := obstacleTriggered_EL_oB_hidden_inarg1 (in30)
reset.A(4,4) = 0; reset.B(4,31) = 1; % inarg11 (4) := obstacleTriggered_EL_oB_hidden_inarg2 (in31)
trans(1) = transition(guard, reset, 11, 'obstacleTriggered_EL_oB_hidden_end');

locs(10) = location(inv, trans, dyn);

%% obstacle_Buffer, Loc 11: obstacle_Buffer_11 assignment (-> 3)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
reset.A(1,1) = 0; reset.A(1,3) = 1; % obstacleTrig (1) := inarg10 (3)
reset.A(2,2) = 0; reset.A(2,4) = 1; % params (2) := inarg11 (4)
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
% 15: value
% 16: inarg1_x
% 17: inarg2_x
% 18: inarg2_y
% 19: inarg3_x
% 20: inarg3_y
% 21: inarg4_x
% 22: inarg4_y
% 23: inarg5
% 24: inarg6
% 25: inarg7
% 26: inarg8
% 27: inarg9
% 28: obstacleTrig
% 29: params
% 30: inarg10
% 31: inarg11
input_size = 31;

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

%% moveToLocation_Buffer, Loc 1: moveToLocation_Buffer_2 assignment (-> 3)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
reset.A(1,1) = 0; % moveToLocationTime (1) := 0
reset.A(2,2) = 0; % moveToLocationOccurred (2) := false (const 0)
trans(1) = transition(guard, reset, 3);

locs(1) = location(inv, trans, dyn);

%% moveToLocation_Buffer, Loc 2: unused end location

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();

locs(2) = location(inv, trans, dyn);

%% moveToLocation_Buffer, Loc 3: moveToLocation_Buffer_3 moveToLocationHappened comm (-> 4)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
trans(1) = transition(guard, reset, 4, 'moveToLocationHappened_mTLB_mTLM_hidden');

locs(3) = location(inv, trans, dyn);

%% moveToLocation_Buffer, Loc 4: moveToLocation_Buffer_5 assignment (-> 1)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
reset.B = zeros(state_size,input_size);
reset.A(2,2) = 0; % moveToLocationOccurred (2) := true (const 0)
reset.A(1,1) = 0; reset.B(1,10) = 1; % moveToLocationTime (1) := time (in10)
trans(1) = transition(guard, reset, 1);

locs(4) = location(inv, trans, dyn);

comp(3) = hybridAutomaton(locs);

%% moveToLocation_Semantics

% moveToLocation_Semantics state:
% 1: inarg12_x
% 2: inarg12_y
% 3: inarg13_x
% 4: inarg13_y
% 5: _timer
% 6: setRobotOrientation_EL_mTLS_hidden_arg
% 7: setRobotVelocity_EL_mTLS_hidden_arg_x
% 8: setRobotVelocity_EL_mTLS_hidden_arg_y
state_size = 8;

% moveToLocation_Semantics inputs:
% 1: inarg14_x
% 2: inarg14_y
% 3: moveToLocationCall_mTLS_mTLM_inarg_x
% 4: moveToLocationCall_mTLS_mTLM_inarg_y
% 5: getRobotPosition_EL_mTLS_hidden_inarg_x
% 6: getRobotPosition_EL_mTLS_hidden_inarg_y
input_size = 31; % expand to maximum input size

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

%% moveToLocation_Semantics, Loc 1: moveToLocation_Semantics_1 external choice (-> 3, 1)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();

guard = fullspace(state_size);
reset = identity_reset;
trans(1) = transition(guard, reset, 3, 'moveToLocationCall_mTLS_mTLM_start');

guard = fullspace(state_size);
reset = identity_reset;
trans(2) = transition(guard, reset, 1, 'proceed_EL_mTLS_hidden');

locs(1) = location(inv, trans, dyn);

%% moveToLocation_Semantics, Loc 2: unused end location

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();

locs(2) = location(inv, trans, dyn);

%% moveToLocation_Semantics, Loc 3: moveToLocation_Semantics_1 moveToLocationCall comm intermediate (-> 4)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
reset.B = zeros(state_size, input_size);
reset.A(1,1) = 0; reset.B(1,3) = 1; % inarg12_x (1) := moveToLocationCall_mTLS_mTLM_inarg_x (in3)
reset.A(2,2) = 0; reset.B(2,4) = 1; % inarg12_y (2) := moveToLocationCall_mTLS_mTLM_inarg_y (in4)
trans(1) = transition(guard, reset, 4, 'moveToLocationCall_mTLS_mTLM_end');

locs(3) = location(inv, trans, dyn);

%% moveToLocation_Semantics, Loc 4: moveToLocation_Semantics_2 getRobotPosition comm start (-> 5)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
trans(1) = transition(guard, reset, 5, 'getRobotPosition_EL_mTLS_hidden_start');

locs(4) = location(inv, trans, dyn);

%% moveToLocation_Semantics, Loc 5: moveToLocation_Semantics_2 getRobotPosition comm intermediate (-> 6)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
reset.B = zeros(state_size,input_size);
reset.A(3,3) = 0; reset.B(3,5) = 1; % inarg13_x (3) := getRobotPosition_EL_mTLS_hidden_inarg_x (in5)
reset.A(4,4) = 0; reset.B(4,6) = 1; % inarg13_y (4) := getRobotPosition_EL_mTLS_hidden_inarg_y (in6)
trans(1) = transition(guard, reset, 6, 'getRobotPosition_EL_mTLS_hidden_end');

locs(5) = location(inv, trans, dyn);

%% moveToLocation_Semantics, Loc 6: moveToLocation_Semantics_4 setRobotOrientation comm start (-> 7)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
% nonlinear reset
reset = struct('f', @moveToLocation_Semantics_4_start_reset);
trans(1) = transition(guard, reset, 7, 'setRobotOrientation_EL_mTLS_hidden_start');

locs(6) = location(inv, trans, dyn);

%% moveToLocation_Semantics, Loc 7: moveToLocation_Semantics_4 setRobotOrientation comm intermediate (-> 8)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
trans(1) = transition(guard, reset, 8, 'setRobotOrientation_EL_mTLS_hidden_end');

locs(7) = location(inv, trans, dyn);

%% moveToLocation_Semantics, Loc 8: moveToLocation_Semantics_6 Skip (-> 9)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
trans(1) = transition(guard, reset, 9);

locs(8) = location(inv, trans, dyn);

%% moveToLocation_Semantics, Loc 9: moveToLocation_Semantics_7 setRobotVelocity comm start (-> 10)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
% nonlinear reset
reset = struct('f', @moveToLocation_Semantics_7_start_reset);
trans(1) = transition(guard, reset, 10, 'setRobotVelocity_EL_mTLS_hidden_start');

locs(9) = location(inv, trans, dyn);

%% moveToLocation_Semantics, Loc 10: moveToLocation_Semantics_7 setRobotVelocity comm intermediate (-> 11)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
trans(1) = transition(guard, reset, 11, 'setRobotVelocity_EL_mTLS_hidden_end');

locs(10) = location(inv, trans, dyn);

%% moveToLocation_Semantics, Loc 11: moveToLocation_Semantics_8 Skip (-> 1)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
trans(1) = transition(guard, reset, 1);

locs(11) = location(inv, trans, dyn);


comp(4) = hybridAutomaton(locs);

%% moveToLocation_Monitor

% moveToLocation_Monitor state:
% 1: inarg14_x
% 2: inarg14_y
% 3: _timer
state_size = 3;

% moveToLocation_Monitor inputs:
% 1: inarg12_x
% 2: inarg12_y
% 3: inarg13_x
% 4: inarg13_y
% 5: moveToLocationCall_mTLS_mTLM_inarg_x
% 6: moveToLocationCall_mTLS_mTLM_inarg_y
input_size = 31; % expand to maximum input size

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

%% moveToLocation_Monitor, Loc 1: moveToLocation_Monitor_1 moveToLocationCall comm start (-> 3)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
trans(1) = transition(guard, reset, 3, 'moveToLocationCall_mTLS_mTLM_start');

locs(1) = location(inv, trans, dyn);

%% moveToLocation_Monitor, Loc 2: unused end location

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();

locs(2) = location(inv, trans, dyn);

%% moveToLocation_Monitor, Loc 3: moveToLocation_Monitor_1 moveToLocationCall comm intermediate (-> 4)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
reset.B = zeros(state_size, input_size);
reset.A(1,1) = 0; reset.B(1,5) = 1; % inarg14_x (1) := moveToLocationCall_mTLS_mTLM_inarg_x (in5)
reset.A(2,2) = 0; reset.B(2,6) = 1; % inarg14_y (2) := moveToLocationCall_mTLS_mTLM_inarg_y (in6)
trans(1) = transition(guard, reset, 4, 'moveToLocationCall_mTLS_mTLM_end');

locs(3) = location(inv, trans, dyn);

%% moveToLocation_Monitor, Loc 4: moveToLocation_Monitor_2 moveToLocationHappened comm (-> 1)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
trans(1) = transition(guard, reset, 1, 'moveToLocationHappened_mTLB_mTLM_hidden');

locs(4) = location(inv, trans, dyn);

comp(5) = hybridAutomaton(locs);

%% composition

ibinds = [];

% EnvironmentLoop inputs:
ibinds{1} = [
    [3, 1]; % 1: moveToLocationTime - from moveToLocation_Buffer (3, 1)
    [3, 2]; % 2: moveToLocationOccurred - from moveToLocation_Buffer (3, 2)
    [2, 1]; % 3: obstacleTrig - from obstacle_Buffer (2, 1)
    [2, 2]; % 4: params - from obstacle_Buffer (2, 2)
    [2, 3]; % 5: inarg10 - from obstacle_Buffer (2, 3)
    [2, 4]; % 6: inarg11 - from obstacle_Buffer (2, 4)
    [1, 30]; % 7: obstacleTriggered_EL_oB_hidden_inarg2 - from EnvironmentLoop (1, 30)
    [4, 7]; % 8: setRobotVelocity_EL_mTLS_hidden_inarg_x - from moveToLocation_Semantics (4, 7)
    [4, 8]; % 9: setRobotVelocity_EL_mTLS_hidden_inarg_y - from moveToLocation_Semantics (4, 8)
    [4, 6]; % 10: setRobotOrientation_EL_mTLS_hidden_inarg - from moveToLocation_Semantics (4, 6);
    [0, 1]; % 11: dummy input
    [0, 2]; % 12: dummy input
    [0, 3]; % 13: dummy input
    [0, 4]; % 14: dummy input
    [0, 5]; % 15: dummy input
    [0, 6]; % 16: dummy input
    [0, 7]; % 17: dummy input
    [0, 8]; % 18: dummy input
    [0, 9]; % 19: dummy input
    [0, 10]; % 20: dummy input
    [0, 11]; % 21: dummy input
    [0, 12]; % 22: dummy input
    [0, 13]; % 23: dummy input
    [0, 14]; % 24: dummy input
    [0, 15]; % 25: dummy input
    [0, 16]; % 26: dummy input
    [0, 17]; % 27: dummy input
    [0, 18]; % 28: dummy input
    [0, 19]; % 29: dummy input
    [0, 20]; % 30: dummy input
    [0, 21]; % 31: dummy input
];

% obstacle_Buffer inputs:
ibinds{2} = [
    [1, 1]; % 1: robot_pos_x - from EnvironmentLoop (1, 1)
    [1, 2] % 2: robot_pos_y - from EnvironmentLoop (1, 2)
    [1, 3] % 3: robot_vel_x - from EnvironmentLoop (1, 3)
    [1, 4]; % 4: robot_vel_y - from EnvironmentLoop (1, 4)
    [1, 5]; % 5: robot_acc_x - from EnvironmentLoop (1, 5)
    [1, 6]; % 6: robot_acc_y - from EnvironmentLoop (1, 6)
    [1, 7]; % 7: robot_ori - from EnvironmentLoop (1, 7)
    [1, 8]; % 8: robot_angVel - from EnvironmentLoop (1, 8)
    [1, 9]; % 9: robot_angAcc - from EnvironmentLoop (1, 9)
    [1, 10]; % 10: time - from EnvironmentLoop (1, 10)
    [1, 11]; % 11: stepTimer - from EnvironmentLoop (1, 11)
    [1, 12]; % 12: tockTimer - from EnvironmentLoop (1, 12)
    [1, 13]; % 13: obstacleTime - from EnvironmentLoop (1, 13)
    [1, 14]; % 14: obstacleOccurred - from EnvironmentLoop (1, 14)
    [3, 1]; % 15: moveToLocationTime - from moveToLocation_Buffer (3, 1)
    [3, 2]; % 16: moveToLocationOccurred - from moveToLocation_Buffer (3, 2)
    [1, 15]; % 17: value - from EnvironmentLoop (1,15)
    [1, 16]; % 18: inarg1 - from EnvironmentLoop (1, 16)
    [1, 17]; % 19: inarg2_x - from EnvironmentLoop (1, 17)
    [1, 18]; % 20: inarg2_y - from EnvironmentLoop (1, 18)
    [1, 19]; % 21: inarg3_x - from EnvironmentLoop (1, 19)
    [1, 20]; % 22: inarg3_y - from EnvironmentLoop (1, 20)
    [1, 21]; % 23: inarg4_x - from EnvironmentLoop (1, 21)
    [1, 22]; % 24: inarg4_y - from EnvironmentLoop (1, 22)
    [1, 23]; % 25: inarg5 - from EnvironmentLoop (1, 23)
    [1, 24]; % 26: inarg6 - from EnvironmentLoop (1, 24)
    [1, 25]; % 27: inarg7 - from EnvironmentLoop (1, 25)
    [1, 26]; % 28: inarg8 - from EnvironmentLoop (1, 26)
    [1, 27]; % 29: inarg9 - from EnvironmentLoop (1, 27)
    [1, 29]; % 30: obstacleTriggered_EL_oB_hidden_inarg1 - from EnvironmentLoop (1, 29)
    [1, 30]; % 31: obstacleTriggered_EL_oB_hidden_inarg2 - from EnvironmentLoop (1, 30)
];

% moveToLocation_Buffer input:
ibinds{3} = [
    [1, 1]; % 1: robot_position_x - from EnvironmentLoop (1, 1)
    [1, 2]; % 2: robot_position_y - from EnvironmentLoop (1, 2)
    [1, 3]; % 3: robot_velocity_x - from EnvironmentLoop (1, 3)
    [1, 4]; % 4: robot_velocity_y - from EnvironmentLoop (1, 4)
    [1, 5]; % 5: robot_acceleration_x - from EnvironmentLoop (1, 5)
    [1, 6]; % 6: robot_acceleration_y - from EnvironmentLoop (1, 6)
    [1, 7]; % 7: robot_orientation - from EnvironmentLoop (1, 7)
    [1, 8]; % 8: robot_angularVelocity - from EnvironmentLoop (1, 8)
    [1, 9]; % 9: robot_angularAcceleration - from EnvironmentLoop (1, 9)
    [1, 10]; % 10: time - from EnvironmentLoop (1, 10)
    [1, 11]; % 11: stepTimer - from EnvironmentLoop (1, 11)
    [1, 12]; % 12: tockTimer - from EnvironmentLoop (1, 12)
    [1, 13]; % 13: obstacleTime - from EnvironmentLoop (1, 13)
    [1, 14]; % 14: obstacleOccurred - from EnvironmentLoop (1, 14)
    [1, 15]; % 15: value - from EnvironmentLoop (1, 15)
    [1, 16]; % 16: inarg1_x - from EnvironmentLoop (1, 16)
    [1, 17]; % 17: inarg2_x - from EnvironmentLoop (1, 17)
    [1, 18]; % 18: inarg2_y - from EnvironmentLoop (1, 18)
    [1, 19]; % 19: inarg3_x - from EnvironmentLoop (1, 19)
    [1, 20]; % 20: inarg3_y - from EnvironmentLoop (1, 20)
    [1, 21]; % 21: inarg4_x - from EnvironmentLoop (1, 21)
    [1, 22]; % 22: inarg4_y - from EnvironmentLoop (1, 22)
    [1, 23]; % 23: inarg5 - from EnvironmentLoop (1, 23)
    [1, 24]; % 24: inarg6 - from EnvironmentLoop (1, 24)
    [1, 25]; % 25: inarg7 - from EnvironmentLoop (1, 25)
    [1, 26]; % 26: inarg8 - from EnvironmentLoop (1, 26)
    [1, 27]; % 27: inarg9 - from EnvironmentLoop (1, 27)
    [2, 1]; % 28: obstacleTrig - from obstacle_Buffer (2, 1)
    [2, 2]; % 29: params - from obstacle_Buffer (2, 2)
    [2, 3]; % 30: inarg10 - from obstacle_Buffer (2, 3)
    [2, 4]; % 31: inarg11 - from obstacle_Buffer (2, 4)
];

% moveToLocation_Semantics inputs:
ibinds{4} = [
    [5, 1]; % 1: inarg14_x - from moveToLocation_Monitor (5, 1)
    [5, 2]; % 2: inarg14_y - from moveToLocation_Monitor (5, 2)
    [0, 1]; % 3: moveToLocationCall_mTLS_mTLM_inarg_x - external (0, 1)
    [0, 2]; % 4: moveToLocationCall_mTLS_mTLM_inarg_y - external (0, 2)
    [1, 31]; % 5: getRobotPosition_EL_mTLS_hidden_inarg_x - from EnvironmentLoop (1, 31)
    [1, 32]; % 6: getRobotPosition_EL_mTLS_hidden_inarg_y - from EnvironmentLoop (1, 32)
    [0, 22]; % 7: dummy input
    [0, 23]; % 8: dummy input
    [0, 24]; % 9: dummy input
    [0, 25]; % 10: dummy input
    [0, 26]; % 11: dummy input
    [0, 27]; % 12: dummy input
    [0, 28]; % 13: dummy input
    [0, 29]; % 14: dummy input
    [0, 30]; % 15: dummy input
    [0, 31]; % 16: dummy input
    [0, 3]; % 17: dummy input
    [0, 4]; % 18: dummy input
    [0, 5]; % 19: dummy input
    [0, 6]; % 20: dummy input
    [0, 7]; % 21: dummy input
    [0, 8]; % 22: dummy input
    [0, 9]; % 23: dummy input
    [0, 10]; % 24: dummy input
    [0, 11]; % 25: dummy input
    [0, 12]; % 26: dummy input
    [0, 13]; % 27: dummy input
    [0, 14]; % 28: dummy input
    [0, 15]; % 29: dummy input
    [0, 16]; % 30: dummy input
    [0, 17]; % 31: dummy input
];

% moveToLocation_Monitor inputs:
ibinds{5} = [
    [4, 1]; % 1: inarg12_x - from moveToLocation_Semantics (4, 1)
    [4, 2]; % 2: inarg12_y - from moveToLocation_Semantics (4, 2)
    [4, 3]; % 3: inarg13_x - from moveToLocation_Semantics (4, 3)
    [4, 4]; % 4: inarg13_y - from moveToLocation_Semantics (4, 4)
    [0, 1]; % 5: moveToLocationCall_mTLS_mTLM_inarg_x - external (0, 1)
    [0, 2]; % 6: moveToLocationCall_mTLS_mTLM_inarg_y - external (0, 2)
    [0, 16]; % 7: dummy input
    [0, 17]; % 8: dummy input
    [0, 18]; % 9: dummy input
    [0, 19]; % 10: dummy input
    [0, 20]; % 11: dummy input
    [0, 21]; % 12: dummy input
    [0, 22]; % 13: dummy input
    [0, 23]; % 14: dummy input
    [0, 24]; % 15: dummy input
    [0, 25]; % 16: dummy input
    [0, 26]; % 17: dummy input
    [0, 27]; % 18: dummy input
    [0, 28]; % 19: dummy input
    [0, 29]; % 20: dummy input
    [0, 30]; % 21: dummy input
    [0, 31]; % 22: dummy input
    [0, 3]; % 23: dummy input
    [0, 4]; % 24: dummy input
    [0, 5]; % 25: dummy input
    [0, 6]; % 26: dummy input
    [0, 7]; % 27: dummy input
    [0, 8]; % 28: dummy input
    [0, 9]; % 29: dummy input
    [0, 10]; % 30: dummy input
    [0, 11]; % 31: dummy input
];

pHA = parallelHybridAutomaton(comp, ibinds);

%% functions


function [x2] = HandleCollision_start_trans1_reset(x, u, collisionEps)
    % Assign type of input to output variable, type safety feature
    if isa(x,'double')
        x2(1,1) = 0;
    else
        x2(1,1) = sym(0);
    end

    inp = u(1:10);
    for i_inner = 1:32
        x2(i_inner,1) = x(i_inner);
    end
    
    % robot.position (1,2) := robot.position (1,2) − (collisionEps/2)∗ (2 ∗ robot.velocity (1,2) − collisionEps ∗ robot.acceleration (5,6))
    % robot.velocity (3,4) := robot.velocity (3,4) − collisionEps ∗ robot.acceleration
    x2(1,1) = x(1) - (collisionEps/2) * (2 * x(3) - collisionEps * x(5));
    x2(2,1) = x(2) - (collisionEps/2) * (2 * x(4) - collisionEps * x(6));
    x2(3,1) = x(3) - collisionEps * x(5);
    x2(4,1) = x(4) - collisionEps * x(6);
end

% value! = vectorToOrientation~(obs1.position \vecminus robot.position)
function [x2] = obstacle_InputEventMapping_5_start_trans1_reset(x, u, obs1)
    % Assign type of input to output variable, type safety feature
    if isa(x,'double')
        x2(1,1) = 0;
    else
        x2(1,1) = sym(0);
    end

    inp = u(1:10);
    for i_inner = 1:32
        x2(i_inner,1) = x(i_inner);
    end

    % exp1: obs1.position \vecminus robot.position
    exp1_x = obs1.position_x - x(1);
    exp1_y = obs1.position_y - x(2);
    % val: vectorToOrientation~(obs1.position \vecminus robot.position)
    val = atan2(exp1_y, exp1_x);
    x2(30) = val;
end

function [x2] =  moveToLocation_Semantics_4_start_reset(x, u)
    % Assign type of input to output variable, type safety feature
    if isa(x,'double')
        x2(1,1) = 0;
    else
        x2(1,1) = sym(real(0));
    end

    inp = u(1:5);
    for i = 1:8
        x2(i,1) = x(i);
    end

    % setRobotOrientation_EL_mTLS_hidden_arg (6) := vectorToOrientation(towards(inarg13 (3,4), inarg12 (1,2)))
    exp1_x = x(1) - x(3); % inarg12 - inarg13 (x component)
    exp1_y = x(2) - x(4); % inarg12 - inarg13 (y component)
    exp2 = sqrt(exp1_x^2 + exp1_y^2); % norm(inarg12 - inarg13)
    exp3_x = exp1_x / exp2; %  towards(inarg13, inarg12) (x component)
    exp3_y = exp1_y / exp2; %  towards(inarg13, inarg12) (y component)
    exp4 = atan2(exp3_y, exp3_x); % vectorToOrientation(towards(inarg13, inarg12))

    x2(6,1) = exp4;
end

function [x2] =  moveToLocation_Semantics_7_start_reset(x, u)
    % Assign type of input to output variable, type safety feature
    if isa(x,'double')
        x2(1,1) = 0;
    else
        x2(1,1) = sym(real(0));
    end

    inp = u(1:5);
    for i = 1:8
        x2(i,1) = x(i);
    end

    % setRobotVelocity_EL_mTLS_hidden_arg_x (7) := 1.0 * (towards~inarg13_x(3)~inarg12_x(1))
    % setRobotVelocity_EL_mTLS_hidden_arg_y (8) := 1.0 * (towards~inarg13_y(4)~inarg12_y(2))
    exp1_x = x(1) - x(3); % inarg12 - inarg13 (x component)
    exp1_y = x(2) - x(4); % inarg12 - inarg13 (y component)
    exp2 = sqrt(exp1_x^2 + exp1_y^2); % norm(inarg12 - inarg13)
    exp3_x = exp1_x / exp2; %  towards(inarg13, inarg12) (x component)
    exp3_y = exp1_y / exp2; %  towards(inarg13, inarg12) (y component)
    x2(7,1) = exp3_x;
    x2(8,1) = exp3_y;
end

end
