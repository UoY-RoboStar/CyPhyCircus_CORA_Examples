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
timeStep = 0.1;
collisionEps = 0.05;

clear comp;

%% reachability checking

pHA = navigation_translation(eps, obstacles, numObstacles, arena, tockLength, timeStep, collisionEps);

c = [...
    % STATE (EnvironmentLoop)
    0; % 1: robot_pos_x
    0; % 2: robot_pos_y
    0; % 3: robot_vel_x
    0; % 4: robot_vel_y
    0; % 5: robot_acc_x
    0; % 6: robot_acc_y
    0; % 7: robot_ori
    0; % 8: robot_angVel
    0; % 9: robot_angAcc
    0; % 10: time
    0; % 11: stepTimer
    0; % 12: tockTimer
    0; % 13: obstacleTime
    0; % 14: obstacleOccurred
    0; % 15: value
    0; % 16: inarg1
    0; % 17: inarg2_x
    0; % 18: inarg2_y
    0; % 19: inarg3_x
    0; % 20: inarg3_y
    0; % 21: inarg4_x
    0; % 22: inarg4_y
    0; % 23: inarg5
    0; % 24: inarg6
    0; % 25: inarg7
    0; % 26: inarg8
    0; % 27: inarg9
    0; % 28: _timer
    0; % 29: obstacleTriggered_EL_oB_hidden_arg1
    0; % 30: obstacleTriggered_EL_oB_hidden_arg2
    0; % 31: getRobotPosition_EL_mTLS_hidden_arg_x
    0; % 32: getRobotPosition_EL_mTLS_hidden_arg_y
    % STATE (obstacle_Buffer)
    0; % 33: obstacleTrig
    0; % 34: params
    0; % 35: inarg10
    0; % 36: inarg11
    0; % 37: _timer
    0; % 38: obstacle_oB_arg
    % STATE (moveToLocation_Buffer)
    0; % 39: moveTime
    0; % 40: moveOccurred
    0; % 41: _timer
    % STATE (moveToLocation_Semantics)
    0; % 42: inarg12_x
    0; % 43: inarg12_y
    0; % 44: inarg13_x
    0; % 45: inarg13_y
    0; % 46: _timer
    0; % 47: setRobotOrientation_EL_mTLS_hidden_arg
    0; % 48: setRobotVelocity_EL_mTLS_hidden_arg_x
    0; % 49: setRobotVelocity_EL_mTLS_hidden_arg_y
    % STATE (moveToLocation_Monitor)
    0; % 50: inarg14_x
    0; % 51: inarg14_y
    0; % 52: _timer
];

G = [...
    % STATE (EnvironmentLoop)
    0; % 1: robot_pos_x
    0; % 2: robot_pos_y
    0; % 3: robot_vel_x
    0; % 4: robot_vel_y
    0; % 5: robot_acc_x
    0; % 6: robot_acc_y
    0; % 7: robot_ori
    0; % 8: robot_angVel
    0; % 9: robot_angAcc
    0; % 10: time
    0; % 11: stepTimer
    0; % 12: tockTimer
    0; % 13: obstacleTime
    0; % 14: obstacleOccurred
    0; % 15: value
    0; % 16: inarg1
    0; % 17: inarg2_x
    0; % 18: inarg2_y
    0; % 19: inarg3_x
    0; % 20: inarg3_y
    0; % 21: inarg4_x
    0; % 22: inarg4_y
    0; % 23: inarg5
    0; % 24: inarg6
    0; % 25: inarg7
    0; % 26: inarg8
    0; % 27: inarg9
    0; % 28: _timer
    0; % 29: obstacleTriggered_EL_oB_hidden_arg1
    0; % 30: obstacleTriggered_EL_oB_hidden_arg2
    0; % 31: getRobotPosition_EL_mTLS_hidden_arg_x
    0; % 32: getRobotPosition_EL_mTLS_hidden_arg_y
    % STATE (obstacle_Buffer)
    0; % 33: obstacleTrig
    0; % 34: params
    0; % 35: inarg10
    0; % 36: inarg11
    0; % 37: _timer
    0; % 38: obstacle_oB_arg
    % STATE (moveToLocation_Buffer)
    0; % 39: moveTime
    0; % 40: moveOccurred
    0; % 41: _timer
    % STATE (moveToLocation_Semantics)
    0; % 42: inarg12_x
    0; % 43: inarg12_y
    0; % 44: inarg13_x
    0; % 45: inarg13_y
    0; % 46: _timer
    0; % 47: setRobotOrientation_EL_mTLS_hidden_arg
    0; % 48: setRobotVelocity_EL_mTLS_hidden_arg_x
    0; % 49: setRobotVelocity_EL_mTLS_hidden_arg_y
    % STATE (moveToLocation_Monitor)
    0; % 50: inarg14_x
    0; % 51: inarg14_y
    0; % 52: _timer
];

params = [];
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
params.U = zonotope([12; 12; zeros(29,1)], zeros(31,1));
params.tu = [0];
% ...

options = [];
options.verbose = false;
options.timeStep = 0.0001;
options.taylorTerms = 1;
options.zonotopeOrder = 1;
options.linAlg = 'standard';
options.guardOrder = 1;
options.guardIntersect = 'levelSet';
options.enclose = {'box'};
%options.error = 0.01;

tic
R = reach(pHA, params, options);
tEval1 = toc;

% filter out reach sets that don't have time interval sets
Rfiltered = R(arrayfun(@(x) not(isempty(x.timeInterval)), R));

% merge together reach sets for faster plotting
Rcompressed(length(Rfiltered),1) = reachSet();
for i = 1:length(Rfiltered)
    timePoint.set{1} = fold(@(x,y) or(x,y, 'iterative'), Rfiltered(i).timePoint.set);
    timePoint.time{1} = fold(@(x,y) or(x,y), Rfiltered(i).timePoint.time);
    timePoint.error = NaN;
    timeInterval.set{1} = fold(@(x,y) or(x,y, 'iterative'), Rfiltered(i).timeInterval.set);
    timeInterval.time{1} = fold(@(x,y) or(x,y), Rfiltered(i).timeInterval.time);
    timeInterval.error = NaN;
    Rcompressed(i) = reachSet(timePoint, timeInterval);
    disp("Compressing reach set: " + num2str(i) + "/" +  num2str(length(Rfiltered)));
end

%% checks of properties
disp('');

% avoid {x : R^n | (x1 - 3)^2 + (x2 - 5)^2 <= 0.1^2}
vars = sym('x', [pHA.nrOfOutputs,1]);
eq = (vars(1) - 3)^2 + (vars(2) - 5)^2 - 0.1^2;
compOp = {'<='};
S = levelSet(eq, vars, compOp);
spec = specification(S, 'unsafeSet');
check1 = spec.check(Rfiltered);

% avoid {x : R^n | (x1 - 5)^2 + (x2 - 3)^2 <= 0.1^2}
vars = sym('x', [pHA.nrOfOutputs,1]);
eq = (vars(1) - 5)^2 + (vars(2) - 3)^2 - 0.1^2;
compOp = {'<='};
S = levelSet(eq, vars, compOp);
spec = specification(S, 'unsafeSet');
check2 = spec.check(Rfiltered);

check3 = 1;
for i = 1:length(R)
    if R(i).loc(1) == 6
        check3 = 0;
    end
end

%% trace checking

pHA2 = navigation_translation(eps, obstacles, numObstacles, arena, timeStep*2, timeStep, collisionEps);
comp = pHA2.components;
ibinds = pHA2.bindsInputs;

% 2 variables (outputs to move call)
vars = sym('x',[2,1]);
% 31 inputs (dummy)
A = zeros(2,2);
B = zeros(2,31);
c = zeros(2,1);
C = eye(2);
D = zeros(2,31);
k = zeros(2,1);
dyn = linearSys(A, B, c, C, D, k);

id_reset.A = eye(2,2);
id_reset.c = zeros(2,1);

inv = fullspace(2);

locs = location();

% loc 1: move call 1 start
trans = transition();
guard = fullspace(2);
reset = id_reset;
reset.c(1) = 1;
reset.c(2) = 1;
trans(1) = transition(guard, reset, 2, 'moveToLocationCall_mTLS_mTLM_start');

locs(1) = location(inv, trans, dyn);

% loc 2: move call 1 end
trans = transition();
guard = fullspace(2);
reset = id_reset;
trans(1) = transition(guard, reset, 3, 'moveToLocationCall_mTLS_mTLM_end');

locs(2) = location(inv, trans, dyn);


% loc 3: move call 2 start
trans = transition();
guard = fullspace(2);
reset = id_reset;
reset.c(1) = 1;
reset.c(2) = 1;
trans(1) = transition(guard, reset, 4, 'moveToLocationCall_mTLS_mTLM_start');

locs(3) = location(inv, trans, dyn);

% loc 4: move call 2 end
trans = transition();
guard = fullspace(2);
reset = id_reset;
trans(1) = transition(guard, reset, 5, 'moveToLocationCall_mTLS_mTLM_end');

locs(4) = location(inv, trans, dyn);

% loc 5: move call 3 start
trans = transition();
guard = fullspace(2);
reset = id_reset;
reset.c(1) = 1;
reset.c(2) = 1;
trans(1) = transition(guard, reset, 6, 'moveToLocationCall_mTLS_mTLM_start');

locs(5) = location(inv, trans, dyn);

% loc 6: move call 3 end
trans = transition();
guard = fullspace(2);
reset = id_reset;
trans(1) = transition(guard, reset, 7, 'moveToLocationCall_mTLS_mTLM_end');

locs(6) = location(inv, trans, dyn);

% loc 7: move call 4 start
trans = transition();
guard = fullspace(2);
reset = id_reset;
reset.c(1) = 1;
reset.c(2) = 1;
trans(1) = transition(guard, reset, 8, 'moveToLocationCall_mTLS_mTLM_start');
eq = realmax - vars(1);
op = '<';
guard = levelSet(eq, vars, op);
reset = id_reset;
trans(2) = transition(guard, reset, 7);

locs(7) = location(inv, trans, dyn);

% loc 8: move call 4 end
trans = transition();
guard = fullspace(2);
reset = id_reset;
trans(1) = transition(guard, reset, 9, 'moveToLocationCall_mTLS_mTLM_end');

locs(8) = location(inv, trans, dyn);

% loc 9: end location
inv = emptySet(2);
trans = transition();
reset = id_reset;
guard = fullspace(2);
trans(1) = transition(guard, reset, 9, 'tock_EL');
locs(9) = location(inv, trans, dyn);

controllerHA = hybridAutomaton(locs);

% add robochart to components
controllerID = length(comp)+1;
comp(controllerID) = controllerHA;

% link controller outputs to existing external input bounds
for i = 1:length(ibinds)
    for j = 1:length(ibinds{i}(:,1))
        if ibinds{i}(j,1) == 0
            switch ibinds{i}(j,2)
                case 1
                    % moveToLocationCall_mS_mM_arg1_x
                    ibinds{i}(j,1) = controllerID;
                    ibinds{i}(j,2) = 1;
                case 2
                    % moveToLocationCall_mS_mM_arg1_y
                    ibinds{i}(j,1) = controllerID;
                    ibinds{i}(j,2) = 2;
            end
        end
    end
end

% link navigation outputs to controller` inputs
ibinds{controllerID} = [
    [0, 1];
    [0, 2];
    [0, 3];
    [0, 4];
    [0, 5];
    [0, 6];
    [0, 7];
    [0, 8];
    [0, 9];
    [0, 10];
    [0, 11];
    [0, 12];
    [0, 13];
    [0, 14];
    [0, 15];
    [0, 16];
    [0, 17];
    [0, 18];
    [0, 19];
    [0, 20];
    [0, 21];
    [0, 22];
    [0, 23];
    [0, 24];
    [0, 25];
    [0, 26];
    [0, 27];
    [0, 28];
    [0, 29];
    [0, 30];
    [0, 31];
    ];

% create composed parallel hybrid automaton
pHA2 = parallelHybridAutomaton(comp, ibinds);

c = zeros(54,1);
G = zeros(54,1);
params.R0 = zonotope(c, G);
params.U = zonotope(zeros(31,1),zeros(31,1));
params.startLoc = [1; 1; 1; 1; 1; 1];

tic
R2 = reach(pHA2, params, options);
tEval2 = toc;

% check the final location was not reached
trace_check = not(any(arrayfun(@(x) x.loc(6) == 9, R2)));

%% trace and reachability checking

% 2 variables (outputs to move call)
vars = sym('x',[2,1]);
% 31 inputs (dummy)
A = zeros(2,2);
B = zeros(2,31);
c = zeros(2,1);
C = eye(2);
D = zeros(2,31);
k = zeros(2,1);
dyn = linearSys(A, B, c, C, D, k);

id_reset.A = eye(2,2);
id_reset.c = zeros(2,1);

inv = fullspace(2);

locs = location();

% loc 1: move call start
trans = transition();
guard = fullspace(2);
reset = id_reset;
reset.c(1) = 5;
reset.c(2) = 3;
trans(1) = transition(guard, reset, 2, 'moveToLocationCall_mTLS_mTLM_start');

locs(1) = location(inv, trans, dyn);

% loc 2: move call end
trans = transition();
guard = fullspace(2);
reset = id_reset;
trans(1) = transition(guard, reset, 3, 'moveToLocationCall_mTLS_mTLM_end');

locs(2) = location(inv, trans, dyn);

% loc 3: start location
trans = transition();
reset = id_reset;
guard = fullspace(2);
trans(1) = transition(guard, reset, 4, 'obstacle_oB_start');
reset = id_reset;
guard = fullspace(2);
trans(2) = transition(guard, reset, 3, 'tock_EL');

locs(3) = location(inv, trans, dyn);

% loc 4: obstacle happened start
inv = fullspace(2);
trans = transition();
reset = id_reset;
guard = fullspace(2);
trans(1) = transition(guard, reset, 5, 'obstacle_oB_end');

locs(4) = location(inv, trans, dyn);

% loc 5: obstacle happened end
inv = emptySet(2);
trans = transition();
reset = id_reset;
guard = fullspace(2);
trans(1) = transition(guard, reset, 5, 'tock_EL');
locs(5) = location(inv, trans, dyn);

controllerHA = hybridAutomaton(locs);

% add monitoring automaton to components
comp(controllerID) = controllerHA;

% iBinds same as for previous check

% create composed parallel hybrid automaton
pHA = parallelHybridAutomaton(comp, ibinds);

c = zeros(54,1);
G = zeros(54,1);
params.R0 = zonotope(c, G);
params.U = zonotope(zeros(31,1),zeros(31,1));
params.startLoc = [1; 1; 1; 1; 1; 1];

tic
R2 = reach(pHA, params, options);
tEval3 = toc;

%% checks

% check whether the monitor ends in its final location (so obstacle occurs)
parents = [];
for i = 1:size(R2,1)
    parents = [parents;R2(i,1).parent];
end
parents = unique(parents);
ind = setdiff(1:size(R2,1),parents);
check4 = all(arrayfun(@(x) R2(x).loc(6), ind) == 5);

% filter out reach sets that don't have time interval sets
R2filtered = R(arrayfun(@(x) not(isempty(x.timeInterval)), R));

% check whether the first obstacle is avoided
% avoid {x : R^n | (x1 - 3)^2 + (x2 - 5)^2} <= 0.35^2
vars = sym('x', [pHA.nrOfOutputs,1]);
eq = (vars(1) - 3)^2 + (vars(2) - 5)^2 - 0.35^2;
compOp = {'<='};
S = levelSet(eq, vars, compOp);
spec = specification(S, 'unsafeSet');
check5 = spec.check(R2filtered);

% avoid {x : R^n | (x1 - 5)^2 + (x2 - 3)^2} <= 0.35^2
vars = sym('x', [pHA.nrOfOutputs,1]);
eq = (vars(1) - 5)^2 + (vars(2) - 3)^2 - 0.35^2;
compOp = {'<='};
S = levelSet(eq, vars, compOp);
spec = specification(S, 'unsafeSet');
check6 = spec.check(R2filtered);

%% Results

disp("Reachability checking time: " + tEval1);

if check1
    disp('Avoid {x : R^n | (x1 - 3)^2 + (x2 - 5)^2 <= 0.1^2}: true');
else
    disp('Avoid {x : R^n | (x1 - 3)^2 + (x2 - 5)^2 <= 0.1^2}: false');
end

if check2
    disp('Avoid {x : R^n | (x1 - 5)^2 + (x2 - 3)^2 <= 0.1^2}: true');
else
    disp('Avoid {x : R^n | (x1 - 5)^2 + (x2 - 3)^2 <= 0.1^2}: false');
end

if check3
    disp('Avoid location 6: true');
else
    disp('Avoid location 6: false');
end

disp(' ')
disp("Trace checking time: " + tEval2);
if trace_check
    disp('Trace check result: true')
else
    disp('Trace check result: false')
end

disp(' ')
disp("Trace and reachability checking time: " + tEval3);
if check4
    if check5 || check6
        disp('Result: obstacle detected and robot is never within 0.25 m of an obstacle')
    else
        if ~check5
            disp('Result: obstacle detected but robot comes within 0.25 m of obstacle at (3,5)')
        end
        if ~check6
            disp('Result: obstacle detected but robot comes within 0.25 m of obstacle at (5,3)')
        end
    end
else
    disp('Result: obstacle is never detected (the robot may not have come near the obstacle)')
end

