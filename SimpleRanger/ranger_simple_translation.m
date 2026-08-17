timeStep = 0.1;
targetPos = 5;
targetRadius = 1;
eps = 0.05;

clear comp;

%%
%% EnvLoop

% state:
% 1: pos
% 2: vel
% 3: stepTimer
% 4: inarg1
% 5: inarg2
% 6: _timer
% 7: _deadlineTimer
% 8: targetTriggered_outarg
state_size = 8;

% inputs:
% 1: targetTrig
% 2: inarg3
% 3: setVel_inarg
input_size = 3;

A = zeros(state_size,state_size);
B = zeros(state_size,input_size);
c = [0; 0; 0; 0; 0; 1; 1; 0];
C = eye(state_size);
D = zeros(state_size,input_size);
k = zeros(state_size,1);
default_dyn = linearSys(A, B, c, C, D, k);

identity_reset = [];
identity_reset.A = eye(state_size);
%identity_reset.B = zeros(state_size,input_size);
identity_reset.c = zeros(state_size,1);

locs = location();


%% EnvLoop, Loc 1: EnvLoop_2 assignment start

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
reset.A(3,3) = 0; % stepTimer := 0
trans(1) = transition(guard, reset, 3);

locs(1) = location(inv, trans, dyn);

%% EnvLoop, Loc 2: Unused end location

inv = fullspace(state_size);
dyn = default_dyn;
trans = transition();

locs(2) = location(inv, trans, dyn);

%% EnvLoop, Loc 3: Movement_2 continuous evolution

C = [0 0 1 0 0 0 0 0];
d = timeStep; 
inv = polytope(C, d);

% 1: pos
% 2: vel
% 3: stepTimer
% 4: inarg1
% 5: inarg2
% 6: _timer
% 7: _deadlineTimer
% 8: targetTriggered_outarg
A = [0 1 0 0 0 0 0 0;  % pos' == vel
     0 0 0 0 0 0 0 0;  % vel' == 0
     0 0 0 0 0 0 0 0;  % stepTimer' == 1
     0 0 0 0 0 0 0 0;  % inarg1' == 0
     0 0 0 0 0 0 0 0;  % inarg2' == 0
     0 0 0 0 0 0 0 0;  % _timer' == 1
     0 0 0 0 0 0 0 0;  % _deadlineTimer' == 1
     0 0 0 0 0 0 0 0]; % targetTriggered_outarg' == 0
B = zeros(state_size,input_size);
c = [0;  % pos' == vel
     0;  % vel' == 0
     1;  % stepTimer' == 1
     0;  % inarg1' == 0
     0;  % inarg2' == 0
     1;  % _timer' == 1
     1;  % _deadlineTimer' == 1
     0]; % targetTriggered_outarg' == 0
C = eye(state_size);
D = zeros(state_size,input_size);
k = zeros(state_size,1);
dyn = linearSys(A, B, c, C, D, k);

trans = transition();
% stepTimer >= timeStep
C = [0 0 -1 0 0 0 0 0];
d = -timeStep;
guard = polytope(C, d);
reset = identity_reset;
trans(1) = transition(guard, reset, 4);

locs(3) = location(inv, trans, dyn);

%% EnvLoop, Loc 4: InputTriggers_1 conditional start


inv = emptySet(state_size);
dyn = default_dyn;
trans = transition();

% abs(pos-targetPos) <= targetRadius-eps (pos - targetPos <= 0)
C = [-1 0 0 0 0 0 0 0;  % -pos <= -targetPos+targetRadius-eps 
      1 0 0 0 0 0 0 0]; % pos <= targetPos
d = [-targetPos+targetRadius-eps; % -pos <= -targetPos+targetRadius-eps
     targetPos];          % pos <= targetPos
guard = polytope(C, d);
reset = identity_reset;
trans(1) = transition(guard, reset, 7);

% abs(pos-targetPos) <= targetRadius-eps (pos - targetPos >= 0)
C = [ 1 0 0 0 0 0 0 0;  % pos <= targetPos+targetRadius-eps 
     -1 0 0 0 0 0 0 0]; % -pos <= -targetPos
d = [targetPos+targetRadius-eps; % pos <= targetPos+targetRadius-eps
     -targetPos];         % -pos <= -targetPos
guard = polytope(C, d); 
reset = identity_reset;
trans(2) = transition(guard, reset, 7);

% abs(pos-targetPos) >= targetRadius (pos - targetPos <= 0)
C = [1 0 0 0 0 0 0 0;  % pos <= targetPos-targetRadius
     1 0 0 0 0 0 0 0]; % pos <= targetPos
d = [targetPos-targetRadius; % pos <= targetPos-targetRadius
     targetPos];             % pos <= targetPos
guard = polytope(C, d); 
reset = identity_reset;
trans(3) = transition(guard, reset, 8);

% abs(pos-targetPos) >= targetRadius (pos - targetPos >= 0)
C = [-1 0 0 0 0 0 0 0;  % -pos <= -targetPos-targetRadius
     -1 0 0 0 0 0 0 0]; % -pos <= -targetPos
d = [-targetPos-targetRadius; % -pos <= -targetPos-targetRadius
     -targetPos];             % -pos <= -targetPos
guard = polytope(C, d);
reset = identity_reset;
trans(4) = transition(guard, reset, 8);

% targetRadius-eps <= abs(pos-targetPos) /\ abs(pos-targetPos) <= targetRadius (pos - targetPos <= 0)
C = [ 1 0 0 0 0 0 0 0;  % pos <= targetPos-targetRadius+eps 
     -1 0 0 0 0 0 0 0;  % -pos <= targetRadius-targetPos
      1 0 0 0 0 0 0 0]; % pos <= targetPos
d = [targetPos-targetRadius+eps; % pos <= targetPos-targetRadius+eps
     targetRadius-targetPos      % -pos <= targetRadius-targetPos
     targetPos];                 % pos <= targetPos
guard = polytope(C, d);
reset = identity_reset;
trans(5) = transition(guard, reset, 9);

% targetRadius-eps <= abs(pos-targetPos) /\ abs(pos-targetPos) <= targetRadius (pos - targetPos >= 0)
C = [-1 0 0 0 0 0 0 0;  % -pos <= -targetPos-targetRadius+eps
      1 0 0 0 0 0 0 0;  % pos <= targetPos+targetRadius
     -1 0 0 0 0 0 0 0]; % -pos <= -targetPos
d = [-targetPos-targetRadius+eps; % -pos <= -targetPos-targetRadius+eps
     targetPos+targetRadius       % pos <= targetPos+targetRadius
     -targetPos];              % -pos <= -targetPos
guard = polytope(C, d);
reset = identity_reset;
trans(6) = transition(guard, reset, 9);

locs(4) = location(inv, trans, dyn);

%% EnvLoop, Loc 5: Movement_2 end location (unused)

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
% stepTimer >= timeStep
C = [0 0 -1 0 0 0 0 0];
d = -timeStep;
guard = polytope(C, d);
reset = identity_reset;
trans(1) = transition(guard, reset, 4);

guard = fullspace(state_size);
reset = identity_reset;
trans(2) = transition(guard, reset, 4);

locs(5) = location(inv, trans, dyn);

%% EnvLoop, Loc 6: EnvLoop_6 timeout start, zero timeout timer


inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
reset.A(7,7) = 0; % _deadlineTimer (7) := 0
trans(1) = transition(guard, reset, 14);

locs(6) = location(inv, trans, dyn);

%% EnvLoop, Loc 7: InputTriggers_2 communication start

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
reset.A(8,8) = 0; reset.c(8) = 1; % targetTriggered_outarg (8) := True (1)
trans(1) = transition(guard, reset, 10, 'targetTriggered_EnvLoop_EventBuffer_start');

locs(7) = location(inv, trans, dyn);

%% EnvLoop, Loc 8: InputTriggers_3 communication start

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
reset.A(8,8) = 0; reset.c(8) = 0; % targetTriggered_outarg (8) := Falae (0)
trans(1) = transition(guard, reset, 12, 'targetTriggered_EnvLoop_EventBuffer_start');

locs(8) = location(inv, trans, dyn);

%% EnvLoop, Loc 9: InputTriggers_4 Skip start

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
trans(1) = transition(guard, reset, 6);

locs(9) = location(inv, trans, dyn);

%% EnvLoop, Loc 10: InputTriggers_2 communication intermediate

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
trans(1) = transition(guard, reset, 11, 'targetTriggered_EnvLoop_EventBuffer_end');

locs(10) = location(inv, trans, dyn);

%% EnvLoop, Loc 11: InputTriggers_2 communication end, InputTriggers_5 Skip start

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
trans(1) = transition(guard, reset, 6);

locs(11) = location(inv, trans, dyn);

%% EnvLoop, Loc 12: InputTriggers_3 communication intermediate

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
trans(1) = transition(guard, reset, 13, 'targetTriggered_EnvLoop_EventBuffer_end');

locs(12) = location(inv, trans, dyn);

%% EnvLoop, Loc 13: InputTriggers_3 communication end, InputTriggers_6 Skip start

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
trans(1) = transition(guard, reset, 6);

locs(13) = location(inv, trans, dyn);

%% EnvLoop, Loc 14: QueryUpd_1 external choice start

% invariant: _deadlineTimer (7) <= 0
C = [0 0 0 0 0 0 1 0];
d = 0; 
inv = polytope(C, d);

dyn = default_dyn;

trans = transition();

guard = fullspace(state_size);
reset = identity_reset;
trans(1) = transition(guard, reset, 17, 'setVel_EnvLoop_Controller_start');

guard = fullspace(state_size);
reset = identity_reset;
trans(2) = transition(guard, reset, 19, 'proceed_EnvLoop_Controller');

% _deadlineTimer >= 0
C = [0 0 0 0 0 0 1 0];
d = 0; 
guard = polytope(-C, -d);
reset = identity_reset;
trans(3) = transition(guard, reset, 16);

locs(14) = location(inv, trans, dyn);

%% EnvLoop, Loc 15: QueryUpd_1 end

% invariant: _deadlineTimer (7) <= 0
C = [0 0 0 0 0 0 1 0];
d = 0; 
inv = polytope(C, d);

dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
trans(1) = transition(guard, reset, 1);

% _deadlineTimer >= 0
C = [0 0 0 0 0 0 1 0];
d = 0; 
guard = polytope(-C, -d);
reset = identity_reset;
trans(2) = transition(guard, reset, 16);

locs(15) = location(inv, trans, dyn);

%% EnvLoop, Loc 16: EnvLoop_6 timeout, EnvLoop_7 Skip start

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
trans(1) = transition(guard, reset, 1);

locs(16) = location(inv, trans, dyn);

%% EnvLoop, Loc 17: QueryUpd_1 setVel communication intermediate

% invariant: _deadlineTimer (7) <= 0
C = [0 0 0 0 0 0 1 0];
d = 0; 
inv = polytope(C, d);

dyn = default_dyn;

trans = transition();

guard = fullspace(state_size);
reset = identity_reset; % inarg2 (5) := setVel_inarg (2)
reset.A(5,5) = 0;
reset.B = zeros(state_size,input_size);
reset.B(5,3) = 1;
trans(1) = transition(guard, reset, 18, 'setVel_EnvLoop_Controller_end');

% _deadlineTimer >= 0
C = [0 0 0 0 0 0 1 0];
d = 0; 
guard = polytope(-C, -d);
reset = identity_reset;
trans(2) = transition(guard, reset, 16);

locs(17) = location(inv, trans, dyn);

%% EnvLoop, Loc 18: QueryUpd_1 setVel communication end, QueryUpd_10 assignment start

% invariant: _deadlineTimer (7) <= 0
C = [0 0 0 0 0 0 1 0];
d = 0; 
inv = polytope(C, d);

dyn = default_dyn;

trans = transition();

guard = fullspace(state_size);
reset = identity_reset; % vel := inarg2
reset.A(2,2) = 0;
reset.A(2,5) = 1;
trans(1) = transition(guard, reset, 14);

% _deadlineTimer (7) >= 0
C = [0 0 0 0 0 0 1 0];
d = 0; 
guard = polytope(-C, -d);
reset = identity_reset;
trans(2) = transition(guard, reset, 16);

locs(18) = location(inv, trans, dyn);

%% EnvLoop, Loc 19: QueryUpd_1 proceed communication end, QueryUpd_6 Skip start

% invariant: _deadlineTimer (7) <= 0
C = [0 0 0 0 0 0 1 0];
d = 0; 
inv = polytope(C, d);

dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
trans(1) = transition(guard, reset, 15);

% _deadlineTimer (7) >= 0
C = [0 0 0 0 0 0 0 1];
d = 0; 
guard = polytope(-C, -d);
reset = identity_reset;
trans(2) = transition(guard, reset, 16);

locs(19) = location(inv, trans, dyn);

comp(1) = hybridAutomaton(locs);

%%
%% EventBuffer

% state:
% 1: targetTrig
% 2: inarg3
% 3: _timer
state_size = 3;

% inputs:
% 1: pos
% 2: vel
% 3: stepTimer
% 4: inarg1
% 5: inarg2
% 6: targetTriggered_inarg
input_size = 6;

A = zeros(state_size,state_size);
B = zeros(state_size,input_size);
c = [0; 0; 1];
C = eye(state_size);
D = zeros(state_size,input_size);
k = zeros(state_size,1);
default_dyn = linearSys(A, B, c, C, D, k);

identity_reset = [];
identity_reset.A = eye(state_size);
identity_reset.B = zeros(state_size,input_size);
identity_reset.c = zeros(state_size,1);

locs = location();

%% EventBuffer, Loc 1: EventBuffer_2 assignment start

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
reset.A(1,1) = 0; % targetTrig := 0
trans(1) = transition(guard, reset, 3);

locs(1) = location(inv, trans, dyn);

%% EventBuffer, Loc 2: Unused end location

inv = fullspace(state_size);
dyn = default_dyn;
trans = transition();

locs(2) = location(inv, trans, dyn);

%% EventBuffer, Loc 3: EventBuffer_3 conditional choice start

inv = emptySet(state_size);
dyn = default_dyn;

trans = transition();

C = [-1 0 0];
d = -0.5; % targetTrig > 0.5
guard = polytope(C,d);
reset = identity_reset;
trans(1) = transition(guard, reset, 4);

C = [1 0 0];
d = 0.5; % targetTrig < 0.5
guard = polytope(C,d);
reset = identity_reset;
trans(2) = transition(guard, reset, 5);

locs(3) = location(inv, trans, dyn);

%% EventBuffer, Loc 4: EventBuffer_4 external choice start

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();

guard = fullspace(state_size);
reset = identity_reset;
trans(1) = transition(guard, reset, 6, 'targetTriggered_EnvLoop_EventBuffer_start');

guard = fullspace(state_size);
reset = identity_reset;
trans(2) = transition(guard, reset, 3, 'target_EventBuffer_Controller');

locs(4) = location(inv, trans, dyn);

%% EventBuffer, Loc 5: EventBuffer_5 input communication start

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();

guard = fullspace(state_size);
reset = identity_reset;
trans(1) = transition(guard, reset, 9, 'targetTriggered_EnvLoop_EventBuffer_start');

locs(5) = location(inv, trans, dyn);

%% EventBuffer, Loc 6: EventBuffer_4 targetTriggered communication intermediate

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();

guard = fullspace(state_size);
reset = identity_reset;
reset.A(2,2) = 0; reset.B(2,6) = 1; % inarg3 (2) := targetTriggered_inarg (6)
trans(1) = transition(guard, reset, 7, 'targetTriggered_EnvLoop_EventBuffer_end');

locs(6) = location(inv, trans, dyn);

%% EventBuffer, Loc 7: EventBuffer_4 targetTriggered communication end, EventBuffer_9 start

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
reset.A(1,1) = 0; reset.A(1,2) = 1; % targetTrig (1) := inarg3 (2)
trans(1) = transition(guard, reset, 3);

locs(7) = location(inv, trans, dyn);


%% EventBuffer, Loc 8: EventBuffer_4 target communication end, EventBuffer 10 Skip start

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
trans(1) = transition(guard, reset, 3);

locs(8) = location(inv, trans, dyn);

%% EventBuffer, Loc 9: EventBuffer_5 targetTriggered communication intermediate

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();

guard = fullspace(state_size);
reset = identity_reset;
reset.A(2,2) = 0; reset.B(2,6) = 1; % inarg3 (2) := targetTriggered_inarg (6)
trans(1) = transition(guard, reset, 10, 'targetTriggered_EnvLoop_EventBuffer_end');

locs(9) = location(inv, trans, dyn);

%% EventBuffer, Loc 10: EventBuffer_5 target communication end, EventBuffer_11 assignnment start

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
reset.A(1,1) = 0; reset.A(1,2) = 1; % targetTrig (1) := inarg3 (2)
trans(1) = transition(guard, reset, 3);

locs(10) = location(inv, trans, dyn);

comp(2) = hybridAutomaton(locs);

%%
%% Controller

% state:
%   1: _timer
%   2: set_vel_arg
state_size = 2;

% inputs:
%   1: dummy
input_size = 1;

A = zeros(state_size,state_size);
B = zeros(state_size,input_size);
c = [0; 1];
C = eye(state_size);
D = zeros(state_size,input_size);
k = zeros(state_size,1);
default_dyn = linearSys(A, B, c, C, D, k);

identity_reset = [];
identity_reset.A = eye(state_size);
%identity_reset.B = zeros(state_size,input_size);
identity_reset.c = zeros(state_size,1);

locs = location();

%% Controller, Loc 1: Controller_2 set_vel!1 start -> 4

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
reset.A(2,2) = 0; reset.c(2) = 1; % set_vel_arg (2) := 1
trans(1) = transition(guard, reset, 4, 'setVel_EnvLoop_Controller_start');

locs(1) = location(inv, trans, dyn);

%% Controller, loc 2: unused end location

inv = fullspace(state_size);
dyn = default_dyn;
trans = transition();

locs(2) = location(inv, trans, dyn);

%% Controller, loc 3: Controller_3 set_vel!(-1) start -> 7

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
reset.A(2,2) = 0; reset.c(2) = -1; % set_vel_arg (2) := -1
trans(1) = transition(guard, reset, 7, 'setVel_EnvLoop_Controller_start');

locs(3) = location(inv, trans, dyn);

%% Controller, loc 4: Controller_2 set_vel!1 intermediate -> 5

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
trans(1) = transition(guard, reset, 5, 'setVel_EnvLoop_Controller_end');

locs(4) = location(inv, trans, dyn);

%% Controller, loc 5: ext_choice of proceed and target -> 6, 5

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();

guard = fullspace(state_size);
reset = identity_reset;
trans(1) = transition(guard, reset, 6, 'target_EventBuffer_Controller');

guard = fullspace(state_size);
reset = identity_reset;
trans(2) = transition(guard, reset, 5, 'proceed_EnvLoop_Controller');

locs(5) = location(inv, trans, dyn);

%% Controller, loc 6: Controller_6 Skip start -> 3

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
trans(1) = transition(guard, reset, 3);

locs(6) = location(inv, trans, dyn);

%% Controller, loc 7: Controller_3 set_vel(-1) intermediate -> 8

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();
guard = fullspace(state_size);
reset = identity_reset;
trans(1) = transition(guard, reset, 8, 'setVel_EnvLoop_Controller_end');

locs(7) = location(inv, trans, dyn);

%% Controller, loc 8: Controller_5 proceed loop -> 8

inv = fullspace(state_size);
dyn = default_dyn;

trans = transition();

guard = fullspace(state_size);
reset = identity_reset;
trans(1) = transition(guard, reset, 8, 'proceed_EnvLoop_Controller');

locs(8) = location(inv, trans, dyn);

comp(3) = hybridAutomaton(locs);

%%
%% Composition

% EnvLoop state:
% 1: pos
% 2: vel
% 3: stepTimer
% 4: inarg1
% 5: inarg2
% 6: _timer
% 7: _deadlineTimer
% 8: targetTriggered_outarg

% EnvLoop inputs:
% 1: targetTrig
% 2: inarg3
% 3: setVel_inarg

% EventBuffer state:
% 1: targetTrig
% 2: inarg3
% 3: _timer

% EventBuffer inputs:
% 1: pos
% 2: vel
% 3: stepTimer
% 4: inarg1
% 5: inarg2
% 6: targetTriggered_inarg

% Controller state:
% 1: _timer
% 2: set_vel_arg

% Controller inputs:
% 1: dummy

iBinds = {};
iBinds{1} = [
    [2, 1];  % targetTrig is connected to targetTrig (1) in EventBuffer (2)
    [2, 2];  % inarg3 is connected to inarg3 (2) in EventBuffer (2)
    [3, 2]]; % setVel_inarg is connected to set_vel_arg (2) in Controller (3)
iBinds{2} = [
    [1, 1];  % pos is connected to pos (1) in EnvLoop (1)
    [1, 2];  % vel is connected to vel (2) in EnvLoop (1)
    [1, 3];  % stepTimer is connected to stepTimer (3) in EnvLoop (1)
    [1, 4];  % inarg1 is connected to inarg1 (4) in EnvLoop (1)
    [1, 5];  % inarg2 is connected to inarg2 (5) in EnvLoop (1)
    [1, 8]]; % targetTriggered_inarg is connected to targetTriggered_outarg (8) in EnvLoop (1)
iBinds{3} = ...
    [0, 1]; % dummy input - connect externally

pHA = parallelHybridAutomaton(comp, iBinds);

c = [...
    % EnvLoop state:
    0; % 1: pos
    0; % 2: vel
    0; % 3: stepTimer
    0; % 4: inarg1
    0; % 5: inarg2
    0; % 6: _timer
    0; % 7: _deadlineTimer
    0; % 8: targetTriggered_outarg
    % EventBuffer state:
    0; % 9: targetTrig
    0; % 10: inarg3
    0; % 11: _timer
    % Controller state:
    0; % 12: _timer
    0; % 13: set_vel_arg
];

G = [...
    % EnvLoop state:
    0; % 1: pos
    0; % 2: vel
    0; % 3: stepTimer
    0; % 4: inarg1
    0; % 5: inarg2
    0; % 6: _timer
    0; % 7: _deadlineTimer
    0; % 8: targetTriggered_outarg
    % EventBuffer state:
    0; % 9: targetTrig
    0; % 10: inarg3
    0; % 11: _timer
    % Controller state:
    0; % 12: _timer
    0; % 13: set_vel_arg
];

% start time
params.tStart = 0;
% final time
params.tFinal = 10;
params.startLoc = [1; 1; 1];
params.R0 = zonotope(c,G);
params.U = zonotope(0, 0);

options.verbose = false;
options.timeStep = 0.001;
options.taylorTerms = 1;
options.zonotopeOrder = 1;
options.linAlg = 'standard';
options.guardOrder = 1;
options.guardIntersect = ['polytope'];
options.enclose = {'box'};
%options.error = 0.1;

tic
R = reach(pHA, params, options);
tEval = toc;

%% checks of properties

Rfiltered = R(arrayfun(@(x) not(isempty(x.timeInterval)), R));

% avoid {x : R^13 | x >= 5}
C = zeros(1,13);
C(1) = 1;
d = 5;
S = polytope(-C, -d);
spec = specification(S);
check1 = spec.check(Rfiltered);

%% Results

disp("Reachability checking time: " + tEval);

if check1
    disp('Avoid {x : R^13 | x >= 5}: true');
else
    disp('Avoid {x : R^13 | x >= 5}: false');
end